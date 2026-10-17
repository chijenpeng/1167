#include "Circuit.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <cmath>
#include <stdexcept>

namespace ar::circuit
{
namespace
{
/** Solves the S x S row-major system a x = b in place (b becomes x), Gaussian elimination
    with partial pivoting; a zero pivot leaves that unknown at zero. For S of 1-3. */
void solveDense (Real* a, Real* b, int S) noexcept
{
    for (int k = 0; k < S; ++k)
    {
        int best = k;
        for (int i = k + 1; i < S; ++i)
            if (std::abs (a[i * S + k]) > std::abs (a[best * S + k])) best = i;
        if (best != k)
        {
            for (int j = 0; j < S; ++j) std::swap (a[k * S + j], a[best * S + j]);
            std::swap (b[k], b[best]);
        }
        const Real d = a[k * S + k];
        if (d == 0.0) continue;
        for (int i = k + 1; i < S; ++i)
        {
            const Real f = a[i * S + k] / d;
            for (int j = k; j < S; ++j) a[i * S + j] -= f * a[k * S + j];
            b[i] -= f * b[k];
        }
    }
    for (int i = S - 1; i >= 0; --i)
    {
        Real v = b[i];
        for (int j = i + 1; j < S; ++j) v -= a[i * S + j] * b[j];
        const Real d = a[i * S + i];
        b[i] = d != 0.0 ? v / d : 0.0;
    }
}
} // namespace

LinearNetwork& Circuit::addNetwork (const std::string& name)
{
    nets.push_back (std::make_unique<LinearNetwork> (name));
    return *nets.back();
}

int Circuit::indexOf (const LinearNetwork& net) const
{
    for (size_t i = 0; i < nets.size(); ++i)
        if (nets[i].get() == &net)
            return (int) i;
    throw std::logic_error ("network is not part of this circuit");
}

PortRef Circuit::port (LinearNetwork& net, int node)
{
    return { &net, net.port (node), indexOf (net) };
}

PortRef Circuit::port (LinearNetwork& net, const std::string& nodeName)
{
    return port (net, net.node (nodeName));
}

DeviceGroup& Circuit::addGroup (std::vector<std::unique_ptr<Device>> devices)
{
    elements.push_back (std::make_unique<DeviceGroup> (std::move (devices)));
    return *elements.back();
}

void Circuit::addBuffer (LinearNetwork& from, int fromNode, LinearNetwork& to, int toSource, Real gain)
{
    buffers.push_back ({ indexOf (from), from.probe (fromNode), indexOf (to), toSource, gain });
}

void Circuit::addSlowSupply (LinearNetwork& net, int sourceIndex, Real farads)
{
    slow.push_back ({ indexOf (net), sourceIndex, farads });
    const size_t S = slow.size();
    slowJ.assign (S * S, 0.0);
    slowA.assign (S * S, 0.0);
    slowB.assign (S, 0.0);
}

bool Circuit::isSlowSource (int net, int source) const noexcept
{
    for (const auto& s : slow)
        if (s.net == net && s.source == source)
            return true;
    return false;
}

void Circuit::resetSlowSupplies() noexcept
{
    for (auto& s : slow)
    {
        s.state = s.applied = nets[(size_t) s.net]->getSource (s.source);
        s.ramp = s.sumI = s.sumV = 0.0;
    }
    slowCount = 0;
    slowRefactorSum = ~0u;
}

void Circuit::stepSlowSupplies() noexcept
{
    const int S = (int) slow.size();
    for (auto& s : slow)
    {
        s.sumI += nets[(size_t) s.net]->sourceCurrent (s.source);
        s.sumV += s.applied;
    }
    ++slowCount;
    if (! slowDc && slowCount < kSlowSupplyInterval)
    {
        for (auto& s : slow)
        {
            s.applied += s.ramp;
            nets[(size_t) s.net]->setSource (s.source, s.applied);
        }
        return;
    }

    // The admittance between the slow sources through the linear network (devices open),
    // refreshed whenever a network involved is refactored.
    unsigned sum = 0;
    for (const auto& s : slow) sum += nets[(size_t) s.net]->refactorCount();
    if (sum != slowRefactorSum)
    {
        slowRefactorSum = sum;
        std::vector<Real> resp;
        for (int b = 0; b < S; ++b)
        {
            nets[(size_t) slow[(size_t) b].net]->sourceAdmittance (slow[(size_t) b].source, resp);
            for (int a = 0; a < S; ++a)
                slowJ[(size_t) (a * S + b)] = slow[(size_t) a].net == slow[(size_t) b].net ? resp[(size_t) slow[(size_t) a].source] : 0.0;
        }
    }

    // Capacitor currents j (into each cap) depend on the cap voltages V through J = dj/dV.
    // AC: C dV/dt = j, backward Euler over the interval, with the measured mean current
    // moved from the mean applied voltage to the present state:
    //     (C/h - J) dV = jMean + J (V - Vmean)
    // DC (the operating-point search): no capacitor current, J dV = -j.
    const Real n = (Real) slowCount;
    const Real h = slowStepSeconds;
    for (int a = 0; a < S; ++a)
    {
        const auto& sa = slow[(size_t) a];
        Real rhs = sa.sumI / n;
        for (int b = 0; b < S; ++b)
        {
            const Real jab = slowJ[(size_t) (a * S + b)];
            if (! slowDc)
                rhs += jab * (slow[(size_t) b].state - slow[(size_t) b].sumV / n);
            slowA[(size_t) (a * S + b)] = (a == b && ! slowDc ? sa.farads / h : 0.0) - jab;
        }
        slowB[(size_t) a] = rhs;
    }
    solveDense (slowA.data(), slowB.data(), S);

    for (int a = 0; a < S; ++a)
    {
        auto& s = slow[(size_t) a];
        s.state += slowB[(size_t) a];
        s.sumI = s.sumV = 0.0;
        if (slowDc)
        {
            s.applied = s.state;
            s.ramp = 0.0;
        }
        else
        {
            // Move to the new state over the next interval rather than in one step.
            s.ramp = (s.state - s.applied) / (Real) kSlowSupplyInterval;
            s.applied += s.ramp;
        }
        nets[(size_t) s.net]->setSource (s.source, s.applied);
    }
    slowCount = 0;
}

void Circuit::setInput (LinearNetwork& net, int sourceIndex)   { inputNet = indexOf (net); inputSource = sourceIndex; }
void Circuit::setOutput (LinearNetwork& net, int node)          { outputNet = indexOf (net); outputProbe = net.probe (node); }

void Circuit::setNetworkSweeps (LinearNetwork& net, int n)
{
    const auto i = (size_t) indexOf (net);
    if (netSweeps.size() <= i)
        netSweeps.resize (i + 1, 0);
    netSweeps[i] = n;
}

void Circuit::finalize()
{
    ops.clear();
    sweepLists.clear();
    sweepPasses.clear();
    const int n = (int) nets.size();
    std::vector<bool> ended ((size_t) n, false), solved (elements.size(), false);

    auto netIsDone = [&] (int k)
    {
        for (size_t e = 0; e < elements.size(); ++e)
            if (! solved[e])
                for (const auto& p : elements[e]->getPorts())
                    if (p.netIndex == k)
                        return false;
        return true;
    };

    for (int k = 0; k < n; ++k)
    {
        for (size_t b = 0; b < buffers.size(); ++b)
            if (buffers[b].toNet == k && ! ended[(size_t) buffers[b].fromNet])
            {
                if (! netIsDone (buffers[b].fromNet))
                    throw std::logic_error ("buffer source network " + nets[(size_t) buffers[b].fromNet]->getName()
                                            + " still has unsolved elements when " + nets[(size_t) k]->getName() + " begins");
                ops.push_back ({ Op::end, buffers[b].fromNet });
                ended[(size_t) buffers[b].fromNet] = true;
            }

        for (size_t b = 0; b < buffers.size(); ++b)
            if (buffers[b].toNet == k)
                ops.push_back ({ Op::buffer, (int) b });

        ops.push_back ({ Op::begin, k });

        for (size_t e = 0; e < elements.size(); ++e)
            if (! solved[e] && elements[e]->maxNetIndex() == k)
            {
                ops.push_back ({ Op::solve, (int) e });
                solved[e] = true;
            }

        for (int j = 0; j <= k; ++j)
            if (! ended[(size_t) j] && netIsDone (j))
            {
                const int passes = std::max (groupSweeps, (size_t) j < netSweeps.size() ? netSweeps[(size_t) j] : 0);
                if (passes > 1)
                {
                    std::vector<int> list;
                    for (size_t e = 0; e < elements.size(); ++e)
                        if (solved[e] && elements[e]->singleNetwork() && elements[e]->maxNetIndex() == j)
                            list.push_back ((int) e);
                    if (list.size() > 1)
                    {
                        sweepLists.push_back (std::move (list));
                        sweepPasses.push_back (passes);
                        ops.push_back ({ Op::sweep, (int) sweepLists.size() - 1 });
                    }
                }
                ops.push_back ({ Op::end, j });
                ended[(size_t) j] = true;
            }
    }

    for (int j = 0; j < n; ++j)
        if (! ended[(size_t) j])
            ops.push_back ({ Op::end, j });

    finalized = true;
}

void Circuit::mergeCoupledGroups()
{
    // Groups in one network that load each other strongly (a stage driving the next through a
    // few kilohms, an EQ section) are merged so they are solved jointly: solved separately each
    // sees the other's current a sample late, which loses top end. The coupling of two groups
    // is the largest |R(p, q)| / sqrt(R(p, p) R(q, q)) over their port pairs, from the networks'
    // port resistances at the default control positions.
    const size_t n = elements.size();
    std::vector<size_t> parent (n);
    for (size_t i = 0; i < n; ++i) parent[i] = i;
    std::function<size_t (size_t)> find = [&] (size_t i) { return parent[i] == i ? i : (parent[i] = find (parent[i])); };
    const bool verbose = std::getenv ("AR_SHOW_COUPLING") != nullptr;
    for (size_t a = 0; a < n; ++a)
        for (size_t b = a + 1; b < n; ++b)
        {
            const auto& ea = *elements[a]; const auto& eb = *elements[b];
            if (! ea.singleNetwork() || ! eb.singleNetwork() || ea.getPorts()[0].net != eb.getPorts()[0].net)
                continue;
            const auto* net = ea.getPorts()[0].net;
            // Groups are solved in index order, so a (earlier) sees b's (later) currents from the
            // previous sample. What matters is how much each current-carrying port of b moves a's
            // port voltages, relative to a's own effect there.
            Real c = 0.0;
            const auto& pa = ea.getPorts(); const auto& pb = eb.getPorts();
            for (size_t j = 0; j < pb.size(); ++j)
            {
                if (! eb.injects ((int) j)) continue;
                for (size_t i = 0; i < pa.size(); ++i)
                {
                    const Real raa = std::abs (net->portResistance (pa[i].port, pa[i].port));
                    if (raa < 1.0e-6) continue;   // a port on a fixed node
                    c = std::max (c, std::abs (net->portResistance (pa[i].port, pb[j].port)) / raa);
                }
            }
            if (verbose) std::printf ("coupling %zu-%zu: %.3f\n", a, b, c);
            if (c > mergeThreshold)
                parent[find (b)] = find (a);
        }
    std::vector<std::vector<std::unique_ptr<Device>>> merged (n);
    bool any = false;
    for (size_t i = 0; i < n; ++i)
        if (find (i) != i) { any = true; break; }
    if (! any)
        return;
    std::vector<std::unique_ptr<DeviceGroup>> next;
    for (size_t i = 0; i < n; ++i)
        for (auto& d : elements[i]->takeDevices())
            merged[find (i)].push_back (std::move (d));
    for (size_t i = 0; i < n; ++i)
        if (! merged[i].empty())
            next.push_back (std::make_unique<DeviceGroup> (std::move (merged[i])));
    elements = std::move (next);
    finalize();   // the op sequence refers to group indices
}

void Circuit::prepare (Real sampleRate)
{
    if (! finalized)
        finalize();
    for (auto& net : nets)
        net->prepare (sampleRate);
    if (mergeGroups && ! merged)
    {
        mergeCoupledGroups();
        merged = true;
    }
    for (auto& e : elements)
        e->resetState();
    slowStepSeconds = (Real) kSlowSupplyInterval / sampleRate;
    resetSlowSupplies();
}

void Circuit::refactorIfNeeded()
{
    for (auto& net : nets)
        net->refactorIfNeeded();
}

void Circuit::findOperatingPoint (int iterations)
{
    for (auto& net : nets) net->setDcMode (true);
    refactorIfNeeded();
    resetSlowSupplies();
    slowDc = true;
    // Source stepping, as SPICE does when a cold start fails: every supply ramps up over the
    // first half of the iterations, so direct-coupled stages (a follower whose cathode load
    // returns to a negative rail, say) never see hundreds of volts of grid drive at once.
    std::vector<std::vector<Real>> saved (nets.size());
    for (size_t n = 0; n < nets.size(); ++n)
        for (int k = 0; k < nets[n]->numSources(); ++k)
            saved[n].push_back (nets[n]->getSource (k));
    const int ramp = std::max (1, iterations / 2);
    for (int i = 0; i < iterations; ++i)
    {
        const Real scale = std::min (1.0, (Real) (i + 1) / (Real) ramp);
        for (size_t n = 0; n < nets.size(); ++n)
            for (int k = 0; k < nets[n]->numSources(); ++k)
                if (! isSlowSource ((int) n, k))
                    nets[n]->setSource (k, saved[n][(size_t) k] * scale);
        process (0.0);
    }
    for (size_t n = 0; n < nets.size(); ++n)
        for (int k = 0; k < nets[n]->numSources(); ++k)
            if (! isSlowSource ((int) n, k))
                nets[n]->setSource (k, saved[n][(size_t) k]);
    for (auto& net : nets) net->setDcMode (false);
    refactorIfNeeded();
    slowDc = false;
    for (auto& s : slow) { s.ramp = s.sumI = s.sumV = 0.0; }
    slowCount = 0;
}

#if AR_PROFILE_OPS
unsigned long long profBegin = 0, profSolve = 0, profEnd = 0, profSamples = 0;
unsigned long long profGroupIters[64] = {}, profGroupTicks[64] = {};
static inline unsigned long long cyc() noexcept { return __builtin_readcyclecounter(); }
#endif

Real Circuit::process (Real inputVolts) noexcept
{
    if (inputNet >= 0)
        nets[(size_t) inputNet]->setSource (inputSource, inputVolts);
#if AR_PROFILE_OPS
    ++profSamples;
#endif

    for (const auto& op : ops)
    {
        switch (op.type)
        {
#if AR_PROFILE_OPS
            case Op::begin:  { const auto t = cyc(); nets[(size_t) op.index]->beginSample(); profBegin += cyc() - t; break; }
            case Op::solve:  { const auto t = cyc(); const auto it0 = DeviceGroup::statIterations; elements[(size_t) op.index]->solve(); const auto d = cyc() - t; profSolve += d; if (op.index < 64) { profGroupIters[op.index] += DeviceGroup::statIterations - it0; profGroupTicks[op.index] += d; } break; }
            case Op::end:    { const auto t = cyc(); nets[(size_t) op.index]->endSample(); profEnd += cyc() - t; break; }
            case Op::sweep:  for (int pass = 1; pass < sweepPasses[(size_t) op.index]; ++pass) for (int e : sweepLists[(size_t) op.index]) elements[(size_t) e]->solve (true); break;
#else
            case Op::begin:  nets[(size_t) op.index]->beginSample(); break;
            case Op::solve:  elements[(size_t) op.index]->solve(); break;
            case Op::end:    nets[(size_t) op.index]->endSample(); break;
            case Op::sweep:
                for (int pass = 1; pass < sweepPasses[(size_t) op.index]; ++pass)
                    for (int e : sweepLists[(size_t) op.index])
                        elements[(size_t) e]->solve (true);
                break;
#endif
            case Op::buffer:
            {
                const auto& b = buffers[(size_t) op.index];
                nets[(size_t) b.toNet]->setSource (b.toSource, b.gain * nets[(size_t) b.fromNet]->probeVoltage (b.fromProbe));
                break;
            }
        }
    }

    if (! slow.empty())
        stepSlowSupplies();
    return outputNet >= 0 ? nets[(size_t) outputNet]->probeVoltage (outputProbe) : 0.0;
}
} // namespace ar::circuit
