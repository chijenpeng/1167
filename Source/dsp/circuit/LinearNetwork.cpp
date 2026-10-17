#include "LinearNetwork.h"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <stdexcept>

namespace ar::circuit
{
namespace
{
    constexpr Real kOpenConductance = 1.0e-12;   // an open switch
    constexpr Real kDcCapConductance = 1.0e-10;  // keeps cap-only nodes defined in DC mode
    constexpr Real kDcInductorOhms = 0.05;
    constexpr Real kMinPotOhms = 5.0;
    constexpr Real kPi = 3.14159265358979323846;
}

Real taperFraction (Taper taper, Real p) noexcept
{
    p = std::clamp (p, 0.0, 1.0);
    constexpr Real b = 20.0; // ~17% of the track at half rotation, like a modern audio pot
    switch (taper)
    {
        case Taper::audio:        return (std::pow (b, p) - 1.0) / (b - 1.0);
        case Taper::reverseAudio: return 1.0 - (std::pow (b, 1.0 - p) - 1.0) / (b - 1.0);
        case Taper::linear:
        default:                  return p;
    }
}

LinearNetwork::LinearNetwork (std::string n) : name (std::move (n))
{
    nodeNames.push_back ("gnd");
}

int LinearNetwork::node (const std::string& nodeName)
{
    if (nodeName == "gnd" || nodeName == "0")
        return 0;
    for (size_t i = 0; i < nodeNames.size(); ++i)
        if (nodeNames[i] == nodeName)
            return (int) i;
    nodeNames.push_back (nodeName);
    needsRefactor = true;
    return (int) nodeNames.size() - 1;
}

void LinearNetwork::resistor (int a, int b, Real ohms)          { resistors.push_back ({ a, b, ohms }); needsRefactor = true; }
void LinearNetwork::capacitor (int a, int b, Real farads)       { capacitors.push_back ({ a, b, farads }); needsRefactor = true; }
void LinearNetwork::inductor (int a, int b, Real henries)       { inductors.push_back ({ a, b, henries }); needsRefactor = true; }

int LinearNetwork::warpedCapacitor (int a, int b, Real farads)
{
    capacitor (a, b, farads);
    return (int) capacitors.size() - 1;
}

Real LinearNetwork::companionConductance (const Capacitor& c) const noexcept
{
    const Real g = 2.0 * c.farads / T;
    if (c.warpHz <= 0.0)
        return g;
    const Real x = std::min (kPi * c.warpHz * T, 0.49 * kPi);   // kept below Nyquist
    return g * x / std::tan (x);
}

void LinearNetwork::setCapacitorWarp (int index, Real hz)
{
    auto& c = capacitors[(size_t) index];
    if (c.warpHz == hz)
        return;
    c.warpHz = hz;
    if (! prepared)
        return;   // prepare() sets the conductance
    const Real g = companionConductance (c);
    // Keep the capacitor's voltage and current: the next history term is -(g v + i).
    if (c.enabled && ! dcMode)
        c.hist += (c.g - g) * c.vPrev;
    c.g = g;
    needsRefactor = true;
}

int LinearNetwork::potentiometer (int a, int w, int b, Real ohms, Taper taper, Real position)
{
    pots.push_back ({ a, w, b, ohms, taper, position });
    needsRefactor = true;
    return (int) pots.size() - 1;
}

int LinearNetwork::switchResistor (int a, int b, Real ohms, bool closed)
{
    resistors.push_back ({ a, b, ohms });
    switches.push_back ({ (int) resistors.size() - 1, false, closed });
    needsRefactor = true;
    return (int) switches.size() - 1;
}

int LinearNetwork::switchCapacitor (int a, int b, Real farads, bool closed)
{
    capacitors.push_back ({ a, b, farads });
    capacitors.back().enabled = closed;
    switches.push_back ({ (int) capacitors.size() - 1, true, closed });
    needsRefactor = true;
    return (int) switches.size() - 1;
}

int LinearNetwork::voltageSource (int n, Real volts)
{
    sources.push_back ({ n, volts });
    needsRefactor = true;
    return (int) sources.size() - 1;
}

void LinearNetwork::coupledInductors (int a1, int b1, int a2, int b2, Real L1, Real L2, Real M)
{
    coupled.push_back ({ a1, b1, a2, b2, L1, L2, M });
    needsRefactor = true;
}

void LinearNetwork::vcvs (int out, int inPlus, int inMinus, Real gain)
{
    vcvsList.push_back ({ out, inPlus, inMinus, gain });
    needsRefactor = true;
}

int LinearNetwork::port (int n)   { ports.push_back (n); needsRefactor = true; return (int) ports.size() - 1; }
int LinearNetwork::probe (int n)  { probes.push_back (n); return (int) probes.size() - 1; }

void LinearNetwork::setPot (int index, Real position)
{
    auto& pot = pots[(size_t) index];
    position = std::clamp (position, 0.0, 1.0);
    if (pot.position != position)
    {
        pot.position = position;
        needsRefactor = true;
    }
}

void LinearNetwork::setSwitch (int index, bool closed)
{
    auto& sw = switches[(size_t) index];
    if (sw.closed == closed)
        return;
    sw.closed = closed;
    if (sw.isCapacitor)
    {
        auto& c = capacitors[(size_t) sw.component];
        c.enabled = closed;
        c.hist = 0; c.vPrev = 0;
    }
    needsRefactor = true;
}

void LinearNetwork::setSource (int index, Real volts)
{
    sources[(size_t) index].volts = volts;
}

void LinearNetwork::prepare (Real sampleRate)
{
    T = 1.0 / sampleRate;
    full = numNodes() - 1 + (int) sources.size() + (int) vcvsList.size();
    A.assign ((size_t) full * (size_t) full, 0.0);
    rhsFull.assign ((size_t) full, 0.0);
    x.assign ((size_t) full, 0.0);

    // A node driven by a grounded voltage source has a known voltage: it and the source's
    // branch row leave the system (the first source on a node only; sources on ground are
    // dummies). The rest is the reduced system the per-sample solves work on.
    const int nn = numNodes() - 1;
    srcEliminated.assign (sources.size(), 0);
    nodeSource.assign ((size_t) nn, -1);
    for (size_t k = 0; k < sources.size(); ++k)
    {
        const int node = sources[k].node;
        if (node <= 0) { srcEliminated[k] = 1; continue; }
        if (nodeSource[(size_t) node - 1] >= 0) continue;
        nodeSource[(size_t) node - 1] = (int) k;
        srcEliminated[k] = 1;
    }
    posOf.assign ((size_t) full, -1);
    freeIdx.clear();
    for (int i = 0; i < full; ++i)
    {
        const bool fixedNode = i < nn && nodeSource[(size_t) i] >= 0;
        const bool elimSource = i >= nn && i < nn + (int) sources.size() && srcEliminated[(size_t) (i - nn)];
        if (fixedNode || elimSource) continue;
        posOf[(size_t) i] = (int) freeIdx.size();
        freeIdx.push_back (i);
    }
    dim = (int) freeIdx.size();
    fixedNodes.clear();
    for (int i = 0; i < nn; ++i)
        if (nodeSource[(size_t) i] >= 0)
            fixedNodes.push_back ({ i, nodeSource[(size_t) i] });
    srcPorts.assign (sources.size(), {});
    for (size_t q = 0; q < ports.size(); ++q)
        if (ports[q] > 0 && nodeSource[(size_t) ports[q] - 1] >= 0)
            srcPorts[(size_t) nodeSource[(size_t) ports[q] - 1]].push_back ((int) q);

    lu.assign ((size_t) dim * (size_t) dim, 0.0);
    pivot.assign ((size_t) dim, 0);
    xh.assign ((size_t) dim, 0.0);
    xr.assign ((size_t) dim, 0.0);
    scratch.assign ((size_t) dim, 0.0);
    unitRhs.assign ((size_t) dim, 0.0);
    // Worst-case capacity for what factorise() refills, so refactoring on the audio thread
    // never reallocates (urei1176 change, see docs).
    lCol.reserve ((size_t) dim * (size_t) dim); lVal.reserve ((size_t) dim * (size_t) dim);
    uCol.reserve ((size_t) dim * (size_t) dim); uVal.reserve ((size_t) dim * (size_t) dim);
    couplings.reserve ((size_t) dim * (size_t) nn);
    srcRowCol.reserve (sources.size() * (size_t) full); srcRowVal.reserve (sources.size() * (size_t) full);
    portSlot.assign (ports.size(), -1);
    slotPos.clear();
    for (size_t q = 0; q < ports.size(); ++q)
    {
        const int pos = reducedIndex (ports[q]);
        if (pos < 0) continue;
        const auto it = std::find (slotPos.begin(), slotPos.end(), pos);
        portSlot[q] = (int) (it - slotPos.begin());
        if (it == slotPos.end())
            slotPos.push_back (pos);
    }
    slotZ.assign (slotPos.size() * (size_t) dim, 0.0);
    slotI.assign (slotPos.size(), 0.0);
    portR.assign (ports.size() * ports.size(), 0.0);
    portI.assign (ports.size(), 0.0);
    probeV.assign (probes.size(), 0.0);

    for (auto& c : capacitors) { c.g = companionConductance (c); c.hist = 0; c.vPrev = 0; }
    for (auto& l : inductors)  { l.g = T / (2.0 * l.henries); l.hist = 0; }
    for (auto& p : coupled)
    {
        // i = (T/2) L^-1 (v + v_prev) + i_prev, with L = [[L1, M], [M, L2]].
        const Real det = p.L1 * p.L2 - p.M * p.M;
        p.g11 = 0.5 * T * p.L2 / det;
        p.g22 = 0.5 * T * p.L1 / det;
        p.g12 = -0.5 * T * p.M / det;
        p.hist1 = p.hist2 = 0.0;
    }

    prepared = true;
    needsRefactor = true;
    refactorIfNeeded();
}

void LinearNetwork::setDcMode (bool dc)
{
    if (dc == dcMode)
        return;

    if (! dc)
    {
        // Leaving DC: seed the reactive histories from the operating point.
        for (auto& c : capacitors)
        {
            c.vPrev = nodeVoltage (c.a) - nodeVoltage (c.b);
            c.hist = c.enabled ? -c.g * c.vPrev : 0.0;
        }
        for (auto& l : inductors)
        {
            const Real i = (nodeVoltage (l.a) - nodeVoltage (l.b)) / kDcInductorOhms;
            l.hist = i; // trapezoidal: next step's history is g*v + i, with v ~ 0 at DC
        }
        for (auto& p : coupled)
        {
            p.hist1 = (nodeVoltage (p.a1) - nodeVoltage (p.b1)) / kDcInductorOhms;
            p.hist2 = (nodeVoltage (p.a2) - nodeVoltage (p.b2)) / kDcInductorOhms;
        }
    }
    dcMode = dc;
    needsRefactor = true;
}

void LinearNetwork::stampConductance (int a, int b, Real g) noexcept
{
    const int ia = a - 1, ib = b - 1;
    if (ia >= 0) A[(size_t) ia * (size_t) full + (size_t) ia] += g;
    if (ib >= 0) A[(size_t) ib * (size_t) full + (size_t) ib] += g;
    if (ia >= 0 && ib >= 0)
    {
        A[(size_t) ia * (size_t) full + (size_t) ib] -= g;
        A[(size_t) ib * (size_t) full + (size_t) ia] -= g;
    }
}

void LinearNetwork::refactorIfNeeded()
{
    if (! needsRefactor || ! prepared)
        return;

    std::fill (A.begin(), A.end(), 0.0);

    for (size_t i = 0; i < resistors.size(); ++i)
    {
        Real g = 1.0 / resistors[i].ohms;
        for (const auto& sw : switches)
            if (! sw.isCapacitor && sw.component == (int) i && ! sw.closed)
                g = kOpenConductance;
        stampConductance (resistors[i].a, resistors[i].b, g);
    }

    for (const auto& c : capacitors)
    {
        Real g = c.enabled ? (dcMode ? kDcCapConductance : c.g) : kOpenConductance;
        stampConductance (c.a, c.b, g);
    }

    for (const auto& l : inductors)
        stampConductance (l.a, l.b, dcMode ? 1.0 / kDcInductorOhms : l.g);

    for (const auto& p : coupled)
    {
        if (dcMode)
        {
            stampConductance (p.a1, p.b1, 1.0 / kDcInductorOhms);
            stampConductance (p.a2, p.b2, 1.0 / kDcInductorOhms);
            continue;
        }
        stampConductance (p.a1, p.b1, p.g11);
        stampConductance (p.a2, p.b2, p.g22);
        // Cross terms: branch 1's current depends on branch 2's voltage and vice versa.
        auto stampCross = [this] (int a, int b, int c, int d, Real g)
        {
            // current g*(v_c - v_d) flows from a to b
            const int ia = a - 1, ib = b - 1, ic = c - 1, id = d - 1;
            if (ia >= 0 && ic >= 0) A[(size_t) ia * (size_t) full + (size_t) ic] += g;
            if (ia >= 0 && id >= 0) A[(size_t) ia * (size_t) full + (size_t) id] -= g;
            if (ib >= 0 && ic >= 0) A[(size_t) ib * (size_t) full + (size_t) ic] -= g;
            if (ib >= 0 && id >= 0) A[(size_t) ib * (size_t) full + (size_t) id] += g;
        };
        stampCross (p.a1, p.b1, p.a2, p.b2, p.g12);
        stampCross (p.a2, p.b2, p.a1, p.b1, p.g12);
    }

    for (const auto& p : pots)
    {
        const Real f = taperFraction (p.taper, p.position);
        const Real rWB = std::max (kMinPotOhms, p.ohms * f);
        const Real rAW = std::max (kMinPotOhms, p.ohms * (1.0 - f));
        stampConductance (p.a, p.w, 1.0 / rAW);
        stampConductance (p.w, p.b, 1.0 / rWB);
    }

    const int n = numNodes() - 1;
    for (size_t k = 0; k < sources.size(); ++k)
    {
        const int in = sources[k].node - 1, row = n + (int) k;
        if (in >= 0)
        {
            A[(size_t) in * (size_t) full + (size_t) row] += 1.0;
            A[(size_t) row * (size_t) full + (size_t) in] += 1.0;
        }
        else
        {
            A[(size_t) row * (size_t) full + (size_t) row] = 1.0; // source on ground: dummy row
        }
    }

    for (size_t k = 0; k < vcvsList.size(); ++k)
    {
        const auto& v = vcvsList[k];
        const int row = n + (int) sources.size() + (int) k;
        const int io = v.out - 1, ip = v.inPlus - 1, im = v.inMinus - 1;
        // Branch current enters the output node; row: v[out] - g v[in+] + g v[in-] = 0.
        if (io >= 0)
        {
            A[(size_t) io * (size_t) full + (size_t) row] += 1.0;
            A[(size_t) row * (size_t) full + (size_t) io] += 1.0;
        }
        if (ip >= 0) A[(size_t) row * (size_t) full + (size_t) ip] -= v.gain;
        if (im >= 0) A[(size_t) row * (size_t) full + (size_t) im] += v.gain;
    }

    factorise();
    rebuildPortResponses();
    needsRefactor = false;
    ++refactors;
}

void LinearNetwork::factorise()
{
    // Reduced matrix, the fixed-node columns as right-hand-side couplings, and the rows of
    // the fixed nodes (for their sources' currents).
    const int nn = numNodes() - 1;
    for (int i = 0; i < dim; ++i)
        for (int j = 0; j < dim; ++j)
            lu[(size_t) i * (size_t) dim + (size_t) j] = A[(size_t) freeIdx[(size_t) i] * (size_t) full + (size_t) freeIdx[(size_t) j]];
    couplings.clear();
    for (int i = 0; i < dim; ++i)
        for (int c = 0; c < nn; ++c)
        {
            const int k = nodeSource[(size_t) c];
            const Real g = A[(size_t) freeIdx[(size_t) i] * (size_t) full + (size_t) c];
            if (k >= 0 && g != 0.0)
                couplings.push_back ({ i, k, g });
        }
    srcRowStart.assign (sources.size() + 1, 0);
    srcRowCol.clear(); srcRowVal.clear();
    for (size_t k = 0; k < sources.size(); ++k)
    {
        const int node = sources[k].node;
        if (srcEliminated[k] && node > 0 && nodeSource[(size_t) node - 1] == (int) k)
            for (int c = 0; c < full; ++c)
            {
                const Real g = A[(size_t) (node - 1) * (size_t) full + (size_t) c];
                if (g != 0.0 && c != nn + (int) k)
                {
                    srcRowCol.push_back (c);
                    srcRowVal.push_back (g);
                }
            }
        srcRowStart[k + 1] = (int) srcRowCol.size();
    }

    for (int i = 0; i < dim; ++i)
        pivot[(size_t) i] = i;

    // LU with partial pivoting (Doolittle), stored in place.
    for (int k = 0; k < dim; ++k)
    {
        int best = k;
        Real bestAbs = std::abs (lu[(size_t) k * (size_t) dim + (size_t) k]);
        for (int i = k + 1; i < dim; ++i)
        {
            const Real v = std::abs (lu[(size_t) i * (size_t) dim + (size_t) k]);
            if (v > bestAbs) { bestAbs = v; best = i; }
        }
        if (best != k)
        {
            for (int j = 0; j < dim; ++j)
                std::swap (lu[(size_t) k * (size_t) dim + (size_t) j], lu[(size_t) best * (size_t) dim + (size_t) j]);
            std::swap (pivot[(size_t) k], pivot[(size_t) best]);
        }
        Real& d = lu[(size_t) k * (size_t) dim + (size_t) k];
        if (std::abs (d) < 1.0e-18)
            d = 1.0e-18; // floating node: keep going rather than divide by zero
        for (int i = k + 1; i < dim; ++i)
        {
            Real& m = lu[(size_t) i * (size_t) dim + (size_t) k];
            m /= d;
            if (m != 0.0)
                for (int j = k + 1; j < dim; ++j)
                    lu[(size_t) i * (size_t) dim + (size_t) j] -= m * lu[(size_t) k * (size_t) dim + (size_t) j];
        }
    }

    // Circuit matrices are sparse and nearly banded, so keep only the non-zeros of L and U.
    lRowStart.assign ((size_t) dim + 1, 0); lCol.clear(); lVal.clear();
    uRowStart.assign ((size_t) dim + 1, 0); uCol.clear(); uVal.clear();
    uDiag.assign ((size_t) dim, 0.0);
    for (int i = 0; i < dim; ++i)
    {
        const Real* row = &lu[(size_t) i * (size_t) dim];
        for (int j = 0; j < i; ++j)
            if (row[j] != 0.0) { lCol.push_back (j); lVal.push_back (row[j]); }
        lRowStart[(size_t) i + 1] = (int) lCol.size();
        uDiag[(size_t) i] = 1.0 / row[i];
        for (int j = i + 1; j < dim; ++j)
            if (row[j] != 0.0) { uCol.push_back (j); uVal.push_back (row[j]); }
        uRowStart[(size_t) i + 1] = (int) uCol.size();
    }
}

void LinearNetwork::solveInPlace (std::vector<Real>& b) const noexcept
{
    // b holds the right-hand side; on return it holds the solution.
    auto& tmp = scratch;
    for (int i = 0; i < dim; ++i)
        tmp[(size_t) i] = b[(size_t) pivot[(size_t) i]];
    for (int i = 0; i < dim; ++i)
    {
        Real s = tmp[(size_t) i];
        for (int k = lRowStart[(size_t) i]; k < lRowStart[(size_t) i + 1]; ++k)
            s -= lVal[(size_t) k] * tmp[(size_t) lCol[(size_t) k]];
        tmp[(size_t) i] = s;
    }
    for (int i = dim - 1; i >= 0; --i)
    {
        Real s = tmp[(size_t) i];
        for (int k = uRowStart[(size_t) i]; k < uRowStart[(size_t) i + 1]; ++k)
            s -= uVal[(size_t) k] * tmp[(size_t) uCol[(size_t) k]];
        tmp[(size_t) i] = s * uDiag[(size_t) i];   // uDiag holds reciprocals
    }
    b.swap (tmp);
}

void LinearNetwork::rebuildPortResponses()
{
    // One response per node that carries ports; a port on a fixed node (or ground) has none:
    // its voltage is the source's.
    auto& e = unitRhs;   // preallocated: no allocation on the audio thread
    for (size_t k = 0; k < slotPos.size(); ++k)
    {
        std::fill (e.begin(), e.end(), 0.0);
        e[(size_t) slotPos[k]] = 1.0;
        solveInPlace (e);
        std::copy (e.begin(), e.end(), slotZ.begin() + (long) (k * (size_t) dim));
    }
    for (size_t p = 0; p < ports.size(); ++p)
    {
        const int pos = reducedIndex (ports[p]);
        for (size_t q = 0; q < ports.size(); ++q)
        {
            const int slot = portSlot[q];
            portR[p * ports.size() + q] = pos >= 0 && slot >= 0 ? slotZ[(size_t) slot * (size_t) dim + (size_t) pos] : 0.0;
        }
    }
}

void LinearNetwork::beginSample()
{
    // The full right-hand side (history currents into node rows), gathered into the reduced
    // system with the fixed nodes' voltages moved across, and solved in place.
    auto& rhs = rhsFull;
    std::fill (rhs.begin(), rhs.end(), 0.0);
    const int n = numNodes() - 1;

    if (! dcMode)
    {
        for (const auto& c : capacitors)
        {
            if (! c.enabled) continue;
            // Companion current source c.hist flows from a to b.
            if (c.a > 0) rhs[(size_t) c.a - 1] -= c.hist;
            if (c.b > 0) rhs[(size_t) c.b - 1] += c.hist;
        }
        for (const auto& l : inductors)
        {
            if (l.a > 0) rhs[(size_t) l.a - 1] -= l.hist;
            if (l.b > 0) rhs[(size_t) l.b - 1] += l.hist;
        }
        for (const auto& p : coupled)
        {
            if (p.a1 > 0) rhs[(size_t) p.a1 - 1] -= p.hist1;
            if (p.b1 > 0) rhs[(size_t) p.b1 - 1] += p.hist1;
            if (p.a2 > 0) rhs[(size_t) p.a2 - 1] -= p.hist2;
            if (p.b2 > 0) rhs[(size_t) p.b2 - 1] += p.hist2;
        }
    }
    for (size_t k = 0; k < sources.size(); ++k)
        rhs[(size_t) n + k] = sources[k].node > 0 ? sources[k].volts : 0.0;

    const int* fi = freeIdx.data();
    Real* r = xh.data();
    for (int i = 0; i < dim; ++i)
        r[i] = rhs[(size_t) fi[i]];
    for (const auto& c : couplings)
        r[c.row] -= c.g * sources[(size_t) c.source].volts;
    solveInPlace (xh);
}

Real LinearNetwork::sourceCurrent (int k) const noexcept
{
    if (! srcEliminated[(size_t) k])
        return x[(size_t) (numNodes() - 1 + k)];
    const int node = sources[(size_t) k].node;
    if (node <= 0)
        return 0.0;
    // KCL at the fixed node: the source supplies what the rest of the row does not.
    Real j = rhsFull[(size_t) node - 1];
    for (int q : srcPorts[(size_t) k])
        j += portI[(size_t) q];
    for (int i = srcRowStart[(size_t) k]; i < srcRowStart[(size_t) k + 1]; ++i)
        j -= srcRowVal[(size_t) i] * x[(size_t) srcRowCol[(size_t) i]];
    return j;
}

void LinearNetwork::sourceAdmittance (int index, std::vector<Real>& out) const
{
    // Response of every source current to a unit step of source `index`, devices open.
    const int nn = numNodes() - 1;
    std::vector<Real> e ((size_t) dim, 0.0), dx ((size_t) full, 0.0);
    if (srcEliminated[(size_t) index])
    {
        for (const auto& c : couplings)
            if (c.source == index)
                e[(size_t) c.row] -= c.g;
        const int node = sources[(size_t) index].node;
        if (node > 0 && nodeSource[(size_t) node - 1] == index)
            dx[(size_t) node - 1] = 1.0;
    }
    else
    {
        e[(size_t) posOf[(size_t) (nn + index)]] = 1.0;
    }
    solveInPlace (e);
    for (int i = 0; i < dim; ++i)
        dx[(size_t) freeIdx[(size_t) i]] = e[(size_t) i];
    out.assign (sources.size(), 0.0);
    for (size_t m = 0; m < sources.size(); ++m)
    {
        if (! srcEliminated[m])
        {
            out[m] = dx[(size_t) nn + m];
            continue;
        }
        Real j = 0.0;
        for (int i = srcRowStart[m]; i < srcRowStart[m + 1]; ++i)
            j -= srcRowVal[(size_t) i] * dx[(size_t) srcRowCol[(size_t) i]];
        out[m] = j;
    }
}

std::string LinearNetwork::describe() const
{
    return name + ": dim=" + std::to_string (dim) + " ports=" + std::to_string (ports.size()) + " caps=" + std::to_string (capacitors.size())
         + " sources=" + std::to_string (sources.size()) + " probes=" + std::to_string (probes.size())
         + " nnzLU=" + std::to_string (lVal.size() + uVal.size() + (size_t) dim);
}

void LinearNetwork::dumpReactive (const char* label) const
{
    std::printf ("[%s %s]\n", name.c_str(), label);
    for (const auto& c : capacitors)
    {
        const Real v = nodeVoltage (c.a) - nodeVoltage (c.b);
        std::printf ("  C %s-%s %.3g F: v=%.3f vPrev=%.3f i=%.4g hist=%.4g\n", nodeNames[(size_t) c.a].c_str(), nodeNames[(size_t) c.b].c_str(), c.farads, v, c.vPrev, c.g * v + c.hist, c.hist);
    }
    for (const auto& l : inductors)
    {
        const Real v = nodeVoltage (l.a) - nodeVoltage (l.b);
        std::printf ("  L %s-%s %.3g H: v=%.4f i=%.4g hist=%.4g\n", nodeNames[(size_t) l.a].c_str(), nodeNames[(size_t) l.b].c_str(), l.henries, v, l.g * v + l.hist, l.hist);
    }
    for (const auto& p : coupled)
        std::printf ("  K %s-%s / %s-%s hist=%.4g %.4g\n", nodeNames[(size_t) p.a1].c_str(), nodeNames[(size_t) p.b1].c_str(), nodeNames[(size_t) p.a2].c_str(), nodeNames[(size_t) p.b2].c_str(), p.hist1, p.hist2);
}

void LinearNetwork::endSample()
{
    Real* __restrict xp = xr.data();
    const Real* xhp = xh.data();
    for (int k = 0; k < dim; ++k)
        xp[k] = xhp[k];
    std::fill (slotI.begin(), slotI.end(), 0.0);
    for (size_t q = 0; q < ports.size(); ++q)
        if (const int slot = portSlot[q]; slot >= 0)
            slotI[(size_t) slot] += portI[q];
    for (size_t k = 0; k < slotPos.size(); ++k)
    {
        const Real i = slotI[k];
        if (i == 0.0) continue;
        const Real* __restrict z = &slotZ[k * (size_t) dim];
        for (int j = 0; j < dim; ++j)
            xp[j] += i * z[j];
    }
    // Back to the full layout: free unknowns, then the fixed nodes' voltages.
    {
        Real* xf = x.data();
        const int* fi = freeIdx.data();
        for (int k = 0; k < dim; ++k)
            xf[fi[k]] = xp[k];
        for (const auto& f : fixedNodes)
            xf[f.first] = sources[(size_t) f.second].volts;
    }

    if (! dcMode)
    {
        for (auto& c : capacitors)
        {
            if (! c.enabled) continue;
            const Real v = nodeVoltage (c.a) - nodeVoltage (c.b);
            // i_n = g v_n + hist_n ; hist_{n+1} = -(g v_n + i_n)
            c.hist = -(2.0 * c.g * v + c.hist);
            c.vPrev = v;
        }
        for (auto& l : inductors)
        {
            const Real v = nodeVoltage (l.a) - nodeVoltage (l.b);
            l.hist = 2.0 * l.g * v + l.hist;
        }
        for (auto& p : coupled)
        {
            const Real v1 = nodeVoltage (p.a1) - nodeVoltage (p.b1), v2 = nodeVoltage (p.a2) - nodeVoltage (p.b2);
            // i_n = G v_n + hist_n ; hist_{n+1} = G v_n + i_n
            const Real i1 = p.g11 * v1 + p.g12 * v2 + p.hist1, i2 = p.g12 * v1 + p.g22 * v2 + p.hist2;
            p.hist1 = p.g11 * v1 + p.g12 * v2 + i1;
            p.hist2 = p.g12 * v1 + p.g22 * v2 + i2;
        }
    }

    for (size_t k = 0; k < probes.size(); ++k)
        probeV[k] = nodeVoltage (probes[k]);
}
} // namespace ar::circuit
