#include "Elements.h"
#include "FastMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

namespace ar::circuit
{
namespace
{
    constexpr Real kVt = 0.02585;

    /** Solves the small dense system a·x = b in place (a is n x n row-major, b becomes x). */
    void solveSmall (Real* a, Real* b, int n) noexcept
    {
        for (int k = 0; k < n; ++k)
        {
            int best = k;
            for (int i = k + 1; i < n; ++i)
                if (std::abs (a[i * n + k]) > std::abs (a[best * n + k]))
                    best = i;
            if (best != k)
            {
                for (int j = 0; j < n; ++j) std::swap (a[k * n + j], a[best * n + j]);
                std::swap (b[k], b[best]);
            }
            Real d = a[k * n + k];
            if (std::abs (d) < 1.0e-30) d = 1.0e-30;
            for (int i = k + 1; i < n; ++i)
            {
                const Real f = a[i * n + k] / d;
                if (f == 0.0) continue;
                for (int j = k; j < n; ++j) a[i * n + j] -= f * a[k * n + j];
                b[i] -= f * b[k];
            }
        }
        for (int i = n - 1; i >= 0; --i)
        {
            Real s = b[i];
            for (int j = i + 1; j < n; ++j) s -= a[i * n + j] * b[j];
            b[i] = s / a[i * n + i];
        }
    }

    /** solveSmall for a size known at compile time (unrolled by the compiler). */
    template <int n>
    inline void solveFixed (Real* a, Real* b) noexcept
    {
        for (int k = 0; k < n; ++k)
        {
            int best = k;
            for (int i = k + 1; i < n; ++i)
                if (std::abs (a[i * n + k]) > std::abs (a[best * n + k]))
                    best = i;
            if (best != k)
            {
                for (int j = k; j < n; ++j) std::swap (a[k * n + j], a[best * n + j]);
                std::swap (b[k], b[best]);
            }
            Real d = a[k * n + k];
            if (std::abs (d) < 1.0e-30) d = 1.0e-30;
            const Real inv = 1.0 / d;
            for (int i = k + 1; i < n; ++i)
            {
                const Real f = a[i * n + k] * inv;
                for (int j = k + 1; j < n; ++j) a[i * n + j] -= f * a[k * n + j];
                b[i] -= f * b[k];
            }
            a[k * n + k] = inv;
        }
        for (int i = n - 1; i >= 0; --i)
        {
            Real s = b[i];
            for (int j = i + 1; j < n; ++j) s -= a[i * n + j] * b[j];
            b[i] = s * a[i * n + i];
        }
    }

    /** Fast path for the PI + output-tube Jacobian. Most solves never need a row exchange;
        return false if a pivot becomes unusable so the caller can retry the same system
        with the regular partial-pivoting solver. */
    template <int n>
    inline bool solveFixedUnpivoted (Real* a, Real* b) noexcept
    {
        for (int k = 0; k < n; ++k)
        {
            const Real d = a[k * n + k];
            if (! std::isfinite (d) || std::abs (d) < 1.0e-12)
                return false;
            const Real inv = 1.0 / d;
            for (int i = k + 1; i < n; ++i)
            {
                const Real f = a[i * n + k] * inv;
                for (int j = k + 1; j < n; ++j) a[i * n + j] -= f * a[k * n + j];
                b[i] -= f * b[k];
            }
            a[k * n + k] = inv;
        }
        for (int i = n - 1; i >= 0; --i)
        {
            Real s = b[i];
            for (int j = i + 1; j < n; ++j) s -= a[i * n + j] * b[j];
            b[i] = s * a[i * n + i];
            if (! std::isfinite (b[i]))
                return false;
        }
        return true;
    }

    /** The Newton groups' dense solve (urei1176 change, see docs): LU with partial pivoting in
        place, for a size fixed at compile time (N > 0; the loops then unroll and vectorise) or
        given at run time (N == 0). Same pivot choice and tiny-pivot guard as solveSmall; the
        factors are kept (unit lower multipliers, upper with reciprocal diagonal, whole rows
        exchanged: piv[k] is the row exchanged with k at step k) so the next sample's predictor
        can reuse them. Two rows are eliminated per pass, sharing the loads of the pivot row. */
    template <int N>
    void luFactorFixed (Real* a, int* piv, int nRun) noexcept
    {
        const int n = N > 0 ? N : nRun;
        for (int k = 0; k < n; ++k)
        {
            int best = k;
            Real bestAbs = std::abs (a[k * n + k]);
            for (int i = k + 1; i < n; ++i)
            {
                const Real v = std::abs (a[i * n + k]);
                if (v > bestAbs) { bestAbs = v; best = i; }
            }
            piv[k] = best;
            Real* __restrict rk = a + k * n;   // rows are disjoint
            if (best != k)
            {
                Real* __restrict rb = a + best * n;
                for (int j = 0; j < n; ++j) std::swap (rk[j], rb[j]);
            }
            Real d = rk[k];
            if (std::abs (d) < 1.0e-30) d = 1.0e-30;
            const Real inv = 1.0 / d;
            rk[k] = inv;
            int i = k + 1;
            for (; i + 1 < n; i += 2)
            {
                Real* __restrict r0 = a + i * n;
                Real* __restrict r1 = a + (i + 1) * n;
                const Real f0 = r0[k] * inv, f1 = r1[k] * inv;
                r0[k] = f0;
                r1[k] = f1;
                for (int j = k + 1; j < n; ++j)
                {
                    const Real p = rk[j];
                    r0[j] -= f0 * p;
                    r1[j] -= f1 * p;
                }
            }
            if (i < n)
            {
                Real* __restrict ri = a + i * n;
                const Real f = ri[k] * inv;
                ri[k] = f;
                for (int j = k + 1; j < n; ++j) ri[j] -= f * rk[j];
            }
        }
    }

    /** Solves with luFactorFixed's factors, b in place. Both triangles are swept by columns
        (independent updates rather than one dependent chain of dot products); a fixed size
        works on a local copy of b, which stays in registers. */
    template <int N>
    void luSolveFixed (const Real* __restrict a, const int* __restrict piv, Real* __restrict bInOut, int nRun) noexcept
    {
        const int n = N > 0 ? N : nRun;
        Real local[N > 0 ? N : 1];
        Real* __restrict b = N > 0 ? local : bInOut;
        if constexpr (N > 0)
            for (int k = 0; k < N; ++k) local[k] = bInOut[k];
        for (int k = 0; k < n; ++k)
            if (piv[k] != k) std::swap (b[k], b[piv[k]]);
        for (int k = 0; k + 1 < n; ++k)
        {
            const Real bk = b[k];
            for (int i = k + 1; i < n; ++i) b[i] -= a[i * n + k] * bk;
        }
        for (int i = n - 1; i >= 0; --i)
        {
            const Real x = b[i] * a[i * n + i];
            b[i] = x;
            for (int r = 0; r < i; ++r) b[r] -= a[r * n + i] * x;
        }
        if constexpr (N > 0)
            for (int k = 0; k < N; ++k) bInOut[k] = local[k];
    }

    using LuFactorFn = void (*) (Real*, int*, int) noexcept;
    using LuSolveFn = void (*) (const Real*, const int*, Real*, int) noexcept;
    /** Kernels compiled for sizes 1 ... kFixedKernels; entry 0 is the run-time-size one. */
    constexpr int kFixedKernels = 24;
    template <int... Ns>
    constexpr std::array<LuFactorFn, sizeof... (Ns)> factorTable (std::integer_sequence<int, Ns...>) { return { &luFactorFixed<Ns>... }; }
    template <int... Ns>
    constexpr std::array<LuSolveFn, sizeof... (Ns)> solveTable (std::integer_sequence<int, Ns...>) { return { &luSolveFixed<Ns>... }; }
    constexpr auto kFactorKernels = factorTable (std::make_integer_sequence<int, kFixedKernels + 1> {});
    constexpr auto kSolveKernels = solveTable (std::make_integer_sequence<int, kFixedKernels + 1> {});

    /** exp() that turns linear above a limit, so Newton steps through a forward-biased
        junction stay finite. */
    inline Real limitedExp (Real x, Real& d) noexcept
    {
        constexpr Real xMax = 40.0; // ~ 1 V across a junction with n*Vt = 25 mV
        if (x > xMax)
        {
            const Real e = std::exp (xMax);
            d = e;
            return e * (1.0 + (x - xMax));
        }
        const Real e = std::exp (x);
        d = e;
        return e;
    }
}

//==============================================================================
DeviceGroup::DeviceGroup (std::vector<std::unique_ptr<Device>> d) : devices (std::move (d))
{
    int np = 0;
    for (const auto& dev : devices)
    {
        np += dev->numPorts();
        m += dev->numUnknowns();
        nV += dev->numModelVoltages();
        for (const auto& p : dev->ports)
            ports.push_back (p);
    }
    C.assign ((size_t) np * (size_t) m, 0.0);
    P.assign ((size_t) nV * (size_t) np, 0.0);
    Rint.assign ((size_t) nV * (size_t) m, 0.0);
    u.assign ((size_t) m, 0.0);
    uPrev = u;
    uPrev2 = u;
    uLast = u;
    for (const auto& dev : devices)
    {
        devUnknowns.push_back (dev->numUnknowns());
        devVoltages.push_back (dev->numModelVoltages());
    }
    v0.assign ((size_t) np, 0.0);
    reff.assign ((size_t) np * (size_t) m, 0.0);
    M.assign ((size_t) nV * (size_t) m, 0.0);
    vm0.assign ((size_t) nV, 0.0);
    vm.assign ((size_t) nV, 0.0);
    vEval.assign ((size_t) nV, 0.0);
    vSolved.assign ((size_t) nV, 0.0);
    um.assign ((size_t) m, 0.0);
    dUdV.assign ((size_t) m * (size_t) nV, 0.0);
    F.assign ((size_t) m, 0.0);
    J.assign ((size_t) m * (size_t) m, 0.0);
    du.assign ((size_t) m, 0.0);
    luPivot.assign ((size_t) m, 0);
    luFactorFn = kFactorKernels[(size_t) (m <= kFixedKernels ? m : 0)];
    luSolveFn = kSolveKernels[(size_t) (m <= kFixedKernels ? m : 0)];
    int blockSize = 0;
    for (size_t d = 0; d < devices.size(); ++d)
    {
        gOffset.push_back (blockSize);
        blockSize += devUnknowns[d] * devVoltages[d];
    }
    gBlock.assign ((size_t) blockSize, 0.0);
    vm0Last.assign ((size_t) nV, 0.0);
    dvStep.assign ((size_t) nV, 0.0);
    Mt.assign ((size_t) m * (size_t) nV, 0.0);
    for (const auto& dev : devices)
        for (int i = 0; i < dev->numModelVoltages(); ++i)
            voltageOwner.push_back ({ dev.get(), i });
    for (const auto& p : ports)
        if (std::find (networks.begin(), networks.end(), p.net) == networks.end())
            networks.push_back (p.net);
    rebuildCoefficients();
}

void DeviceGroup::rebuildCoefficients() noexcept
{
    cachedRefactorSum = ~0u;
    std::fill (uPrev.begin(), uPrev.end(), 0.0);
    std::fill (uPrev2.begin(), uPrev2.end(), 0.0);
    std::fill (u.begin(), u.end(), 0.0);
    std::fill (C.begin(), C.end(), 0.0);
    std::fill (P.begin(), P.end(), 0.0);
    std::fill (Rint.begin(), Rint.end(), 0.0);
    const int np = (int) ports.size();
    int p0 = 0, m0 = 0, v0i = 0;
    std::vector<Real> c, p, r;
    for (const auto& dev : devices)
    {
        const int dp = dev->numPorts(), dm = dev->numUnknowns(), dv = dev->numModelVoltages();
        c.assign ((size_t) dp * (size_t) dm, 0.0);
        p.assign ((size_t) dv * (size_t) dp, 0.0);
        r.assign ((size_t) dv * (size_t) dm, 0.0);
        dev->coefficients (c.data(), p.data(), r.data());
        for (int i = 0; i < dp; ++i)
            for (int j = 0; j < dm; ++j)
                C[(size_t) (p0 + i) * (size_t) m + (size_t) (m0 + j)] = c[(size_t) i * (size_t) dm + (size_t) j];
        for (int i = 0; i < dv; ++i)
        {
            for (int j = 0; j < dp; ++j)
                P[(size_t) (v0i + i) * (size_t) np + (size_t) (p0 + j)] = p[(size_t) i * (size_t) dp + (size_t) j];
            for (int j = 0; j < dm; ++j)
                Rint[(size_t) (v0i + i) * (size_t) m + (size_t) (m0 + j)] = r[(size_t) i * (size_t) dm + (size_t) j];
        }
        p0 += dp; m0 += dm; v0i += dv;
    }
    // P and C as sparse lists, in the order of the dense products they replace (the same sums).
    pStart.assign (1, 0);
    pTerms.clear();
    for (int i = 0; i < nV; ++i)
    {
        for (int q = 0; q < np; ++q)
            if (const Real c = P[(size_t) i * (size_t) np + (size_t) q]; c != 0.0)
                pTerms.push_back ({ q, c });
        pStart.push_back ((int) pTerms.size());
    }
    cStart.assign (1, 0);
    cTerms.clear();
    for (int q = 0; q < np; ++q)
    {
        for (int j = 0; j < m; ++j)
            if (const Real c = C[(size_t) q * (size_t) m + (size_t) j]; c != 0.0)
                cTerms.push_back ({ j, c });
        cStart.push_back ((int) cTerms.size());
    }
    predictorReady = false;
}

int DeviceGroup::maxNetIndex() const noexcept
{
    int best = -1;
    for (const auto& p : ports)
        best = std::max (best, p.netIndex);
    return best;
}

void DeviceGroup::refreshCoupling() noexcept
{
    predictorReady = false;   // the factored Jacobian was built on the old M
    const int np = (int) ports.size();
    portNode.assign ((size_t) np, 0);
    otherStart.assign ((size_t) np + 1, 0);
    otherPort.clear();
    otherR.clear();
    auto isOwnPort = [this] (const LinearNetwork* net, int q)
    {
        for (const auto& own : ports)
            if (own.net == net && own.port == q)
                return true;
        return false;
    };
    for (int p = 0; p < np; ++p)
    {
        auto* net = ports[(size_t) p].net;
        const int pp = ports[(size_t) p].port;
        portNode[(size_t) p] = net->portNode (pp);
        for (int q = 0; q < net->numPorts(); ++q)
        {
            const Real r = net->portResistance (pp, q);
            if (r != 0.0 && ! isOwnPort (net, q))
            {
                otherPort.push_back (q);
                otherR.push_back (r);
            }
        }
        otherStart[(size_t) p + 1] = (int) otherPort.size();
    }
    for (int p = 0; p < np; ++p)
    {
        auto* net = ports[(size_t) p].net;
        for (int j = 0; j < m; ++j)
        {
            Real r = 0.0;
            for (int q = 0; q < np; ++q)
                if (ports[(size_t) q].net == net)
                {
                    const Real c = C[(size_t) q * (size_t) m + (size_t) j];
                    if (c != 0.0)
                        r += net->portResistance (ports[(size_t) p].port, ports[(size_t) q].port) * c;
                }
            reff[(size_t) p * (size_t) m + (size_t) j] = r;
        }
    }
    for (int i = 0; i < nV; ++i)
        for (int j = 0; j < m; ++j)
        {
            Real t = Rint[(size_t) i * (size_t) m + (size_t) j];
            for (int p = 0; p < np; ++p)
                t += P[(size_t) i * (size_t) np + (size_t) p] * reff[(size_t) p * (size_t) m + (size_t) j];
            M[(size_t) i * (size_t) m + (size_t) j] = t;
        }
    Mt.resize ((size_t) m * (size_t) nV);   // sized in the constructor: no allocation here
    for (int i = 0; i < nV; ++i)
        for (int j = 0; j < m; ++j)
            Mt[(size_t) j * (size_t) nV + (size_t) i] = M[(size_t) i * (size_t) m + (size_t) j];
}

#if AR_PROFILE_OPS
unsigned long long profModel = 0, profModelCalls = 0, profPrologue = 0, profNewton = 0, profEpilogue = 0;
#endif
bool DeviceGroup::iterate (Real limitScale, int iterations) noexcept
{
    // Newton with per-voltage step limiting (as SPICE does for junctions): the full step is
    // scaled so no model voltage moves further than its device allows.
    auto evaluateAt = [this] (Real& residualNorm)
    {
        for (int i = 0; i < nV; ++i)
        {
            Real s = vm0[(size_t) i];
            for (int j = 0; j < m; ++j)
                s += M[(size_t) i * (size_t) m + (size_t) j] * u[(size_t) j];
            vm[(size_t) i] = s;
        }
        for (int j = 0; j < m; ++j)
            for (int k = 0; k < m; ++k)
                J[(size_t) j * (size_t) m + (size_t) k] = (j == k) ? 1.0 : 0.0;
        int m0 = 0, v0i = 0;
        for (size_t d = 0; d < devices.size(); ++d)
        {
            const int dm = devUnknowns[d], dv = devVoltages[d];
            Real* localJ = dUdV.data();
            std::fill (localJ, localJ + (size_t) dm * (size_t) dv, 0.0);
#if AR_PROFILE_OPS
            { const auto t0 = __builtin_readcyclecounter(); devices[d]->evaluate (vm.data() + v0i, um.data() + m0, localJ); profModel += __builtin_readcyclecounter() - t0; ++profModelCalls; }
#else
            devices[d]->evaluate (vm.data() + v0i, um.data() + m0, localJ);
#endif
            for (int i = 0; i < dm; ++i)
            {
                Real* jrow = &J[(size_t) (m0 + i) * (size_t) m];
                for (int l = 0; l < dv; ++l)
                {
                    const Real g = localJ[(size_t) i * (size_t) dv + (size_t) l];
                    if (g == 0.0) continue;
                    const Real* mrow = &M[(size_t) (v0i + l) * (size_t) m];
                    for (int k = 0; k < m; ++k)
                        jrow[k] -= g * mrow[k];
                }
            }
            m0 += dm; v0i += dv;
        }
        residualNorm = 0.0;
        for (int j = 0; j < m; ++j)
        {
            F[(size_t) j] = um[(size_t) j] - u[(size_t) j];
            residualNorm += F[(size_t) j] * F[(size_t) j];
        }
    };

    Real norm = 0.0;
    evaluateAt (norm);
    if (! std::isfinite (norm))
        return false;

    for (int iter = 0; iter < iterations; ++iter)
    {
        ++statIterations; ++newtonIterations;
        std::copy (F.begin(), F.end(), du.begin());
        solveSmall (J.data(), du.data(), m);

        Real lambda = 1.0, biggest = 0.0;
        for (int i = 0; i < nV; ++i)
        {
            Real dv = 0.0;
            for (int j = 0; j < m; ++j)
                dv += M[(size_t) i * (size_t) m + (size_t) j] * du[(size_t) j];
            // Convergence is judged relative to the size of the voltage: a plate at 400 V
            // does not need the same absolute precision as a grid at -1 V.
            biggest = std::max (biggest, std::abs (dv) / std::max (1.0, std::abs (vm[(size_t) i]) * 0.05));
            if (dv == 0.0) continue;
            const Real allowed = voltageOwner[(size_t) i].first->limitStep (voltageOwner[(size_t) i].second, vm[(size_t) i], dv * limitScale);
            const Real ratio = std::abs (allowed) / std::abs (dv);
            if (ratio < lambda)
                lambda = std::max (ratio, 1.0e-6);
        }
        if (! std::isfinite (biggest))
            return false;
        if (biggest < toleranceVolts)
        {
            // Converged; the step just computed is free and leaves only a quadratic error.
            // (Returning without it left up to the tolerance at every plate, which the stages
            // downstream amplify into audible hash.)
            for (int j = 0; j < m; ++j)
                u[(size_t) j] += du[(size_t) j];
            return true;
        }

        for (int j = 0; j < m; ++j)
            u[(size_t) j] += lambda * du[(size_t) j];
        if (lambda == 1.0 && biggest < acceptLimit (iter == 0 && limitScale == 1.0))
            return true;   // a full, tiny step: converged to working precision, no re-evaluation needed
        evaluateAt (norm);
        if (! std::isfinite (norm))
            return false;
    }
    return false;
}

bool DeviceGroup::iterateLimited (int iterations) noexcept
{
    // SPICE's scheme: every model voltage is limited on its own (pnjlim), the devices are
    // linearised at those limited voltages, and Newton takes the full step of the linearised
    // system. In iterate() one junction far from its solution scales down every variable's
    // step, which makes a large group (four merged fuzz stages) crawl through 10-30 steps.
    // The limits are taken from the last solution, which also tames the predictor's
    // overshoot on a junction that switches.
    std::copy (vSolved.begin(), vSolved.end(), vEval.begin());
    const Real* Mp = M.data();
    const Real* Mtp = Mt.data();
    Real* Jp = J.data();
    // The model voltages of the starting point; after that they follow the steps (vm += M du).
    // M x is summed column by column (the same additions, in the same order, as row by row).
    const int mm = m, nv = nV;
#ifndef PRODUCT_ROWS
    auto productM = [mm, nv, Mtp] (const Real* __restrict x, Real* __restrict out)
    {
        for (int j = 0; j < mm; ++j)
        {
            const Real xj = x[j];
            const Real* __restrict col = Mtp + (size_t) j * (size_t) nv;
            for (int i = 0; i < nv; ++i)
                out[i] += col[i] * xj;
        }
    };
#else
    auto productM = [mm, nv, Mp] (const Real* __restrict x, Real* __restrict out)
    {
        for (int i = 0; i < nv; ++i)
        {
            Real s = out[i];
            const Real* __restrict row = Mp + (size_t) i * (size_t) mm;
            for (int j = 0; j < mm; ++j)
                s += row[j] * x[j];
            out[i] = s;
        }
    };
#endif
    std::copy (vm0.begin(), vm0.end(), vm.begin());
    productM (u.data(), vm.data());

    for (int iter = 0; iter < iterations; ++iter)
    {
        bool limited = false;
        for (int i = 0; i < nV; ++i)
        {
            const Real old = vEval[(size_t) i], want = vm[(size_t) i] - old;
            const auto& owner = voltageOwner[(size_t) i];
            const Real allowed = want == 0.0 ? 0.0 : owner.first->limitStep (owner.second, old, want);
            limited = limited || allowed != want;
            vEval[(size_t) i] = old + allowed;
        }
        // The devices at the limited voltages, and J = I - G M row by row (G, each device's
        // dU/dV, kept in gBlock for the next sample's predictor).
        std::fill (gBlock.begin(), gBlock.end(), 0.0);
        int m0 = 0, v0i = 0;
        for (size_t d = 0; d < devices.size(); ++d)
        {
            const int dm = devUnknowns[d], dv = devVoltages[d];
            Real* localJ = gBlock.data() + gOffset[d];
            devices[d]->evaluate (vEval.data() + v0i, um.data() + m0, localJ);
            for (int i = 0; i < dm; ++i)
            {
                Real* jrow = Jp + (size_t) (m0 + i) * (size_t) m;
                for (int k = 0; k < m; ++k)
                    jrow[k] = 0.0;
                jrow[m0 + i] = 1.0;
                for (int l = 0; l < dv; ++l)
                {
                    const Real g = localJ[(size_t) i * (size_t) dv + (size_t) l];
                    if (g == 0.0) continue;
                    // The device's tangent at the limited voltage, carried to the actual one.
                    um[(size_t) (m0 + i)] += g * (vm[(size_t) (v0i + l)] - vEval[(size_t) (v0i + l)]);
                    const Real* mrow = Mp + (size_t) (v0i + l) * (size_t) m;
                    for (int k = 0; k < m; ++k)
                        jrow[k] -= g * mrow[k];
                }
            }
            m0 += dm; v0i += dv;
        }
        Real norm = 0.0;
        for (int j = 0; j < m; ++j)
        {
            const Real f = um[(size_t) j] - u[(size_t) j];
            du[(size_t) j] = f;
            norm += f * f;
        }
        if (! std::isfinite (norm))
            return false;
        ++statIterations; ++newtonIterations;
        luFactorFn (Jp, luPivot.data(), m);
        luSolveFn (Jp, luPivot.data(), du.data(), m);

        Real biggest = 0.0;
        Real* dvm = dvStep.data();
        std::fill (dvStep.begin(), dvStep.end(), 0.0);
        productM (du.data(), dvm);
        for (int i = 0; i < nV; ++i)
        {
            const Real dv = dvm[i];
            biggest = std::max (biggest, std::abs (dv) / std::max (1.0, std::abs (vm[(size_t) i]) * 0.05));
        }
        if (! std::isfinite (biggest))
            return false;
        for (int j = 0; j < m; ++j)
            u[(size_t) j] += du[(size_t) j];
        for (int i = 0; i < nV; ++i)
            vm[(size_t) i] += dvm[i];
        // Converged when the step was exact, or small with the devices evaluated where the
        // solution is (no voltage held back by a limit).
        if (! limited && (biggest < toleranceVolts || biggest < acceptLimit (iter == 0)))
        {
            predictorReady = true;   // J holds the factors of the last step's Jacobian
            return true;
        }
    }
    return false;
}

void DeviceGroup::predict() noexcept
{
    if (predictorReady)
    {
        // The solution's first-order response to the change in the open-circuit model
        // voltages: J du = G dvm0, with last sample's factors and conductances.
        Real* corr = du.data();
        int m0 = 0, v0i = 0;
        for (size_t d = 0; d < devices.size(); ++d)
        {
            const int dm = devUnknowns[d], dv = devVoltages[d];
            const Real* g = gBlock.data() + gOffset[d];
            for (int i = 0; i < dm; ++i)
            {
                Real sum = 0.0;
                for (int l = 0; l < dv; ++l)
                    sum += g[i * dv + l] * (vm0[(size_t) (v0i + l)] - vm0Last[(size_t) (v0i + l)]);
                corr[m0 + i] = sum;
            }
            m0 += dm; v0i += dv;
        }
        luSolveFn (J.data(), luPivot.data(), corr, m);
        bool finite = true;
        for (int j = 0; j < m; ++j)
            finite = finite && std::isfinite (corr[j]);
        if (finite)
        {
            for (int j = 0; j < m; ++j)
                u[(size_t) j] += corr[j];
            return;
        }
    }
    // Quadratic extrapolation of the last three solutions.
    for (int j = 0; j < m; ++j)
        u[(size_t) j] = 3.0 * (u[(size_t) j] - uPrev[(size_t) j]) + uPrev2[(size_t) j];
}

void DeviceGroup::computeOpenVoltages() noexcept
{
    const int np = (int) ports.size();
    const int* start = otherStart.data();
    const int* oq = otherPort.data();
    const Real* orr = otherR.data();
    for (int p = 0; p < np; ++p)
    {
        const auto* net = ports[(size_t) p].net;
        Real v = net->openNodeVoltage (portNode[(size_t) p]);
        for (int k = start[p]; k < start[p + 1]; ++k)
            v += orr[k] * net->portCurrent (oq[k]);
        v0[(size_t) p] = v;
    }
}

void DeviceGroup::solve (bool resweep) noexcept
{
#if AR_PROFILE_OPS
    const auto tp0 = __builtin_readcyclecounter();
#endif
    const int np = (int) ports.size();

    unsigned sum = 0;
    for (const auto* net : networks)
        sum += net->refactorCount();
    if (sum != cachedRefactorSum)
    {
        refreshCoupling();
        cachedRefactorSum = sum;
    }
    computeOpenVoltages();

    for (int i = 0; i < nV; ++i)
    {
        Real s = 0.0;
        for (int t = pStart[(size_t) i]; t < pStart[(size_t) i + 1]; ++t)
            s += pTerms[(size_t) t].coefficient * v0[(size_t) pTerms[(size_t) t].index];
        vm0[(size_t) i] = s;
    }

#if AR_PROFILE_OPS
    profPrologue += __builtin_readcyclecounter() - tp0;
    const auto tn0 = __builtin_readcyclecounter();
#endif
    ++statSolves; ++newtonSolves;

    // Start from a prediction; if it fails to converge, retry from the last solution with smaller steps.
    if (! resweep)
    {
        std::copy (u.begin(), u.end(), uLast.begin());
        predict();
    }

    bool ok = iterateLimited (maxIterations);
    // iterateLimited leaves vm at the solution's model voltages; the fallback does not.
    const bool vmCurrent = ok;
    if (! ok)
    {
        predictorReady = false;   // the fallback reuses J and does not keep the conductances
        std::copy (uLast.begin(), uLast.end(), u.begin());
        ok = iterate (0.25, maxIterations * 3);
        if (! ok)
        {
            ++statFailures;
            ++failures;
            static int debugLeft = std::getenv ("AR_DEBUG_NEWTON") ? 3 : 0;
            if (debugLeft > 0)
            {
                --debugLeft;
                std::printf ("Newton failure: m=%d nV=%d\n  u:", m, nV);
                for (Real v : u) std::printf (" %.4g", v);
                std::printf ("\n  uLast:");
                for (Real v : uLast) std::printf (" %.4g", v);
                std::printf ("\n  vm0:");
                for (Real v : vm0) std::printf (" %.3f", v);
                std::printf ("\n  vm:");
                for (Real v : vm) std::printf (" %.3f", v);
                std::printf ("\n  F:");
                for (Real v : F) std::printf (" %.4g", v);
                std::printf ("\n  M:");
                for (Real v : M) std::printf (" %.4g", v);
                std::printf ("\n");
            }
            bool finite = true;
            for (Real v : u) finite = finite && std::isfinite (v);
            if (! finite)
                std::copy (uLast.begin(), uLast.end(), u.begin());
        }
    }
    if (vmCurrent)
        std::copy (vm.begin(), vm.end(), vSolved.begin());
    else
        for (int i = 0; i < nV; ++i)
        {
            Real s = vm0[(size_t) i];
            for (int j = 0; j < m; ++j)
                s += M[(size_t) i * (size_t) m + (size_t) j] * u[(size_t) j];
            vSolved[(size_t) i] = s;
        }
    std::copy (vm0.begin(), vm0.end(), vm0Last.begin());   // what u was solved against
    if (! resweep)
    {
        std::swap (uPrev2, uPrev);   // uPrev2 <- uPrev
        std::copy (uLast.begin(), uLast.end(), uPrev.begin());
    }
#if AR_PROFILE_OPS
    profNewton += __builtin_readcyclecounter() - tn0;
    const auto te0 = __builtin_readcyclecounter();
#endif

    for (int p = 0; p < np; ++p)
    {
        Real i = 0.0;
        for (int t = cStart[(size_t) p]; t < cStart[(size_t) p + 1]; ++t)
            i += cTerms[(size_t) t].coefficient * u[(size_t) cTerms[(size_t) t].index];
        ports[(size_t) p].net->setPortCurrent (ports[(size_t) p].port, i);
    }
#if AR_PROFILE_OPS
    profEpilogue += __builtin_readcyclecounter() - te0;
#endif
}

//==============================================================================
DiodePairDevice::DiodePairDevice (DiodeSpec f, DiodeSpec r, PortRef a, PortRef b, Real s) : fwd (f), rev (r), scale (s)
{
    ports = { a, b };
    updateLimits();
}

void DiodePairDevice::coefficients (Real* C, Real* P, Real* Rint) const noexcept
{
    C[0] = -1.0;  // current a->b leaves node a
    C[1] = 1.0;
    P[0] = 1.0; P[1] = -1.0;
    Rint[0] = -std::max (fwd.is > 0.0 ? fwd.rs : 0.0, rev.is > 0.0 ? rev.rs : 0.0);   // v_junction = v_ab - rs i
}

/** A junction whose reverse current is its spec's leakage instead of is:
    i = is (e^x - r - 2 (1 - r) s(x)), r = leakage / is, s the logistic. The Shockley current
    for x >> 0, zero at x = 0, -leakage for x << 0, smooth throughout. di is d i / d x. */
static void leakyJunction (const DiodeSpec& spec, Real x, Real& i, Real& di) noexcept
{
    Real de;
    const Real e = limitedExp (x, de);
    const Real r = spec.leakage / spec.is, s = 1.0 / (1.0 + std::exp (-x));
    i = spec.is * (e - r - 2.0 * (1.0 - r) * s);
    di = spec.is * (de - 2.0 * (1.0 - r) * s * (1.0 - s));
}

void DiodePairDevice::evaluate (const Real* v, Real* u, Real* j) const noexcept
{
    Real i = 0.0, d = 0.0;
    if (fwd.leakage >= 0.0 || rev.leakage >= 0.0)
    {
        if (fwd.is > 0.0)
        {
            Real ij, dj;
            if (fwd.leakage >= 0.0)
                leakyJunction (fwd, v[0] / (fwd.n * kVt), ij, dj);
            else
            {
                ij = fwd.is * (limitedExp (v[0] / (fwd.n * kVt), dj) - 1.0);
                dj *= fwd.is;
            }
            i += scale * ij;
            d += scale * dj / (fwd.n * kVt);
        }
        if (rev.is > 0.0)
        {
            Real ij, dj;
            if (rev.leakage >= 0.0)
                leakyJunction (rev, -v[0] / (rev.n * kVt), ij, dj);
            else
            {
                ij = rev.is * (limitedExp (-v[0] / (rev.n * kVt), dj) - 1.0);
                dj *= rev.is;
            }
            i -= scale * ij;
            d += scale * dj / (rev.n * kVt);
        }
        u[0] = i;
        j[0] = d;
        return;
    }
    if (fwd.is > 0.0)
    {
        Real de;
        const Real e = limitedExp (v[0] / (fwd.n * kVt), de);
        i += scale * fwd.is * (e - 1.0);
        d += scale * fwd.is * de / (fwd.n * kVt);
    }
    if (rev.is > 0.0)
    {
        Real de;
        const Real e = limitedExp (-v[0] / (rev.n * kVt), de);
        i -= scale * rev.is * (e - 1.0);
        d += scale * rev.is * de / (rev.n * kVt);
    }
    u[0] = i;
    j[0] = d;
}

JfetDevice::JfetDevice (JfetSpec s, PortRef g, PortRef d, PortRef src) : spec (s)
{
    ports = { g, d, src };
}

void JfetDevice::coefficients (Real* C, Real* P, Real*) const noexcept
{
    // ports: 0 gate, 1 drain, 2 source; unknown: Id (into the drain, out of the source); model: vgs, vds
    C[1] = -1.0;
    C[2] = 1.0;
    P[0 * 3 + 0] = 1.0; P[0 * 3 + 2] = -1.0;
    P[1 * 3 + 1] = 1.0; P[1 * 3 + 2] = -1.0;
}

void JfetDevice::evaluate (const Real* v, Real* u, Real* j) const noexcept
{
    // Square law written symmetrically in source and drain (forward minus reverse):
    //     id = beta (sp(vgs - vp)^2 - sp(vgd - vp)^2) (1 + lambda |vds|),  vgd = vgs - vds,
    // sp a softplus with a 30 mV knee for the subthreshold tail. Ohmic region: beta (2 vov vds
    // - vds^2); saturation (vgd below pinch-off): beta vov^2; vds < 0 swaps the roles of
    // drain and source by itself. id(vds = 0) is exactly 0 at every vgs, with no step in
    // id or its slope there: a voltage-controlled resistor (the 1176's gain cell) sees
    // tens of millivolts across it, where the earlier form (sp(sp(vgs - vp) - vds) in the
    // reverse term, mirrored for vds < 0) jumped by up to 0.8 uA near pinch-off.
    constexpr Real k = 0.03;
    const Real vgs = v[0], vds = v[1];
    Real spF, sgF, spR, sgR;
    fast::softplusSigmoid ((vgs - spec.vp) / k, spF, sgF);
    fast::softplusSigmoid ((vgs - vds - spec.vp) / k, spR, sgR);
    spF *= k; spR *= k;
    const Real f = spF * spF - spR * spR;
    const Real m = 1.0 + spec.lambda * std::abs (vds);
    const Real dm = vds >= 0.0 ? spec.lambda : -spec.lambda; // dm/dvds
    u[0] = spec.beta * f * m;
    j[0] = spec.beta * 2.0 * (spF * sgF - spR * sgR) * m;
    j[1] = spec.beta * (2.0 * spR * sgR * m + f * dm);
}

BjtDevice::BjtDevice (BjtSpec s, PortRef b, PortRef c, PortRef e) : spec (s), vOnGuard (0.026 * std::log (kJunctionGuardAmps / s.is))
{
    ports = { b, c, e };
}

void BjtDevice::coefficients (Real* C, Real* P, Real*) const noexcept
{
    // ports: 0 base, 1 collector, 2 emitter; unknowns: 0 Ic, 1 Ib; model: vbe, vbc. A PNP has
    // the same model on veb and vcb, its currents flowing the other way: both matrices negated.
    const Real s = spec.pnp ? -1.0 : 1.0;
    C[0 * 2 + 1] = -s;
    C[1 * 2 + 0] = -s;
    C[2 * 2 + 0] = s; C[2 * 2 + 1] = s;
    P[0 * 3 + 0] = s; P[0 * 3 + 2] = -s;
    P[1 * 3 + 0] = s; P[1 * 3 + 1] = -s;
}

void BjtDevice::evaluate (const Real* v, Real* u, Real* j) const noexcept
{
    Real dF, dR;
    const Real eF = limitedExp (v[0] / kVt, dF), eR = limitedExp (v[1] / kVt, dR);
    const Real is = spec.is;
    const Real iF = is * (eF - 1.0), iR = is * (eR - 1.0);
    const Real diF = is * dF / kVt, diR = is * dR / kVt;
    u[0] = iF - iR - iR / spec.betaR;                 // Ic
    u[1] = iF / spec.betaF + iR / spec.betaR;         // Ib
    j[0 * 2 + 0] = diF;                               // dIc/dvbe
    j[0 * 2 + 1] = -diR - diR / spec.betaR;           // dIc/dvbc
    j[1 * 2 + 0] = diF / spec.betaF;
    j[1 * 2 + 1] = diR / spec.betaR;
}
} // namespace ar::circuit
