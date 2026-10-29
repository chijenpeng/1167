#pragma once

// A small dense linear network, discretised by the trapezoidal rule exactly as the reference
// engine (circuit/LinearNetwork) does, and reduced to a discrete state-space system:
//
//     y   = C h + D u          h' = A h + B u
//
// The states h are the capacitors' and inductors' companion history sources, so a network of
// n_c capacitors and n_l inductors has n_c + n_l states and nothing else: no algebraic nodes
// remain at run time. The inputs u are ideal voltage sources at nodes and injected currents (the
// nonlinear devices' ports); the outputs are node voltages and source currents.
//
// Nothing here is used per sample except through the plain matrices; build() is for prepare()
// and for control ticks (a few thousand flops for the sizes used, no allocation).

#include <algorithm>
#include <cmath>
#include <cstring>

namespace urei1176::fast
{
class TrapNet
{
public:
    static constexpr int kMaxM = 30;      // MNA dimension (nodes + sources + inductor branches)
    static constexpr int kMaxS = 12;      // states
    static constexpr int kMaxI = 8;       // inputs
    static constexpr int kMaxO = 10;      // outputs
    static constexpr int kMaxEl = 64;     // elements of each kind

    void clear() noexcept
    {
        nNodes = 0; nSrc = 0; nInj = 0; nCoup = 0; nOut = 0; nR = nC = nG = nQ = 0; nVcvs = 0;
        nIn = 0;
    }

    int node() noexcept { return ++nNodes; }

    void resistor (int a, int b, double ohms) noexcept { r[nR++] = { a, b, 1.0 / ohms }; }
    void capacitor (int a, int b, double farads) noexcept { c[nC++] = { a, b, farads }; }
    /** Current gm (v_c - v_d) flows from a to b. */
    void vccs (int a, int b, int cc, int d, double gm) noexcept { g[nG++] = { a, b, cc, d, gm }; }
    /** An ideal voltage-controlled voltage source: v(out) = gain (v(cp) - v(cm)), the node `out` driven against ground (an extra MNA
        row, no state). For the op-amp macro-model of rev G's and J's input. */
    void vcvs (int out, int cp, int cm, double gain) noexcept { vc[nVcvs++] = { out, cp, cm, gain }; }
    /** An ideal voltage source from ground to `node`: input `index`, its delivered current output
        `outSourceCurrent(index)`. */
    int source (int node) noexcept { src[nSrc] = node; inputKind[nIn] = 0; inputSlot[nIn] = nSrc; ++nSrc; return nIn++; }
    /** A current of value u flowing from a to b through a device (leaves a). */
    int inject (int a, int b) noexcept { inj[nInj] = { a, b }; inputKind[nIn] = 1; inputSlot[nIn] = nInj; ++nInj; return nIn++; }
    void coupledInductors (int a1, int b1, int a2, int b2, double l1, double l2, double m) noexcept
    {
        coup[nCoup++] = { a1, b1, a2, b2, l1, l2, m };
    }
    /** Output: v(a) - v(b). */
    int outVoltage (int a, int b = 0) noexcept { out[nOut] = { 0, a, b }; return nOut++; }
    /** Output: the current source input `inputIndex` delivers into the network. */
    int outSourceCurrent (int inputIndex) noexcept { out[nOut] = { 1, inputSlot[inputIndex], 0 }; return nOut++; }

    /** Builds the system for sample period T. Returns false when the matrix is singular. */
    bool build (double T) noexcept
    {
        const int nn = nNodes;
        const int mm = nn + nSrc + 2 * nCoup + nVcvs;
        if (mm > kMaxM) return false;
        ns = nC + 2 * nCoup;
        if (ns > kMaxS || nIn > kMaxI || nOut > kMaxO) return false;
        m = mm;
        std::memset (M, 0, sizeof (M));
        std::memset (Bu, 0, sizeof (Bu));
        std::memset (Bh, 0, sizeof (Bh));
        std::memset (Q, 0, sizeof (Q));
        std::memset (kappa, 0, sizeof (kappa));
        auto ix = [] (int node) { return node - 1; };   // node 0 (ground) has no row
        auto stampG = [&] (int a, int b, double gg)
        {
            if (a > 0) M[ix (a)][ix (a)] += gg;
            if (b > 0) M[ix (b)][ix (b)] += gg;
            if (a > 0 && b > 0) { M[ix (a)][ix (b)] -= gg; M[ix (b)][ix (a)] -= gg; }
        };
        for (int k = 0; k < nR; ++k) stampG (r[k].a, r[k].b, r[k].g);
        for (int k = 0; k < nG; ++k)
        {
            const auto& e = g[k];
            if (e.a > 0 && e.c > 0) M[ix (e.a)][ix (e.c)] += e.gm;
            if (e.a > 0 && e.d > 0) M[ix (e.a)][ix (e.d)] -= e.gm;
            if (e.b > 0 && e.c > 0) M[ix (e.b)][ix (e.c)] -= e.gm;
            if (e.b > 0 && e.d > 0) M[ix (e.b)][ix (e.d)] += e.gm;
        }
        // capacitors: companion conductance 2C/T and history current h (i = g v + h), h' = -h - 2 g v
        for (int k = 0; k < nC; ++k)
        {
            const double gg = 2.0 * c[k].f / T;
            stampG (c[k].a, c[k].b, gg);
            if (c[k].a > 0) Bh[ix (c[k].a)][k] -= 1.0;
            if (c[k].b > 0) Bh[ix (c[k].b)][k] += 1.0;
            if (c[k].a > 0) Q[k][ix (c[k].a)] += 1.0;
            if (c[k].b > 0) Q[k][ix (c[k].b)] -= 1.0;
            kappa[k] = 2.0 * gg;   // h' = -h - 2 g v
        }
        // sources: row nn + k, unknown = current delivered into the network
        for (int k = 0; k < nSrc; ++k)
        {
            const int row = nn + k;
            M[row][ix (src[k])] += 1.0;
            M[ix (src[k])][row] -= 1.0;
        }
        for (int k = 0; k < nIn; ++k)
        {
            if (inputKind[k] == 0)
                Bu[nn + inputSlot[k]][k] = 1.0;
            else
            {
                const auto& e = inj[inputSlot[k]];
                if (e.a > 0) Bu[ix (e.a)][k] -= 1.0;
                if (e.b > 0) Bu[ix (e.b)][k] += 1.0;
            }
        }
        // coupled inductors: branch currents i1, i2; v1 - (2/T)(L1 i1 + M i2) = e1
        for (int k = 0; k < nCoup; ++k)
        {
            const auto& e = coup[k];
            const int r1 = nn + nSrc + 2 * k, r2 = r1 + 1, s1 = nC + 2 * k, s2 = s1 + 1;
            const double a = 2.0 / T;
            if (e.a1 > 0) { M[ix (e.a1)][r1] += 1.0; M[r1][ix (e.a1)] += 1.0; }
            if (e.b1 > 0) { M[ix (e.b1)][r1] -= 1.0; M[r1][ix (e.b1)] -= 1.0; }
            if (e.a2 > 0) { M[ix (e.a2)][r2] += 1.0; M[r2][ix (e.a2)] += 1.0; }
            if (e.b2 > 0) { M[ix (e.b2)][r2] -= 1.0; M[r2][ix (e.b2)] -= 1.0; }
            M[r1][r1] -= a * e.l1; M[r1][r2] -= a * e.m;
            M[r2][r1] -= a * e.m;  M[r2][r2] -= a * e.l2;
            Bh[r1][s1] += 1.0;
            Bh[r2][s2] += 1.0;
            // e' = -e - (4/T) (L i)
            Q[s1][r1] = e.l1; Q[s1][r2] = e.m; kappa[s1] = 2.0 * a;
            Q[s2][r1] = e.m;  Q[s2][r2] = e.l2; kappa[s2] = 2.0 * a;
        }
        for (int k = 0; k < nVcvs; ++k)
        {
            const auto& e = vc[k];
            const int row = nn + nSrc + 2 * nCoup + k;
            M[ix (e.out)][row] -= 1.0;          // the current the source delivers into `out`
            M[row][ix (e.out)] += 1.0;
            if (e.cp > 0) M[row][ix (e.cp)] -= e.gain;
            if (e.cm > 0) M[row][ix (e.cm)] += e.gain;
        }
        if (! invert (M, m)) return false;     // M now holds the inverse
        // Xu = Minv Bu, Xh = Minv Bh
        for (int i = 0; i < m; ++i)
        {
            for (int j = 0; j < nIn; ++j)
            {
                double s = 0.0;
                for (int k = 0; k < m; ++k) s += M[i][k] * Bu[k][j];
                Xu[i][j] = s;
            }
            for (int j = 0; j < ns; ++j)
            {
                double s = 0.0;
                for (int k = 0; k < m; ++k) s += M[i][k] * Bh[k][j];
                Xh[i][j] = s;
            }
        }
        for (int o = 0; o < nOut; ++o)
        {
            for (int j = 0; j < nIn; ++j) D[o][j] = 0.0;
            for (int j = 0; j < ns; ++j) C[o][j] = 0.0;
            auto addRow = [&] (int row, double w)
            {
                for (int j = 0; j < nIn; ++j) D[o][j] += w * Xu[row][j];
                for (int j = 0; j < ns; ++j) C[o][j] += w * Xh[row][j];
            };
            if (out[o].kind == 0)
            {
                if (out[o].a > 0) addRow (ix (out[o].a), 1.0);
                if (out[o].b > 0) addRow (ix (out[o].b), -1.0);
            }
            else
                addRow (nn + out[o].a, 1.0);
        }
        // state update: h' = -h - kappa (Q x)
        for (int s = 0; s < ns; ++s)
        {
            for (int j = 0; j < ns; ++j) A[s][j] = (s == j ? -1.0 : 0.0);
            for (int j = 0; j < nIn; ++j) B[s][j] = 0.0;
            for (int row = 0; row < m; ++row)
            {
                const double q = Q[s][row];
                if (q == 0.0) continue;
                const double w = -kappa[s] * q;
                for (int j = 0; j < ns; ++j) A[s][j] += w * Xh[row][j];
                for (int j = 0; j < nIn; ++j) B[s][j] += w * Xu[row][j];
            }
        }
        return true;
    }

    int numStates() const noexcept { return ns; }
    int numInputs() const noexcept { return nIn; }
    int numOutputs() const noexcept { return nOut; }

    // results (valid after build)
    double A[kMaxS][kMaxS] {}, B[kMaxS][kMaxI] {}, C[kMaxO][kMaxS] {}, D[kMaxO][kMaxI] {};

private:
    struct Res { int a, b; double g; };
    struct Cap { int a, b; double f; };
    struct Vccs { int a, b, c, d; double gm; };
    struct Inj { int a, b; };
    struct Coup { int a1, b1, a2, b2; double l1, l2, m; };
    struct Vcvs { int out, cp, cm; double gain; };
    struct Out { int kind, a, b; };

    static bool invert (double (&a)[kMaxM][kMaxM], int n) noexcept
    {
        double inv[kMaxM][kMaxM];
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                inv[i][j] = (i == j ? 1.0 : 0.0);
        for (int col = 0; col < n; ++col)
        {
            int p = col;
            double best = std::abs (a[col][col]);
            for (int i = col + 1; i < n; ++i)
                if (std::abs (a[i][col]) > best) { best = std::abs (a[i][col]); p = i; }
            if (best < 1.0e-300) return false;
            if (p != col)
                for (int j = 0; j < n; ++j) { std::swap (a[p][j], a[col][j]); std::swap (inv[p][j], inv[col][j]); }
            const double d = 1.0 / a[col][col];
            for (int j = 0; j < n; ++j) { a[col][j] *= d; inv[col][j] *= d; }
            for (int i = 0; i < n; ++i)
            {
                if (i == col) continue;
                const double f = a[i][col];
                if (f == 0.0) continue;
                for (int j = 0; j < n; ++j) { a[i][j] -= f * a[col][j]; inv[i][j] -= f * inv[col][j]; }
            }
        }
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                a[i][j] = inv[i][j];
        return true;
    }

    int nNodes = 0, nSrc = 0, nInj = 0, nCoup = 0, nOut = 0, nR = 0, nC = 0, nG = 0, nQ = 0, nIn = 0, nVcvs = 0;
    int ns = 0, m = 0;
    Res r[kMaxEl]; Cap c[kMaxEl]; Vccs g[kMaxEl]; Inj inj[kMaxI]; Coup coup[4]; Out out[kMaxO]; Vcvs vc[8];
    int src[kMaxI] {}, inputKind[kMaxI] {}, inputSlot[kMaxI] {};
    double M[kMaxM][kMaxM], Bu[kMaxM][kMaxI], Bh[kMaxM][kMaxS], Q[kMaxS][kMaxM], kappa[kMaxS];
    double Xu[kMaxM][kMaxI], Xh[kMaxM][kMaxS];
};
} // namespace urei1176::fast
