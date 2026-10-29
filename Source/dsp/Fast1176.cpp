// The reduced-circuit engine for the 1176LN: see Fast1176.h and docs/research/urei1176.md.
//
// Every block is built from the same component values as Urei1176.cpp, by the trapezoidal rule the
// circuit engine uses (FastNet.h turns a small linear netlist into a discrete state-space system),
// so a linear block has the circuit's own response at every sample rate. Semiconductors are
// linearised around the idle operating point, except:
//   Q1       the gain cell's JFET, with the square-law equations of the circuit engine: a scalar Newton
//            solve per sample on its channel current, the plant eliminated (the "K method")
//   Q12/Q14  the GR amp's input and second-stage transistors, as Ebers-Moll junctions: their exponential excess beyond the small-signal
//            model, a 2 x 2 Newton warm-started from the last sample; Q13 / Q15 (emitter followers at 5-7 mA) stay small-signal
//   CR3/CR4  the rectifier diodes: a scalar solve of the cathode node (Lambert W start)
// and the loop through them (the control voltage the gate sees is the one the rectifier produces from
// what the gate lets through) is closed inside the sample by a bracketed Newton iteration on that
// control voltage, as the circuit engine closes it.
//
// Blocks:
//   input    600 ohm source, bridged-T pad, T1, R5: linear, rebuilt when Input moves. Rev G and J: the 5532 receiver (IC2, C30-C33, R1-R2), with IC2's two output
//            clamps solved inside the sample when they conduct, then C33 into the Input pot R5
//   cell     R6, Q1, the Q2-Q4 preamp as its small-signal circuit, the gate network (R7 / R9 / C3 / C4 / C5)
//   side     ratio ladder + GR amp (one block per ratio, small signal plus the Q12 / Q14 junction solve; loaded by what C19 and C20 draw),
//            CR3 / CR4, attack, release, the threshold reference (ratio section A), the bias
//   line     Output pot, line amp (small signal), its rails, C15, T2 (strapped per revision; G and J: with R84 across its secondary), load
//   meters   the same laws as the circuit engine
// Large-signal behaviour beyond what the linearised blocks give (the preamp's and line amp's rails and
// the slide of their operating points, Q1's gate junctions) is approximated by fitted static limits.

#include "Fast1176.h"

#include "FastNet.h"
#include "Urei1176Circuit.h"
#include "circuit/FastMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cassert>
#include <cstring>

// Forced inlining of the per-sample stages (the solves are latency-bound; call overhead shows).
#if defined (_MSC_VER)
 #define UREI1176_FORCE_INLINE __forceinline
#else
 #define UREI1176_FORCE_INLINE __attribute__((always_inline)) inline
#endif

namespace urei1176
{
using fast::TrapNet;
namespace afast = ar::circuit::fast;

namespace
{
constexpr double kVt = 0.02585;
constexpr double kBetaF = 300.0;

/** GR amp block size: inputs the preamp output, the currents drawn at e15 and e13, and the excess collector currents of Q12 and Q14;
    outputs e13, e15 and the base-emitter deviations of Q12 and Q14. */
constexpr int kGrU = 5, kGrO = 4;

//== component values shared by every version (rev F, as Urei1176.cpp; the ones that differ are in VersionSpec) ========
constexpr double kR6 = 27e3;
/** Q1 as Urei1176.cpp: UREI's selected 2N5457. */
constexpr double kQ1Beta = 1.2e-3, kQ1Vp = -1.5, kQ1Lambda = 0.01, kSoftKnee = 0.03;
constexpr double kChannelShare = 0.48;

constexpr double collectorCurrent (double ie) { return ie * kBetaF / (1.0 + kBetaF); }

//== overload: the parts of the reference's behaviour that its small-signal blocks cannot give ====
// Everything below sits outside the linear blocks, which are exact (small signal) around the idle operating point. Each is a
// static limit with a soft knee, or the slide of an operating point with overdrive, fitted to the circuit engine's waveforms
// (docs/research/urei-1176.md, "The fast engine"). The fitted constants are per version (VersionSpec::overload); Q1's gate junctions
// are the same in every version:
/** Q1's gate junctions (to source and drain) hold vgs near +0.4 V: the largest reductions stop there (the reference's gate peaks at +0.6 V). */
constexpr double kGateMax = 0.40, kGateKnee = 0.20;

/** The Q1 reduction law's conductance: d id / d vds at vds = 0. */
inline double q1Conductance (double vgs) noexcept
{
    double sp, sg;
    ar::circuit::fast::softplusSigmoid ((vgs - kQ1Vp) / kSoftKnee, sp, sg);
    return 2.0 * kQ1Beta * (sp * kSoftKnee) * sg;
}

/** Q1 (Urei1176's square law with a 30 mV softplus knee, both polarities of vds): current and its
    partial derivatives. */
inline void q1Eval (double vgs, double vds, double& id, double& dVgs, double& dVds) noexcept
{
    double spF, sgF, spR, sgR;
    ar::circuit::fast::softplusSigmoid ((vgs - kQ1Vp) / kSoftKnee, spF, sgF);
    ar::circuit::fast::softplusSigmoid ((vgs - vds - kQ1Vp) / kSoftKnee, spR, sgR);
    spF *= kSoftKnee; spR *= kSoftKnee;
    const double f = spF * spF - spR * spR;
    const double m = 1.0 + kQ1Lambda * std::abs (vds);
    const double dm = vds >= 0.0 ? kQ1Lambda : -kQ1Lambda;
    id = kQ1Beta * f * m;
    dVgs = kQ1Beta * 2.0 * (spF * sgF - spR * sgR) * m;
    dVds = kQ1Beta * (2.0 * spR * sgR * m + f * dm);
}

/** A small-signal BJT between nodes (b, c, e) at collector current ic (NPN and PNP alike). */
void bjt (TrapNet& n, int b, int c, int e, double ic)
{
    const double gm = ic / kVt;
    n.resistor (b, e, kBetaF / gm);
    n.vccs (c, e, b, e, gm);
}

/** Dense Gaussian elimination for the small DC networks (n <= 12). */
bool solveDense (int n, double (&a)[12][12], double (&b)[12]) noexcept
{
    for (int col = 0; col < n; ++col)
    {
        int p = col;
        for (int i = col + 1; i < n; ++i)
            if (std::abs (a[i][col]) > std::abs (a[p][col])) p = i;
        if (std::abs (a[p][col]) < 1e-300) return false;
        if (p != col) { for (int j = 0; j < n; ++j) std::swap (a[p][j], a[col][j]); std::swap (b[p], b[col]); }
        for (int i = col + 1; i < n; ++i)
        {
            const double f = a[i][col] / a[col][col];
            if (f == 0.0) continue;
            for (int j = col; j < n; ++j) a[i][j] -= f * a[col][j];
            b[i] -= f * b[col];
        }
    }
    for (int i = n - 1; i >= 0; --i)
    {
        double s = b[i];
        for (int j = i + 1; j < n; ++j) s -= a[i][j] * b[j];
        b[i] = s / a[i][i];
    }
    return true;
}

/** A discrete state-space system of fixed size, copied from a TrapNet. The matrices are kept row-major for
    the odd scalar access and column-major (the t-copies) for the per-sample products, which then run as
    short vector operations over the outputs. */
template <int NS, int NU, int NO>
struct Lti
{
    double A[NS][NS] {}, B[NS][NU] {}, C[NO][NS] {}, D[NO][NU] {};
    double At[NS][NS] {}, Bt[NU][NS] {}, Ct[NS][NO] {}, Dt[NU][NO] {};
    double h[NS] {};

    bool load (const TrapNet& n) noexcept
    {
        if (n.numStates() != NS || n.numInputs() != NU || n.numOutputs() != NO) return false;
        for (int i = 0; i < NS; ++i)
        {
            for (int j = 0; j < NS; ++j) { A[i][j] = n.A[i][j]; At[j][i] = A[i][j]; }
            for (int j = 0; j < NU; ++j) { B[i][j] = n.B[i][j]; Bt[j][i] = B[i][j]; }
        }
        for (int o = 0; o < NO; ++o)
        {
            for (int j = 0; j < NS; ++j) { C[o][j] = n.C[o][j]; Ct[j][o] = C[o][j]; }
            for (int j = 0; j < NU; ++j) { D[o][j] = n.D[o][j]; Dt[j][o] = D[o][j]; }
        }
        return true;
    }
    /** Loads and insists that the network has the size this block was declared with. */
    void loadChecked (const TrapNet& n) noexcept
    {
        [[maybe_unused]] const bool ok = load (n);
        assert (ok);
    }
    void clearState() noexcept { for (auto& v : h) v = 0.0; }
    /** y = C h + D u, over the first N inputs. */
    template <int N>
    inline void output (const double* u, double* y) const noexcept
    {
        double acc[NO] {};
        for (int j = 0; j < NS; ++j)
        {
            const double hj = h[j];
            for (int o = 0; o < NO; ++o) acc[o] += Ct[j][o] * hj;
        }
        for (int k = 0; k < N; ++k)
        {
            const double uk = u[k];
            for (int o = 0; o < NO; ++o) acc[o] += Dt[k][o] * uk;
        }
        for (int o = 0; o < NO; ++o) y[o] = acc[o];
    }
    inline void advance (const double* u) noexcept
    {
        double nh[NS] {};
        for (int j = 0; j < NS; ++j)
        {
            const double hj = h[j];
            for (int i = 0; i < NS; ++i) nh[i] += At[j][i] * hj;
        }
        for (int k = 0; k < NU; ++k)
        {
            const double uk = u[k];
            for (int i = 0; i < NS; ++i) nh[i] += Bt[k][i] * uk;
        }
        for (int i = 0; i < NS; ++i) h[i] = nh[i];
    }
};

constexpr double kDbuVolts = 0.7745966692;
constexpr double kVuZeta = 0.8127, kVuOmega = 13.512;
constexpr double kVuPin = 1.5;
constexpr double kSineAverageOverRms = 0.9003163162;
double dbuToVolts (double dbu) noexcept { return kDbuVolts * std::pow (10.0, dbu / 20.0); }

constexpr int kControlInterval = 32;
constexpr double kPotStep = 0.02;
} // namespace

//==============================================================================================
struct FastChannel::Impl
{
    // --- controls ------------------------------------------------------------------------------
    Controls ctl;
    float calCurrent = Range::calibrationDefault;
    float posInputT = 0.5f, posInputC = 0.5f, posOutputT = 0.5f, posOutputC = 0.5f;
    float posAttackT = 0.5f, posAttackC = 0.5f, posReleaseT = 0.5f, posReleaseC = 0.5f;
    int ratioT = 0, ratioC = 0;
    bool offT = false, offC = false;
    int countdown = 0;

    // --- the detector's high-pass (a plug-in addition: the reference circuit has none) -------------------------------------
    // The GR amp sees hp = preIn - g * lp, lp being a one-pole low-pass of preIn (lp' = lp + k (preIn - lp), k = 1 - exp (-2 pi fc / fs)),
    // so hp = hpF * preIn + hpC with the feedthrough hpF = 1 - g k and the history term hpC = -g (1 - k) lp, both known before the sample
    // is solved. g (0 = off, 1 = on) and k glide linearly over ~10 ms when the choice changes; lp keeps running while off. With g = 0
    // hpF is exactly 1 and hpC exactly -0, so off is bit-identical to a unit without the filter.
    int hpIdxT = 0, hpIdxC = 0;
    double hpG = 0.0, hpK = 0.0, hpGT = 0.0, hpKT = 0.0, hpDG = 0.0, hpDK = 0.0, hpLp = 0.0, hpF = 1.0, hpC = -0.0;
    int hpLeft = 0;
    double fs = 0.0, T = 0.0;
    bool prepared = false;
    /** The version this engine is built as, and the GR amp's Q12 / Q14 collector currents IF0 / IF1 (read by the hot solve). */
    const VersionSpec* spec = &specFor (Version::F);
    double if0 = collectorCurrent (specFor (Version::F).ieQ12), if1 = collectorCurrent (specFor (Version::F).ieQ14);

    // --- levels --------------------------------------------------------------------------------
    double unitVolts = 1.0, inGain = 1.0, outGain = 1.0, vuReference = 1.0;
    bool trimsOn = true;   // the version's level trims (Versions.h), applied in updateScale(); setVersionTrims (false) takes them off

    // --- the shared scratch network for builds ---------------------------------------------------
    TrapNet net;

    // --- input block ---------------------------------------------------------------------------
    Lti<4, 1, 1> inputBlk;
    // rev G's and J's bridging input in place of inputBlk: the receiver (IC2 and its network, with IC2's two output clamps), then C33 into the Input pot
    // R5 (its open-circuit wiper voltage; rTh below is the pot's source resistance)
    bool bridging = false;
    Lti<6, 3, 2> inputRx;   // inputs: the EMF and the clamp currents of IC2-B's and IC2-A's outputs; outputs: those two output voltages
    Lti<1, 1, 1> inputPot;
    double clampH = 0.0, clampSafe = 0.0, clampIs = 0.0;   // the clamps' AC threshold, the output level below which they carry nothing, Is
    /** clampSafe is clampH less this: 0.5 V inside the threshold the diodes carry e^-19 of Is, nothing, so the clamps are skipped. */
    static constexpr double kClampQuietVolts = 0.5;
    double rTh = 145.0;
    double seriesOhms = 0.0, calibrationOhms = 0.0;

    // --- cell block
    Lti<6, 3, 6> cell;   // inputs: T1 volts (open circuit), control volts, Q1 current; outputs: d, s, c5, fb, o1 (Q4's emitter), the current the control node supplies
    /** The cell's outputs one sample ahead, without waiting for the state update: with y0 = C h, the next sample's C h' is
        (C A) h + (C B) u, and (C A) h can be formed while the loop is still solving. */
    double cellQt[6][6] {}, cellRt[3][6] {}, cellY0[6] {}, cellCinv[6][6] {};
    bool cellInvValid = false;
    Lti<1, 1, 1> outBlk;      // C9 into the Output pot's track: the preamp output's AC coupling
    double ctlRef = -1.52;
    double iPrev = 0.0, iPrev2 = 0.0;

    // --- GR amp, one block per ratio 4, 8, 12, 20, All: input pre, outputs e13, e15 ----------------
    std::array<Lti<1, kGrU, kGrO>, 5> grBlk;     // inputs: preamp output, the currents drawn at e15 and e13, the excess collector currents of Q12 and Q14
    /** Q12 and Q14 as Ebers-Moll junctions. The block above carries their small-signal models (gm at the idle currents); what they add
        beyond that, G = IF (e^(x/Vt) - 1 - x/Vt) of the base-emitter deviation x, is solved per evaluation (2 x 2 Newton, warm-started
        from the last sample) and injected. Q13 and Q15 are emitter followers at 5-7 mA whose exponential matters little: they stay small-signal. */
    double grGWarm[2] {}, grX0[2] {};
    /** The GR amp's outputs loaded by the currents C19 and C20 draw: v_loaded = v_open + Z c, with c = (iC3, iC4) and v = (e15, e13),
        and the anode equations solved for that coupling: c = N (g19 (v_open - va) + hh), va = Ainv (N (g19 v_open + hh) + ref / R - i_D). */
    struct GrLoad { double Z[2][2], N[2][2], Ainv[2][2], NG[2][2]; };
    std::array<GrLoad, 5> grLoad;
    double aAnode3 = 1.5, aAnode4 = 1.5;   // the diagonal of Ainv of the present variant (for the cathode solve)
    double e13 = 0.0, e15 = 0.0;

    // --- DC bias per ratio ---------------------------------------------------------------------
    double biasV[6] {}, refVA6[6] {}, refRA6[6] {};
    double biasPos = 0.37, gateAtIdle = -1.52;

    // --- rectifier, attack, release --------------------------------------------------------------
    double g19 = 0, g21 = 0, g22 = 0, g27 = 0;
    double va3 = 0, va4 = 0, vc19 = 0, vc20 = 0, i19 = 0, i20 = 0;
    double ref = 0, i21 = 0;
    double ctlV = 0, i27 = 0, vz = 0, i22 = 0;
    double Iprev = 0.0;
    double vk = 0, iCtlPlant = 0;
    double rRel = 1e6, rAw = 0, rho = 0, kap27 = 0;
    double biasNow = -1.52, refVA = -1.78, refRA = 100.0;
    double biasOverRel = 0.0, refDrive = 0.0, refInvDen = 0.0;   // bias / R_rel, V_A / R_A and 1 / (g21 + 1 / R_A + 2 / 47k) for the present settings
    double q1Id = 0.0, q1Gsg = 0.0, q1Dd = 0.0, q1VgsLin = 0.0, q1Vds = 0.0, q1Tol = 3.0e-4;
    bool q1Ok = false;
    double q1InvDF = 1.0, cellKDs = 0.0, cellKGs = 0.0;   // 1 / (dF of the last Q1 evaluation); Q1's feedthrough from the cell, set by buildCell   // Q1 at the last point evaluated
    double iWarm = 0.0, loopTol = 1.0e-5, smallStep = 2.0e-4;
    int grVariant = 0;

    // --- stereo link (the 1176SA tie of two units' control nodes) -------------------------------------
    /** Added to the shared control node's voltage on the way into this unit's gate network. It is zero except while the tie is
        engaging: each unit then starts from the control voltage it had and the offsets glide to zero (a non-physical glide, chosen
        so that engaging the link while playing does not click). */
    double ctlOffset = 0.0;
    double offsetDecay = 0.0;   // per sample
    bool linked = false;

    // --- meters ------------------------------------------------------------------------------------
    double fbAvg = 0, srcAvg = 0, avgCoeff = 0, vgsControl = 0, vgsPeak = -10.0;
    // The gate-drive meter works on segments: per sample only the raw drive's peak and last values are kept (the soft limit at the
    // gate junctions is monotonic, so it applies to the peak afterwards), and the 50 ms averages step once per segment.
    double rawBase = 0.0, rawLast = 0.0, rawPeak = -1.0e9, fbLast = 0.0, srcLast = 0.0;
    double avgSeg[kControlInterval + 1] {};
    double driveSum = 0.0;   // the needle's drive, summed over the segment (|output| in the +4 / +8 modes)
    double controlIdle = -1.52, sourceIdle = 0.0, idleConductance = 0.0, meterSlope = 1.0;
    double needle = 0.0, needleSpeed = 0.0;

    // --- line: pot + line amp (small signal), a static nonlinearity, C15 + T2 + load --------------------
    Lti<5, 1, 1> lineA;
    Lti<3, 1, 1> lineB;
    double lineEnv = 0.0, lineEnvRelease = 0.0;   // peak follower of the line amp's linear output
    // the overload parameters, as members: prepare() loads them from the version's spec (the fitting tools can move them)
    double gateMax = kGateMax, gateKnee = kGateKnee;
    double o1Lo = 0.0, o1Hi = 0.0, o1Knee = 0.0, o1KneeLo = 0.0, o1ShiftPerVolt = 0.0, o1ShiftStart = 0.0, o1EnvSeconds = 1.0, o1EnvRelease = 0.0, o1Env = 0.0;
    double lineLo = 0.0, lineHi = 0.0, lineKneeHi = 0.0, lineKneeLo = 0.0;
    double lineShiftPerVolt = 0.0, lineShiftStart = 0.0, lineEnvSeconds = 1.0;
#if UREI1176_FIT_HOOKS
    bool overloadOverridden = false;   // setOverloadParams() was called: prepare() keeps those values
#endif

    /** Loads the 14 overload constants from the spec (unless a fitting tool set its own). */
    void loadOverload (const OverloadParams& p) noexcept
    {
#if UREI1176_FIT_HOOKS
        if (overloadOverridden) return;
#endif
        o1Lo = p.o1Lo; o1Hi = p.o1Hi; o1Knee = p.o1KneeHi; o1KneeLo = p.o1KneeLo;
        o1ShiftPerVolt = p.o1ShiftPerVolt; o1ShiftStart = p.o1ShiftStart; o1EnvSeconds = p.o1EnvSeconds;
        lineLo = p.lineLo; lineHi = p.lineHi; lineKneeHi = p.lineKneeHi; lineKneeLo = p.lineKneeLo;
        lineShiftPerVolt = p.lineShiftPerVolt; lineShiftStart = p.lineShiftStart; lineEnvSeconds = p.lineEnvSeconds;
    }

    // ==============================================================================================
    static constexpr double kIs = 2.5e-9, kPsi = 1.8 * kVt, kLeak = 1.0e-10;
    static constexpr double kRatioLeak = kLeak / kIs;
    static constexpr double kInvPsi = 1.0 / kPsi, kInv47k = 1.0 / 47e3;

    /** FD333 (Is 2.5 nA, n 1.8, 0.1 nA leakage): current and slope at junction voltage v. */
    static inline void diode (double v, double& i, double& di) noexcept
    {
        double x = v / kPsi;
        if (x > 600.0) x = 600.0;
        const double e = ar::circuit::fast::exp (x);
        const double sg = e / (1.0 + e);
        i = kIs * (e - kRatioLeak - 2.0 * (1.0 - kRatioLeak) * sg);
        di = kIs / kPsi * (e - 2.0 * (1.0 - kRatioLeak) * sg * (1.0 - sg));
    }

    /** The diode's current and slope tabulated over x = -16 .. 4 (1/32 steps, linear interpolation: 1e-4 relative at worst,
        in the range where the currents are nanoamps and the exponentials are not worth their cost). */
    struct DiodeTable
    {
        static constexpr double lo = -16.0, hi = 4.0;
        static constexpr int per = 32;
        static constexpr int count = (int) ((hi - lo) * per) + 2;
        double cur[count], slope[count];
        DiodeTable() noexcept
        {
            for (int k = 0; k < count; ++k)
            {
                const double x = lo + (double) k / per;
                const double e = std::exp (x), sg = e / (1.0 + e);
                cur[k] = kIs * (e - kRatioLeak - 2.0 * (1.0 - kRatioLeak) * sg);
                slope[k] = kIs / kPsi * (e - 2.0 * (1.0 - kRatioLeak) * sg * (1.0 - sg));
            }
        }
    };
    static const DiodeTable& diodeTable() noexcept { static const DiodeTable t; return t; }
    const DiodeTable* dtab = &diodeTable();

    /** diode(), from the table for x in [-16, 4]. */
    inline void diodeTabulated (double v, double& i, double& di) const noexcept
    {
        const double x = v * kInvPsi;
        if (x <= DiodeTable::lo) { i = -kIs * kRatioLeak; di = 0.0; return; }
        if (x >= DiodeTable::hi) { diode (v, i, di); return; }
        const double f = (x - DiodeTable::lo) * DiodeTable::per;
        const int k = (int) f;
        const double t = f - (double) k;
        i = dtab->cur[k] + t * (dtab->cur[k + 1] - dtab->cur[k]);
        di = dtab->slope[k] + t * (dtab->slope[k + 1] - dtab->slope[k]);
    }

    /** W(e^c), the Lambert W of exp(c) (the principal branch), tabulated over c = -12 .. 64 in 1/8 steps for cubic Hermite interpolation
        (W' = W / (1 + W) is exact): the start of the forward rectifier solve, good to about 1e-8. */
    struct LambertTable
    {
        static constexpr double lo = -12.0, hi = 64.0;
        static constexpr int per = 8;
        static constexpr int count = (int) ((hi - lo) * per) + 2;
        double w[count], d[count];
        LambertTable() noexcept
        {
            for (int k = 0; k < count; ++k)
            {
                const double c = lo + (double) k / per;
                // Newton on w + ln w = c (c > 1) or w e^w = e^c, in full double precision
                double x;
                if (c > 1.0)
                {
                    x = c - std::log (c) + std::log (c) / c;
                    for (int it = 0; it < 8; ++it) x -= (x + std::log (x) - c) / (1.0 + 1.0 / x);
                }
                else
                {
                    const double lam = std::exp (c);
                    x = lam / (1.0 + lam);
                    for (int it = 0; it < 8; ++it) { const double ew = std::exp (x); x -= (x * ew - lam) / (ew * (x + 1.0)); }
                }
                w[k] = x;
                d[k] = x / (1.0 + x);
            }
        }
    };
    static const LambertTable& lambertTable() noexcept { static const LambertTable t; return t; }
    const LambertTable* ltab = &lambertTable();

    /** W(e^c) for c in the table's range (cubic Hermite); the caller handles c outside it. */
    inline double lambertTabulated (double c) const noexcept
    {
        const double f = (c - LambertTable::lo) * LambertTable::per;
        const int k = (int) f;
        const double t = f - (double) k;
        const double t2 = t * t, t3 = t2 * t;
        constexpr double h = 1.0 / LambertTable::per;
        const double w0 = ltab->w[k], w1 = ltab->w[k + 1];
        return (2.0 * t3 - 3.0 * t2 + 1.0) * w0 + (t3 - 2.0 * t2 + t) * h * ltab->d[k]
             + (-2.0 * t3 + 3.0 * t2) * w1 + (t3 - t2) * h * ltab->d[k + 1];
    }

    // The rectifier's series resistance seen by each anode (rk0 + the anode's own), with its logarithm and reciprocal, cached for the control settings
    double rkKey0 = -1.0, rkKey3 = -1.0, rkKey4 = -1.0;
    double rkc[2] {}, irkc[2] {}, lnRkc[2] {};
    inline void rectConstants (double rk0) noexcept
    {
        if (rk0 == rkKey0 && aAnode3 == rkKey3 && aAnode4 == rkKey4) return;
        rkKey0 = rk0; rkKey3 = aAnode3; rkKey4 = aAnode4;
        rkc[0] = rk0 + aAnode3; rkc[1] = rk0 + aAnode4;
        for (int w = 0; w < 2; ++w) { irkc[w] = 1.0 / rkc[w]; lnRkc[w] = std::log (rkc[w] * kIs / kPsi); }
    }

    // ==============================================================================================
    // builds

    void buildInput()
    {
        if (bridging) buildPot();   // the receiver ahead of it (G and J) is built once, in prepare()
        else buildInputBridgedT();
    }

    /** The 5532 macro-model of the reference (Circuit1176::addOpAmp), its non-inverting input on the bias node, an AC ground: Rid, a VCVS of
        the open-loop gain into a 1 k / C pole at GBW / A0, a unity buffer and the output resistance. The caller allocates the internal nodes
        a, p and o, so that the receiver keeps its node order. */
    void addOpAmp (const BridgingInput& b, int inv, int a, int p, int o, int out) noexcept
    {
        static constexpr double twoPi = 6.283185307179586;
        net.resistor (0, inv, b.opAmpDiffOhms);
        net.vcvs (a, 0, inv, b.opAmpGain);
        net.resistor (a, p, b.opAmpPoleOhms);
        net.capacitor (p, 0, 1.0 / (twoPi * b.opAmpPoleOhms * (b.opAmpGbwHz / b.opAmpGain)));
        net.vcvs (o, p, 0, 1.0);
        net.resistor (o, out, b.opAmpOutOhms);
    }

    /** Rev G's and J's receiver: the reference's IC2-B and IC2-A stages (Circuit1176::buildInputBridging) with the bias node as an AC ground. The EMF
        and the two output clamps' currents go in, the two op-amp output voltages come out (the clamps are solved per sample in
        inputStepBridging; C33 and R5 follow in buildPot). Depends only on the spec and T: prepare() builds it. */
    void buildReceiver()
    {
        const auto& b = spec->bridging;
        net.clear();
        const int gen = net.node(), in = net.node(), c30o = net.node(), bInv = net.node(), bA = net.node(), bP = net.node(), bO = net.node(), bOut = net.node();
        const int r2Top = net.node(), aInv = net.node(), c31o = net.node(), r2Bot = net.node(), aA = net.node(), aP = net.node(), aO = net.node(), aOut = net.node();
        net.source (gen);
        net.inject (bOut, 0);                                 // the output clamps' currents (solved per sample, see inputStepBridging)
        net.inject (aOut, 0);
        net.resistor (gen, in, Circuit1176::kSourceOhms);
        net.capacitor (in, 0, b.rfFarads);
        net.capacitor (in, c30o, b.c30Farads);                // C30
        net.resistor (c30o, bInv, b.r1Ohms);                  // R1-D
        net.resistor (bOut, bInv, b.r1Ohms);                  // R1-C
        addOpAmp (b, bInv, bA, bP, bO, bOut);                 // IC2-B
        net.resistor (bOut, r2Top, b.r1Ohms);                 // R1-B
        net.capacitor (0, c31o, b.c31Farads);                 // C31: INPUT COM (ground) to R1-E
        net.resistor (c31o, r2Bot, b.r1Ohms);                 // R1-E
        net.resistor (r2Top, aInv, 0.5 * b.r2Ohms);           // R2, wiper centred
        net.resistor (aInv, r2Bot, 0.5 * b.r2Ohms);
        net.resistor (aOut, aInv, b.r1Ohms);                  // R1-A
        net.capacitor (aOut, aInv, b.c32Farads);              // C32
        addOpAmp (b, aInv, aA, aP, aO, aOut);                 // IC2-A
        net.outVoltage (bOut);
        net.outVoltage (aOut);
        net.build (T);
        inputRx.loadChecked (net);
        // The clamps hang from each output to a node `headroom + on` inside the rails, which sit at the bias, half the supply: in AC terms +-clampH.
        clampH = 0.5 * spec->supplyVolts - b.clampHeadroomVolts - b.clampOnVolts;
        clampSafe = clampH - kClampQuietVolts;
        clampIs = b.clampIs;
    }

    /** C33 into R5 and the wiper at the Input position: the open-circuit wiper voltage, and the pot's source resistance for R6. */
    void buildPot()
    {
        const auto& b = spec->bridging;
        const double f = Circuit1176::inputPotFraction (Circuit1176::inputPosition (posInputC));   // wiper to ground is f of the track
        net.clear();
        const int src = net.node(), top = net.node(), wiper = net.node();
        net.source (src);
        net.capacitor (src, top, b.c33Farads);                // C33
        net.resistor (top, wiper, std::max (1.0, b.potOhms * (1.0 - f)));
        net.resistor (wiper, 0, std::max (1.0, b.potOhms * f));
        net.outVoltage (wiper);
        net.build (T);
        inputPot.loadChecked (net);   // load() leaves the state alone: C33's charge carries through the rebuild
        rTh = f * (1.0 - f) * b.potOhms;
        seriesOhms = (kR6 + rTh) * kChannelShare;
    }

    void buildInputBridgedT()
    {
        double bridge, shunt;
        Circuit1176::bridgedTArms (Circuit1176::inputPosition (posInputC), bridge, shunt);
        const double r1a = std::max (5.0, bridge), r1b = std::max (1.25, shunt);
        const auto& t = Circuit1176::inputTransformer();
        net.clear();
        const int gen = net.node(), in = net.node(), mid = net.node(), pri = net.node(), pw = net.node(), sec = net.node(), sw = net.node();
        net.source (gen);
        net.resistor (gen, in, Circuit1176::kSourceOhms);
        net.resistor (in, mid, 620.0);
        net.resistor (mid, pri, 620.0);
        net.resistor (in, pri, r1a);
        net.resistor (mid, 0, r1b);
        net.resistor (pri, pw, t.primaryOhms);
        net.resistor (sec, sw, t.secondaryOhms);
        net.capacitor (pri, 0, t.primaryFarads);
        net.capacitor (sec, 0, t.secondaryFarads);
        net.resistor (sec, 0, 270.0);
        const double l1 = t.primaryHenries, l2 = l1 * t.turnsRatio * t.turnsRatio;
        net.coupledInductors (pw, 0, sw, 0, l1, l2, t.coupling * std::sqrt (l1 * l2));
        net.outVoltage (sec);
        net.build (T);
        // keep the state through a rebuild (the history sources are the same physical quantities)
        double keep[4];
        std::memcpy (keep, inputBlk.h, sizeof (keep));
        inputBlk.loadChecked (net);
        std::memcpy (inputBlk.h, keep, sizeof (keep));

        // Thevenin resistance seen by R6 and the gain cell's series ohms (as Circuit1176::gainCellOhms)
        const double gS = 1.0 / Circuit1176::kSourceOhms, gB = 1.0 / std::max (bridge, 1.0), gA = 1.0 / 620.0, gSh = 1.0 / std::max (shunt, 1.0);
        const double a11 = gS + gB + gA, a12 = -gA, a13 = -gB, a22 = 2.0 * gA + gSh, a23 = -gA, a33 = gB + gA;
        const double det = a11 * (a22 * a33 - a23 * a23) - a12 * (a12 * a33 - a23 * a13) + a13 * (a12 * a23 - a22 * a13);
        const double rPad = (a11 * a22 - a12 * a12) / det;
        const double n2 = t.turnsRatio * t.turnsRatio;
        const double rSec = t.secondaryOhms + n2 * (t.primaryOhms + rPad);
        rTh = rSec * 270.0 / (rSec + 270.0);
        seriesOhms = (kR6 + rTh) * kChannelShare;
    }

    double gainCellOhmsAtFull() const
    {
        if (bridging)
        {
            const double f = Circuit1176::inputPotFraction (Circuit1176::inputPosition (Range::inputMax));
            return (kR6 + f * (1.0 - f) * spec->bridging.potOhms) * kChannelShare;
        }
        double bridge, shunt;
        Circuit1176::bridgedTArms (Circuit1176::inputPosition (Range::inputMax), bridge, shunt);
        const double gS = 1.0 / Circuit1176::kSourceOhms, gB = 1.0 / std::max (bridge, 1.0), gA = 1.0 / 620.0, gSh = 1.0 / std::max (shunt, 1.0);
        const double a11 = gS + gB + gA, a12 = -gA, a13 = -gB, a22 = 2.0 * gA + gSh, a23 = -gA, a33 = gB + gA;
        const double det = a11 * (a22 * a33 - a23 * a23) - a12 * (a12 * a33 - a23 * a13) + a13 * (a12 * a23 - a22 * a13);
        const double rPad = (a11 * a22 - a12 * a12) / det;
        const auto& t = Circuit1176::inputTransformer();
        const double rSec = t.secondaryOhms + t.turnsRatio * t.turnsRatio * (t.primaryOhms + rPad);
        return (kR6 + rSec * 270.0 / (rSec + 270.0)) * kChannelShare;
    }

    void buildCell()
    {
        net.clear();
        const int src = net.node(), d = net.node(), s = net.node(), c1 = net.node(), b2 = net.node(), e2 = net.node(), e2m = net.node();
        const int c2 = net.node(), r15 = net.node(), fb = net.node(), c3 = net.node(), e3 = net.node(), e3m = net.node(), o1 = net.node();
        const int c5 = net.node(), ctlN = net.node();
        net.source (src);                  // 0: T1 secondary, open circuit
        const int inCtl = net.source (ctlN);   // 1: control node
        net.inject (d, s);                 // 2: Q1 channel current
        net.resistor (src, d, kR6 + rTh);                    // R6 and T1's source resistance
        net.resistor (s, 0, 10e3);                           // R8
        net.capacitor (s, e2m, spec->c2Farads);              // C2
        net.capacitor (d, c1, 1.0e-6);                       // C1
        net.resistor (c1, b2, 1e3);                          // R10
        net.resistor (b2, e3m, 560e3);                       // R11
        net.resistor (c2, 0, 1e6);                           // R12 (B+ is an AC ground)
        net.resistor (e2, e2m, 180.0);                       // R13
        net.resistor (e2m, 0, 180.0);                        // R14
        net.resistor (e2, r15, 150.0);                       // R15
        net.resistor (r15, fb, spec->r16Ohms);               // R16
        net.resistor (fb, o1, spec->r21Ohms);                // R21
        net.resistor (c3, 0, 22e3);                          // R18
        net.resistor (e3, e3m, 82.0);                        // R19
        net.resistor (e3m, 0, spec->r20Ohms);                // R20
        net.capacitor (e3m, 0, spec->c8Farads);               // C8
        bjt (net, b2, c2, e2, collectorCurrent (spec->ieQ2)); // Q2
        bjt (net, c2, c3, e3, collectorCurrent (spec->ieQ3)); // Q3
        bjt (net, c3, 0, o1, collectorCurrent (spec->ieQ4)); // Q4
        net.capacitor (c5, fb, 0.22e-6);                     // C5
        net.resistor (c5, ctlN, 4.4e6);                      // R7 + R9
        net.capacitor (c5, ctlN, 100e-12);                   // C3 and C4 in series
        net.capacitor (b2, 0, 27e-12);                       // C6: with the ~25 kohm that drives Q2's base, a pole near 250 kHz
        (void) inCtl;
        net.outVoltage (d);
        net.outVoltage (s);
        net.outVoltage (c5);
        net.outVoltage (fb);
        net.outVoltage (o1);
        net.outSourceCurrent (1);
        net.build (T);
        // The cell's state is kept as its own outputs without the input's share, q = C h (the part of y = C h + D u that the present
        // input does not decide): q' = (C A C^-1) q + (C B) u. One 6 x 6 and one 6 x 3 product a sample instead of those and the same
        // again for h. A rebuild (the Input knob moving) keeps h, so h is recovered with the old C^-1 first.
        if (cellInvValid)
            for (int i = 0; i < 6; ++i)
            {
                double sum = 0.0;
                for (int j = 0; j < 6; ++j) sum += cellCinv[i][j] * cellY0[j];
                cell.h[i] = sum;
            }
        cell.loadChecked (net);
        {
            double inv[6][6] {};
            for (int col = 0; col < 6; ++col)
            {
                double a[12][12] {}, b[12] {};
                for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) a[i][j] = cell.C[i][j];
                b[col] = 1.0;
                const bool ok = solveDense (6, a, b);
                assert (ok);
                (void) ok;
                for (int i = 0; i < 6; ++i) inv[i][col] = b[i];
            }
            std::memcpy (cellCinv, inv, sizeof (inv));
            cellInvValid = true;
            double CA[6][6] {};
            for (int o = 0; o < 6; ++o)
                for (int j = 0; j < 6; ++j)
                {
                    double q = 0.0;
                    for (int k = 0; k < 6; ++k) q += cell.C[o][k] * cell.A[k][j];
                    CA[o][j] = q;
                }
            for (int o = 0; o < 6; ++o)
            {
                for (int j = 0; j < 6; ++j)
                {
                    double q = 0.0;
                    for (int k = 0; k < 6; ++k) q += CA[o][k] * inv[k][j];
                    cellQt[j][o] = q;
                }
                for (int j = 0; j < 3; ++j)
                {
                    double r = 0.0;
                    for (int k = 0; k < 6; ++k) r += cell.C[o][k] * cell.B[k][j];
                    cellRt[j][o] = r;
                }
            }
        }
        cellKDs = cell.D[0][2] - cell.D[1][2];
        cellKGs = 0.5 * cell.D[2][2] - cell.D[1][2];
        q1Ok = false;   // the chord of the last evaluation belongs to the old feedthrough
        refreshCellOutputs();
    }

    void refreshCellOutputs() noexcept
    {
        for (int o = 0; o < 6; ++o)
        {
            double sum = 0.0;
            for (int j = 0; j < 6; ++j) sum += cell.C[o][j] * cell.h[j];
            cellY0[o] = sum;
        }
    }

    void buildOutBlock()
    {
        // pre is loaded by R27's track and by the ratio ladder R26 ... R23 (283 kohm to ground, or 103 kohm with every button in)
        const double ladder = ratioC == (int) Ratio::all ? 103e3 : 283e3;
        net.clear();
        const int o1 = net.node(), pre = net.node();
        net.source (o1);
        net.capacitor (o1, pre, spec->c9Farads);                // C9
        net.resistor (pre, 0, 1.0 / (1.0 / spec->outputPotOhms + 1.0 / ladder));
        net.outVoltage (pre);
        net.build (T);
        double keep = outBlk.h[0];
        outBlk.loadChecked (net);
        outBlk.h[0] = keep;
    }


    /** BJT with its own beta. */
    static void bjtBeta (TrapNet& n, int b, int c, int e, double ic, double beta)
    {
        const double gm = ic / kVt;
        n.resistor (b, e, beta / gm);
        n.vccs (c, e, b, e, gm);
    }

    void buildLineA()
    {
        const double f = ar::circuit::taperFraction (ar::circuit::Taper::audio, posOutputC);
        const double rWb = std::max (5.0, spec->outputPotOhms * f), rAw = std::max (5.0, spec->outputPotOhms * (1.0 - f));
        net.clear();
        const int pre = net.node(), ow = net.node(), b5 = net.node(), e5 = net.node(), c13 = net.node(), q5c = net.node(), e6 = net.node(), e7 = net.node();
        const int x = net.node(), cr = net.node(), y = net.node(), z = net.node(), r37 = net.node(), e8 = net.node(), e9 = net.node(), lo = net.node();
        net.source (pre);
        net.resistor (pre, ow, rAw);                 // R27 above the wiper
        net.resistor (ow, 0, rWb);                   // R27 below the wiper
        net.capacitor (ow, b5, spec->c10Farads);        // C10
        net.resistor (b5, 0, 1.0 / (1.0 / spec->r29Ohms + 1.0 / spec->r30Ohms));   // R29 (to the filtered supply) and R30
        net.resistor (q5c, 0, spec->r31Ohms);       // R31
        net.resistor (e5, 0, spec->r32Ohms);       // R32
        net.resistor (e5, c13, spec->r33Ohms);     // R33
        net.capacitor (c13, 0, spec->c13Farads);   // C13
        net.resistor (lo, e5, spec->r34Ohms);      // R34
        net.resistor (e6, 0, 10e3);                  // R35
        net.resistor (e7, 0, 100.0);                 // R36
        net.capacitor (q5c, r37, spec->c12Farads); // C12
        net.resistor (r37, y, 1e3);                  // R37
        net.resistor (y, z, 1.2e3);                  // R38
        net.resistor (z, 0, 1.2e3);                  // R39
        net.capacitor (z, lo, 22e-6);                // C14
        net.capacitor (y, 0, 47e-12);                // C29
        net.resistor (e8, lo, 4.3);                  // R40
        net.resistor (lo, e9, 4.3);                  // R41
        const double t2Load = spec->r84Ohms > 0.0 ? 1.0 / (1.0 / Circuit1176::kLoadOhms + 1.0 / spec->r84Ohms) : Circuit1176::kLoadOhms;
        net.resistor (lo, 0, t2Load / 3.0);          // T2's load (R84 across it on G) reflected to the primary
        bjtBeta (net, b5, q5c, e5, spec->icQ5, 300.0);    // Q5
        bjtBeta (net, q5c, x, e6, spec->icQ6, 250.0);     // Q6
        bjtBeta (net, e6, x, e7, spec->icQ7, 250.0);      // Q7
        const double rd = 1.984 * kVt / spec->idBias + 0.034;
        net.resistor (x, cr, rd);                    // CR1
        net.resistor (cr, y, rd);                    // CR2
        bjtBeta (net, x, 0, e8, spec->icQ8, 100.0);       // Q8
        bjtBeta (net, y, 0, e9, spec->icQ9, 100.0);       // Q9
        net.outVoltage (lo);
        net.build (T);
        double keep[5];
        std::memcpy (keep, lineA.h, sizeof (keep));
        lineA.loadChecked (net);
        std::memcpy (lineA.h, keep, sizeof (keep));
    }

    void buildLineB()
    {
        const TransformerSpec t = Circuit1176::outputTransformer (spec->version);   // T2 as the version straps it (the reference's winding data)
        net.clear();
        const int lo = net.node(), c15 = net.node(), pw = net.node(), sec = net.node(), sw = net.node();
        net.source (lo);
        net.capacitor (lo, c15, spec->c15Farads);      // C15
        net.resistor (c15, pw, t.primaryOhms);
        net.resistor (sec, sw, t.secondaryOhms);
        const double l1 = t.primaryHenries, l2 = l1 * t.turnsRatio * t.turnsRatio;
        net.coupledInductors (pw, 0, sw, 0, l1, l2, t.coupling * std::sqrt (l1 * l2));
        if (t.coreOhms > 0.0) net.resistor (pw, 0, t.coreOhms);
        net.resistor (sec, 0, Circuit1176::kLoadOhms);
        if (spec->r84Ohms > 0.0) net.resistor (sec, 0, spec->r84Ohms);   // R84 across T2's secondary
        net.outVoltage (sec);
        net.build (T);
        lineB.loadChecked (net);
    }

    /** The GR block with each transistor's collector and base excess merged into one input (the base share is 1 / beta of the collector's). */
    static void loadGrBjt (Lti<1, kGrU, kGrO>& blk, const TrapNet& n) noexcept
    {
        [[maybe_unused]] const bool ok = n.numStates() == 1 && n.numInputs() == 7 && n.numOutputs() == kGrO;
        assert (ok);
        blk.A[0][0] = n.A[0][0]; blk.At[0][0] = blk.A[0][0];
        for (int j = 0; j < kGrU; ++j)
        {
            const double b = j < 3 ? n.B[0][j] : n.B[0][3 + 2 * (j - 3)] + n.B[0][4 + 2 * (j - 3)] / kBetaF;
            blk.B[0][j] = b; blk.Bt[j][0] = b;
        }
        for (int o = 0; o < kGrO; ++o)
        {
            blk.C[o][0] = n.C[o][0]; blk.Ct[0][o] = blk.C[o][0];
            for (int j = 0; j < kGrU; ++j)
            {
                const double d = j < 3 ? n.D[o][j] : n.D[o][3 + 2 * (j - 3)] + n.D[o][4 + 2 * (j - 3)] / kBetaF;
                blk.D[o][j] = d; blk.Dt[j][o] = d;
            }
        }
    }

    void buildGr()
    {
        for (int variant = 0; variant < 5; ++variant)
        {
            net.clear();
            const int pre = net.node(), rb4 = net.node(), rb3 = net.node(), rb2 = net.node(), rb1 = net.node(), bus = net.node(), n22 = net.node();
            const int b12 = net.node(), c12 = net.node(), e12 = net.node(), r64 = net.node(), e13n = net.node(), b14 = net.node(), c14 = net.node();
            const int e14 = net.node(), r72 = net.node(), e15n = net.node();
            net.source (pre);
            net.resistor (pre, rb4, 56e3);       // R26
            net.resistor (rb4, rb3, 68e3);       // R25
            net.resistor (rb3, rb2, 56e3);       // R24
            net.resistor (rb2, rb1, 56e3);       // R22
            net.resistor (rb1, 0, 47e3);         // R23
            const int taps[4] = { rb1, rb2, rb3, rb4 };
            if (variant == 4)
                for (int tap : taps) net.resistor (tap, bus, 1.0);
            else
                net.resistor (taps[variant], bus, 1.0);
            net.resistor (bus, n22, 1.0);        // S9
            net.capacitor (n22, b12, 1.0e-6);    // C17
            net.resistor (c12, b12, 1e6);        // R60
            net.resistor (b12, 0, 470e3);        // R61
            net.resistor (c12, 0, 47e3);         // R62 (B+ AC ground)
            net.resistor (e12, r64, 4.7e3);      // R63
            net.resistor (r64, 0, 270.0);        // R65
            net.resistor (e13n, r64, 2.4e3);     // R64
            net.resistor (e12, e15n, 47e3);      // R68
            net.resistor (e13n, b14, spec->grR66Ohms); // R66
            net.resistor (b14, r72, spec->grR67Ohms); // R67
            net.resistor (c14, 0, spec->grR69Ohms);   // R69
            net.resistor (e14, r72, spec->grR71Ohms); // R71
            net.resistor (e15n, r72, 2.4e3);     // R72
            net.resistor (r72, 0, 180.0);        // R73
            net.inject (e15n, 0);                // 1: what C19 draws from Q15's emitter
            net.inject (e13n, 0);                // 2: what C20 draws from Q13's emitter
            // large-signal excess of Q12 and Q14: collector to emitter, and the base share (1 / beta) of it (inputs 3 + 2k, 4 + 2k)
            net.inject (c12, e12);  net.inject (b12, e12);
            net.inject (c14, e14);  net.inject (b14, e14);
            bjt (net, b12, c12, e12, collectorCurrent (spec->ieQ12)); // Q12
            bjt (net, c12, 0, e13n, collectorCurrent (spec->ieQ13)); // Q13
            bjt (net, b14, c14, e14, collectorCurrent (spec->ieQ14)); // Q14
            bjt (net, c14, 0, e15n, collectorCurrent (spec->ieQ15)); // Q15
            net.outVoltage (e13n);
            net.outVoltage (e15n);
            net.outVoltage (b12, e12);       // Q12 base-emitter
            net.outVoltage (b14, e14);       // Q14
            net.build (T);
            loadGrBjt (grBlk[(size_t) variant], net);
            {
                auto& gl = grLoad[(size_t) variant];
                gl.Z[0][0] = net.D[1][1]; gl.Z[0][1] = net.D[1][2];     // e15 per (iC3, iC4)
                gl.Z[1][0] = net.D[0][1]; gl.Z[1][1] = net.D[0][2];     // e13 per (iC3, iC4)
            }
        }
    }

    // --- DC networks: the bias and the threshold reference per ratio -------------------------------------------------
    /** Bias node (R81 wiper) and the section A reference, with ratio `sel` (0..3 = 4..20, 4 = All, 5 = None). */
    void solveBias (int sel, double pos, double& bias, double& refV, double& refR) const
    {
        // nodes: 0 bp, 1 bias, 2 a1, 3 a2, 4 a3, 5 a4, 6 ref
        auto solve = [&] (double injectRef, double* v)
        {
            double a[12][12] {}, b[12] {};
            auto stamp = [&] (int i, int j, double g)   // j < 0: ground, j == -2: -10 V
            {
                if (i >= 0) a[i][i] += g;
                if (j >= 0) { a[j][j] += g; if (i >= 0) { a[i][j] -= g; a[j][i] -= g; } }
                if (j == -2 && i >= 0) b[i] += g * -10.0;
            };
            stamp (0, -2, 1.0 / 10e3);                                // R82
            stamp (0, -1, 1.0 / 3.9e3);                               // R83
            stamp (0, 1, 1.0 / std::max (5.0, 2e3 * (1.0 - pos)));    // R81 upper
            stamp (1, -1, 1.0 / std::max (5.0, 2e3 * pos));           // R81 lower
            stamp (1, 2, 1.0 / 150.0);                                // R87
            stamp (2, 3, 1.0 / 470.0);                                // R42
            stamp (3, 4, 1.0 / 560.0);                                // R43
            stamp (4, 5, 1.0 / 1.5e3);                                // R44
            stamp (5, -2, 1.0 / 2.2e3);                               // R45
            stamp (2, 6, 1.0 / 10e6);                                 // R46
            for (int k = 0; k < 4; ++k)
            {
                const bool closed = sel == 4 || sel == k;
                stamp (2 + k, 6, closed ? 1.0 : 1e-12);               // S1A ... S4A
            }
            b[6] += injectRef;
            if (! solveDense (7, a, b)) { for (int i = 0; i < 7; ++i) v[i] = 0; return; }
            for (int i = 0; i < 7; ++i) v[i] = b[i];
        };
        double v0[12], v1[12];
        solve (0.0, v0);
        solve (1.0, v1);
        bias = v0[1];
        refV = v0[6];
        refR = v1[6] - v0[6];
    }

    /** R81's position that puts the bias node at `volts` with the 4:1 chain hanging on it (as Urei1176.cpp). */
    static double biasPosition (double volts)
    {
        auto biasAt = [] (double f)
        {
            const double rLow = std::max (1.0, 2e3 * f), rHigh = std::max (1.0, 2e3 * (1.0 - f));
            const double gChain = 1.0 / 4880.0;
            const double a11 = 1.0 / 10e3 + 1.0 / 3.9e3 + 1.0 / rHigh, a12 = -1.0 / rHigh, b1 = -10.0 / 10e3;
            const double a21 = -1.0 / rHigh, a22 = 1.0 / rHigh + 1.0 / rLow + gChain, b2 = -10.0 * gChain;
            return (a11 * b2 - a21 * b1) / (a11 * a22 - a12 * a21);
        };
        double lo = 0.0, hi = 1.0;
        for (int i = 0; i < 60; ++i)
        {
            const double mid = 0.5 * (lo + hi);
            (biasAt (mid) > volts ? lo : hi) = mid;
        }
        return 0.5 * (lo + hi);
    }

    static double idleGate (double series)
    {
        double lo = kQ1Vp - 0.5, hi = 0.0;
        for (int i = 0; i < 60; ++i)
        {
            const double mid = 0.5 * (lo + hi);
            const double dB = 20.0 * std::log10 (1.0 + series * q1Conductance (mid));
            (dB < 1.0 ? lo : hi) = mid;
        }
        return 0.5 * (lo + hi);
    }

    void computeBias()
    {
        calibrationOhms = gainCellOhmsAtFull();
        gateAtIdle = idleGate (calibrationOhms);
        biasPos = biasPosition (gateAtIdle);
        for (int sel = 0; sel < 6; ++sel)
            solveBias (sel, biasPos, biasV[sel], refVA6[sel], refRA6[sel]);
    }

    void applyGrCoupling() noexcept
    {
        const auto& gl = grLoad[(size_t) grVariant];
        aAnode3 = gl.Ainv[0][0];
        aAnode4 = gl.Ainv[1][1];
    }

    static int variantOf (int ratio) noexcept { return ratio == (int) Ratio::all ? 4 : std::min (ratio, 3); }

    void applyRatio()
    {
        const int v = variantOf (ratioC);
        if (v != grVariant)
        {
            grBlk[(size_t) v].h[0] = grBlk[(size_t) grVariant].h[0];   // C17 carries its charge across the switch
            grVariant = v;
        }
        applyGrCoupling();
        biasNow = biasV[ratioC];
        refVA = refVA6[ratioC];
        refRA = refRA6[ratioC];
        updateDerived();
    }

    void applyTimes()
    {
        const double fa = posAttackC, fr = posReleaseC;
        const double rAwFull = std::max (5.0, 25e3 * (1.0 - fa)), rWk = std::max (5.0, 25e3 * fa);
        rAw = rAwFull;
        const double rp = 1.0 / (1.0 / rWk + 1.0 / 470.0);
        rho = 1.0 / (1.0 / rp + g22);
        const double r1 = std::max (5.0, 5e6 * (1.0 - fr)), r2 = std::max (5.0, 5e6 * fr);
        rRel = r1 + 1.0 / (1.0 / r2 + 1.0 / 270e3);
        kap27 = 1.0 / (g27 + 1.0 / rRel);
        updateDerived();
    }

    void updateDerived() noexcept
    {
        biasOverRel = biasNow / rRel;
        refDrive = refVA / refRA;
        refInvDen = 1.0 / (g21 + 1.0 / refRA + 2.0 * kInv47k);
    }

    void updateScale()
    {
        unitVolts = dbuToVolts (calCurrent) * std::sqrt (2.0);
        // The version's level trims sit in this calibration layer: ahead of the circuit and after it, and ahead of the +4 / +8 meter's tap, so
        // that a given plug-in output level reads the same on every version (the make-up is taken out of the meter's reference). Both are
        // exactly 1 on F, whose products are then bit for bit what they were without them.
        const double inTrim = trimsOn ? std::pow (10.0, spec->inputTrimDb / 20.0) : 1.0;
        const double outTrim = trimsOn ? std::pow (10.0, spec->outputTrimDb / 20.0) : 1.0;
        inGain = 2.0 * unitVolts * inTrim;
        outGain = 1.0 / unitVolts * outTrim;
        const double refDbu = ctl.meter == MeterMode::plus8 ? 4.0 + 20.0 * std::log10 ((spec->meterR59Ohms + spec->meterPlus8SeriesOhms) / (spec->meterR59Ohms + spec->meterPlus8ReferenceOhms)) : 4.0;
        vuReference = kSineAverageOverRms * dbuToVolts (refDbu) / outTrim;
    }

    // ==============================================================================================
    // meters' reference

    double reductionDbAt (double vgs) const noexcept
    {
        const double g = q1Conductance (vgs);
        return 20.0 * std::log10 ((1.0 + seriesOhms * g) / (1.0 + seriesOhms * idleConductance));
    }

    double controlForReduction (double dB) const
    {
        auto reduction = [this] (double vc)
        {
            const double g = q1Conductance (vc - sourceIdle);
            return 20.0 * std::log10 ((1.0 + calibrationOhms * g) / (1.0 + calibrationOhms * idleConductance));
        };
        double lo = controlIdle, hi = 0.5;
        for (int i = 0; i < 60; ++i)
        {
            const double mid = 0.5 * (lo + hi);
            (reduction (mid) < dB ? lo : hi) = mid;
        }
        return 0.5 * (lo + hi);
    }

    double meterDeflection() const noexcept { return std::max (0.0, 1.0 - meterSlope * (ctlV - controlIdle)); }

    // ==============================================================================================
    // state

    /** The reference's DC operating point of the slow nodes, which has 0.1 nA / V of conductance across every
        capacitor: C5 and C27's leaks and the diodes' leakage put the control node a fraction of a millivolt
        off the bias and C5's node about a millivolt above it. The transient starts from there. */
    void idleControl (int sel, double& ctl0, double& c5Offset) const
    {
        constexpr double kFbDc = 0.8127, kLeakS = 1.0e-10;
        double c = biasV[sel];
        for (int it = 0; it < 4; ++it)
            c = biasV[sel] + rRel * (kLeakS * (kFbDc - c) + kLeakS * (0.0 - c) - 2.0 * kIs * kRatioLeak);
        ctl0 = c;
        c5Offset = 4.4e6 * kLeakS * (kFbDc - c);
    }

    void settleCell (double ctlAbs, double c5Offset)
    {
        // the plant's DC steady state under a constant control voltage: h = (I - A)^-1 B u
        double a[12][12] {}, b[12] {};
        for (int i = 0; i < 6; ++i)
        {
            for (int j = 0; j < 6; ++j) a[i][j] = (i == j ? 1.0 : 0.0) - cell.A[i][j];
            b[i] = cell.B[i][1] * (ctlAbs - ctlRef);
        }
        if (solveDense (6, a, b))
            for (int i = 0; i < 6; ++i) cell.h[i] = b[i];
        else
            cell.clearState();
        cell.h[3] -= 2.0 * 0.22e-6 / T * c5Offset;      // C5 and the gate arm hold the offset
        cell.h[4] -= 2.0 * 100e-12 / T * c5Offset;
        refreshCellOutputs();
    }

    void clearStates()
    {
        inputBlk.clearState();
        inputRx.clearState();
        inputPot.clearState();
        cell.clearState();
        refreshCellOutputs();
        outBlk.clearState();
        for (auto& g : grBlk) g.clearState();
        applyRatio();
        double c5Off;
        idleControl (ratioC, ctlV, c5Off);
        settleCell (ctlV, c5Off);
        ref = refVA;
        va3 = va4 = ref;
        vc19 = vc20 = -ref; i19 = i20 = 0.0;
        i21 = i27 = vz = i22 = 0.0;
        Iprev = 0.0; vk = ctlV; iCtlPlant = 0.0;
        e13 = e15 = 0.0;
        grGWarm[0] = grGWarm[1] = 0.0;
        iPrev = iPrev2 = 0.0;
        ctlOffset = 0.0; linked = false;
        q1Ok = false;
        fbAvg = srcAvg = 0.0;
        vgsControl = ctlV + 0.5 * c5Off;
        vgsPeak = vgsControl;
        rawBase = 0.5 * ctlRef + 0.5 * fbAvg - srcAvg;
        rawLast = vgsControl; rawPeak = -1.0e9; fbLast = srcLast = 0.0; driveSum = 0.0;
    }

    // ==============================================================================================
    void prepare (double sampleRate, Version version)
    {
        spec = &specFor (version);
        bridging = spec->inputBridging;
        if0 = collectorCurrent (spec->ieQ12);
        if1 = collectorCurrent (spec->ieQ14);
        loadOverload (spec->overload);
        fs = sampleRate;
        T = 1.0 / fs;
        calCurrent = ctl.calibrationDbu;
        g19 = 2.0 * 6.8e-6 / T;
        g21 = 2.0 * 100e-6 / T;
        g22 = 2.0 * 22e-9 / T;
        g27 = 2.0 * 0.22e-6 / T;
        avgCoeff = 1.0 - std::exp (-1.0 / (0.05 * fs));
        for (int n = 0; n <= kControlInterval; ++n) avgSeg[n] = 1.0 - std::exp (-(double) n / (0.05 * fs));
        lineEnvRelease = 1.0 - std::exp (-1.0 / (lineEnvSeconds * fs));
        o1EnvRelease = 1.0 - std::exp (-1.0 / (o1EnvSeconds * fs));
        if (bridging) buildReceiver();
        computeBias();
        buildGr();
        for (auto& gl : grLoad)
        {
            auto inv2 = [] (const double (&m)[2][2], double (&r)[2][2])
            {
                const double det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
                r[0][0] = m[1][1] / det; r[0][1] = -m[0][1] / det; r[1][0] = -m[1][0] / det; r[1][1] = m[0][0] / det;
            };
            double M[2][2] = { { 1.0 - g19 * gl.Z[0][0], -g19 * gl.Z[0][1] }, { -g19 * gl.Z[1][0], 1.0 - g19 * gl.Z[1][1] } };
            inv2 (M, gl.N);
            double B[2][2];
            for (int i = 0; i < 2; ++i)
                for (int j = 0; j < 2; ++j)
                    B[i][j] = g19 * gl.N[i][j] + (i == j ? 1.0 / 47e3 : 0.0);
            inv2 (B, gl.Ainv);
            for (int i = 0; i < 2; ++i)
                for (int j = 0; j < 2; ++j)
                    gl.NG[i][j] = gl.Ainv[i][0] * gl.N[0][j] * g19 + gl.Ainv[i][1] * gl.N[1][j] * g19;   // d va-offset / d v_open
        }
        applyGrCoupling();
        prepared = true;
        reset();
    }

    void reset()
    {
        if (! prepared) return;
        posInputC = posInputT; posOutputC = posOutputT; posAttackC = posAttackT; posReleaseC = posReleaseT;
        ratioC = ratioT; offC = offT;
        hpIdxC = hpIdxT;
        hpKT = hpKFor (hpIdxC > 0 ? hpIdxC : 3);
        hpGT = hpIdxC > 0 ? 1.0 : 0.0;
        hpG = hpGT; hpK = hpKT; hpLeft = 0; hpLp = 0.0;
        hpF = 1.0 - hpG * hpK; hpC = -hpG * (1.0 - hpK) * hpLp;
        updateScale();
        applyTimes();
        // the meter zero (R55) is set at idle with a 20:1 button in and limiting off
        buildInput();
        buildCell();
        buildOutBlock();
        buildLineA();
        buildLineB();
        lineA.clearState();
        lineB.clearState();
        lineEnv = 0.0;
        o1Env = 0.0;
        // idle ctl at 20:1
        double c5Off20;
        idleControl (3, controlIdle, c5Off20);
        sourceIdle = 0.0;
        idleConductance = q1Conductance (controlIdle + 0.5 * c5Off20 - sourceIdle);
        meterSlope = (1.0 - std::pow (10.0, -10.0 / 20.0)) / (controlForReduction (10.0) - controlIdle);
        clearStates();
        calCurrent = ctl.calibrationDbu;
        updateScale();
        countdown = 0;
        needle = ctl.meter == MeterMode::gainReduction ? meterDeflection() : 0.0;
        needleSpeed = 0.0;
    }

    // ==============================================================================================
    /** The one-pole's coefficient k for the S/C HPF choice (1 = 30 Hz ... 5 = 185 Hz) at the sample rate in use. */
    double hpKFor (int idx) const noexcept
    {
        static constexpr double kCorners[6] = { 0.0, 30.0, 60.0, 105.0, 125.0, 185.0 };
        return 1.0 - std::exp (-2.0 * 3.14159265358979323846 * kCorners[std::clamp (idx, 1, 5)] / fs);
    }
    /** A new choice: off glides g to 0 and keeps the corner; on glides g to 1 (the corner is taken at once while off, and glides
        from one corner to another). About 10 ms either way. */
    void hpStart() noexcept
    {
        hpIdxC = hpIdxT;
        if (hpIdxC == 0) { hpGT = 0.0; hpKT = hpK; }
        else
        {
            hpGT = 1.0;
            hpKT = hpKFor (hpIdxC);
            if (hpG == 0.0) hpK = hpKT;
        }
        hpLeft = std::max (1, (int) std::lround (0.010 * fs));
        hpDG = (hpGT - hpG) / hpLeft;
        hpDK = (hpKT - hpK) / hpLeft;
    }

    void controlTick()
    {
        auto glide = [] (float& cur, float want)
        {
            if (cur == want) return false;
            if (std::abs (want - cur) > (float) kPotStep) cur += want > cur ? (float) kPotStep : -(float) kPotStep;
            else cur = want;
            return true;
        };
        if (glide (posInputC, posInputT)) { buildInput(); buildCell(); }
        if (glide (posOutputC, posOutputT)) buildLineA();
        const bool a = glide (posAttackC, posAttackT), r = glide (posReleaseC, posReleaseT);
        if (a || r) applyTimes();
        if (ratioC != ratioT) { ratioC = ratioT; applyRatio(); buildOutBlock(); }
        offC = offT;
        if (hpIdxC != hpIdxT) hpStart();
        if (calCurrent != ctl.calibrationDbu)
        {
            const float step = 0.25f, diff = ctl.calibrationDbu - calCurrent;
            calCurrent = std::abs (diff) <= step ? ctl.calibrationDbu : calCurrent + (diff > 0.0f ? step : -step);
            updateScale();
        }
    }

    // ==============================================================================================
    /** Lambert W of exp(c) (the principal branch, argument >= 0). */
    static inline double lambertFromLog (double c) noexcept
    {
        if (c > 1.0)
        {
            double t = c - afast::log (c) + afast::log (c) / c;
            for (int it = 0; it < 3; ++it)
                t -= (t + afast::log (t) - c) / (1.0 + 1.0 / t);
            return t;
        }
        const double lam = afast::exp (c);
        double w = lam / (1.0 + lam);
        for (int it = 0; it < 3; ++it)
        {
            const double ew = afast::exp (w);
            w -= (w * ew - lam) / (ew * (w + 1.0));
        }
        return w;
    }

    /** tanh to about 1e-9 (the soft limits are fitted shapes; the library tanh costs several times as much). */
    static inline double fastTanh (double x) noexcept
    {
        const double a = std::abs (x);
        const double th = a > 20.0 ? 1.0 : 1.0 - 2.0 / (afast::exp (2.0 * a) + 1.0);
        return x < 0.0 ? -th : th;
    }

    /** A soft limit: x passes unchanged between lo + kwLo and hi - kwHi and bends smoothly (tanh) into lo and hi. `slope` is dy/dx. */
    static inline double softLimit (double x, double lo, double hi, double& slope, double kwHi = 3.0, double kwLo = -1.0) noexcept
    {
        if (kwLo < 0.0) kwLo = kwHi;
        slope = 1.0;
        if (x > hi - kwHi)
        {
            const double th = fastTanh ((x - (hi - kwHi)) / kwHi);
            slope = 1.0 - th * th;
            return (hi - kwHi) + kwHi * th;
        }
        if (x < lo + kwLo)
        {
            const double th = fastTanh (((lo + kwLo) - x) / kwLo);
            slope = 1.0 - th * th;
            return (lo + kwLo) - kwLo * th;
        }
        return x;
    }

    /** One evaluation of the closed loop at a trial control voltage x: the cell (Newton on Q1's
        current, plant eliminated), the GR amp, and the rectifier / C27 update, with the slope of the
        control voltage that comes out against the one that went in. Nothing is committed. */
    struct LoopEval
    {
        double y[6];                 // cell outputs (y[4]: the preamp output stage's node, before its limit and C9)
        double o1c, pre;
        double i;                    // Q1 current
        double e13, e15;
        double A3, A4, I, i3, i4, hh3, hh4;
        double ctlOut, slope, vk;
        double vz0, c0n, V0, Rk;
        double dg, dd, dF, didx, clipSlope, j3, j4, denom;
        double G[2];                 // the excess collector currents of Q12 and Q14
    };

    /** The cell and the preamp for a trial control voltage x: fills L up to the preamp output after C9 (L.pre). */
    UREI1176_FORCE_INLINE void evalCell (const double* p0, double x, LoopEval& L, bool newton = true, double iGiven = 0.0, bool first = false) noexcept
    {
        // --- cell (p0: the part of the outputs that does not depend on the control voltage)
        const double uc = (x + ctlOffset) - ctlRef;
        double p[6];
        for (int o = 0; o < 6; ++o) p[o] = p0[o] + cell.D[o][1] * uc;
        const double kDs = cellKDs, kGs = cellKGs;
        const double vds0 = p[0] - p[1];
        const double vgs0 = ctlRef + 0.5 * (p[2] + uc) - p[1];
        double i = newton ? iWarm : iGiven;
        // The first guess for Q1's current: Q1's law linearised at the last point it was evaluated at (a chord, no table look-ups)
        if (first && newton && q1Ok)
        {
            i = (q1Id + q1Gsg * (vgs0 - q1VgsLin) + q1Dd * (vds0 - q1Vds)) * q1InvDF;
        }
        double id = 0.0, dg = 0.0, dd = 0.0, dF = 1.0;
        for (int it = 0; it < 12; ++it)
        {
            // Q1's gate junctions (to source and drain) hold the gate about half a volt above them: the largest reductions stop there
            double gs;
            const double vgsLinear = vgs0 + kGs * i;
            double vgs = vgsLinear;
            gs = 1.0;
            if (vgsLinear > gateMax - gateKnee) vgs = softLimit (vgsLinear, -1e9, gateMax, gs, gateKnee);
            const double vds = vds0 + kDs * i;
            q1Eval (vgs, vds, id, dg, dd);
            const double F = i - id;
            dF = 1.0 - dg * gs * kGs - dd * kDs;
            q1Ok = true; q1Id = id; q1Gsg = dg * gs; q1Dd = dd; q1VgsLin = vgsLinear; q1Vds = vds;
            if (! newton) break;
            const double invDF = 1.0 / dF;
            q1InvDF = invDF;
            double di = -F * invDF;
            const double lim = std::max (std::abs (kGs * di) / 0.5, std::abs (kDs * di) / 2.0);
            if (lim > 1.0) di /= lim;
            i += di;
            if (std::abs (kDs * di) < q1Tol && std::abs (kGs * di) < q1Tol) break;   // quadratic: the error left is ~1e-9 of a volt
        }
        L.i = i; L.dg = dg; L.dd = dd; L.dF = dF;
        for (int o = 0; o < 6; ++o) L.y[o] = p[o] + cell.D[o][2] * i;
        // the preamp output stage limits (Q4 follows Q3's collector between its saturation and cut-off), then C9
        L.o1c = softLimit (L.y[4] - o1ShiftPerVolt * std::max (0.0, o1Env - o1ShiftStart), o1Lo, o1Hi, L.clipSlope, o1Knee, o1KneeLo);
        outBlk.output<1> (&L.o1c, &L.pre);
    }

    UREI1176_FORCE_INLINE void evalLoop (const double* p0, double x, LoopEval& L, bool newton = true, double iGiven = 0.0, bool first = false) noexcept
    {
        evalCell (p0, x, L, newton, iGiven, first);
        sideEval (L.pre, L.y[5], L);
    }

    /** phi(z) = e^z - 1 - z and phi'(z) = e^z - 1 (the series where the exponential would lose them to rounding). */
    static inline void phiOf (double z, double& phi, double& dphi) noexcept
    {
        if (std::abs (z) < 0.7)
        {
            phi = z * z * (1.0 / 2 + z * (1.0 / 6 + z * (1.0 / 24 + z * (1.0 / 120 + z * (1.0 / 720 + z * (1.0 / 5040 + z * (1.0 / 40320 + z * (1.0 / 362880))))))));
            dphi = phi + z;
            return;
        }
        double e;
        if (z > 40.0) e = afast::exp (40.0) * (1.0 + (z - 40.0));
        else          e = afast::exp (z);
        phi = e - 1.0 - z; dphi = e - 1.0;
    }

    /** The GR amp's Q12 and Q14 large-signal excess currents G (collector; the base carries G / beta) for the base-emitter deviations
        x = x0 + K G the linear block gives, by Newton (2 x 2) from the last solution: G = IF (e^(x/Vt) - 1 - x/Vt) per transistor. Fills the
        followers' outputs e13 and e15 (before the load C19 and C20 draw, which enters through the last sample's currents). */
    UREI1176_FORCE_INLINE void solveGr (const Lti<1, kGrU, kGrO>& gr, const double* y, LoopEval& L) noexcept
    {
        const double IF0 = if0, IF1 = if1;
        const double x0a = y[2] + gr.D[2][1] * i19 + gr.D[2][2] * i20, x0b = y[3] + gr.D[3][1] * i19 + gr.D[3][2] * i20;
        grX0[0] = x0a; grX0[1] = x0b;
        const double k00 = gr.D[2][3], k01 = gr.D[2][4], k10 = gr.D[3][3], k11 = gr.D[3][4];
        double g0 = grGWarm[0], g1 = grGWarm[1];
        for (int it = 0; it < 30; ++it)
        {
            const double xa = x0a + k00 * g0 + k01 * g1, xb = x0b + k10 * g0 + k11 * g1;
            double pa, da, pb, db;
            phiOf (xa * (1.0 / kVt), pa, da);
            phiOf (xb * (1.0 / kVt), pb, db);
            const double fa = g0 - IF0 * pa, fb = g1 - IF1 * pb;
            const double sa = IF0 * da * (1.0 / kVt), sb = IF1 * db * (1.0 / kVt);
            const double j00 = 1.0 - sa * k00, j01 = -sa * k01, j10 = -sb * k10, j11 = 1.0 - sb * k11;
            const double idet = 1.0 / (j00 * j11 - j01 * j10);
            const double d0 = -(j11 * fa - j01 * fb) * idet, d1 = -(j00 * fb - j10 * fa) * idet;
            const double dxa = k00 * d0 + k01 * d1, dxb = k10 * d0 + k11 * d1;
            const double dxMax = std::max (std::abs (dxa), std::abs (dxb));
            const double alpha = dxMax > 0.1 ? 0.1 / dxMax : 1.0;
            g0 += alpha * d0; g1 += alpha * d1;
            if (alpha == 1.0 && dxMax < 5.0e-5) break;   // quadratic: the error left is ~1e-7 of a volt
        }
        L.G[0] = g0; L.G[1] = g1;
        grGWarm[0] = g0; grGWarm[1] = g1;
        L.e13 = y[0] + gr.D[0][3] * g0 + gr.D[0][4] * g1;
        L.e15 = y[1] + gr.D[1][3] * g0 + gr.D[1][4] * g1;
    }

    /** d e13 / d pre and d e15 / d pre of the Q12 / Q14 solve at the present point (for the loop's Newton step; rare). */
    void grSlopes (const Lti<1, kGrU, kGrO>& gr, const double* G, double& g13, double& g15) const noexcept
    {
        const double IF0 = if0, IF1 = if1;
        const double k00 = gr.D[2][3], k01 = gr.D[2][4], k10 = gr.D[3][3], k11 = gr.D[3][4];
        double pa, da, pb, db;
        phiOf ((grX0[0] + k00 * G[0] + k01 * G[1]) * (1.0 / kVt), pa, da);
        phiOf ((grX0[1] + k10 * G[0] + k11 * G[1]) * (1.0 / kVt), pb, db);
        const double sa = IF0 * da * (1.0 / kVt), sb = IF1 * db * (1.0 / kVt);
        const double j00 = 1.0 - sa * k00, j01 = -sa * k01, j10 = -sb * k10, j11 = 1.0 - sb * k11;
        const double idet = 1.0 / (j00 * j11 - j01 * j10);
        const double ra = sa * gr.D[2][0], rb = sb * gr.D[3][0];   // dF/dpre
        const double d0 = (j11 * ra - j01 * rb) * idet, d1 = (j00 * rb - j10 * ra) * idet;
        g13 = gr.D[0][0] + gr.D[0][3] * d0 + gr.D[0][4] * d1;
        g15 = gr.D[1][0] + gr.D[1][3] * d0 + gr.D[1][4] * d1;
    }

    /** The slopes of Q1's current and of the control voltage the rectifier produces against the control voltage that went in
        (for the loop's Newton step; not needed when the first guess was good enough). */
    UREI1176_FORCE_INLINE void loopSlopes (LoopEval& L) const noexcept
    {
        const double gx = 0.5 * (cell.D[2][1] + 1.0) - cell.D[1][1], dxs = cell.D[0][1] - cell.D[1][1];
        const double didx = (L.dg * gx + L.dd * dxs) / L.dF;
        L.didx = didx;
        const bool active = ! offC && ratioC != (int) Ratio::none;
        const auto& gr = grBlk[(size_t) grVariant];
        const auto& gl = grLoad[(size_t) grVariant];
        const double dpre = hpF * outBlk.D[0][0] * L.clipSlope * (cell.D[4][1] + cell.D[4][2] * didx);   // hpF: the S/C HPF's feedthrough
        const double dictl = cell.D[5][1] + cell.D[5][2] * didx;
        double g13 = 0.0, g15 = 0.0;
        if (active) grSlopes (gr, L.G, g13, g15);
        const double de13 = g13 * dpre, de15 = g15 * dpre;
        const double dI = (L.j3 * gl.NG[0][0] * de15 + L.j4 * gl.NG[1][1] * de13) / L.denom;
        L.slope = kap27 * (dI - dictl);
    }

    /** The GR amp and the rectifier for a preamp output `preIn` (volts) and the current the gate network draws
        from the control node; fills L.ctlOut and its slope against the control voltage that went in. */
    UREI1176_FORCE_INLINE void sideEval (double preIn, double iCtl, LoopEval& L) noexcept
    {
        sideAnodes (preIn, iCtl, L);
        // --- rectifier
        L.V0 = kap27 * L.c0n + L.vz0;
        L.Rk = kap27 + rAw + rho;
        double j3, j4;
        L.I = solveRectifier (L.A3, L.A4, L.V0, L.Rk, L.i3, j3, j4);
        L.i4 = L.I - L.i3;
        L.ctlOut = kap27 * (L.I + L.c0n);
        L.vk = L.V0 + L.Rk * L.I;
        L.j3 = j3; L.j4 = j4;
        L.denom = 1.0 + (L.Rk + 0.5 * (aAnode3 + aAnode4)) * (j3 + j4);
    }

    /** The GR amp and what the rectifier needs of it: the anode voltages A3 and A4, C22's and C27's history terms, and the current
        the control node sees apart from the diodes (L.c0n). */
    UREI1176_FORCE_INLINE void sideAnodes (double preIn, double iCtl, LoopEval& L) noexcept
    {
        // --- GR amp
        const bool active = ! offC && ratioC != (int) Ratio::none;
        const double grIn = active ? hpF * preIn + hpC : 0.0;   // the S/C HPF (exactly preIn while it is off)
        const int variant = grVariant;
        const auto& gr = grBlk[(size_t) variant];
        double e[kGrO];
        gr.output<1> (&grIn, e);   // with the load currents of the last committed sample folded in below
        solveGr (gr, e, L);

        // --- rectifier
        const auto& gl = grLoad[(size_t) grVariant];
        L.hh3 = -g19 * vc19 - i19;     // the caps' history sources
        L.hh4 = -g19 * vc20 - i20;
        {
            const double r0 = ref * kInv47k;
            const double q0 = g19 * L.e15 + L.hh3, q1 = g19 * L.e13 + L.hh4;   // what each cap would push in
            const double b0 = gl.N[0][0] * q0 + gl.N[0][1] * q1 + r0, b1 = gl.N[1][0] * q0 + gl.N[1][1] * q1 + r0;
            L.A3 = gl.Ainv[0][0] * b0 + gl.Ainv[0][1] * b1;
            L.A4 = gl.Ainv[1][0] * b0 + gl.Ainv[1][1] * b1;
        }
        L.vz0 = rho * (g22 * vz + i22);
        L.c0n = -iCtl + biasOverRel + g27 * ctlV + i27;
    }

    /** Solves the cathode node for the diode currents; returns the total current into the node and
        the slopes dI/dA of the two diodes (before the series factor). */
    UREI1176_FORCE_INLINE double solveRectifier (double a3, double a4, double v0, double rk0, double& i3out, double& j3out, double& j4out) noexcept
    {
        const double u3 = a3 - v0, u4 = a4 - v0;
        rectConstants (rk0);
        const int wd = u3 > u4 ? 0 : 1;
        const double rk = rkc[wd];   // the anode's own series resistance (one diode conducts at a time)
        const double xm = std::max (u3, u4) * kInvPsi;
        double i3, i4, j3, j4;
        if (xm < -14.0)
        {
            // both diodes far into reverse: their leakage, nothing to solve
            i3out = -kIs * kRatioLeak; j3out = j4out = 0.0;
            return -2.0 * kIs * kRatioLeak;
        }
        if (xm < 2.0)
        {
            // barely conducting (at most tens of nA): the drop across Rk is a first-order correction
            diodeTabulated (u3, i3, j3);
            diodeTabulated (u4, i4, j4);
            i3 *= 1.0 - rk * j3;   // first order (rk j is below 1e-3 here)
            i4 *= 1.0 - rk * j4;
            i3out = i3; j3out = j3; j4out = j4;
            return i3 + i4;
        }
        // Lambert-W start for the forward exponentials (log domain, no overflow), then Newton on the exact node equation. The current
        // only has to be right to a fraction of a microamp (C27's 47 ohm turns that into volts), so the Newton step is accepted as soon as its
        // own quadratic error (dI^2 / 2I) is a nanoamp; the diodes' values at the new current follow from the last evaluation to first order.
        const double dx = std::abs (u3 - u4) * kInvPsi;
        double sp, sg;
        afast::softplusSigmoid (-dx, sp, sg);   // ln (1 + e^-dx): the second diode's share
        const double lnLam = lnRkc[wd] + xm + sp;
        double I;
        if (lnLam > LambertTable::lo && lnLam < LambertTable::hi - 1.0 / LambertTable::per)
            I = kPsi * lambertTabulated (lnLam) * irkc[wd];
        else
            I = kPsi * lambertFromLog (lnLam) * irkc[wd];
        bool exact = false;
        double dI = 0.0;
        for (int it = 0; it < 30; ++it)
        {
            diode (u3 - rk * I, i3, j3);
            diode (u4 - rk * I, i4, j4);
            const double F = I - (i3 + i4), dF = 1.0 + rk * (j3 + j4);
            dI = -F / dF;
            if (I + dI < -1.0e-8) dI = -1.0e-8 - I;
            I += dI;
            if (dI * dI < 4.0e-9 * (std::abs (I) + 1.0e-9) || std::abs (dI) < 1.0e-13) { exact = true; break; }
        }
        if (exact)
        {
            i3 -= rk * dI * j3;
            i4 -= rk * dI * j4;
        }
        else
        {
            diode (u3 - rk * I, i3, j3);
            diode (u4 - rk * I, i4, j4);
        }
        const double sum = i3 + i4;
        i3out = sum != 0.0 ? I * i3 / sum : 0.5 * I;
        j3out = j3; j4out = j4;
        return I;
    }

    UREI1176_FORCE_INLINE double lineStep (double pre) noexcept
    {
        double lo, out;
        lineA.output<1> (&pre, &lo);
        lineA.advance (&pre);
        // The output stage's rails, with the shift the real amplifier shows when it is overdriven (its operating point slides as the
        // clipping gets asymmetric): the signal is moved by 0.4 V per volt that its peak exceeds 14 V before it meets the limits.
        const double a = std::abs (lo);
        lineEnv = a > lineEnv ? a : lineEnv + (a - lineEnv) * lineEnvRelease;
        const double shift = lineShiftPerVolt * std::max (0.0, lineEnv - lineShiftStart);
        double slope;
        lo = softLimit (lo - shift, lineLo, lineHi, slope, lineKneeHi, lineKneeLo);
        lineB.output<1> (&lo, &out);
        lineB.advance (&lo);
        return out;
    }

    /** Advances the GR amp, the rectifier, C27 and C22 and the threshold reference with the solved sample. */
    UREI1176_FORCE_INLINE void sideCommit (const LoopEval& L, double preN, double ctlN) noexcept
    {
        const bool active = ! offC && ratioC != (int) Ratio::none;
        vk = L.vk;
        const auto& gl = grLoad[(size_t) grVariant];
        va3 = L.A3 - gl.Ainv[0][0] * L.i3 - gl.Ainv[0][1] * L.i4;
        va4 = L.A4 - gl.Ainv[1][0] * L.i3 - gl.Ainv[1][1] * L.i4;
        {
            // the currents C19 and C20 draw, and the followers' outputs under that load
            const double q0 = g19 * (L.e15 - va3) + L.hh3, q1 = g19 * (L.e13 - va4) + L.hh4;
            const double iC3 = gl.N[0][0] * q0 + gl.N[0][1] * q1, iC4 = gl.N[1][0] * q0 + gl.N[1][1] * q1;
            e15 = L.e15 + gl.Z[0][0] * iC3 + gl.Z[0][1] * iC4;
            e13 = L.e13 + gl.Z[1][0] * iC3 + gl.Z[1][1] * iC4;
            vc19 = e15 - va3; i19 = iC3;
            vc20 = e13 - va4; i20 = iC4;
            const double ug[kGrU] = { active ? hpF * preN + hpC : 0.0, iC3, iC4, L.G[0], L.G[1] };
            grBlk[(size_t) grVariant].advance (ug);
        }
        const double refN = (g21 * ref + i21 + refDrive + (va3 + va4) * kInv47k) * refInvDen;
        i21 = g21 * (refN - ref) - i21;
        ref = refN;
        i27 = g27 * (ctlN - ctlV) - i27;
        const double vzN = rho * L.I + L.vz0;
        i22 = g22 * (vzN - vz) - i22;
        vz = vzN;
        ctlV = ctlN;
        Iprev = L.I;
        // the S/C HPF: this sample's low-pass state, the glide, and the feedthrough and history term for the next sample
        hpLp += hpK * (preN - hpLp);
        if (hpLeft > 0)
        {
            if (--hpLeft == 0) { hpG = hpGT; hpK = hpKT; }
            else { hpG += hpDG; hpK += hpDK; }
        }
        hpF = 1.0 - hpG * hpK;
        hpC = -hpG * (1.0 - hpK) * hpLp;
    }

    /** The input network: the plug-in's level in volts at the 600 ohm source's EMF, T1's secondary open-circuit volts out. */
    struct Segment;
    /** End of a loop segment of `count` samples: the gate-drive meter's averages and peak. */
    inline void segmentEnd (int count) noexcept
    {
        auto limit = [this] (double v)
        {
            double unused;
            return v > gateMax - gateKnee ? softLimit (v, -1e9, gateMax, unused, gateKnee) : v;
        };
        vgsControl = limit (rawLast);
        vgsPeak = std::max (vgsPeak, limit (rawPeak));
        rawPeak = -1.0e9;
        fbAvg += avgSeg[count] * (fbLast - fbAvg);
        srcAvg += avgSeg[count] * (srcLast - srcAvg);
        rawBase = 0.5 * ctlRef + 0.5 * fbAvg - srcAvg;
    }
    void inputPass (const float* seg, int count, Segment& sg) noexcept;
    bool linePass (float* seg, int count, Segment& sg) noexcept;
    inline void lineSample (float* seg, int k, const Segment& sg, bool absDrive) noexcept;
    inline void needleStep (int count, const Segment& sg) noexcept;
    bool lineOk (const float* seg, int count, const Segment& sg) const noexcept;
    void recover (float* seg, int count) noexcept;
    void tick() noexcept;
    UREI1176_FORCE_INLINE double inputStep (double emf) noexcept
    {
        double voc;
        inputBlk.output<1> (&emf, &voc);
        inputBlk.advance (&emf);
        return voc;
    }
    /** One clamp's current at output voltage v (AC, re the bias): the top diode to +H and the bottom to -H, Is (limitedExp (x) - 1), x = (v -+ H) / Vt. */
    static inline void clampCurrent (double v, double H, double Is, double& i, double& di) noexcept
    {
        static constexpr double xMax = 40.0;   // limitedExp's knee, as the reference's diode device
        auto limitedExp = [] (double x, double& d)
        {
            const double e = std::exp (std::min (x, xMax));
            d = e;
            return x > xMax ? e * (1.0 + (x - xMax)) : e;
        };
        double dt, db;
        const double et = limitedExp ((v - H) / kVt, dt), eb = limitedExp ((-H - v) / kVt, db);
        i = Is * (et - eb);
        di = Is * (dt + db) / kVt;
    }

    /** v = c - R i (v) for one clamped output, by Newton from the right side of the root (the function is convex and increasing there, so the iteration
        descends monotonically); the lower clamp is the mirror image. Returns i (v). */
    static inline double solveClamp (double c, double R, double H, double Is) noexcept
    {
        const double sign = c < 0.0 ? -1.0 : 1.0;
        const double cc = std::abs (c);
        if (cc <= H - kClampQuietVolts) return 0.0;
        // start right of the root: at cc, or 1.1 V past H, just beyond limitedExp's knee (40 Vt = 1.03 V), where R i already exceeds any level
        double v = std::min (cc, H + 1.1), i = 0.0, di = 0.0;
        for (int it = 0; it < 60; ++it)
        {
            clampCurrent (v, H, Is, i, di);
            const double g = v - cc + R * i, step = g / (1.0 + R * di);
            v -= step;
            if (std::abs (step) < 1.0e-12) break;
        }
        clampCurrent (v, H, Is, i, di);
        return sign * i;
    }

    /** Rev G and J: the receiver with IC2's output clamps, then C33 and the pot. Between the clamps' thresholds the receiver is the linear block; past
        them the two clamp currents are solved inside the sample (the K method: each output is its linear prediction less the block's own
        feedthrough times the clamp's current), as the reference's Newton group does, and fed back into the block's states, so the op-amp
        macro-models' internal poles wind up as the reference's do. */
    UREI1176_FORCE_INLINE double inputStepBridging (double emf) noexcept
    {
        double y0[2];
        inputRx.output<1> (&emf, y0);
        double u[3] = { emf, 0.0, 0.0 };
        double a = y0[1];
        if (std::abs (y0[0]) > clampSafe || std::abs (y0[1]) > clampSafe)
        {
            // v = y0 + D i: each output is its prediction plus the feedthrough from the two clamp currents (negative: current leaving a node lowers it)
            const double rB = -inputRx.D[0][1], rA = -inputRx.D[1][2];
            double iB = 0.0, iA = 0.0;
            for (int pass = 0; pass < 3; ++pass)
            {
                iB = solveClamp (y0[0] + inputRx.D[0][2] * iA, rB, clampH, clampIs);
                iA = solveClamp (y0[1] + inputRx.D[1][1] * iB, rA, clampH, clampIs);
            }
            u[1] = iB; u[2] = iA;
            a = y0[1] + inputRx.D[1][1] * iB + inputRx.D[1][2] * iA;
        }
        inputRx.advance (u);
        double v;
        inputPot.output<1> (&a, &v);
        inputPot.advance (&a);
        return v;
    }

    /** One sample of the compression loop in four stages, so that two channels can be stepped alternately (see FastChannel::processPair):
        the cell, the preamp, the GR amp, the rectifier and the control voltage, closed within the sample. */
    struct StepCtx
    {
        LoopEval L;
        double p0[6], qh[6];
        double x, voc, c0p;
    };

    /** Stage 1: the part of the cell's outputs that the control voltage does not decide, the next sample's, and the first guess. */
    UREI1176_FORCE_INLINE void stepPrep (StepCtx& S, double voc) noexcept
    {
        S.voc = voc;
        for (int o = 0; o < 6; ++o) S.p0[o] = cellY0[o] + cell.D[o][0] * voc;
        // (C A C^-1) q, for the next sample's outputs: independent of the solve below
        for (int o = 0; o < 6; ++o) S.qh[o] = 0.0;
        for (int j = 0; j < 6; ++j)
        {
            const double qj = cellY0[j];
            for (int o = 0; o < 6; ++o) S.qh[o] += cellQt[j][o] * qj;
        }
        // first guess: C27's trapezoid equation with last sample's diode current and the gate network's current at the old control voltage
        const double iCtlPred = S.p0[5] + cell.D[5][1] * ((ctlV + ctlOffset) - ctlRef) + cell.D[5][2] * iPrev;
        const double c0p = -iCtlPred + biasOverRel + g27 * ctlV + i27;
        S.c0p = c0p;
        S.x = kap27 * (Iprev + c0p);
        iWarm = iPrev + 0.9 * (iPrev - iPrev2);
    }

    /** Stage 2: Q1 and the preamp at the guess. */
    UREI1176_FORCE_INLINE void stepCell (StepCtx& S) noexcept { evalCell (S.p0, S.x, S.L, true, 0.0, true); iWarm = S.L.i; }

    /** Stage 3: the GR amp and the rectifier at the guess. */
    UREI1176_FORCE_INLINE void stepSide (StepCtx& S) noexcept { sideEval (S.L.pre, S.L.y[5], S.L); }

    /** Stage 4: the guess is nearly always right; otherwise the loop iterates (bracketed Newton) from it. Then the sample is committed.
        Returns the preamp output after C9 (volts). */
    UREI1176_FORCE_INLINE double stepFinish (StepCtx& S) noexcept
    {
        LoopEval& L = S.L;
        double x = S.x;
        if (std::abs (L.ctlOut - x) >= loopTol)
        {
            double lo = -1e9, hi = 1e9;
            double widthBack1 = 1e30, widthBack2 = 1e30;   // the bracket's width after the last two iterations
            bool newton = true;
            double iGiven = 0.0;
            for (int it = 0; it < 48; ++it)
            {
                if (it > 0)
                {
                    evalLoop (S.p0, x, L, newton, iGiven, false);
                    if (newton) iWarm = L.i;
                }
                const double R = L.ctlOut - x;
                if (std::abs (R) < loopTol || ! newton) break;
                loopSlopes (L);
                if (R > 0.0) lo = std::max (lo, x); else hi = std::min (hi, x);
                const double width = hi - lo;
                const double dx = -R / (L.slope - 1.0);
                double xn = x + dx;
                // R falls with x (more control voltage, less signal): [lo, hi] brackets the root. Newton, unless it leaves the
                // bracket or the bracket has not halved in two iterations (a kink in the diode's turn-on can make it cycle).
                const bool bracketed = lo > -1e8 && hi < 1e8;
                if (xn <= lo || xn >= hi || (bracketed && width > 0.5 * widthBack2))
                    xn = bracketed ? 0.5 * (lo + hi) : x + std::copysign (std::min (std::abs (dx), 1.0), dx);
                widthBack2 = widthBack1;
                widthBack1 = width;
                if (std::abs (xn - x) < smallStep && bracketed == false)
                {
                    // a small correction: Q1's current follows linearly, no new Newton solve
                    iGiven = L.i + L.didx * (xn - x);
                    newton = false;
                }
                x = xn;
            }
        }

        return stepCommit (S, x, L.ctlOut);
    }

    /** Commits the solved sample: x is the control voltage the loop converged on, ctlN the one the side chain produced (equal at convergence;
        a linked pair passes the shared node's). Returns the preamp output after C9 (volts). */
    UREI1176_FORCE_INLINE double stepCommit (StepCtx& S, double x, double ctlN) noexcept
    {
        const LoopEval& L = S.L;
        const double u0 = S.voc, u1 = (x + ctlOffset) - ctlRef, u2 = L.i;
        for (int o = 0; o < 6; ++o)
            cellY0[o] = S.qh[o] + cellRt[0][o] * u0 + cellRt[1][o] * u1 + cellRt[2][o] * u2;
        iPrev2 = iPrev; iPrev = L.i;
        outBlk.advance (&L.o1c);
        { const double a = std::abs (L.y[4]); o1Env = a > o1Env ? a : o1Env + (a - o1Env) * o1EnvRelease; }
        const double sN = L.y[1], c5N = L.y[2], fbN = L.y[3], preN = L.pre;
        iCtlPlant = L.y[5];
        sideCommit (L, preN, ctlN);

        // the meters' state
        rawLast = rawBase + 0.5 * (c5N + (ctlN + ctlOffset) - fbN);
        rawPeak = std::max (rawPeak, rawLast);
        fbLast = fbN; srcLast = sN;

        return preN;
    }

    UREI1176_FORCE_INLINE double loopStep (double voc) noexcept
    {
        StepCtx S;
        stepPrep (S, voc);
        stepCell (S);
        stepSide (S);
        return stepFinish (S);
    }

    // ==============================================================================================
    // The 1176SA tie: two units' control nodes (C27) joined. Both rectifiers charge one node, both C27s are in parallel, both release /
    // bias returns load it, and each gain cell's gate network draws from it; each unit keeps its own attack path (R + C22), GR amp and
    // cell. One control-voltage unknown x is solved for the pair inside the sample, and the two rectifier currents with it.
    //
    // The shared node: x = kapS (I_a + I_b + c0_a + c0_b), with kapS = 1 / (the two C27s' companion conductances + the two release /
    // bias returns). Unit k's cathode sits at x + (R_attack + rho) I_k + vz0_k.

    /** The shared node's resistance factor for a pair: the reciprocal of the sum of both units' conductances at their C27. */
    static inline double sharedKappa (const Impl& a, const Impl& b) noexcept
    {
        return 1.0 / ((a.g27 + 1.0 / a.rRel) + (b.g27 + 1.0 / b.rRel));
    }

    /** The two rectifiers on one node, for the units' side chains already evaluated up to their anode voltages (A3, A4, c0n, vz0). Fills
        both L's I, i3, i4, j3, j4, vk, denom and the shared ctlOut. The same three regimes and the same Newton start and acceptance as
        solveRectifier, with the Jacobian of the pair: with two identical units it is solveRectifier's iteration on the pair's mean. */
    static void solveRectifierPair (Impl& ma, Impl& mb, LoopEval& La, LoopEval& Lb, double kapS) noexcept
    {
        Impl* const m[2] = { &ma, &mb };
        LoopEval* const L[2] = { &La, &Lb };
        const double c0sum = La.c0n + Lb.c0n;
        const double kEq = kapS + kapS;   // what one unit alone would have: its own C27 and returns
        double u3[2], u4[2], xm[2], rks[2], rkStart[2];
        int wd[2];
        for (int k = 0; k < 2; ++k)
        {
            const double v0 = kapS * c0sum + L[k]->vz0;
            const double rk0 = kEq + m[k]->rAw + m[k]->rho;
            u3[k] = L[k]->A3 - v0;
            u4[k] = L[k]->A4 - v0;
            m[k]->rectConstants (rk0);
            wd[k] = u3[k] > u4[k] ? 0 : 1;
            rkStart[k] = m[k]->rkc[wd[k]];
            rks[k] = m[k]->rAw + m[k]->rho + (wd[k] == 0 ? m[k]->aAnode3 : m[k]->aAnode4);
            xm[k] = std::max (u3[k], u4[k]) * kInvPsi;
        }
        const double xmax = std::max (xm[0], xm[1]);
        double I[2], i3[2], i4[2], j3[2], j4[2], share[2];
        if (xmax < -14.0)
        {
            for (int k = 0; k < 2; ++k)
            {
                I[k] = -2.0 * kIs * kRatioLeak; share[k] = -kIs * kRatioLeak; j3[k] = j4[k] = 0.0;
            }
        }
        else if (xmax < 2.0)
        {
            double i0[2];
            for (int k = 0; k < 2; ++k)
            {
                m[k]->diodeTabulated (u3[k], i3[k], j3[k]);
                m[k]->diodeTabulated (u4[k], i4[k], j4[k]);
                i0[k] = i3[k] + i4[k];
            }
            for (int k = 0; k < 2; ++k)
            {
                // first order, as solveRectifier: the drop across the series resistance, here with the other unit's current too
                const double other = i0[1 - k], own = i0[k];
                const double ratio = own > 1.0e-12 ? std::min(std::max (other / own, 0.0), 10.0) : 1.0;
                const double rk = rks[k] + kapS + kapS * ratio;
                i3[k] *= 1.0 - rk * j3[k];
                i4[k] *= 1.0 - rk * j4[k];
                I[k] = i3[k] + i4[k];
                share[k] = i3[k];
            }
        }
        else
        {
            double dxx[2], sp[2], sg[2];
            for (int k = 0; k < 2; ++k)
            {
                dxx[k] = std::abs (u3[k] - u4[k]) * kInvPsi;
                afast::softplusSigmoid (-dxx[k], sp[k], sg[k]);
                const double lnLam = m[k]->lnRkc[wd[k]] + xm[k] + sp[k];
                if (lnLam > LambertTable::lo && lnLam < LambertTable::hi - 1.0 / LambertTable::per)
                    I[k] = kPsi * m[k]->lambertTabulated (lnLam) * m[k]->irkc[wd[k]];
                else
                    I[k] = kPsi * lambertFromLog (lnLam) * m[k]->irkc[wd[k]];
            }
            bool exact = false;
            double dI[2] = { 0.0, 0.0 };
            for (int it = 0; it < 30; ++it)
            {
                const double tie = kapS * (I[0] + I[1]);
                double F[2], J[2];
                for (int k = 0; k < 2; ++k)
                {
                    const double drop = tie + rks[k] * I[k];
                    diode (u3[k] - drop, i3[k], j3[k]);
                    diode (u4[k] - drop, i4[k], j4[k]);
                    F[k] = I[k] - (i3[k] + i4[k]);
                    J[k] = j3[k] + j4[k];
                }
                const double m11 = 1.0 + J[0] * (rks[0] + kapS), m12 = J[0] * kapS, m21 = J[1] * kapS, m22 = 1.0 + J[1] * (rks[1] + kapS);
                const double invDet = 1.0 / (m11 * m22 - m12 * m21);
                dI[0] = (-F[0] * m22 + F[1] * m12) * invDet;
                dI[1] = (-m11 * F[1] + m21 * F[0]) * invDet;
                for (int k = 0; k < 2; ++k)
                {
                    if (I[k] + dI[k] < -1.0e-8) dI[k] = -1.0e-8 - I[k];
                    I[k] += dI[k];
                }
                exact = true;
                for (int k = 0; k < 2; ++k)
                    exact = exact && (dI[k] * dI[k] < 4.0e-9 * (std::abs (I[k]) + 1.0e-9) || std::abs (dI[k]) < 1.0e-13);
                if (exact) break;
            }
            if (exact)
            {
                const double dTie = kapS * (dI[0] + dI[1]);
                for (int k = 0; k < 2; ++k)
                {
                    const double d = dTie + rks[k] * dI[k];
                    i3[k] -= d * j3[k];
                    i4[k] -= d * j4[k];
                }
            }
            else
            {
                const double tie = kapS * (I[0] + I[1]);
                for (int k = 0; k < 2; ++k)
                {
                    const double drop = tie + rks[k] * I[k];
                    diode (u3[k] - drop, i3[k], j3[k]);
                    diode (u4[k] - drop, i4[k], j4[k]);
                }
            }
            for (int k = 0; k < 2; ++k)
            {
                const double sum = i3[k] + i4[k];
                share[k] = sum != 0.0 ? I[k] * i3[k] / sum : 0.5 * I[k];
            }
        }
        const double ctlOut = kapS * ((I[0] + I[1]) + c0sum);
        for (int k = 0; k < 2; ++k)
        {
            LoopEval& l = *L[k];
            l.I = I[k];
            l.i3 = share[k];
            l.i4 = l.I - l.i3;
            l.j3 = j3[k]; l.j4 = j4[k];
            l.ctlOut = ctlOut;
            l.vk = ctlOut + l.vz0 + (m[k]->rAw + m[k]->rho) * l.I;
        }
    }

    /** Stage 3 of a linked pair. */
    static UREI1176_FORCE_INLINE void stepSideLink (Impl& ma, Impl& mb, StepCtx& Sa, StepCtx& Sb, double kapS) noexcept
    {
        ma.sideAnodes (Sa.L.pre, Sa.L.y[5], Sa.L);
        mb.sideAnodes (Sb.L.pre, Sb.L.y[5], Sb.L);
        solveRectifierPair (ma, mb, Sa.L, Sb.L, kapS);
    }

    /** The slopes of Q1's current and of the shared node's voltage against the voltage that went in, for a pair. */
    UREI1176_FORCE_INLINE void channelSlopes (LoopEval& L, double& q, double& dictl, double& rkm, double& jj) const noexcept
    {
        const double gx = 0.5 * (cell.D[2][1] + 1.0) - cell.D[1][1], dxs = cell.D[0][1] - cell.D[1][1];
        const double didx = (L.dg * gx + L.dd * dxs) / L.dF;
        L.didx = didx;
        const bool active = ! offC && ratioC != (int) Ratio::none;
        const auto& gr = grBlk[(size_t) grVariant];
        const auto& gl = grLoad[(size_t) grVariant];
        const double dpre = hpF * outBlk.D[0][0] * L.clipSlope * (cell.D[4][1] + cell.D[4][2] * didx);   // hpF: the S/C HPF's feedthrough
        dictl = cell.D[5][1] + cell.D[5][2] * didx;
        double g13 = 0.0, g15 = 0.0;
        if (active) grSlopes (gr, L.G, g13, g15);
        const double de13 = g13 * dpre, de15 = g15 * dpre;
        q = L.j3 * gl.NG[0][0] * de15 + L.j4 * gl.NG[1][1] * de13;
        rkm = rAw + rho + 0.5 * (aAnode3 + aAnode4);
        jj = L.j3 + L.j4;
    }

    static UREI1176_FORCE_INLINE void loopSlopesLink (Impl& ma, Impl& mb, LoopEval& La, LoopEval& Lb, double kapS, double& slope) noexcept
    {
        double qa, qb, da, db, ra, rb, ja, jb;
        ma.channelSlopes (La, qa, da, ra, ja);
        mb.channelSlopes (Lb, qb, db, rb, jb);
        const double m11 = 1.0 + ja * (ra + kapS), m12 = ja * kapS, m21 = jb * kapS, m22 = 1.0 + jb * (rb + kapS);
        const double invDet = 1.0 / (m11 * m22 - m12 * m21);
        const double dIa = (qa * m22 - m12 * qb) * invDet, dIb = (m11 * qb - m21 * qa) * invDet;
        slope = kapS * ((dIa + dIb) - (da + db));
    }

    /** Stage 1 of a linked pair: both units' preparation, with the shared node's first guess. */
    static UREI1176_FORCE_INLINE void stepPrepLink (Impl& ma, Impl& mb, StepCtx& Sa, StepCtx& Sb, double vocA, double vocB, double kapS) noexcept
    {
        ma.stepPrep (Sa, vocA);
        mb.stepPrep (Sb, vocB);
        const double x = kapS * ((ma.Iprev + mb.Iprev) + (Sa.c0p + Sb.c0p));
        Sa.x = Sb.x = x;
    }

    /** Stage 4 of a linked pair: the shared control voltage is closed as stepFinish closes one unit's. Commits both. */
    static UREI1176_FORCE_INLINE void stepFinishLink (Impl& ma, Impl& mb, StepCtx& Sa, StepCtx& Sb, double kapS, double& preA, double& preB) noexcept
    {
        LoopEval& La = Sa.L;
        LoopEval& Lb = Sb.L;
        double x = Sa.x;
        if (std::abs (La.ctlOut - x) >= ma.loopTol)
        {
            double lo = -1e9, hi = 1e9;
            double widthBack1 = 1e30, widthBack2 = 1e30;
            bool newton = true;
            double iGivenA = 0.0, iGivenB = 0.0;
            for (int it = 0; it < 48; ++it)
            {
                if (it > 0)
                {
                    ma.evalCell (Sa.p0, x, La, newton, iGivenA, false);
                    mb.evalCell (Sb.p0, x, Lb, newton, iGivenB, false);
                    stepSideLink (ma, mb, Sa, Sb, kapS);
                    if (newton) { ma.iWarm = La.i; mb.iWarm = Lb.i; }
                }
                const double R = La.ctlOut - x;
                if (std::abs (R) < ma.loopTol || ! newton) break;
                double slope;
                loopSlopesLink (ma, mb, La, Lb, kapS, slope);
                if (R > 0.0) lo = std::max (lo, x); else hi = std::min (hi, x);
                const double width = hi - lo;
                const double dx = -R / (slope - 1.0);
                double xn = x + dx;
                const bool bracketed = lo > -1e8 && hi < 1e8;
                if (xn <= lo || xn >= hi || (bracketed && width > 0.5 * widthBack2))
                    xn = bracketed ? 0.5 * (lo + hi) : x + std::copysign (std::min (std::abs (dx), 1.0), dx);
                widthBack2 = widthBack1;
                widthBack1 = width;
                if (std::abs (xn - x) < ma.smallStep && bracketed == false)
                {
                    iGivenA = La.i + La.didx * (xn - x);
                    iGivenB = Lb.i + Lb.didx * (xn - x);
                    newton = false;
                }
                x = xn;
            }
        }
        const double ctlN = La.ctlOut;
        preA = ma.stepCommit (Sa, x, ctlN);
        preB = mb.stepCommit (Sb, x, ctlN);
        ma.ctlOffset = std::abs (ma.ctlOffset) < 1.0e-12 ? 0.0 : ma.ctlOffset * ma.offsetDecay;
        mb.ctlOffset = std::abs (mb.ctlOffset) < 1.0e-12 ? 0.0 : mb.ctlOffset * mb.offsetDecay;
    }

    /** Ties (or unties) the pair's control nodes. Tying: the shared node starts at the mean of the two control voltages, each cell keeps
        seeing the one it had (the offsets) and the offsets glide to zero over ~15 ms, so the gains move together without a step; the cells'
        and the other states continue. Untying: the offsets are folded into each unit's own control voltage, so nothing moves. */
    static void setLinked (Impl& ma, Impl& mb, bool on) noexcept
    {
        if (on)
        {
            if (ma.linked && mb.linked) return;
            const double mean = 0.5 * ((ma.ctlV + ma.ctlOffset) + (mb.ctlV + mb.ctlOffset));
            const double ia = 0.5 * (ma.i27 + mb.i27);
            for (Impl* m : { &ma, &mb })
            {
                m->ctlOffset = (m->ctlV + m->ctlOffset) - mean;
                m->ctlV = mean;
                m->i27 = ia;
                m->offsetDecay = std::exp (-1.0 / (0.015 * m->fs));
                m->linked = true;
            }
        }
        else
        {
            for (Impl* m : { &ma, &mb })
            {
                m->ctlV += m->ctlOffset;
                m->ctlOffset = 0.0;
                m->linked = false;
            }
        }
    }
};

//==============================================================================================
FastChannel::FastChannel() : impl (std::make_unique<Impl>()) {}
FastChannel::~FastChannel() = default;
const FastChannel::Controls& FastChannel::controls() const noexcept { return impl->ctl; }
double FastChannel::voltsPerUnit() const noexcept { return impl->unitVolts; }
double FastChannel::controlVolts() const noexcept { return impl->ctlV + impl->ctlOffset; }

void FastChannel::setBiasPotDiag (double position) noexcept
{
    auto& m = *impl;
    m.biasPos = std::clamp (position, 0.0, 1.0);
    for (int sel = 0; sel < 6; ++sel)
        m.solveBias (sel, m.biasPos, m.biasV[sel], m.refVA6[sel], m.refRA6[sel]);
}

#if UREI1176_FIT_HOOKS
OverloadParams FastChannel::shippedOverload (Version v) noexcept
{
    return specFor (v).overload;
}

BakedConstants FastChannel::bakedConstants (Version v) noexcept
{
    const auto& f = specFor (v);
    return { f.ieQ2, f.ieQ3, f.ieQ4, f.ieQ12, f.ieQ13, f.ieQ14, f.ieQ15, f.icQ5, f.icQ6, f.icQ7, f.icQ8, f.icQ9, f.idBias,
             collectorCurrent (f.ieQ12), collectorCurrent (f.ieQ14) };
}

void FastChannel::setOverloadParams (const OverloadParams& p) noexcept
{
    auto& m = *impl;
    m.o1Lo = p.o1Lo; m.o1Hi = p.o1Hi; m.o1Knee = p.o1KneeHi; m.o1KneeLo = p.o1KneeLo;
    m.o1ShiftPerVolt = p.o1ShiftPerVolt; m.o1ShiftStart = p.o1ShiftStart; m.o1EnvSeconds = p.o1EnvSeconds;
    m.lineLo = p.lineLo; m.lineHi = p.lineHi; m.lineKneeHi = p.lineKneeHi; m.lineKneeLo = p.lineKneeLo;
    m.lineShiftPerVolt = p.lineShiftPerVolt; m.lineShiftStart = p.lineShiftStart; m.lineEnvSeconds = p.lineEnvSeconds;
    m.overloadOverridden = true;
    if (m.prepared)
    {
        m.lineEnvRelease = 1.0 - std::exp (-1.0 / (m.lineEnvSeconds * m.fs));
        m.o1EnvRelease = 1.0 - std::exp (-1.0 / (m.o1EnvSeconds * m.fs));
    }
}
#endif


Version FastChannel::version() const noexcept { return impl->spec->version; }

void FastChannel::prepare (double sampleRate, Version version)
{
    impl->prepare (sampleRate, version);
    needleDb.store ((float) std::clamp (20.0 * std::log10 (std::max (impl->needle, 1e-6)), (double) Range::meterMinDb, (double) Range::meterMaxDb), std::memory_order_relaxed);
}

void FastChannel::adoptControlState (const FastChannel& from) noexcept
{
    auto& m = *impl;
    const auto& f = *from.impl;
    if (! m.prepared || ! f.prepared || &m == &f)
        return;

    // The control voltage: a linked source's gate sees ctlV + ctlOffset, so the effective voltage is taken (this engine is not tied yet;
    // runPair engages the tie from here).
    const double ctlEff = f.ctlV + f.ctlOffset;
    m.ctlV = ctlEff;
    m.ctlOffset = 0.0;
    m.linked = false;
    m.i27 = f.i27;
    m.vz = f.vz; m.i22 = f.i22;                 // C22 and the attack path
    m.ref = f.ref; m.i21 = f.i21;               // the threshold reference
    m.Iprev = f.Iprev; m.vk = ctlEff;           // the rectifier's last current (the next sample's first guess)

    // The GR amp and its coupling: C19 / C20 hold the peak-detector's DC (a 0.3 s memory), with their anodes and history currents;
    // C17 carries its charge in whichever ratio block is running.
    m.va3 = f.va3; m.va4 = f.va4; m.vc19 = f.vc19; m.vc20 = f.vc20; m.i19 = f.i19; m.i20 = f.i20;
    m.e13 = f.e13; m.e15 = f.e15;
    m.grGWarm[0] = f.grGWarm[0]; m.grGWarm[1] = f.grGWarm[1];
    for (size_t v = 0; v < m.grBlk.size(); ++v) m.grBlk[v].h[0] = f.grBlk[v].h[0];

    // The S/C HPF's low-pass (the filter's own glide state follows the controls, which the processor pushes to both).
    m.hpLp = f.hpLp;
    m.hpC = -m.hpG * (1.0 - m.hpK) * m.hpLp;

    // The cell's state (Q1's plant: C1, C2, C5, C8 and the rest). Not the DC steady state at this control voltage (settleCell): with a
    // signal running, the capacitors hold the rectified shift of the signal as well, and starting from the idle solution leaves the
    // control voltage drifting by tens of millivolts over the next 20 ms (measured: 1.1 dB of level at 12 dB of reduction). The state is
    // recovered as buildCell does, h = C^-1 q, from the source's own outputs-as-state; the target's blocks are the same circuit.
    for (int i = 0; i < 6; ++i)
    {
        double sum = 0.0;
        for (int j = 0; j < 6; ++j) sum += f.cellCinv[i][j] * f.cellY0[j];
        m.cell.h[i] = sum;
    }
    m.refreshCellOutputs();

    // The gate-drive meter's state (the 50 ms averages and the last values) and the overload followers.
    m.fbAvg = f.fbAvg; m.srcAvg = f.srcAvg; m.fbLast = f.fbLast; m.srcLast = f.srcLast;
    m.vgsControl = f.vgsControl; m.vgsPeak = f.vgsControl;
    m.rawBase = f.rawBase; m.rawLast = f.rawLast; m.rawPeak = -1.0e9; m.driveSum = 0.0;
    m.lineEnv = f.lineEnv; m.o1Env = f.o1Env;

    // The VU needle and the published meters, so that a meter does not jump at the hand-off; and the control tick, so that the
    // two engines of a pair stay in step.
    m.needle = f.needle; m.needleSpeed = f.needleSpeed;
    m.countdown = f.countdown;
    needleDb.store (from.needleDb.load (std::memory_order_relaxed), std::memory_order_relaxed);
    blockGrDb.store (from.blockGrDb.load (std::memory_order_relaxed), std::memory_order_relaxed);
    grNow.store (from.grNow.load (std::memory_order_relaxed), std::memory_order_relaxed);
}

void FastChannel::reset()
{
    impl->reset();
    const float gr = (float) impl->reductionDbAt (impl->vgsControl);
    grNow.store (gr, std::memory_order_relaxed);
    blockGrDb.store (std::max (0.0f, gr), std::memory_order_relaxed);
    needleDb.store ((float) std::clamp (20.0 * std::log10 (std::max (impl->needle, 1e-6)), (double) Range::meterMinDb, (double) Range::meterMaxDb), std::memory_order_relaxed);
}

void FastChannel::setInput (float rotation) noexcept
{
    impl->ctl.input = std::clamp (rotation, Range::inputMin, Range::inputMax);
    impl->posInputT = (float) Circuit1176::inputPosition (impl->ctl.input);
}
void FastChannel::setOutput (float rotation) noexcept
{
    impl->ctl.output = std::clamp (rotation, Range::outputMin, Range::outputMax);
    impl->posOutputT = (float) Circuit1176::outputPosition (impl->ctl.output);
}
void FastChannel::setAttack (float dial) noexcept
{
    impl->ctl.attack = std::clamp (dial, Range::attackMin, Range::attackMax);
    impl->posAttackT = (float) Circuit1176::attackPosition (impl->ctl.attack);
}
void FastChannel::setRelease (float dial) noexcept
{
    impl->ctl.release = std::clamp (dial, Range::releaseMin, Range::releaseMax);
    impl->posReleaseT = (float) Circuit1176::releasePosition (impl->ctl.release);
}
void FastChannel::setLimitOff (bool off) noexcept { impl->ctl.limitOff = off; impl->offT = off; }
void FastChannel::setRatio (Ratio ratio) noexcept { impl->ctl.ratio = ratio; impl->ratioT = (int) ratio; }
void FastChannel::setSidechainFilter (int choice) noexcept
{
    impl->ctl.sidechainFilter = std::clamp (choice, 0, 5);
    impl->hpIdxT = impl->ctl.sidechainFilter;
}
void FastChannel::setVersionTrims (bool on) noexcept
{
    impl->trimsOn = on;
    impl->updateScale();
}
void FastChannel::setMeterMode (MeterMode mode) noexcept { impl->ctl.meter = mode; impl->updateScale(); }
void FastChannel::setCalibrationDbu (float dbu) noexcept
{
    impl->ctl.calibrationDbu = std::clamp (dbu, Range::calibrationMin, Range::calibrationMax);
}

// One segment of at most kControlInterval samples is three passes. The stages are not all feedback: the input network only feeds the
// loop, and the line amp and the meters only follow it, so each runs over the segment on its own (short, independent dependency chains).
struct FastChannel::Impl::Segment
{
    double voc[kControlInterval], pre[kControlInterval], ctl[kControlInterval];
};

void FastChannel::Impl::inputPass (const float* seg, int count, Segment& sg) noexcept
{
    if (bridging)   // fixed between prepare() calls: one test per segment
    {
        for (int k = 0; k < count; ++k)
            sg.voc[k] = inputStepBridging ((double) seg[k] * inGain);
        return;
    }
    for (int k = 0; k < count; ++k)
        sg.voc[k] = inputStep ((double) seg[k] * inGain);
}

/** One sample of the line amp (and what the +4 / +8 meter modes average). */
UREI1176_FORCE_INLINE void FastChannel::Impl::lineSample (float* seg, int k, const Segment& sg, bool absDrive) noexcept
{
    const double out = lineStep (sg.pre[k]);
    seg[k] = (float) (out * outGain);
    if (absDrive) driveSum += std::abs (out);
}

/** The VU needle over a segment: the second-order ballistics step once, with the segment's mean drive. */
inline void FastChannel::Impl::needleStep (int count, const Segment& sg) noexcept
{
    const MeterMode mode = ctl.meter;
    double drive = 0.0;
    if (mode == MeterMode::gainReduction)
        drive = std::max (0.0, 1.0 - meterSlope * (sg.ctl[count - 1] - controlIdle));
    else if (mode == MeterMode::plus4 || mode == MeterMode::plus8)
        drive = driveSum / (vuReference * (double) count);
    driveSum = 0.0;
    const double dt = (double) count / fs, w2 = kVuOmega * kVuOmega, damp = 2.0 * kVuZeta * kVuOmega;
    needleSpeed += dt * (w2 * (drive - needle) - damp * needleSpeed);
    needle += dt * needleSpeed;
    if (needle > kVuPin) { needle = kVuPin; needleSpeed = std::min (needleSpeed, 0.0); }
    if (needle < 0.0)    { needle = 0.0;    needleSpeed = std::max (needleSpeed, 0.0); }
}

bool FastChannel::Impl::lineOk (const float* seg, int count, const Segment& sg) const noexcept
{
    return std::isfinite (sg.pre[count - 1]) && std::isfinite (ctlV) && std::isfinite (seg[count - 1]) && std::isfinite (needle);
}

/** The line amp and the meters over a segment; false when the loop left a non-finite state (not expected: an overload the solvers cannot
    follow would stay silent for good, so the model starts again from idle). */
bool FastChannel::Impl::linePass (float* seg, int count, Segment& sg) noexcept
{
    const bool absDrive = ctl.meter == MeterMode::plus4 || ctl.meter == MeterMode::plus8;
    for (int k = 0; k < count; ++k)
        lineSample (seg, k, sg, absDrive);
    needleStep (count, sg);
    return lineOk (seg, count, sg);
}

void FastChannel::Impl::recover (float* seg, int count) noexcept
{
    reset();
    for (int k = 0; k < count; ++k) seg[k] = 0.0f;
    countdown = kControlInterval - count;
}

void FastChannel::Impl::tick() noexcept
{
    if (countdown <= 0)
    {
        controlTick();
        countdown = kControlInterval;
    }
}

void FastChannel::publish() noexcept
{
    auto& m = *impl;
    grNow.store ((float) m.reductionDbAt (m.vgsControl), std::memory_order_relaxed);
    blockGrDb.store (std::max (0.0f, (float) m.reductionDbAt (m.vgsPeak)), std::memory_order_relaxed);
    needleDb.store ((float) std::clamp (20.0 * std::log10 (std::max (m.needle, 1e-6)), (double) Range::meterMinDb, (double) Range::meterMaxDb),
                    std::memory_order_relaxed);
}

void FastChannel::process (float* samples, int numSamples) noexcept
{
    auto& m = *impl;
    if (! m.prepared)
        return;
    m.vgsPeak = m.vgsControl;
    Impl::Segment sg;
    for (int pos = 0; pos < numSamples;)
    {
        m.tick();
        const int count = std::min (numSamples - pos, m.countdown);
        float* seg = samples + pos;
        m.inputPass (seg, count, sg);
        for (int k = 0; k < count; ++k)
        {
            sg.pre[k] = m.loopStep (sg.voc[k]);
            sg.ctl[k] = m.ctlV;
        }
        m.segmentEnd (count);
        if (! m.linePass (seg, count, sg))
        {
            m.recover (seg, count);
            pos += count;
            continue;
        }
        m.countdown -= count;
        pos += count;
    }
    if (numSamples > 0)
        publish();
}

void FastChannel::processPair (FastChannel& a, FastChannel& b, float* x, float* y, int numSamples) noexcept
{
    runPair (a, b, x, y, numSamples, false);
}

void FastChannel::processLinked (FastChannel& a, FastChannel& b, float* x, float* y, int numSamples) noexcept
{
    runPair (a, b, x, y, numSamples, true);
}

void FastChannel::runPair (FastChannel& a, FastChannel& b, float* x, float* y, int numSamples, bool link) noexcept
{
    auto& ma = *a.impl;
    auto& mb = *b.impl;
    if (! ma.prepared || ! mb.prepared || ma.countdown != mb.countdown)
    {
        Impl::setLinked (ma, mb, false);
        a.process (x, numSamples);
        b.process (y, numSamples);
        return;
    }
    Impl::setLinked (ma, mb, link);
    ma.vgsPeak = ma.vgsControl;
    mb.vgsPeak = mb.vgsControl;
    Impl::Segment sa, sb;
    for (int pos = 0; pos < numSamples;)
    {
        ma.tick();
        mb.tick();
        const int count = std::min (numSamples - pos, ma.countdown);
        float* sx = x + pos;
        float* sy = y + pos;
        ma.inputPass (sx, count, sa);
        mb.inputPass (sy, count, sb);
        if (link)
        {
            // one control node: the two chains are solved together
            const double kapS = Impl::sharedKappa (ma, mb);
            for (int k = 0; k < count; ++k)
            {
                Impl::StepCtx ta, tb;
                Impl::stepPrepLink (ma, mb, ta, tb, sa.voc[k], sb.voc[k], kapS);
                ma.stepCell (ta);
                mb.stepCell (tb);
                Impl::stepSideLink (ma, mb, ta, tb, kapS);
                Impl::stepFinishLink (ma, mb, ta, tb, kapS, sa.pre[k], sb.pre[k]);
                sa.ctl[k] = ma.ctlV + ma.ctlOffset;
                sb.ctl[k] = mb.ctlV + mb.ctlOffset;
            }
        }
        else
        {
            // the two chains are independent: stepping them together lets the processor overlap their latencies
            for (int k = 0; k < count; ++k)
            {
                Impl::StepCtx ta, tb;
                ma.stepPrep (ta, sa.voc[k]);
                mb.stepPrep (tb, sb.voc[k]);
                ma.stepCell (ta);
                mb.stepCell (tb);
                ma.stepSide (ta);
                mb.stepSide (tb);
                sa.pre[k] = ma.stepFinish (ta);
                sb.pre[k] = mb.stepFinish (tb);
                sa.ctl[k] = ma.ctlV;
                sb.ctl[k] = mb.ctlV;
            }
        }
        ma.segmentEnd (count);
        mb.segmentEnd (count);
        const bool absA = ma.ctl.meter == MeterMode::plus4 || ma.ctl.meter == MeterMode::plus8;
        const bool absB = mb.ctl.meter == MeterMode::plus4 || mb.ctl.meter == MeterMode::plus8;
        for (int k = 0; k < count; ++k)
        {
            ma.lineSample (sx, k, sa, absA);
            mb.lineSample (sy, k, sb, absB);
        }
        ma.needleStep (count, sa);
        mb.needleStep (count, sb);
        const bool okA = ma.lineOk (sx, count, sa);
        const bool okB = mb.lineOk (sy, count, sb);
        if (! okA) ma.recover (sx, count); else ma.countdown -= count;
        if (! okB) mb.recover (sy, count); else mb.countdown -= count;
        pos += count;
        // (the two stay in step unless one recovered, which is no longer in step with the other's control ticks)
        if (ma.countdown != mb.countdown)
        {
            Impl::setLinked (ma, mb, false);
            if (pos < numSamples) { a.process (x + pos, numSamples - pos); b.process (y + pos, numSamples - pos); }
            pos = numSamples;
        }
    }
    if (numSamples > 0)
    {
        a.publish();
        b.publish();
    }
}
} // namespace urei1176
