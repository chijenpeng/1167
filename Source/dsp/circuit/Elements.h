#pragma once
#include <string>

#include "LinearNetwork.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace ar::circuit
{
struct PortRef
{
    LinearNetwork* net = nullptr;
    int port = -1;
    int netIndex = -1;   // position of the network in the circuit, for ordering
};

/** One non-linear device: a tube, a diode pair, an op-amp, a transistor.

    The device has m unknown currents u. The current it injects into each of its ports is a
    linear combination C·u. Its model is evaluated on nV "model voltages", each a linear
    combination P of the port voltages plus internal series drops Rint·u (a grid stopper
    kept inside the device, say). */
class Device
{
public:
    virtual ~Device() = default;
    virtual int numPorts() const noexcept = 0;
    virtual int numUnknowns() const noexcept = 0;
    virtual int numModelVoltages() const noexcept = 0;
    /** Fills C (ports x m), P (nV x ports) and Rint (nV x m), all row-major and zeroed. */
    virtual void coefficients (Real* C, Real* P, Real* Rint) const noexcept = 0;
    /** Model currents u and the Jacobian dU/dV (m x nV, row-major, zeroed on entry). */
    virtual void evaluate (const Real* vModel, Real* uModel, Real* dUdV) const noexcept = 0;
    /** Newton step limiting (as SPICE does for junctions): given model voltage i at vOld and
        the proposed change, returns the change actually allowed (same sign, no larger). */
    virtual Real limitStep (int i, Real vOld, Real delta) const noexcept = 0;
    /** Junctions are guarded from where they carry this current. SPICE uses about 1 mA; a
        fuzz's clipping diodes and transistors work at microamps behind high impedances, and
        landing them at 1 mA leaves Newton to walk back down by n Vt per step. */
    static constexpr Real kJunctionGuardAmps = 1.0e-6;

    /** A junction that conducts above vOn with a knee of vKnee: free movement while off, a
        short step once it is on, and landing just inside conduction when switching on. */
    static Real limitJunction (Real vOld, Real delta, Real vOn, Real vKnee) noexcept
    {
        // SPICE's pnjlim: free movement below the critical voltage; above it the step is damped
        // logarithmically so a junction far past its knee comes back in one or two steps and a
        // junction switching on lands just inside conduction.
        const Real vCrit = vOn + 2.0 * vKnee, vNew = vOld + delta;
        if (vNew <= vCrit || std::abs (delta) <= 2.0 * vKnee)
            return delta;
        if (vOld > vCrit)
        {
            const Real arg = 1.0 + delta / vKnee;
            const Real limited = arg > 0.0 ? vOld + vKnee * std::log (arg) : vCrit;
            return limited - vOld;
        }
        return (vCrit + vKnee * std::log (1.0 + (vNew - vCrit) / vKnee)) - vOld;
    }
    static Real limitFlat (Real delta, Real lim) noexcept { return delta > lim ? lim : (delta < -lim ? -lim : delta); }
    /** A plate voltage: capped, and never collapsing past a quarter of its value in one step,
        so a tube is not thrown from its knee into cut-off (where the model has no slope). */
    static Real limitPlate (Real vOld, Real delta, Real lim) noexcept
    {
        delta = limitFlat (delta, lim);
        if (vOld > 0.0 && delta < -0.75 * vOld)
            delta = -0.75 * vOld;
        return delta;
    }

    std::vector<PortRef> ports;
};

/** Experiment: accept the first Newton correction after the predictor whenever it is a full
    step (not limited) that moves no model voltage by more than kSingleCorrectionMax
    (relative units of the convergence test), instead of iterating to the tolerance. */
#ifndef AR_SINGLE_CORRECTION
#define AR_SINGLE_CORRECTION 0
#endif
constexpr bool kSingleCorrection = AR_SINGLE_CORRECTION != 0;
#ifndef AR_SINGLE_CORRECTION_MAX
#define AR_SINGLE_CORRECTION_MAX 0.05
#endif
constexpr double kSingleCorrectionMax = AR_SINGLE_CORRECTION_MAX;

/** A set of devices solved together with Newton's method against the networks' affine port
    relations. Devices that interact strongly through one network (a long-tailed pair, an
    op-amp and its feedback diodes) must share a group; weakly coupled ones can be separate
    groups, in which case their interaction is one sample late. */
class DeviceGroup
{
public:
    explicit DeviceGroup (std::vector<std::unique_ptr<Device>> devices);

    /** One Newton solve for this sample. A resweep re-solves from the current solution with
        the other groups' updated currents and leaves the predictor history alone. */
    void solve (bool resweep = false) noexcept;
    bool singleNetwork() const noexcept { for (const auto& p : ports) if (p.net != ports[0].net) return false; return true; }
    /** Whether port p carries current (an op-amp output does, its inputs do not). */
    bool injects (int p) const noexcept { for (int j = 0; j < m; ++j) if (C[(size_t) p * (size_t) m + (size_t) j] != 0.0) return true; return false; }
    /** Hands the devices over (to be merged into another group); this group is then unusable. */
    std::vector<std::unique_ptr<Device>> takeDevices() noexcept { return std::move (devices); }
    int maxNetIndex() const noexcept;
    void resetState() noexcept
    {
        for (auto* v : { &u, &uPrev, &uPrev2, &vSolved })
            std::fill (v->begin(), v->end(), 0.0);
        predictorReady = false;
    }
    /** Call after changing a device's coefficients. */
    void rebuildCoefficients() noexcept;

    const std::vector<PortRef>& getPorts() const noexcept { return ports; }
    const std::vector<Real>& unknowns() const noexcept { return u; }
    Device& device (int index) noexcept { return *devices[(size_t) index]; }

    int maxIterations = 60;
    Real toleranceVolts = 1.0e-4;   // stop when the model voltages move less than this

    /** Diagnostics: Newton iterations and solves since the counters were last zeroed. */
    static inline unsigned long statIterations = 0, statSolves = 0, statFailures = 0;
    unsigned long failures = 0;   // this group's failed solves
    unsigned long newtonIterations = 0, newtonSolves = 0;   // this group's Newton iterations and solves
    std::string describe() const { return "group: devices=" + std::to_string (devices.size()) + " m=" + std::to_string (m) + " nV=" + std::to_string (nV) + " ports=" + std::to_string (ports.size()); }

private:
    void refreshCoupling() noexcept;
    /** v0[p]: each port's voltage with this group's currents at zero and every other device's
        current (already set this sample, or last sample's) left in. */
    void computeOpenVoltages() noexcept;
    /** One Newton run from the current u, the whole step scaled down when any model voltage
        would move too far. Slow but robust: the fallback. Returns true when it converged. */
    bool iterate (Real stepLimit, int iterations) noexcept;
    /** One Newton run from the current u with each model voltage limited on its own (SPICE's
        scheme). Returns true when it converged; vm then holds the model voltages of u. */
    bool iterateLimited (int iterations) noexcept;
    /** The starting point for this sample's Newton solve (urei1176 change, see docs): the last
        solution moved by the linear response of the devices to the change in their open-circuit
        model voltages, J^-1 G (vm0 - vm0Last), with the Jacobian factored and the conductances
        G of the last Newton iteration. Falls back to extrapolating the last three solutions
        when there is no usable Jacobian (first sample, after a refactor or a fallback solve). */
    void predict() noexcept;
    /** Largest relative step after which a full Newton step is taken as converged. */
    Real acceptLimit (bool firstStep) const noexcept
    {
        return kSingleCorrection && firstStep ? std::max (50.0 * toleranceVolts, kSingleCorrectionMax) : 50.0 * toleranceVolts;
    }
    unsigned cachedRefactorSum = ~0u;
    std::vector<std::unique_ptr<Device>> devices;
    std::vector<PortRef> ports;
    int m = 0, nV = 0;
    std::vector<Real> C, P, Rint, u, uPrev, uPrev2, uLast;
    std::vector<Real> v0, reff, M, vm0, vm, um, dUdV, F, J, du;
    std::vector<Real> vEval, vSolved;   // model voltages: where the devices were evaluated; the last solution
    // urei1176 changes (see docs, "Engine changes"):
    // - the Jacobian is factored in place in J (luPivot) by a kernel compiled for the group's
    //   size, and kept with the device conductances it was built from (gBlock, per device a
    //   dm x dv block at gOffset) for predict();
    // - P and C are applied as sparse lists (they hold a few +-1 per row).
    std::vector<int> luPivot, gOffset;
    std::vector<Real> gBlock, vm0Last, dvStep;   // dvStep: a Newton step's change of each model voltage
    std::vector<Real> Mt;   // M transposed (m x nV): M u as a sum of columns, in the same order
    std::vector<std::pair<Device*, int>> voltageOwner;   // device and local index per model voltage
    bool predictorReady = false;
    using LuFactor = void (*) (Real*, int*, int) noexcept;
    using LuSolve = void (*) (const Real*, const int*, Real*, int) noexcept;
    LuFactor luFactorFn = nullptr;
    LuSolve luSolveFn = nullptr;
    struct Term { int index; Real coefficient; };
    std::vector<int> pStart, cStart;    // per model voltage / per port: range in pTerms / cTerms
    std::vector<Term> pTerms, cTerms;   // P (model voltage from ports), C (port current from unknowns)
    std::vector<LinearNetwork*> networks;   // the distinct networks of the ports
    std::vector<int> devUnknowns, devVoltages;
    // Per port: its node, and the ports of the same network that belong to other devices
    // (otherStart[p]..otherStart[p+1] into otherPort / otherR), rebuilt in refreshCoupling().
    std::vector<int> portNode, otherStart, otherPort;
    std::vector<Real> otherR;
};

//==============================================================================
/** Two diodes between a and b: `forward` conducts a->b, `reverse` b->a. Either may be
    absent (saturation current 0). Unknown: the current a->b. */
struct DiodeSpec
{
    Real is = 2.5e-9, n = 1.8;        // 1N4148-ish
    Real rs = 0.0;                    // series (bulk) resistance, ohms
    /** Reverse current, amps, when it is not the forward fit's saturation current (negative:
        it is). A forward fit with n near 2 has a large is, nanoamps, which a low-leakage part
        does not leak; behind megohms (a compressor's release network) that matters. */
    Real leakage = -1.0;
    static DiodeSpec si1n4148() noexcept { return { 2.5e-9, 1.8 }; }
    /** FDH333 low-leakage diode (UREI's FD333): the 1N4148 forward fit (the datasheet's 0.68 V
        at 1 mA and 0.80-0.89 V at 50 mA bracket it) with 0.1 nA of leakage; the datasheet
        allows 3 nA at 125 V. */
    static DiodeSpec fd333() noexcept    { return { 2.5e-9, 1.8, 0.0, 1.0e-10 }; }
    static DiodeSpec si1n914() noexcept  { return { 2.5e-9, 1.8 }; }
    static DiodeSpec redLed() noexcept   { return { 1.0e-18, 2.0 }; }
    /** A generic low-drop germanium junction: 0.21 V at 1 mA, 0.26 V at 5 mA, no series
        resistance. Lower than a 1N34A; see ge1n34a(). */
    static DiodeSpec germanium() noexcept{ return { 2.0e-6, 1.3 }; }
    /** 1N34A / D9E class germanium point-contact diode, the widely used SPICE model
        (Is 0.2 uA, N 1.3, Rs 7 ohm): 0.21 V at 0.1 mA, 0.29 V at 1 mA, 0.375 V at 5 mA,
        0.43 V at 10 mA. Low knee, and above about 5 mA the series resistance sets the slope.
        Data: 1N34A at most 1 V at 5 mA; D9E sold as 0.27 to 0.30 V at 1 mA; Aion FX measured
        the Klon Centaur's own 1N34As at about 0.35 V at 5 mA. */
    static DiodeSpec ge1n34a() noexcept  { return { 2.0e-7, 1.3, 7.0 }; }
    static DiodeSpec none() noexcept     { return { 0.0, 1.0 }; }
    /** One direction of a back-to-back zener pair: conducts about 1 mA at vz + 0.65 V (the
        reverse breakdown of one zener plus the forward drop of the other). The emission
        coefficient keeps the exponent near limitedExp's linear limit at that current, which
        makes the knee softer than a real zener's; behind a series resistor it hardly shows. */
    static DiodeSpec zenerPair (Real vz) noexcept
    {
        const Real vc = vz + 0.65, nVt = vc / 38.0;
        return { 1.0e-3 * std::exp (-38.0), nVt / 0.025852 };
    }
};

/** Model voltage: the junction voltage, the port voltage less the series resistance's drop.
    The pair shares one series resistance, the larger of the two specs' (only one side
    conducts at a time, so for a matched pair this is exact up to the leakage). */
class DiodePairDevice final : public Device
{
public:
    DiodePairDevice (DiodeSpec forward, DiodeSpec reverse, PortRef a, PortRef b, Real scale = 1.0);
    int numPorts() const noexcept override { return 2; }
    int numUnknowns() const noexcept override { return 1; }
    int numModelVoltages() const noexcept override { return 1; }
    void coefficients (Real* C, Real* P, Real* Rint) const noexcept override;
    void evaluate (const Real* v, Real* u, Real* j) const noexcept override;
    Real limitStep (int, Real vOld, Real delta) const noexcept override
    {
        // Either direction may be a forward-biased diode; the guard voltages are worked out once,
        // in updateLimits() (urei1176 change: they were two logs per call, see docs).
        if (delta > 0.0) return freeF ? delta : limitJunction (vOld, delta, vOnF - 2.0 * kneeF, kneeF);
        return freeR ? delta : -limitJunction (-vOld, -delta, vOnR - 2.0 * kneeR, kneeR);
    }
    /** Multiplies both saturation currents: a blend between diode sets. */
    void setScale (Real s) noexcept { scale = s; updateLimits(); }
private:
    void updateLimits() noexcept
    {
        const Real vt = 0.026;
        const Real nF = fwd.is > 0.0 ? fwd.n : rev.n, nR = rev.is > 0.0 ? rev.n : fwd.n;
        vOnF = nF * vt * std::log (kJunctionGuardAmps / std::max (fwd.is * scale, 1.0e-30));
        vOnR = nR * vt * std::log (kJunctionGuardAmps / std::max (rev.is * scale, 1.0e-30));
        kneeF = nF * vt;
        kneeR = nR * vt;
        // A direction whose diode never carries the guard current before the exponential turns
        // linear (a blocked direction, saturation current ~0) has no junction to guard: guarded,
        // a clamp diode reverse-biased by hundreds of volts crawled after its plate.
        constexpr Real eKnee = 2.35e17;   // limitedExp's exp (40)
        freeF = fwd.is * scale * eKnee < kJunctionGuardAmps;
        freeR = rev.is * scale * eKnee < kJunctionGuardAmps;
    }
    DiodeSpec fwd, rev;
    Real scale;
    Real vOnF = 0.0, vOnR = 0.0, kneeF = 0.0, kneeR = 0.0;
    bool freeF = false, freeR = false;
};

/** Square-law JFET (Shichman-Hodges) with smooth transitions, both polarities of vds. */
struct JfetSpec
{
    Real beta = 1.0e-3, vp = -0.7, lambda = 0.01;
    static JfetSpec j201() noexcept  { return { 1.0e-3, -0.7, 0.01 }; }
    static JfetSpec n2n5457() noexcept { return { 0.6e-3, -1.5, 0.01 }; }
};

class JfetDevice final : public Device
{
public:
    JfetDevice (JfetSpec spec, PortRef gate, PortRef drain, PortRef source);
    int numPorts() const noexcept override { return 3; }
    int numUnknowns() const noexcept override { return 1; }
    int numModelVoltages() const noexcept override { return 2; }
    void coefficients (Real* C, Real* P, Real* Rint) const noexcept override;
    void evaluate (const Real* v, Real* u, Real* j) const noexcept override;
    Real limitStep (int i, Real, Real delta) const noexcept override { return limitFlat (delta, i == 0 ? 0.5 : 2.0); }
private:
    JfetSpec spec;
};

struct BjtSpec
{
    Real is = 6.7e-15, betaF = 200.0, betaR = 4.0;
    /** A PNP: the same model on the emitter-base and collector-base voltages, currents
        reversed. */
    bool pnp = false;
    static BjtSpec n2n3904() noexcept { return { 6.7e-15, 200.0, 4.0 }; }
    static BjtSpec bc550c() noexcept  { return { 7.0e-14, 500.0, 5.0 }; }
};

/** A bipolar transistor (Ebers-Moll), NPN or PNP (BjtSpec::pnp): ports base, collector,
    emitter; unknowns Ic, Ib (positive when it conducts, either polarity). */
class BjtDevice final : public Device
{
public:
    BjtDevice (BjtSpec spec, PortRef base, PortRef collector, PortRef emitter);
    int numPorts() const noexcept override { return 3; }
    int numUnknowns() const noexcept override { return 2; }
    int numModelVoltages() const noexcept override { return 2; }
    void coefficients (Real* C, Real* P, Real* Rint) const noexcept override;
    void evaluate (const Real* v, Real* u, Real* j) const noexcept override;
    Real limitStep (int, Real vOld, Real delta) const noexcept override
    {
        const Real vt = 0.026;
        if (delta > 0.0) return limitJunction (vOld, delta, vOnGuard - 2.0 * vt, vt);
        return delta;   // moving towards cut-off is safe
    }
private:
    BjtSpec spec;
    Real vOnGuard;   // vt log (kJunctionGuardAmps / is), once (urei1176 change: was a log per call)
};
} // namespace ar::circuit
