#pragma once

// The 1176LN netlist behind urei1176::Channel. Internal: the plug-in uses Channel; the tests use
// this for operating-point and gain checks.

#include "CircuitModule.h"
#include "Urei1176.h"

#include <algorithm>
#include <string>

#ifndef UREI1176_FIT_HOOKS
 #define UREI1176_FIT_HOOKS 0
#endif

namespace urei1176
{
#if UREI1176_FIT_HOOKS
/** Ablation (tests only, hooked build): each bit puts one of rev G's (or J's) differences from F back to F's value in the reference
    netlist, to find which one produces a measured effect (Phase 3b-2, the hot-input H3). Read by Circuit1176::build(). */
enum Ablation : unsigned { kAblateR84 = 1, kAblateT2 = 2, kAblateSupply = 4, kAblateC10 = 8, kAblatePreamp = 16, kAblateIc2 = 32,
                        kAblateBias = 64, kAblateR32 = 128, kAblateR33 = 256, kAblateR34 = 512, kAblateGate = 1024 };
extern unsigned gAblation;
#endif

/** Typical parameters of a transformer modelled as two coupled windings. */
struct TransformerSpec
{
    Real turnsRatio = 1.0;          // secondary turns / primary turns
    Real primaryHenries = 10.0;     // primary inductance (secondary open)
    Real coupling = 0.9995;         // k; the leakage inductance is about 2 (1 - k) Lp
    Real primaryOhms = 50.0, secondaryOhms = 50.0;   // winding (DC) resistances
    Real primaryFarads = 0.0, secondaryFarads = 0.0, interwindingFarads = 0.0;
    Real coreOhms = 0.0;            // core loss across the primary (0: none)
};

class Circuit1176 final : public CircuitModule
{
public:
    // Control indices (see addParam calls in build()).
    enum ParamIndex { pInput, pOutput, pAttack, pRelease, pRatio, pLimitOff, pCount };

    void setControl (ParamIndex p, float value) noexcept { setTarget ((int) p, value); }
    void tick() noexcept { controlTick(); }

    /** One sample: EMF of the 600 ohm source in volts, volts across the output load. */
    Real processSample (Real emfVolts) noexcept { return circ.process (emfVolts); }

    /** Q1's gate-source voltage now (includes the audio the gate carries). */
    Real gateSourceVolts() const noexcept;
    /** Q1's gate-source voltage without the audio, updated by trackControl() every sample: the
        gate is exactly the mean of the control node and C5's node (R7 || C3 and R9 || C4 are
        identical arms) and C5's node carries the feedback node's audio, so the control part is
        vg - (v_fb - <v_fb>) / 2 - <v_s>, <> the DC levels (50 ms averages, which follow the
        shifts a heavy overload causes). Nothing of the control voltage itself is smoothed. */
    Real controlGateSourceVolts() const noexcept { return vgsControl; }
    void trackControl() noexcept
    {
        const Real fb = main->nodeVoltage (feedbackNode), src = main->nodeVoltage (sourceNode);
        feedbackAverage += averageCoeff * (fb - feedbackAverage);
        sourceAverage += averageCoeff * (src - sourceAverage);
        vgsControl = main->nodeVoltage (gateNode) - 0.5 * (fb - feedbackAverage) - sourceAverage;
    }
    /** The control node (C27), what the meter's matched FET Q10 sees through R48. */
    Real controlVolts() const noexcept;
    /** The GR meter's needle drive (1 = 0 VU): Q10, a source follower on the control node
        through R48 3.9M, into IC1 and CR11. Linear in the control voltage, zeroed at idle with
        limiting off (R55) and scaled to read -10 VU at 10 dB of steady-state reduction (R54),
        as UREI's test 11 calibrates it; CR11 passes no reverse current. */
    Real meterDeflection() const noexcept
    {
        return std::max (0.0, 1.0 - meterSlope * (controlVolts() - controlIdle));
    }
    /** Steady-state control voltage for `dB` of reduction (gate DC = control node). */
    Real controlForReduction (Real dB) const noexcept;
    /** Diagnostics: moves R81 (factory test 10). */
    void setBiasPot (Real position) noexcept;
    /** Diagnostics: build with the rev K drawing's values where they differ from rev F (from T1's
        secondary to C15), to compare with the voltages marked on that drawing. Before prepare().
        Applies to the rev F netlist only (a diagnostic flag, not a Version: Versions.h keeps its four
        versions and the plug-in's parameter its three choices). */
    void useRevisionKValues (bool k) noexcept { revK = k; }
    /** The revision to build; takes effect at the next prepare(). F is the netlist the plug-in has always had; G has the bridging
        IC2 5532 input and its own line-amp, T2 and supply values; J is G's input with its own C32, line-amp values and T2. D builds as F until it has
        a netlist. A change of version clears setR16(), setR33() and setR20(). */
    void setVersion (Version v) noexcept { if (v != version) r16Override = r33Override = r20Override = 0.0; version = v; }
    Version getVersion() const noexcept { return version; }
    /** Rev G and J have the 5532 receiver and the 10 k Input pot instead of the bridged-T and T1. */
    static constexpr bool hasBridgingInput (Version v) noexcept { return v == Version::G || v == Version::J; }
    /** Diagnostics: R16, the distortion trimmer, in ohms (before prepare(); cleared when the version changes, so call setVersion() first). UREI sets it per unit for
        minimum THD (manual 5.3.5). */
    void setR16 (Real ohms) noexcept { r16Override = ohms; }
    /** The R16 setting a revision is built with unless setR16() was called. */
    static Real factoryR16 (Version v) noexcept;
    /** Diagnostics: R33 (Q5's emitter to C13) and R20 (Q3's emitter leg), in ohms (before prepare(); cleared when the version changes, so call
        setVersion() first). The sheet titled H has R33 as 11 k, 1.1 k or 10 k and R20 as 1.3 k or 1.8 k (schematic-revisions.md 3.1); the
        ambiguity sweep moves them. */
    void setR33 (Real ohms) noexcept { r33Override = ohms; }
    void setR20 (Real ohms) noexcept { r20Override = ohms; }
    /** G's and J's Input pot (R5): the track fraction between the wiper and ground at a knob rotation. Approximation: the sheets give no
        taper, so the divider ratio follows the printed dial's loss (inputLossDb) and the Input dial reads as on F. */
    static Real inputPotFraction (Real rotation) noexcept;
    /** The gain cell's reduction in dB for a gate-source voltage, against the calibrated idle
        point, with the present Input setting's source resistance. */
    Real reductionDbAt (Real vgs) const noexcept;
    /** A node's voltage by network and node name (diagnostics; not for the audio thread). */
    Real nodeVolts (const std::string& network, const std::string& node);

    /** The R81 position that gives the idle gate voltage of the factory calibration. */
    Real biasPotPosition() const noexcept { return biasPos; }

    // Model constants (see Urei1176.cpp)
    static const TransformerSpec& inputTransformer();
    static const TransformerSpec& outputTransformer();
    /** T2 as wired in a revision: F's B11148 with both primaries paralleled; G's 16-11148 with one primary (twice the
        primary winding resistance); J's 163-12614 with one primary and one secondary (both doubled). D gives F's. */
    static TransformerSpec outputTransformer (Version v);
    static constexpr Real kLoadOhms = 10e3;        // bridging load across T2's secondary
    static constexpr Real kSourceOhms = 600.0;     // the plug-in's input: a 600 ohm source

    /** The Input attenuator's loss (dB) at a knob rotation (0 fully counter-clockwise, 1 fully
        clockwise), and its two decks' resistances there. */
    static Real inputLossDb (Real rotation) noexcept;
    static void bridgedTArms (Real rotation, Real& bridge, Real& shunt) noexcept;
    /** The public control values (knob rotation 0..1, dial 1..7) as the pot positions the
        circuit uses (0..1; 1 = fully clockwise, fastest for Attack and Release). */
    static Real inputPosition (float dial) noexcept;
    static Real outputPosition (float dial) noexcept;
    static Real attackPosition (float dial) noexcept;
    static Real releasePosition (float dial) noexcept;

    /** Rereads the meter zero (R55) from the present operating point: call at idle, 4:1-20:1. */
    void zeroMeter() noexcept;
    /** Restarts trackControl()'s DC averages from the present (operating) point. */
    void startTracking() noexcept;

private:
    void build (double sampleRate) override;
    using DeviceList = std::vector<std::unique_ptr<ar::circuit::Device>>;
    int buildInput (ar::circuit::LinearNetwork& n);
    /** Rev G's and J's receiver (IC2, C30-C33, R1-R5): returns the node R6 hangs on; the op-amp output clamps go to `loop`. */
    int buildInputBridging (ar::circuit::LinearNetwork& n, DeviceList& loop, Real supply, Real c32Farads);
    bool bridgingInput() const noexcept { return hasBridgingInput (version); }
    int buildOutput (ar::circuit::LinearNetwork& l, int lineOut);
    void applyInput (Real position);
    void applyRatio (int sel);
    void applyLimitOff (bool off);
    static Real biasPosition (Real volts);
    Real idleGate() const;
    /** What Q1's channel works against at an Input position: R6 plus the source resistance
        T1's secondary (rev G and J: R5's wiper) presents, times the share of the drain swing across the channel. */
    Real gainCellOhms (Real inputPosition) const;

    ar::circuit::LinearNetwork* main = nullptr;
    const ar::circuit::JfetDevice* gainFet = nullptr;
    int gateNode = 0, sourceNode = 0, feedbackNode = 0, controlNode = 0;
    Real feedbackAverage = 0.0, sourceAverage = 0.0, vgsControl = 0.0, averageCoeff = 0.0;
    double fs = 0.0;
    Real controlIdle = 0.0, sourceIdle = 0.0, meterSlope = 0.0, calibrationOhms = 0.0;
    int biasPot = -1;
    bool revK = false;
    int ratioSwitches[8] {};
    static constexpr int kShuntSections = 4;
    int inputBridge = -1, inputShunt[kShuntSections] {};   // the attenuator's two decks
    int inputPot = -1;                                     // rev G and J: R5
    Version version = Version::F;
    Real r16Override = 0.0, r33Override = 0.0, r20Override = 0.0;
    Real seriesOhms = 0.0;                   // what Q1's channel works against (times the share)
    int limitSwitches[2] { -1, -1 };
    Real gateAtIdle = 0.0, idleConductanceMeter = 0.0, biasPos = 0.0;
};
} // namespace urei1176
