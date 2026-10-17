#pragma once

// The 1176LN netlist behind urei1176::Channel. Internal: the plug-in uses Channel; the tests use
// this for operating-point and gain checks.

#include "CircuitModule.h"
#include "Urei1176.h"

#include <algorithm>
#include <string>

namespace urei1176
{
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
        secondary to C15), to compare with the voltages marked on that drawing. Before prepare(). */
    void useRevisionKValues (bool k) noexcept { revK = k; }
    /** The gain cell's reduction in dB for a gate-source voltage, against the calibrated idle
        point, with the present Input setting's source resistance. */
    Real reductionDbAt (Real vgs) const noexcept;
    /** A node's voltage by network and node name (diagnostics; not for the audio thread). */
    Real nodeVolts (const std::string& network, const std::string& node);

    /** Idle gate voltage from the factory calibration and the R81 position that gives it. */
    Real idleGateVolts() const noexcept { return gateAtIdle; }
    Real biasPotPosition() const noexcept { return biasPos; }

    // Model constants (see Urei1176.cpp)
    static const TransformerSpec& inputTransformer();
    static const TransformerSpec& outputTransformer();
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
    int buildInput (ar::circuit::LinearNetwork& n);
    int buildOutput (ar::circuit::LinearNetwork& l, int lineOut);
    void applyInput (Real position);
    void applyRatio (int sel);
    void applyLimitOff (bool off);
    static Real biasPosition (Real volts);
    Real idleGate() const;
    /** What Q1's channel works against at an Input position: R6 plus the source resistance
        T1's secondary presents, times the share of the drain swing across the channel. */
    static Real gainCellOhms (Real inputPosition);

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
    Real seriesOhms = 0.0;                   // what Q1's channel works against (times the share)
    int limitSwitches[2] { -1, -1 };
    Real gateAtIdle = 0.0, idleConductanceMeter = 0.0, biasPos = 0.0;
};
} // namespace urei1176
