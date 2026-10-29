#pragma once

// The reduced-circuit engine for the 1176LN channel: the same netlist values as
// urei1176::Channel (Urei1176.cpp), derived block by block into small state-space systems and
// two scalar nonlinear solves (Q1 and the rectifier), instead of the full 19- and 12-unknown
// Newton groups. This is what the plug-in runs. See docs/research/urei-1176.md, "The fast engine".
//
// FastChannel has Channel's public API (same setters, process(), meter getters and Controls,
// same control smoothing); the circuit engine stays in the tree as the reference the tests
// (urei1176_test, urei1176_compare) check it against.

#include "Urei1176.h"
#include "Versions.h"

#include <atomic>
#include <memory>

// Fitting hooks: 0 in every shipped target. Only the fitting and diagnostic tools (tests/urei1176_fit.cpp, urei1176_opdump.cpp,
// urei1176_ablate.cpp, urei1176_compare_fit) compile the engine with UREI1176_FIT_HOOKS=1, in a library of their own. The hooks add
// setOverloadParams (the 14 constants the fit moves; prepare() then keeps them instead of loading the version's), shippedOverload and
// bakedConstants (a version's VersionSpec, F's by default, for opdump); the reference engine's ablation bits are in Urei1176Circuit.h.
#ifndef UREI1176_FIT_HOOKS
 #define UREI1176_FIT_HOOKS 0
#endif

namespace urei1176
{
#if UREI1176_FIT_HOOKS
/** The operating-point constants of a version's VersionSpec (amps; F's by default), and the GR amp's IF0 / IF1 as the hot functions form them. */
struct BakedConstants
{
    double ieQ2, ieQ3, ieQ4, ieQ12, ieQ13, ieQ14, ieQ15;
    double icQ5, icQ6, icQ7, icQ8, icQ9, idBias;
    double if0, if1;
};
#endif

/** One mono channel of the 1176LN, as urei1176::Channel, about 20 times cheaper (see the research notes).

    Thread-safety is Channel's: prepare() and reset() allocate nothing but take a fraction of a
    millisecond, and belong off the audio thread; the setters and process() are for the audio
    thread and neither allocate nor lock; the meter getters read relaxed atomics from any thread.
    Latency is zero; there is no oversampling. */
class FastChannel
{
public:
    FastChannel();
    ~FastChannel();
    FastChannel (const FastChannel&) = delete;
    FastChannel& operator= (const FastChannel&) = delete;

    /** Builds the blocks for this sample rate (44.1 to 192 kHz) as revision F and settles the idle state. The controls
        keep their values across calls. Never concurrently with process(). */
    void prepare (double sampleRate) { prepare (sampleRate, Version::F); }
    /** As above, for a version: the build functions read that version's VersionSpec (Versions.h); the sample loop does not know the
        version. Allocates nothing (the blocks are fixed-size members), so the processor's settle worker can call it on a standby engine. */
    void prepare (double sampleRate, Version version);
    Version version() const noexcept;

    /** Copies the slow gain-reduction memory of `from` into this (already prepared) engine, so that a freshly settled engine that is about
        to replace `from` starts from the same gain reduction instead of from 0 dB: the control voltage and the whole detector (C27, the
        attack and release networks, the threshold reference, the GR amp's coupling capacitors and its rectifier anodes, the S/C HPF's
        low-pass), the gain cell's whole state (not only its DC solution: see the .cpp), the gate-drive meter's averages, the overload followers, the VU needle
        and the published meters. Audio thread, no allocation, no lock. The two engines must have the same sample rate, ratio and meter
        mode (the processor pushes the same controls to both); the input network, the output coupling and the line amp keep the state prepare() left them (they ring in over the crossfade). */
    void adoptControlState (const FastChannel& from) noexcept;
    /** Clears all state to the idle operating point with the present control values. */
    void reset();

    //== Controls (audio thread), as Channel =====================================================
    void setInput (float rotation) noexcept;
    void setOutput (float rotation) noexcept;
    void setAttack (float dial) noexcept;
    void setRelease (float dial) noexcept;
    void setLimitOff (bool off) noexcept;
    void setRatio (Ratio ratio) noexcept;
    void setMeterMode (MeterMode mode) noexcept;
    void setCalibrationDbu (float dbu) noexcept;
    /** The version's level trims (VersionSpec::inputTrimDb, outputTrimDb: Custom, a plug-in choice) are on by default. false takes them
        off, so that the engine's level is the circuit's own: for the factory-procedure tests and the circuit-level fit. Not concurrently
        with process(); keeps its value across prepare(). */
    void setVersionTrims (bool on) noexcept;
    /** A high-pass on the detector only (a plug-in addition, not in the 1176): 0 = off, 1..5 = 30, 60, 105, 125, 185 Hz, first order. Takes
        effect over about 10 ms; off is bit-identical to a build without it. The reference Channel has no such control. */
    void setSidechainFilter (int choice) noexcept;

    /** In place: floats at the plug-in's input, floats at its output. */
    void process (float* samples, int numSamples) noexcept;

    /** Two channels in place, one block: the same result as a.process (x, n) and b.process (y, n), bit for bit, but the two independent
        solves are stepped together so that the processor can overlap their latencies. Use it for a stereo pair of dual-mono channels. */
    static void processPair (FastChannel& a, FastChannel& b, float* x, float* y, int numSamples) noexcept;

    /** As processPair, with the two units' control nodes tied, as UREI's 1176SA adapter ties two 1176s (see docs/research/urei-1176.md, "Stereo
        link"): both rectifiers charge one shared control node, both C27s and all the node's returns are in parallel, and one control voltage is
        solved for the pair inside every sample. Both units keep their own side chains' anodes, attack paths and gain cells. Zero latency, no
        allocation. The first call after processPair (or after a reset) engages the tie: the shared node starts at the mean of the two control
        voltages and each cell's own offset from it glides to zero over about 15 ms, so no gain steps; the next processPair call unties them in place. */
    static void processLinked (FastChannel& a, FastChannel& b, float* x, float* y, int numSamples) noexcept;

    //== Meters (any thread) =====================================================================
    float getMeterNeedleDb() const noexcept { return needleDb.load (std::memory_order_relaxed); }
    float getGainReductionDb() const noexcept { return blockGrDb.load (std::memory_order_relaxed); }

    //== Diagnostics =============================================================================
    float currentGainReductionDb() const noexcept { return grNow.load (std::memory_order_relaxed); }
    double voltsPerUnit() const noexcept;
    /** The control voltage the gain cell sees (C27's node), in volts, at full precision: the tests compare it between engines. */
    double controlVolts() const noexcept;

    using Controls = Channel::Controls;
    const Controls& controls() const noexcept;

    /** Test hook for UREI's factory test 10: moves R81 (the Q bias pot, 0..1) and recomputes the bias; call reset() after. */
    void setBiasPotDiag (double position) noexcept;

#if UREI1176_FIT_HOOKS
    /** Fitting tools only: the overload constants the engine is built with (before prepare(), or any time; the followers' rates are
        recomputed). The values the engine ships with are shippedOverload(). */
    void setOverloadParams (const OverloadParams& p) noexcept;
    static OverloadParams shippedOverload (Version v = Version::F) noexcept;
    static BakedConstants bakedConstants (Version v = Version::F) noexcept;
#endif

private:
    struct Impl;
    static void runPair (FastChannel& a, FastChannel& b, float* x, float* y, int numSamples, bool link) noexcept;
    void publish() noexcept;
    std::unique_ptr<Impl> impl;
    std::atomic<float> needleDb { -20.0f }, blockGrDb { 0.0f }, grNow { 0.0f };
};
} // namespace urei1176
