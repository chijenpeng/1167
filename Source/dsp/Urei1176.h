#pragma once

// UREI 1176LN limiting amplifier, one channel, modelled component for component from UREI's
// drawing for serial suffix F (2611-7052): the bridged-T Input attenuator and input transformer
// T1 (UTC O-12), the FET gain cell, preamp, side chain with the Attack knob's OFF switch,
// discrete class AB line amp and output transformer T2 (B11148, 1:1.73) into a 10 kohm bridging
// load. See docs/reference/schematic.md and docs/research/urei-1176.md.

#include <atomic>
#include <memory>

namespace urei1176
{
/** The ratio buttons. `all` is every button pressed at once; `none` is no button pressed (both
    switch sections open: the GR amp has no input and the unit does not limit). */
enum class Ratio { r4 = 0, r8, r12, r20, all, none };

/** The meter switch. It never changes the audio. */
enum class MeterMode { off = 0, gainReduction, plus4, plus8 };

/** Control ranges, in the hardware's own terms (see the setters). */
struct Range
{
    /** Input and Output: knob rotation, 0 fully counter-clockwise ("inf") to 1 fully clockwise
        ("0"); 0.5 is "24", the 12 o'clock unity-gain setting. See dialMarking(). */
    static constexpr float inputMin = 0.0f, inputMax = 1.0f, inputDefault = 0.5f;
    static constexpr float outputMin = 0.0f, outputMax = 1.0f, outputDefault = 0.5f;
    static constexpr float attackMin = 1.0f, attackMax = 7.0f, attackDefault = 4.0f;
    static constexpr float releaseMin = 1.0f, releaseMax = 7.0f, releaseDefault = 4.0f;
    static constexpr float calibrationMin = 10.0f, calibrationMax = 26.0f, calibrationDefault = 18.0f;
    static constexpr float meterMinDb = -20.0f, meterMaxDb = 3.0f;
};

/** The Input and Output knobs' printed dial (UA's reissue panel art; the original UREI art is
    not verified): nine labels 40 degrees apart over a 320 degree rotation. */
struct DialLabel { float rotation; const char* text; };
constexpr DialLabel kDialLabels[] {
    { 0.0f / 8.0f, "\xE2\x88\x9E" }, { 1.0f / 8.0f, "48" }, { 2.0f / 8.0f, "36" }, { 3.0f / 8.0f, "30" }, { 4.0f / 8.0f, "24" },
    { 5.0f / 8.0f, "18" }, { 6.0f / 8.0f, "12" }, { 7.0f / 8.0f, "6" }, { 8.0f / 8.0f, "0" } };
/** The printed scale value at a rotation, interpolated between labels (infinity at 0). */
float dialMarking (float rotation) noexcept;
/** The rotation that points at a printed scale value (0..48; larger values give 0, "inf"). */
float rotationForMarking (float marking) noexcept;

class Circuit1176;

/** One mono channel of the 1176LN. A stereo plug-in instantiates two.

    Thread-safety: prepare() builds the circuit (allocates; call it from the host's prepare,
    never concurrently with process()). reset() settles the operating point again; it is not
    real-time safe either (the engine's operating-point search allocates a little and takes a
    few milliseconds), so call it where the host allows that, never concurrently with process().
    Every setter and process() are called from the audio thread: the setters only store a
    target, which process() moves towards at a 32-sample control interval (pots by at most 2 %
    of their rotation per step, switches at once), refactoring the circuit there; neither the
    setters nor process() allocate or lock. The meter getters read relaxed atomics and may be
    called from any thread (a UI timer). */
class Channel
{
public:
    Channel();
    ~Channel();
    Channel (const Channel&) = delete;
    Channel& operator= (const Channel&) = delete;

    /** Builds the circuit for this sample rate (44.1 to 192 kHz) and settles its operating
        point. The controls keep their values across calls. */
    void prepare (double sampleRate);
    /** Clears all state (capacitor charges, the gain reduction, the meter) to the idle
        operating point with the present control values. */
    void reset();

    //== Controls (audio thread) ================================================================
    /** The Input knob (the bridged-T attenuator R1A / R1B ahead of T1): rotation 0..1, fully
        clockwise = least loss (0.15 dB), "24" (0.5) = 24 dB, fully counter-clockwise 54.5 dB. */
    void setInput (float rotation) noexcept;
    /** The Output knob (R27, 100k audio taper, ahead of the line amp): rotation 0..1. */
    void setOutput (float rotation) noexcept;
    /** Attack R77 (25k linear) dial 1..7; 7 (fully clockwise) is the fastest. */
    void setAttack (float dial) noexcept;
    /** Release R78 (5M linear) dial 1..7; 7 (fully clockwise) is the fastest. */
    void setRelease (float dial) noexcept;
    /** The Attack knob's switch at fully counter-clockwise ("OFF" click): disables limiting in
        the circuit (see docs). The Attack pot keeps its dial value. */
    void setLimitOff (bool off) noexcept;
    void setRatio (Ratio ratio) noexcept;
    void setMeterMode (MeterMode mode) noexcept;
    /** The dBu that 0 dBFS stands for (10..26 dBu), at the input and at the output. */
    void setCalibrationDbu (float dbu) noexcept;

    /** In place: floats at the plug-in's input, floats at its output. */
    void process (float* samples, int numSamples) noexcept;

    //== Meters (any thread) =====================================================================
    /** The meter needle on the VU scale in dB, -20 .. +3, for the selected meter mode, with VU
        ballistics (99 % in 300 ms, 1.25 % overshoot, the same falling). GR: the meter amp's
        reading of the control voltage, zeroed at idle (R55) and reading -10 at 10 dB of settled
        reduction (R54), so it rests at 0 and falls to the left (it over-reads fast transients
        and reads above 0 at idle with all buttons in, as the circuit does). +4 / +8: the output
        level across the load in VU (0 VU = +4 dBu, or +8.16 dBu by R50's value). Off: the needle
        falls to rest (-20). */
    float getMeterNeedleDb() const noexcept { return needleDb.load (std::memory_order_relaxed); }
    /** The largest gain reduction (dB, >= 0) during the last process() call, unballistic, from
        the gain cell's own state (Q1's gate without its audio), not the meter's law: for a bar
        meter. Zero at the calibrated idle bias. */
    float getGainReductionDb() const noexcept { return blockGrDb.load (std::memory_order_relaxed); }

    //== Diagnostics (tests, not for the plug-in's audio path) ===================================
    /** The circuit itself, for operating-point and gain checks. */
    Circuit1176& circuitModel() noexcept { return *model; }
    /** The present gain reduction (dB) from the gain cell's state, before the meter. */
    float currentGainReductionDb() const noexcept { return grNow.load (std::memory_order_relaxed); }
    /** Volts at the 600 ohm input that 1.0 (0 dBFS peak) stands for, and at the output load. */
    double voltsPerUnit() const noexcept { return unitVolts; }

    /** The present control values (as last set). */
    struct Controls
    {
        float input = Range::inputDefault, output = Range::outputDefault;
        float attack = Range::attackDefault, release = Range::releaseDefault;
        Ratio ratio = Ratio::r4;
        bool limitOff = false;
        MeterMode meter = MeterMode::gainReduction;
        float calibrationDbu = Range::calibrationDefault;
    };
    const Controls& controls() const noexcept { return ctl; }

private:
    void pushControls() noexcept;
    void controlTick() noexcept;
    void updateScale() noexcept;

    std::unique_ptr<Circuit1176> model;
    Controls ctl;
    float calCurrent = Range::calibrationDefault;
    double fs = 0.0;
    double unitVolts = 1.0;             // peak volts that 1.0 stands for
    double inGain = 1.0, outGain = 1.0; // float -> source EMF, load volts -> float
    double vuReference = 1.0;           // mean |v| at 0 VU for the selected scale
    int countdown = 0;
    double needle = 0.0, needleSpeed = 0.0;   // movement deflection (1 = 0 VU) and its rate
    std::atomic<float> needleDb { -20.0f }, blockGrDb { 0.0f }, grNow { 0.0f };
};
} // namespace urei1176
