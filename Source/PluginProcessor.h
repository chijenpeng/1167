#pragma once

#include "dsp/Fast1176.h"
#include "dsp/StereoModes.h"
#include "dsp/VersionSwitch.h"
#include "dsp/Urei1176.h"

#include <juce_audio_utils/juce_audio_utils.h>

#include <array>
#include <iterator>

/** Parameter IDs: stable API (sessions and automation refer to them). */
namespace ParamID
{
    inline constexpr auto input = "input", output = "output", attack = "attack", release = "release",
                          ratio = "ratio", meter = "meter", calibration = "calibration",
                          stereo = "stereo", attackOff = "attackOff",
                          sidechainFilter = "sidechainFilter",   // appended after the first seven: never reorder
                          model = "model";                       // the 1176 revision (Version): appended last, choices F G J; D is deferred and would be appended
}

/** The plug-in's defaults for Attack and Release (owner choice, 2026-10-29): Attack at its slowest (dial 1) and Release at its fastest (dial 7).
    The engine's own Range::attackDefault / releaseDefault (4) are untouched: the goldens and the circuit fits are built on them. */
namespace Default
{
    inline constexpr float attack = urei1176::Range::attackMin, release = urei1176::Range::releaseMax;
}

/** The Version parameter's choices, index to engine version. D is not exposed (deferred, docs/research/rev-d-data-search.md); if it is ever built it is
    appended here ({ F, G, J, D }) so the stored indices of saved sessions never move. */
inline constexpr urei1176::Version kModelChoices[] = { urei1176::Version::F, urei1176::Version::G, urei1176::Version::J };
inline constexpr int kNumModelChoices = (int) std::size (kModelChoices);
static_assert (kModelChoices[0] == urei1176::Version::F, "choice 0 is F: the parameter's default, and what a session without the parameter loads as");

/** The engine version of a choice index (clamped to the choices). */
inline urei1176::Version versionForChoice (int index) noexcept { return kModelChoices[juce::jlimit (0, kNumModelChoices - 1, index)]; }

/** The choice index of an engine version; D, not exposed, gives F's (0). */
inline constexpr int choiceForVersion (urei1176::Version v) noexcept
{
    for (int i = 0; i < kNumModelChoices; ++i)
        if (kModelChoices[i] == v)
            return i;
    return 0;
}

/** The Version parameter's choice names ("F", "G", "J"), in kModelChoices' order: the parameter's choices and the editor's REV row. */
inline juce::StringArray modelChoiceNames()
{
    juce::StringArray names;
    for (const auto v : kModelChoices)
        names.add (juce::String::charToString (urei1176::specFor (v).name));
    return names;
}

/** How the parameters map to the hardware's controls and print their values. Shared by the
    processor, the editor and the snapshot tool's checks. */
namespace ParamText
{
    /** Input and Output: the printed dial ("∞", "48" ... "0"), interpolated with one decimal
        between the labels ("21.0"). */
    juce::String dial (float rotation);
    /** Typed dial text ("24", "21.5", "inf", "∞") to a rotation; nothing if it means nothing. */
    std::optional<float> rotationForDialText (const juce::String&);

    /** Attack: "OFF" at the click, else "4.0". */
    juce::String attack (float value);
}

/** The Attack parameter, 0..7: below 1 is an OFF span (limiting off) that the earliest versions of this plug-in reached by turning the knob
    past 1; the editor's knob no longer enters it and the Attack Off switch (ParamID::attackOff) is the control now, but the span stays so
    that old sessions and automation, and a host's generic editor, keep working. Its normalised range gives the span a short stretch of
    travel past 1 rather than a dead zone: 0 .. kOffSpan is OFF and everything above it is the dial's 1 .. 7 sweep. */
struct AttackLaw
{
    static constexpr float kOffSpan = 0.06f;
    static float fromNormalised (float n) noexcept;
    static float toNormalised (float value) noexcept;
    static float snap (float value) noexcept;
    static bool isOff (float value) noexcept { return value < 1.0f; }
};

class Urei1176Processor final : public juce::AudioProcessor
{
public:
    static constexpr int kMaxChannels = 2;

    /** Both channels' meters, read on the editor's timer. */
    struct Meters
    {
        std::array<float, kMaxChannels> needleDb {}, gainReductionDb {};
        int channels = 0;   // how many of them are running
    };

    Urei1176Processor();
    ~Urei1176Processor() override;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;
    using AudioProcessor::processBlock;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }

    const juce::String getName() const override { return JucePlugin_Name; }
    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    double getTailLengthSeconds() const override { return 0.0; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram (int) override {}
    const juce::String getProgramName (int) override { return {}; }
    void changeProgramName (int, const juce::String&) override {}

    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    juce::AudioProcessorValueTreeState apvts;

    /** Any thread: the latest needle and gain reduction of every running channel. */
    Meters getMeters() const noexcept;
    /** For tests: a channel's controls() show what the last processBlock() set. (The active engine set's: during a version fade, the outgoing's.) */
    const urei1176::FastChannel& getChannel (int index) const noexcept { return engines.channel (index); }
    /** The version playing as the main set, and the one the engine is heading for (they differ during the 20 ms fade of a switch). */
    urei1176::Version getActiveVersion() const noexcept { return engines.activeVersion(); }
    urei1176::Version getTargetVersion() const noexcept { return engines.targetVersion(); }
    bool isVersionFading() const noexcept { return engines.isFading(); }

private:
    static juce::AudioProcessorValueTreeState::ParameterLayout createLayout();
    void pushControls (urei1176::FastChannel&) const noexcept;
    urei1176::StereoMode currentStereoMode() const noexcept;
    urei1176::Version currentVersion() const noexcept;
    void stopSettleWorker();

    /** Polls VersionSwitch::pollSettle every 2 ms. The audio thread never signals it. */
    class SettleWorker;

    urei1176::VersionSwitch engines;   // two engine sets of two channels each, the settle step and the crossfade (dsp/VersionSwitch.h)
    std::unique_ptr<SettleWorker> settleWorker;
    std::atomic<int> runningChannels { 0 };

    std::atomic<float>* const input;
    std::atomic<float>* const output;
    std::atomic<float>* const attack;
    std::atomic<float>* const release;
    std::atomic<float>* const ratio;
    std::atomic<float>* const meter;
    std::atomic<float>* const calibration;
    std::atomic<float>* const stereo;
    std::atomic<float>* const attackOff;
    std::atomic<float>* const sidechainFilter;
    std::atomic<float>* const model;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Urei1176Processor)
};
