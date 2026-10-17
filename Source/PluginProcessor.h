#pragma once

#include "dsp/Urei1176.h"

#include <juce_audio_utils/juce_audio_utils.h>

#include <array>

/** Parameter IDs: stable API (sessions and automation refer to them). */
namespace ParamID
{
    inline constexpr auto input = "input", output = "output", attack = "attack", release = "release",
                          ratio = "ratio", meter = "meter", calibration = "calibration";
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

/** The Attack parameter, 0..7: below 1 is the knob's OFF click (limiting off). Its normalised
    range gives the click a short stretch of travel past 1 rather than a dead zone: 0 .. kAttackOffSpan
    is OFF and everything above it is the dial's 1 .. 7 sweep. */
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

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}
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
    /** For tests: a channel's controls() show what the last processBlock() set. */
    const urei1176::Channel& getChannel (int index) const noexcept { return channels[(size_t) index]; }

private:
    static juce::AudioProcessorValueTreeState::ParameterLayout createLayout();
    void pushControls (urei1176::Channel&) const noexcept;

    std::array<urei1176::Channel, kMaxChannels> channels;
    std::atomic<int> runningChannels { 0 };

    std::atomic<float>* const input;
    std::atomic<float>* const output;
    std::atomic<float>* const attack;
    std::atomic<float>* const release;
    std::atomic<float>* const ratio;
    std::atomic<float>* const meter;
    std::atomic<float>* const calibration;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Urei1176Processor)
};
