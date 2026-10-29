#include "PluginProcessor.h"
#include "PluginEditor.h"

using urei1176::Range;

//==============================================================================
juce::String ParamText::dial (float rotation)
{
    const float marking = urei1176::dialMarking (rotation);
    if (std::isinf (marking))
        return juce::String::fromUTF8 (urei1176::kDialLabels[0].text);

    // A printed label shows as printed ("24"); anything between them with one decimal ("21.0").
    const juce::String text (marking, 1);
    for (const auto& label : urei1176::kDialLabels)
        if (text == juce::String (label.text) + ".0")
            return label.text;
    return text;
}

std::optional<float> ParamText::rotationForDialText (const juce::String& text)
{
    const auto t = text.trim().toLowerCase();
    if (t.startsWith ("inf") || t.startsWith (juce::String::fromUTF8 (urei1176::kDialLabels[0].text)))
        return 0.0f;
    if (! t.containsAnyOf ("0123456789"))
        return std::nullopt;

    // dialMarking() falls monotonically from just past infinity (rotation 0) to 0 (rotation 1):
    // invert it by bisection so that typed text and displayed text agree everywhere.
    const float marking = t.retainCharacters ("0123456789.").getFloatValue();
    float lo = 0.0f, hi = 1.0f;
    for (int i = 0; i < 40; ++i)
    {
        const float mid = 0.5f * (lo + hi);
        (urei1176::dialMarking (mid) > marking ? lo : hi) = mid;
    }
    const float rotation = hi;
    for (const auto& label : urei1176::kDialLabels)
        if (std::abs (rotation - label.rotation) < 1.0e-4f)
            return label.rotation;
    return rotation < 1.0e-4f ? 0.0f : rotation;
}

juce::String ParamText::attack (float value)
{
    return AttackLaw::isOff (value) ? juce::String ("OFF") : juce::String (value, 1);
}

namespace
{
    std::optional<float> attackForText (const juce::String& text)
    {
        const auto t = text.trim().toLowerCase();
        if (t.startsWith ("off"))
            return 0.0f;
        if (! t.containsAnyOf ("0123456789"))
            return std::nullopt;
        return AttackLaw::snap (t.getFloatValue());
    }
}

//==============================================================================
float AttackLaw::fromNormalised (float n) noexcept
{
    if (n < kOffSpan)
        return 0.0f;
    return Range::attackMin + (Range::attackMax - Range::attackMin) * juce::jmin (1.0f, (n - kOffSpan) / (1.0f - kOffSpan));
}

float AttackLaw::toNormalised (float value) noexcept
{
    if (isOff (value))
        return 0.0f;
    return kOffSpan + (1.0f - kOffSpan) * (juce::jmin (value, Range::attackMax) - Range::attackMin) / (Range::attackMax - Range::attackMin);
}

float AttackLaw::snap (float value) noexcept
{
    return isOff (value) ? 0.0f : juce::jmin (value, Range::attackMax);
}

//==============================================================================
juce::AudioProcessorValueTreeState::ParameterLayout Urei1176Processor::createLayout()
{
    using Float = juce::AudioParameterFloat;
    using Choice = juce::AudioParameterChoice;
    using Attributes = juce::AudioParameterFloatAttributes;

    juce::AudioProcessorValueTreeState::ParameterLayout layout;

    // Input and Output: knob rotation 0..1, shown as the printed dial.
    const auto dial = [] (float fallback)
    {
        return Attributes().withStringFromValueFunction ([] (float v, int) { return ParamText::dial (v); })
                           .withValueFromStringFunction ([fallback] (const juce::String& t)
                                                         { return ParamText::rotationForDialText (t).value_or (fallback); });
    };
    layout.add (std::make_unique<Float> (juce::ParameterID { ParamID::input, 1 }, "Input",
                                         juce::NormalisableRange<float> (Range::inputMin, Range::inputMax),
                                         Range::inputDefault, dial (Range::inputDefault)));
    layout.add (std::make_unique<Float> (juce::ParameterID { ParamID::output, 1 }, "Output",
                                         juce::NormalisableRange<float> (Range::outputMin, Range::outputMax),
                                         Range::outputDefault, dial (Range::outputDefault)));

    // Attack: 1..7, and the OFF click below 1 (see AttackLaw).
    const juce::NormalisableRange<float> attackRange (0.0f, Range::attackMax,
                                                      [] (float, float, float n) { return AttackLaw::fromNormalised (n); },
                                                      [] (float, float, float v) { return AttackLaw::toNormalised (v); },
                                                      [] (float, float, float v) { return AttackLaw::snap (v); });
    layout.add (std::make_unique<Float> (juce::ParameterID { ParamID::attack, 1 }, "Attack", attackRange, Default::attack,
                                         Attributes().withStringFromValueFunction ([] (float v, int) { return ParamText::attack (v); })
                                                     .withValueFromStringFunction ([] (const juce::String& t)
                                                                                   { return attackForText (t).value_or (Default::attack); })));

    layout.add (std::make_unique<Float> (juce::ParameterID { ParamID::release, 1 }, "Release",
                                         juce::NormalisableRange<float> (Range::releaseMin, Range::releaseMax), Default::release,
                                         Attributes().withStringFromValueFunction ([] (float v, int) { return juce::String (v, 1); })));

    // In urei1176::Ratio's order; "None" is every button out.
    layout.add (std::make_unique<Choice> (juce::ParameterID { ParamID::ratio, 1 }, "Ratio",
                                          juce::StringArray { "4", "8", "12", "20", "All", "None" }, (int) urei1176::Ratio::r4));
    // In urei1176::MeterMode's order.
    layout.add (std::make_unique<Choice> (juce::ParameterID { ParamID::meter, 1 }, "Meter",
                                          juce::StringArray { "Off", "GR", "+4", "+8" }, (int) urei1176::MeterMode::gainReduction));

    layout.add (std::make_unique<Float> (juce::ParameterID { ParamID::calibration, 1 }, "0 dBFS",
                                         juce::NormalisableRange<float> (Range::calibrationMin, Range::calibrationMax, 1.0f),
                                         Range::calibrationDefault,
                                         Attributes().withLabel ("dBu")
                                                     .withStringFromValueFunction ([] (float v, int) { return juce::String (juce::roundToInt (v)) + " dBu"; })));

    // Appended after the first seven (never reordered): the stereo mode, in urei1176::StereoMode's order, and the Attack Off switch.
    layout.add (std::make_unique<Choice> (juce::ParameterID { ParamID::stereo, 1 }, "Stereo",
                                          juce::StringArray { "Dual", "Link", "M/S" }, (int) urei1176::StereoMode::dual));
    layout.add (std::make_unique<juce::AudioParameterBool> (juce::ParameterID { ParamID::attackOff, 1 }, "Attack Off", false));
    // Appended after those two: the detector's high-pass (a plug-in addition; the choice index is the FastChannel::setSidechainFilter value).
    layout.add (std::make_unique<Choice> (juce::ParameterID { ParamID::sidechainFilter, 1 }, "S/C HPF",
                                          juce::StringArray { "OFF", "30", "60", "105", "125", "185" }, 0));
    // Appended last: the 1176 revision, in kModelChoices' order (F G J). A version added later (D, deferred) is appended or the stored
    // indices of every saved session would move. The default is F.
    layout.add (std::make_unique<Choice> (juce::ParameterID { ParamID::model, 1 }, "Version",
                                          modelChoiceNames(), choiceForVersion (urei1176::Version::F)));

    return layout;
}

Urei1176Processor::Urei1176Processor()
    : AudioProcessor (BusesProperties().withInput ("Input", juce::AudioChannelSet::stereo(), true)
                                       .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
      apvts (*this, nullptr, "Urei1176", createLayout()),
      input (apvts.getRawParameterValue (ParamID::input)),
      output (apvts.getRawParameterValue (ParamID::output)),
      attack (apvts.getRawParameterValue (ParamID::attack)),
      release (apvts.getRawParameterValue (ParamID::release)),
      ratio (apvts.getRawParameterValue (ParamID::ratio)),
      meter (apvts.getRawParameterValue (ParamID::meter)),
      calibration (apvts.getRawParameterValue (ParamID::calibration)),
      stereo (apvts.getRawParameterValue (ParamID::stereo)),
      attackOff (apvts.getRawParameterValue (ParamID::attackOff)),
      sidechainFilter (apvts.getRawParameterValue (ParamID::sidechainFilter)),
      model (apvts.getRawParameterValue (ParamID::model))
{
    engines.setControlSource ([] (void* self, urei1176::FastChannel& channel) noexcept { static_cast<Urei1176Processor*> (self)->pushControls (channel); }, this);
}

class Urei1176Processor::SettleWorker final : public juce::Thread
{
public:
    explicit SettleWorker (urei1176::VersionSwitch& e) : juce::Thread ("1167 settle"), engines (e) {}
    void run() override
    {
        while (! threadShouldExit())
        {
            engines.pollSettle();
            wait (2);
        }
    }

private:
    urei1176::VersionSwitch& engines;
};

Urei1176Processor::~Urei1176Processor()
{
    stopSettleWorker();
}

void Urei1176Processor::stopSettleWorker()
{
    if (settleWorker != nullptr)
    {
        settleWorker->stopThread (2000);
        settleWorker.reset();
    }
}

void Urei1176Processor::releaseResources()
{
    stopSettleWorker();
}

bool Urei1176Processor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    const auto out = layouts.getMainOutputChannelSet();
    return (out == juce::AudioChannelSet::mono() || out == juce::AudioChannelSet::stereo())
           && out == layouts.getMainInputChannelSet();
}

void Urei1176Processor::pushControls (urei1176::FastChannel& channel) const noexcept
{
    // The setters only store targets; the model moves its pots towards them itself.
    const float attackValue = attack->load (std::memory_order_relaxed);
    // Limiting is off with the Attack Off switch, or with the Attack parameter in its OFF span (old sessions and automation).
    const bool limitOff = attackOff->load (std::memory_order_relaxed) >= 0.5f || AttackLaw::isOff (attackValue);

    channel.setInput (input->load (std::memory_order_relaxed));
    channel.setOutput (output->load (std::memory_order_relaxed));
    channel.setLimitOff (limitOff);
    channel.setAttack (limitOff ? Range::attackMin : attackValue);   // the pot sits at 1 when clicked off
    channel.setRelease (release->load (std::memory_order_relaxed));
    channel.setRatio ((urei1176::Ratio) juce::jlimit (0, 5, juce::roundToInt (ratio->load (std::memory_order_relaxed))));
    channel.setMeterMode ((urei1176::MeterMode) juce::jlimit (0, 3, juce::roundToInt (meter->load (std::memory_order_relaxed))));
    channel.setCalibrationDbu (calibration->load (std::memory_order_relaxed));
    channel.setSidechainFilter (juce::jlimit (0, 5, juce::roundToInt (sidechainFilter->load (std::memory_order_relaxed))));
}

urei1176::Version Urei1176Processor::currentVersion() const noexcept
{
    return versionForChoice (juce::roundToInt (model->load (std::memory_order_relaxed)));
}

urei1176::StereoMode Urei1176Processor::currentStereoMode() const noexcept
{
    return (urei1176::StereoMode) juce::jlimit (0, 2, juce::roundToInt (stereo->load (std::memory_order_relaxed)));
}

void Urei1176Processor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    // The worker is stopped while the engines are rebuilt. prepare() builds the circuit and settles its operating point with the controls as
    // they are now (it allocates, which is fine here and nowhere else): the active set is prepared for the version the parameter holds,
    // with no fade, and the standby is voided; the scratch of the crossfade is sized for the blocks.
    stopSettleWorker();
    engines.prepare (sampleRate, samplesPerBlock, currentVersion(), currentStereoMode());
    runningChannels.store (juce::jmin (getTotalNumInputChannels(), getTotalNumOutputChannels(), kMaxChannels));
    setLatencySamples (0);
    settleWorker = std::make_unique<SettleWorker> (engines);
    settleWorker->startThread (juce::Thread::Priority::normal);
}

void Urei1176Processor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;

    const int numSamples = buffer.getNumSamples();
    for (int ch = getTotalNumInputChannels(); ch < getTotalNumOutputChannels(); ++ch)
        buffer.clear (ch, 0, numSamples);

    const int numChannels = juce::jmin (getTotalNumInputChannels(), getTotalNumOutputChannels(), kMaxChannels);
    if (numChannels > 0)
        // Two units stepped together. Dual: each is a complete unit with its own side chain (bit for bit what it would give alone). Link: the
        // 1176SA tie of their control nodes. M/S: the units see mid and side. Ratio, Attack and Release are shared, so both units always
        // have the same settings. A change of the Version parameter is settled on a standby set and crossfaded in (dsp/VersionSwitch.h).
        engines.process (buffer.getArrayOfWritePointers(), numChannels, numSamples, currentVersion(), currentStereoMode(), isNonRealtime());

    runningChannels.store (numChannels, std::memory_order_relaxed);
}

Urei1176Processor::Meters Urei1176Processor::getMeters() const noexcept
{
    Meters meters;
    meters.channels = juce::jlimit (1, kMaxChannels, runningChannels.load (std::memory_order_relaxed));
    for (int ch = 0; ch < meters.channels; ++ch)
    {
        meters.needleDb[(size_t) ch] = engines.channel (ch).getMeterNeedleDb();
        meters.gainReductionDb[(size_t) ch] = engines.channel (ch).getGainReductionDb();
    }
    return meters;
}

//==============================================================================
void Urei1176Processor::getStateInformation (juce::MemoryBlock& destData)
{
    if (const auto xml = apvts.copyState().createXml())
        copyXmlToBinary (*xml, destData);
}

void Urei1176Processor::setStateInformation (const void* data, int sizeInBytes)
{
    const auto xml = getXmlFromBinary (data, sizeInBytes);
    if (xml == nullptr || ! xml->hasTagName (apvts.state.getType()))
        return;

    const auto loaded = juce::ValueTree::fromXml (*xml);
    const auto hasParameter = [&loaded] (const char* id) { return loaded.getChildWithProperty ("id", id).isValid(); };
    const bool hadStereo = hasParameter (ParamID::stereo), hadAttackOff = hasParameter (ParamID::attackOff),
               hadSidechainFilter = hasParameter (ParamID::sidechainFilter), hadModel = hasParameter (ParamID::model);
    apvts.replaceState (loaded);

    // replaceState leaves a parameter the state does not mention at its live value; a session saved before it existed must not inherit it.
    // (Each is set explicitly in every case: a live instance in Link or M/S, with the Attack Off switch on or the S/C HPF at 185, loads an
    // older state as Dual, as the state says, and with the S/C HPF off.)
    if (! hadStereo)
        apvts.getParameter (ParamID::stereo)->setValueNotifyingHost (apvts.getParameter (ParamID::stereo)->convertTo0to1 ((float) urei1176::StereoMode::dual));
    if (! hadSidechainFilter)
        apvts.getParameter (ParamID::sidechainFilter)->setValueNotifyingHost (0.0f);   // OFF, even on a live instance set to 185
    if (! hadModel)
    {
        // A session saved before the Version parameter existed was made with F, the only model there was: it loads as F, even on an
        // instance that is set to J now.
        auto& modelParam = *apvts.getParameter (ParamID::model);
        modelParam.setValueNotifyingHost (modelParam.convertTo0to1 ((float) choiceForVersion (urei1176::Version::F)));
    }
    if (! hadAttackOff)
    {
        // The OFF click of the earliest versions was the knob's bottom; the switch replaces it, and the knob sits at 1 as the hardware's pot does.
        auto& attackParam = *apvts.getParameter (ParamID::attack);
        const bool wasOff = AttackLaw::isOff (attackParam.convertFrom0to1 (attackParam.getValue()));
        if (wasOff)
            attackParam.setValueNotifyingHost (attackParam.convertTo0to1 (Range::attackMin));
        apvts.getParameter (ParamID::attackOff)->setValueNotifyingHost (wasOff ? 1.0f : 0.0f);
    }

    // Settle the loaded version now, on this (non-audio) thread, so that the next block already finds it ready and fades in at once
    // instead of playing the old version for the few milliseconds the worker would take.
    engines.settleNow (currentVersion());
}

juce::AudioProcessorEditor* Urei1176Processor::createEditor()
{
    return new Urei1176Editor (*this);
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new Urei1176Processor();
}
