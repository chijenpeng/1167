// Checks the plug-in's parameter laws (the Attack OFF click under automation and on the knob,
// the dial text, the ratio and meter mapping), then renders the editor to two PNGs without a
// host or a screen after running a 1 kHz tone through the plug-in so the meter has something
// to show:
//
//   editor.png        meter on GR, ratio 4
//   editor-plus4.png  meter on +4, ratio All
//
//   plugin1167_snapshot [output-directory]   (default: docs; returns 1 if a check fails)

#include "../Source/PluginEditor.h"

#include <cstdio>

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter();

namespace
{
constexpr double kRate = 48000.0;
constexpr int kBlock = 512;

int failures = 0;

bool same (float a, float b) { return std::abs (a - b) < 1.0e-6f; }

void check (bool ok, const juce::String& what)
{
    std::printf ("%s  %s\n", ok ? "ok  " : "FAIL", what.toRawUTF8());
    failures += ok ? 0 : 1;
}

std::unique_ptr<Urei1176Processor> makeProcessor()
{
    std::unique_ptr<Urei1176Processor> p (dynamic_cast<Urei1176Processor*> (createPluginFilter()));
    p->setPlayConfigDetails (2, 2, kRate, kBlock);
    p->prepareToPlay (kRate, kBlock);
    return p;
}

/** As a host's automation does: a normalised value. */
void automate (Urei1176Processor& p, const char* id, float normalised)
{
    p.apvts.getParameter (id)->setValueNotifyingHost (normalised);
}

void set (Urei1176Processor& p, const char* id, float value)
{
    auto* param = p.apvts.getParameter (id);
    param->setValueNotifyingHost (param->convertTo0to1 (value));
}

/** A 1 kHz sine on both channels; amplitude 1 is 0 dBFS. */
void runTone (Urei1176Processor& p, double seconds, float amplitude)
{
    static double phase = 0.0;
    juce::AudioBuffer<float> buffer (2, kBlock);
    juce::MidiBuffer midi;

    for (int done = 0; done < (int) (seconds * kRate); done += kBlock)
    {
        for (int i = 0; i < kBlock; ++i)
        {
            const float v = amplitude * (float) std::sin (phase);
            phase += juce::MathConstants<double>::twoPi * 1000.0 / kRate;
            buffer.setSample (0, i, v);
            buffer.setSample (1, i, v);
        }
        p.processBlock (buffer, midi);
    }
}

void checkParameters()
{
    auto p = makeProcessor();
    auto& attack = *p->apvts.getParameter (ParamID::attack);
    const auto& channel = p->getChannel (0);

    // Automation of the OFF click: 0, and anything in its stretch below 1.
    for (const float n : { 0.0f, 0.5f * AttackLaw::kOffSpan, AttackLaw::kOffSpan - 1.0e-4f })
    {
        automate (*p, ParamID::attack, n);
        runTone (*p, 0.01, 0.0f);
        check (channel.controls().limitOff && same (channel.controls().attack, urei1176::Range::attackMin),
               "attack automated to " + juce::String (n, 4) + " (normalised) clicks limiting off, pot at 1");
    }

    // Automation of the dial, 1 to 7.
    for (const float dial : { 1.0f, 1.5f, 4.0f, 6.3f, 7.0f })
    {
        automate (*p, ParamID::attack, attack.convertTo0to1 (dial));
        runTone (*p, 0.01, 0.0f);
        check (! channel.controls().limitOff && std::abs (channel.controls().attack - dial) < 1.0e-3f,
               "attack automated to " + juce::String (dial, 1) + " limits at that dial setting");
    }
    check (std::abs (attack.convertTo0to1 (1.0f) - AttackLaw::kOffSpan) < 1.0e-6f && attack.convertTo0to1 (7.0f) == 1.0f,
           "attack 1..7 fills the travel above the OFF click");
    check (attack.convertFrom0to1 (attack.convertTo0to1 (0.4f)) == 0.0f, "attack values between 0 and 1 snap to OFF");

    // The knob: the slider takes the parameter's range, so dragging below 1 lands on OFF.
    {
        juce::Slider slider (juce::Slider::RotaryHorizontalVerticalDrag, juce::Slider::NoTextBox);
        juce::SliderParameterAttachment attachment (attack, slider);
        check (slider.proportionOfLengthToValue (0.5 * AttackLaw::kOffSpan) == 0.0
                   && std::abs (slider.proportionOfLengthToValue (AttackLaw::kOffSpan) - 1.0) < 1.0e-5,
               "attack knob: travel below 1 is the OFF click, then 1");
        slider.setValue (0.7, juce::sendNotificationSync);
        check (slider.getValue() == 0.0 && attack.getValue() == 0.0f, "attack knob set to 0.7 snaps to OFF");
        check (slider.getTextFromValue (slider.getValue()) == "OFF", "attack knob shows OFF");
        slider.setValue (5.25, juce::sendNotificationSync);
        check (slider.getTextFromValue (slider.getValue()) == "5.3" || slider.getTextFromValue (slider.getValue()) == "5.2",
               "attack knob shows one decimal (" + slider.getTextFromValue (slider.getValue()) + ")");
    }

    // Typed and host text.
    check (attack.getText (attack.convertTo0to1 (0.0f), 0) == "OFF" && attack.getText (attack.convertTo0to1 (4.0f), 0) == "4.0",
           "attack text: OFF, 4.0");
    check (attack.convertFrom0to1 (attack.getValueForText ("off")) == 0.0f
               && std::abs (attack.convertFrom0to1 (attack.getValueForText ("5.5")) - 5.5f) < 1.0e-4f,
           "attack text in: off, 5.5");

    auto& input = *p->apvts.getParameter (ParamID::input);
    const auto dialText = [&] (float rotation) { return input.getText (rotation, 0); };
    check (dialText (0.5f) == "24" && dialText (1.0f) == "0" && dialText (0.0f) == juce::String::fromUTF8 ("\xE2\x88\x9E")
               && dialText (0.125f) == "48",
           "input dial text: 24 at 12 o'clock, 0 fully clockwise, infinity at the stop, 48");
    check (dialText (0.5625f) == "21.0", "input dial text between labels: " + dialText (0.5625f));
    check (std::abs (input.getValueForText ("21") - 0.5625f) < 1.0e-4f && input.getValueForText ("24") == 0.5f
               && input.getValueForText ("inf") == 0.0f && input.getValueForText ("0") == 1.0f,
           "input dial text in: 21, 24, inf, 0");
    check (input.getDefaultValue() == 0.5f && p->apvts.getParameter (ParamID::output)->getDefaultValue() == 0.5f,
           "input and output default to 24");

    // Ratio and meter in the DSP's order; calibration text.
    const urei1176::Ratio ratios[] { urei1176::Ratio::r4, urei1176::Ratio::r8, urei1176::Ratio::r12,
                                     urei1176::Ratio::r20, urei1176::Ratio::all, urei1176::Ratio::none };
    bool ratiosOk = true;
    for (int i = 0; i < 6; ++i)
    {
        set (*p, ParamID::ratio, (float) i);
        runTone (*p, 0.01, 0.0f);
        ratiosOk = ratiosOk && channel.controls().ratio == ratios[i] && p->getChannel (1).controls().ratio == ratios[i];
    }
    check (ratiosOk, "ratio 4, 8, 12, 20, All, None reach both channels");

    bool metersOk = true;
    for (int i = 0; i < 4; ++i)
    {
        set (*p, ParamID::meter, (float) i);
        runTone (*p, 0.01, 0.0f);
        metersOk = metersOk && channel.controls().meter == (urei1176::MeterMode) i;
    }
    check (metersOk, "meter Off, GR, +4, +8 reach the channels");

    auto& calibration = *p->apvts.getParameter (ParamID::calibration);
    set (*p, ParamID::calibration, 21.0f);
    runTone (*p, 0.01, 0.0f);
    check (calibration.getText (calibration.getValue(), 0) == "21 dBu" && channel.controls().calibrationDbu == 21.0f,
           "calibration 21 dBu");

    // State round trip.
    juce::MemoryBlock state;
    p->getStateInformation (state);
    auto restored = makeProcessor();
    restored->setStateInformation (state.getData(), (int) state.getSize());
    check (same (restored->apvts.getParameter (ParamID::calibration)->getValue(), calibration.getValue())
               && same (restored->apvts.getParameter (ParamID::meter)->getValue(), p->apvts.getParameter (ParamID::meter)->getValue()),
           "state round trip");
}

/** `helpFor`: a parameter ID whose hover help to show, as if the mouse were over its control. */
bool writeSnapshot (Urei1176Editor& editor, const juce::File& file, const juce::String& helpFor = {})
{
    editor.updateMeters();
    juce::MessageManager::getInstance()->runDispatchLoopUntil (100);   // let the editor's timer run too
    if (helpFor.isNotEmpty())
        editor.showHelpForParameter (helpFor);
    const auto image = editor.createComponentSnapshot (editor.getLocalBounds(), true, 2.0f);
    file.deleteFile();
    juce::FileOutputStream out (file);
    const bool ok = out.openedOk() && juce::PNGImageFormat().writeImageToStream (image, out);
    std::printf ("%s %s (%d x %d)\n", ok ? "wrote" : "could not write", file.getFullPathName().toRawUTF8(), image.getWidth(), image.getHeight());
    return ok;
}

void printMeters (const Urei1176Processor& p)
{
    const auto m = p.getMeters();
    std::printf ("  needle %.2f / %.2f dB, gain cell reduction %.2f / %.2f dB\n",
                 m.needleDb[0], m.needleDb[1], m.gainReductionDb[0], m.gainReductionDb[1]);
}
} // namespace

int main (int argc, char** argv)
{
    const juce::ScopedJuceInitialiser_GUI gui;
    std::setvbuf (stdout, nullptr, _IOLBF, 0);
    const auto directory = juce::File::getCurrentWorkingDirectory().getChildFile (argc > 1 ? argv[1] : "docs");
    directory.createDirectory();

    checkParameters();

    // Input at 0.75 ("12"), ratio 4, meter on GR; a 1 kHz tone at -26 dBFS for 1.5 s: about 7 dB
    // of reduction, the needle near -6. (At -12 dBFS it is 17 dB and the needle sits on the stop.)
    auto p = makeProcessor();
    set (*p, ParamID::input, 0.75f);
    set (*p, ParamID::ratio, (float) urei1176::Ratio::r4);
    set (*p, ParamID::meter, (float) urei1176::MeterMode::gainReduction);
    const float amplitude = juce::Decibels::decibelsToGain (-26.0f);
    runTone (*p, 1.5, amplitude);
    std::printf ("GR, ratio 4:\n");
    printMeters (*p);

    bool ok = true;
    {
        Urei1176Editor editor (*p);
        ok = writeSnapshot (editor, directory.getChildFile ("editor.png")) && ok;

        // The same tone with the meter on +4, every ratio button in and Output at "18", so the
        // needle sits near 0 VU; the hover help of the METER row is showing.
        set (*p, ParamID::meter, (float) urei1176::MeterMode::plus4);
        set (*p, ParamID::ratio, (float) urei1176::Ratio::all);
        set (*p, ParamID::output, urei1176::rotationForMarking (18.0f));
        runTone (*p, 1.5, amplitude);
        std::printf ("+4, ratio All, Output 18:\n");
        printMeters (*p);
        ok = writeSnapshot (editor, directory.getChildFile ("editor-plus4.png"), ParamID::meter) && ok;
    }

    std::printf (failures == 0 ? "all checks passed\n" : "%d check(s) failed\n", failures);
    return ok && failures == 0 ? 0 : 1;
}
