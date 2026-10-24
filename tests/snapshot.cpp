// Checks the plug-in's parameter laws (the Attack OFF span under automation, the dial-only Attack
// knob and the Attack Off switch, the dial text, the ratio and meter mapping), the stereo modes through the
// processor, and old-state migration and round trips, then renders the editor to two PNGs without a host or a screen
// after running a 1 kHz tone through the plug-in so the meter has something to show:
//
//   editor.png        meter on GR, ratio 4
//   editor-plus4.png  meter on +4, ratio All
//
//   plugin1167_snapshot [output-directory [extra-directory]]
//
// (default directory: docs; returns 1 if a check fails). With an extra directory it also writes editor-link.png
// and editor-ms.png there, the editor in Link and in M/S with the Attack Off switch lit (not part of docs/).

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

    // The knob covers the dial, 1 to 7: it never enters the parameter's OFF span.
    {
        auto knob = Knob::makeAttackKnob (p->apvts, "", 64);
        auto* slider = dynamic_cast<juce::Slider*> (knob->getChildComponent (0));
        check (slider != nullptr && slider->getMinimum() == 1.0 && slider->getMaximum() == 7.0, "attack knob: the dial, 1 to 7");
        if (slider != nullptr)
        {
            slider->setValue (0.0, juce::sendNotificationSync);
            check (slider->getValue() == 1.0 && attack.convertFrom0to1 (attack.getValue()) == 1.0f,
                   "attack knob: dragged below 1 it stays on 1, the parameter at 1 (not OFF)");
            slider->setValue (5.25, juce::sendNotificationSync);
            check (std::abs (attack.convertFrom0to1 (attack.getValue()) - 5.25f) < 1.0e-4f
                       && (slider->getTextFromValue (slider->getValue()) == "5.3" || slider->getTextFromValue (slider->getValue()) == "5.2"),
                   "attack knob shows one decimal (" + slider->getTextFromValue (slider->getValue()) + ") and writes the parameter");
            check (slider->getValueFromText ("off") == 1.0 && slider->getValueFromText ("0.4") == 1.0 && slider->getValueFromText ("9") == 7.0
                       && std::abs (slider->getValueFromText ("3.5") - 3.5) < 1.0e-9,
                   "attack knob: typed text stays on the dial (off, 0.4 -> 1; 9 -> 7)");
            check (same ((float) slider->getDoubleClickReturnValue(), urei1176::Range::attackDefault), "attack knob: double-click returns to 4");
            // Old automation or a generic UI puts the parameter inside the OFF span: the knob shows 1.
            automate (*p, ParamID::attack, 0.0f);
            check (slider->getValue() == 1.0, "attack parameter in the OFF span: the knob shows 1");
            set (*p, ParamID::attack, 4.0f);
        }
    }

    // The Attack Off switch.
    {
        auto& off = *p->apvts.getParameter (ParamID::attackOff);
        set (*p, ParamID::attack, 5.5f);
        runTone (*p, 0.01, 0.0f);
        check (! channel.controls().limitOff && std::abs (channel.controls().attack - 5.5f) < 1.0e-3f, "attack off, switch out: limits at 5.5");
        automate (*p, ParamID::attackOff, 1.0f);
        runTone (*p, 0.01, 0.0f);
        check (channel.controls().limitOff && same (channel.controls().attack, urei1176::Range::attackMin)
                   && p->getChannel (1).controls().limitOff && same (attack.convertFrom0to1 (attack.getValue()), 5.5f),
               "attack off, switch in: limiting off, pot parked at 1, the Attack parameter keeps 5.5");
        automate (*p, ParamID::attackOff, 0.0f);
        runTone (*p, 0.01, 0.0f);
        check (! channel.controls().limitOff && std::abs (channel.controls().attack - 5.5f) < 1.0e-3f, "attack off, switch out again: back at 5.5");
        check (off.getDefaultValue() == 0.0f && off.getText (1.0f, 0).isNotEmpty(), "attack off: default out");

        AttackOffSwitch sw (p->apvts);
        sw.setSize (100, 18);
        bool lit = false;
        sw.onLitChanged = [&lit] (bool l) { lit = l; };
        check (! sw.isLit(), "switch unlit when limiting");
        automate (*p, ParamID::attackOff, 1.0f);
        check (sw.isLit() && lit, "switch lit when the parameter is on");
        automate (*p, ParamID::attackOff, 0.0f);
        automate (*p, ParamID::attack, 0.0f);
        check (sw.isLit() && lit, "switch lit when Attack is inside its OFF span (the knob was turned there in an old session)");
        sw.mouseDown (juce::MouseEvent (juce::Desktop::getInstance().getMainMouseSource(), { 5.0f, 5.0f }, {}, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &sw, &sw,
                                        juce::Time(), { 5.0f, 5.0f }, juce::Time(), 1, false));
        runTone (*p, 0.01, 0.0f);
        check (! sw.isLit() && ! channel.controls().limitOff && same (channel.controls().attack, urei1176::Range::attackMin),
               "clicking the lit switch from the OFF span releases it and puts the pot at 1");
        set (*p, ParamID::attack, 4.0f);
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

/** A state as an earlier version saved it: without the parameters appended since (`stereo`, `attackOff`). */
juce::MemoryBlock oldState (Urei1176Processor& source)
{
    auto xml = source.apvts.copyState().createXml();
    for (const char* id : { ParamID::stereo, ParamID::attackOff })
        if (auto* child = xml->getChildByAttribute ("id", id))
            xml->removeChildElement (child, true);
    juce::MemoryBlock block;
    juce::AudioProcessor::copyXmlToBinary (*xml, block);
    return block;
}

float value (Urei1176Processor& p, const char* id)
{
    auto& param = *p.apvts.getParameter (id);
    return param.convertFrom0to1 (param.getValue());
}

void load (Urei1176Processor& p, const juce::MemoryBlock& state)
{
    p.setStateInformation (state.getData(), (int) state.getSize());
}

void checkStateAndStereo()
{
    // Parameters appended after the first seven, in this order.
    {
        auto p = makeProcessor();
        const auto& all = p->getParameters();
        check (all.size() == 9 && dynamic_cast<juce::HostedAudioProcessorParameter*> (all[7]) != nullptr
                   && static_cast<juce::AudioProcessorParameterWithID*> (all[7])->paramID == ParamID::stereo
                   && static_cast<juce::AudioProcessorParameterWithID*> (all[8])->paramID == ParamID::attackOff
                   && static_cast<juce::AudioProcessorParameterWithID*> (all[6])->paramID == ParamID::calibration,
               "stereo and attackOff are appended after the first seven parameters");
        auto* stereo = dynamic_cast<juce::AudioParameterChoice*> (p->apvts.getParameter (ParamID::stereo));
        check (stereo != nullptr && stereo->choices == juce::StringArray ({ "Dual", "Link", "M/S" }) && stereo->getIndex() == 0
                   && p->apvts.getParameter (ParamID::stereo)->getName (32) == "Stereo",
               "stereo: Dual, Link, M/S, default Dual");
    }

    // An older state loads with the new parameters set explicitly, whatever the live instance had.
    {
        auto source = makeProcessor();
        set (*source, ParamID::attack, 5.5f);
        const auto notOff = oldState (*source);
        automate (*source, ParamID::attack, 0.0f);
        const auto inOffSpan = oldState (*source);

        auto live = makeProcessor();
        set (*live, ParamID::stereo, 1.0f);
        automate (*live, ParamID::attackOff, 1.0f);
        set (*live, ParamID::attack, 2.0f);
        load (*live, notOff);
        check (value (*live, ParamID::stereo) == 0.0f, "old state: stereo is Dual although the live instance was in Link");
        check (value (*live, ParamID::attackOff) == 0.0f && std::abs (value (*live, ParamID::attack) - 5.5f) < 1.0e-4f,
               "old state, Attack 5.5: attackOff is false although the live switch was on, Attack 5.5");

        set (*live, ParamID::stereo, 2.0f);
        automate (*live, ParamID::attackOff, 0.0f);
        load (*live, inOffSpan);
        check (value (*live, ParamID::stereo) == 0.0f, "old state: stereo is Dual although the live instance was in M/S");
        check (same (value (*live, ParamID::attackOff), 1.0f) && same (value (*live, ParamID::attack), urei1176::Range::attackMin),
               "old state with Attack in its OFF span: attackOff is true and Attack is 1");

        // and it limits off, with the pot at 1
        runTone (*live, 0.01, 0.0f);
        check (live->getChannel (0).controls().limitOff && same (live->getChannel (0).controls().attack, 1.0f), "... which the channels follow");
    }

    // Round trips with each mode, and the switch.
    for (int mode = 0; mode < 3; ++mode)
        for (const bool off : { false, true })
        {
            auto a = makeProcessor();
            set (*a, ParamID::stereo, (float) mode);
            automate (*a, ParamID::attackOff, off ? 1.0f : 0.0f);
            set (*a, ParamID::attack, 6.2f);
            set (*a, ParamID::ratio, 3.0f);
            juce::MemoryBlock block;
            a->getStateInformation (block);
            auto b = makeProcessor();
            set (*b, ParamID::stereo, (float) ((mode + 1) % 3));
            automate (*b, ParamID::attackOff, off ? 0.0f : 1.0f);
            load (*b, block);
            check (same (value (*b, ParamID::stereo), (float) mode) && same (value (*b, ParamID::attackOff), off ? 1.0f : 0.0f)
                       && std::abs (value (*b, ParamID::attack) - 6.2f) < 1.0e-4f && value (*b, ParamID::ratio) == 3.0f,
                   "state round trip: stereo " + juce::String (mode) + ", attack off " + juce::String (off ? "on" : "off"));
        }

    // Through the processor: mono-compatible material in M/S equals Dual on both channels; Link equals Dual for identical channels;
    // a mono layout ignores the parameter.
    {
        const auto render = [] (int mode, bool identical, float* peakOut)
        {
            auto p = makeProcessor();
            set (*p, ParamID::stereo, (float) mode);
            set (*p, ParamID::input, 0.8f);
            p->prepareToPlay (kRate, kBlock);   // as a host does before playing: the mode needs no morph then
            juce::AudioBuffer<float> buffer (2, kBlock), all (2, kBlock * 40);
            juce::MidiBuffer midi;
            double phase = 0.0;
            for (int block = 0; block < 40; ++block)
            {
                for (int i = 0; i < kBlock; ++i)
                {
                    const float v = 0.1f * (float) std::sin (phase);
                    phase += juce::MathConstants<double>::twoPi * 1000.0 / kRate;
                    buffer.setSample (0, i, v);
                    buffer.setSample (1, i, identical ? v : 0.3f * v);
                }
                p->processBlock (buffer, midi);
                all.copyFrom (0, block * kBlock, buffer, 0, 0, kBlock);
                all.copyFrom (1, block * kBlock, buffer, 1, 0, kBlock);
            }
            *peakOut = all.getMagnitude (0, 0, all.getNumSamples());
            return all;
        };
        float peakDual = 0.0f, peakLink = 0.0f, peakMs = 0.0f;
        const auto dual = render (0, true, &peakDual);
        const auto link = render (1, true, &peakLink);
        const auto ms = render (2, true, &peakMs);
        float linkErr = 0.0f, msErr = 0.0f;
        for (int ch = 0; ch < 2; ++ch)
            for (int i = 0; i < dual.getNumSamples(); ++i)
            {
                linkErr = juce::jmax (linkErr, std::abs (dual.getSample (ch, i) - link.getSample (ch, i)));
                if (i > (int) (0.05 * kRate))   // the matrix morphs in over 10 ms
                    msErr = juce::jmax (msErr, std::abs (dual.getSample (ch, i) - ms.getSample (ch, i)));
            }
        std::printf ("  (identical L and R, peak %.4f: link differs by %.2e, mid-side by %.2e of the peak)\n", peakDual, linkErr / peakDual, msErr / peakDual);
        check (linkErr / peakDual < 2.0e-7f, "processor, L == R: Link equals Dual (to a float ulp)");
        check (msErr / peakDual < 1.0e-6f, "processor, L == R: M/S equals Dual on both channels");
        float peakDiff = 0.0f;
        const auto link2 = render (1, false, &peakDiff);
        const auto dual2 = render (0, false, &peakDiff);
        float different = 0.0f;
        for (int i = 0; i < dual2.getNumSamples(); ++i) different = juce::jmax (different, std::abs (dual2.getSample (0, i) - link2.getSample (0, i)));
        check (different > 1.0e-4f, "processor, L != R: Link differs from Dual (the tie does something)");

        // mono layout
        auto mono = std::unique_ptr<Urei1176Processor> (dynamic_cast<Urei1176Processor*> (createPluginFilter()));
        mono->setPlayConfigDetails (1, 1, kRate, kBlock);
        mono->prepareToPlay (kRate, kBlock);
        auto reference = std::unique_ptr<Urei1176Processor> (dynamic_cast<Urei1176Processor*> (createPluginFilter()));
        reference->setPlayConfigDetails (1, 1, kRate, kBlock);
        reference->prepareToPlay (kRate, kBlock);
        set (*mono, ParamID::stereo, 2.0f);
        juce::AudioBuffer<float> a (1, kBlock), b (1, kBlock);
        juce::MidiBuffer midi;
        float monoErr = 0.0f;
        for (int block = 0; block < 10; ++block)
        {
            for (int i = 0; i < kBlock; ++i)
                a.setSample (0, i, 0.2f * (float) std::sin (0.05 * (block * kBlock + i)));
            b.makeCopyOf (a);
            mono->processBlock (a, midi);
            reference->processBlock (b, midi);
            for (int i = 0; i < kBlock; ++i) monoErr = juce::jmax (monoErr, std::abs (a.getSample (0, i) - b.getSample (0, i)));
        }
        check (monoErr == 0.0f, "mono layout: the stereo parameter is ignored (M/S output equals the default's, bit for bit)");
    }
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
    // MSVC's _IOLBF validates the buffer size as 2..INT_MAX even when buffer is null, so 0
    // invokes the invalid-parameter handler and fast-fails before the first test output.
    if (std::setvbuf (stdout, nullptr, _IONBF, 0) != 0)
    {
        std::fprintf (stderr, "failed to disable stdout buffering\n");
        return 1;
    }
    const auto directory = juce::File::getCurrentWorkingDirectory().getChildFile (argc > 1 ? argv[1] : "docs");
    directory.createDirectory();

    checkParameters();
    checkStateAndStereo();

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

    if (argc > 2)
    {
        // Link, then M/S with the Attack Off switch lit and the hover help on the Stereo row: the same tone, two different amplitudes per side.
        const auto extra = juce::File::getCurrentWorkingDirectory().getChildFile (argv[2]);
        extra.createDirectory();
        auto q = makeProcessor();
        set (*q, ParamID::input, 0.75f);
        set (*q, ParamID::stereo, 1.0f);
        {
            Urei1176Editor editor (*q);
            runTone (*q, 1.5, amplitude);
            ok = writeSnapshot (editor, extra.getChildFile ("editor-link.png"), ParamID::stereo) && ok;
        }
        set (*q, ParamID::stereo, 2.0f);
        automate (*q, ParamID::attackOff, 1.0f);
        {
            Urei1176Editor editor (*q);
            runTone (*q, 0.5, amplitude);
            ok = writeSnapshot (editor, extra.getChildFile ("editor-ms.png"), ParamID::attackOff) && ok;
        }
        auto r = makeProcessor();
        set (*r, ParamID::input, 0.75f);
        set (*r, ParamID::stereo, 2.0f);
        {
            Urei1176Editor editor (*r);
            // M/S with the right channel at a quarter of the left, in phase: M carries most of it, S a little
            juce::AudioBuffer<float> buffer (2, kBlock);
            juce::MidiBuffer midi;
            double phase = 0.0;
            for (int block = 0; block < 150; ++block)
            {
                for (int i = 0; i < kBlock; ++i)
                {
                    buffer.setSample (0, i, 3.0f * amplitude * (float) std::sin (phase));
                    buffer.setSample (1, i, 0.5f * amplitude * (float) std::sin (phase));
                    phase += juce::MathConstants<double>::twoPi * 1000.0 / kRate;
                }
                r->processBlock (buffer, midi);
            }
            ok = writeSnapshot (editor, extra.getChildFile ("editor-ms-readout.png"), ParamID::stereo) && ok;
        }
    }

    std::printf (failures == 0 ? "all checks passed\n" : "%d check(s) failed\n", failures);
    return ok && failures == 0 ? 0 : 1;
}
