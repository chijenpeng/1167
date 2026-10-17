#include "PluginEditor.h"

#include <BinaryData.h>

namespace
{
    // The analog-recreation / drum-gate language: flat, dark, no borders. Pale discs are the
    // things you turn; white and pale type is what is selected; the quieter grey names things.
    namespace Theme
    {
        const juce::Colour background { 0xff0e0e0f }, panel { 0xff1c1c1e }, text { 0xffffffff },
                           label { 0xffa6a6a8 }, accent { 0xff5b8cff }, pale { 0xffcfeef8 },
                           clipping { 0xffff5a5a }, field { 0xff38383b };

        // The VU face: the panel grey, a shade warmer, for the 1176's cream meter; its hood is
        // the plain panel grey.
        const juce::Colour face { 0xff24221e }, hood = panel;
        // Its red zone, quieter than the clipping red it comes from.
        const juce::Colour redZone = clipping.withMultipliedSaturation (0.7f).withAlpha (0.7f);

        juce::Font make (const juce::Typeface::Ptr& typeface, float height, float tracking)
        {
            return juce::Font (juce::FontOptions (typeface).withHeight (height)).withExtraKerningFactor (tracking);
        }

        /** Labels are upper-case and tracked out; pass upper-case text. */
        juce::Font labelFont (float height = 11.0f)  { return make (juce::SharedResourcePointer<EditorFonts>()->inter, height, 0.09f); }
        juce::Font titleFont (float height)          { return make (juce::SharedResourcePointer<EditorFonts>()->inter, height, 0.18f); }
        juce::Font bodyFont (float height = 12.0f)   { return make (juce::SharedResourcePointer<EditorFonts>()->inter, height, 0.01f); }
        juce::Font valueFont (float height = 12.0f)  { return make (juce::SharedResourcePointer<EditorFonts>()->mono, height, 0.0f); }

        int widthOf (const juce::Font& font, const juce::String& s)
        {
            return juce::roundToInt (std::ceil (juce::GlyphArrangement::getStringWidth (font, s)));
        }
    }

    // Spacing grammar of an option row: sibling options closest, the label further from its
    // first option.
    constexpr int kOptionGap = 10, kLabelGap = 18, kRowHeight = 18;

    const juce::Identifier helpId ("help"), helpTitleId ("helpTitle"), editorScaleId ("editorScale");

    constexpr auto kDefaultHelpTitle = "1167";
    constexpr auto kDefaultHelp =
        "1167 is an independent model inspired by the UREI 1176LN revision F (blackface). In stereo it runs "
        "two independent channels with no link between them. Hover over a control to see what it does; "
        "double-click a knob for its default, or its value to type one.";

    void setHelp (juce::Component& c, const juce::String& title, const juce::String& help)
    {
        c.getProperties().set (helpTitleId, title.toUpperCase());
        c.getProperties().set (helpId, help);
    }

    juce::String minusSign() { return juce::String::fromUTF8 ("\xE2\x88\x92"); }
}

//==============================================================================
EditorFonts::EditorFonts()
    : inter (juce::Typeface::createSystemTypefaceFor (BinaryData::InterSemiBold_ttf, BinaryData::InterSemiBold_ttfSize)),
      mono (juce::Typeface::createSystemTypefaceFor (BinaryData::JetBrainsMonoMedium_ttf, BinaryData::JetBrainsMonoMedium_ttfSize))
{
}

//==============================================================================
void FlatLookAndFeel::drawRotarySlider (juce::Graphics& g, int x, int y, int width, int height, float position,
                                        float startAngle, float endAngle, juce::Slider& slider)
{
    const auto bounds = juce::Rectangle<int> (x, y, width, height).toFloat();
    const auto centre = bounds.getCentre();
    const float radius = juce::jmin (bounds.getWidth(), bounds.getHeight()) * 0.5f - 1.0f;
    const float angle = startAngle + position * (endAngle - startAngle);
    const float dot = juce::jmax (2.5f, radius * 0.11f);

    g.setColour (slider.isMouseOverOrDragging() ? Theme::pale.brighter (0.1f) : Theme::pale);
    g.fillEllipse (juce::Rectangle<float> (radius * 2.0f, radius * 2.0f).withCentre (centre));

    g.setColour (Theme::background);
    g.fillEllipse (juce::Rectangle<float> (dot * 2.0f, dot * 2.0f).withCentre (centre.getPointOnCircumference (radius * 0.72f, angle)));
}

void FlatLookAndFeel::drawCornerResizer (juce::Graphics& g, int w, int h, bool isMouseOver, bool isMouseDragging)
{
    // Three short diagonals in the label grey, quiet until touched.
    g.setColour (Theme::label.withAlpha (isMouseOver || isMouseDragging ? 0.8f : 0.3f));
    const float size = (float) juce::jmin (w, h);
    for (float i = 0.3f; i < 1.0f; i += 0.3f)
        g.drawLine ((float) w - size * i, (float) h - 2.0f, (float) w - 2.0f, (float) h - size * i, 1.0f);
}

//==============================================================================
Knob::Knob (juce::AudioProcessorValueTreeState& state, const char* paramId, const juce::String& nameToUse, const juce::String& help,
            int diameterToUse, float sweepDegrees)
    : name (nameToUse.toUpperCase()), diameter (diameterToUse), attachment (*state.getParameter (paramId), slider)
{
    const float halfSweep = juce::degreesToRadians (sweepDegrees * 0.5f), twoPi = juce::MathConstants<float>::twoPi;
    slider.setRotaryParameters (twoPi - halfSweep, twoPi + halfSweep, true);
    slider.setVelocityModeParameters (0.15, 1, 0.0, true, juce::ModifierKeys::shiftModifier);   // Shift: fine
    slider.onValueChange = [this] { repaint(); };
    addAndMakeVisible (slider);

    editor.setJustification (juce::Justification::centred);
    editor.setFont (Theme::valueFont (12.0f));
    editor.setIndents (0, 0);
    editor.setColour (juce::TextEditor::backgroundColourId, Theme::field);
    editor.setColour (juce::TextEditor::textColourId, Theme::pale);
    editor.setColour (juce::TextEditor::highlightColourId, Theme::accent.withAlpha (0.5f));
    editor.setColour (juce::TextEditor::outlineColourId, juce::Colours::transparentBlack);
    editor.setColour (juce::TextEditor::focusedOutlineColourId, juce::Colours::transparentBlack);
    editor.onReturnKey = [this] { finishEditing (true); };
    editor.onEscapeKey = [this] { finishEditing (false); };
    editor.onFocusLost = [this] { finishEditing (true); };
    addChildComponent (editor);

    setHelp (*this, name, help);
}

void Knob::resized()
{
    auto area = getLocalBounds();
    area.removeFromTop (kLabelHeight + kGap - 1);
    slider.setBounds (area.removeFromTop (diameter + 2).withSizeKeepingCentre (diameter + 2, diameter + 2));
    editor.setBounds (getLocalBounds().removeFromBottom (kLabelHeight).withSizeKeepingCentre (64, kLabelHeight));
}

void Knob::paint (juce::Graphics& g)
{
    auto area = getLocalBounds();

    g.setColour (Theme::label);
    g.setFont (Theme::labelFont());
    g.drawText (name, area.removeFromTop (kLabelHeight), juce::Justification::centred, false);

    // JetBrains Mono's infinity is tiny at this size; Inter's matches the figures.
    const auto value = slider.getTextFromValue (slider.getValue());
    const bool infinity = value == juce::String::fromUTF8 (urei1176::kDialLabels[0].text);
    g.setFont (infinity ? Theme::bodyFont (14.0f) : Theme::valueFont (12.0f));
    g.drawText (value, area.removeFromBottom (kLabelHeight), juce::Justification::centred, false);
}

void Knob::mouseDoubleClick (const juce::MouseEvent& e)
{
    if (e.y < getHeight() - kLabelHeight - 2)
        return;

    editor.setText (slider.getTextFromValue (slider.getValue()), false);
    editor.setVisible (true);
    editor.selectAll();
    editor.grabKeyboardFocus();
}

void Knob::finishEditing (bool apply)
{
    if (! editor.isVisible())
        return;

    editor.setVisible (false);
    const auto text = editor.getText().trim().toLowerCase();

    // Only text that names a value: the parameters fall back to their defaults otherwise.
    const bool meaningful = text.containsAnyOf ("0123456789") || text.startsWith ("off") || text.startsWith ("inf")
                            || text.startsWith (juce::String::fromUTF8 (urei1176::kDialLabels[0].text));
    if (apply && meaningful)
        slider.setValue (slider.getValueFromText (text), juce::sendNotificationSync);
    repaint();
}

//==============================================================================
OptionRow::OptionRow (juce::RangedAudioParameter& param, const juce::String& labelToUse, juce::StringArray optionsToUse,
                      const juce::String& help, int releasedIndexToUse)
    : label (labelToUse.toUpperCase()), options (std::move (optionsToUse)), releasedIndex (releasedIndexToUse),
      attachment (param, [this] (float v) { selected = juce::roundToInt (v); repaint(); })
{
    labelColumn = labelTextWidth() + kLabelGap;
    attachment.sendInitialUpdate();
    setHelp (*this, label, help);
}

int OptionRow::labelTextWidth() const
{
    return Theme::widthOf (Theme::labelFont(), label);
}

std::vector<juce::Range<int>> OptionRow::optionRanges() const
{
    std::vector<juce::Range<int>> ranges;
    const auto font = Theme::labelFont();
    int x = labelColumn;

    for (const auto& option : options)
    {
        const int width = Theme::widthOf (font, option.toUpperCase());
        ranges.push_back ({ x, x + width });
        x += width + kOptionGap;
    }
    return ranges;
}

int OptionRow::preferredWidth() const
{
    const auto ranges = optionRanges();
    return ranges.empty() ? labelColumn : ranges.back().getEnd();
}

int OptionRow::optionAt (int x) const
{
    const auto ranges = optionRanges();
    for (int i = 0; i < (int) ranges.size(); ++i)
        if (ranges[(size_t) i].expanded (kOptionGap / 2).contains (x))
            return i;
    return -1;
}

void OptionRow::paint (juce::Graphics& g)
{
    g.setFont (Theme::labelFont());
    g.setColour (Theme::label);
    g.drawText (label, 0, 0, labelColumn, getHeight(), juce::Justification::centredLeft, false);

    const auto ranges = optionRanges();
    for (int i = 0; i < options.size(); ++i)
    {
        const auto range = ranges[(size_t) i];
        g.setColour (i == selected ? Theme::pale : (i == hovered ? Theme::text : Theme::label.withAlpha (0.7f)));
        g.drawText (options[i].toUpperCase(), range.getStart(), 0, range.getLength() + 4, getHeight(),
                    juce::Justification::centredLeft, false);
    }
}

void OptionRow::mouseDown (const juce::MouseEvent& e)
{
    const int option = optionAt (e.x);
    if (option < 0)
        return;

    // A lit push button pressed again comes out.
    const int value = option == selected && releasedIndex >= 0 ? releasedIndex : option;
    attachment.setValueAsCompleteGesture ((float) value);
}

void OptionRow::mouseMove (const juce::MouseEvent& e)   { hovered = optionAt (e.x); repaint(); }
void OptionRow::mouseExit (const juce::MouseEvent&)     { hovered = -1; repaint(); }

//==============================================================================
namespace
{
    // The VU scale: deflection is proportional to the rectified voltage, +3 VU at full scale.
    // The -20 mark is the left end here (the meter law clamps the reading there).
    constexpr float kHalfSweep = 0.85f;   // radians either side of 12 o'clock
    constexpr float kArcTop = 25.0f;      // from the top of the face to the top of the arc
    constexpr float kHoodHeight = 19.0f, kCorner = 4.0f;
    constexpr float kMarks[] { -20.0f, -10.0f, -7.0f, -5.0f, -3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f };
}

VuMeter::VuMeter()
{
    setHelp (*this, "Meter",
             "A VU movement (300 ms to settle), as on the hardware. On GR it reads the control voltage: it rests at 0, "
             "falls to the left and over-reads fast peaks as the real meter does. Bottom left: the gain cell's actual "
             "reduction (the larger of the two channels), without the meter's ballistics.");
}

float VuMeter::angleFor (float db) noexcept
{
    const auto deflection = [] (float d) { return std::pow (10.0f, d / 20.0f); };
    const float lo = deflection (urei1176::Range::meterMinDb), hi = deflection (urei1176::Range::meterMaxDb);
    const float clamped = juce::jlimit (urei1176::Range::meterMinDb, urei1176::Range::meterMaxDb, db);
    return kHalfSweep * (2.0f * (deflection (clamped) - lo) / (hi - lo) - 1.0f);
}

juce::Point<float> VuMeter::pivot() const
{
    const auto r = getLocalBounds().toFloat();
    return { r.getCentreX(), r.getY() + kArcTop + radius() };
}

float VuMeter::radius() const
{
    return (float) getWidth() * 0.42f / std::sin (kHalfSweep);
}

void VuMeter::setReading (float newNeedleDb, float newGrDb, urei1176::MeterMode newMode)
{
    const bool needleMoved = std::abs (angleFor (newNeedleDb) - angleFor (needleDb)) > 0.0005f;
    const bool readoutChanged = juce::String (newGrDb, 1) != juce::String (grDb, 1);

    if (needleMoved || readoutChanged || newMode != mode)
    {
        needleDb = newNeedleDb;
        grDb = newGrDb;
        mode = newMode;
        repaint();
    }
}

void VuMeter::paint (juce::Graphics& g)
{
    // The face, and at its foot the hood over the needle's pivot, which carries the readout.
    const auto faceArea = getLocalBounds().toFloat();
    const auto hood = faceArea.withTop (faceArea.getBottom() - kHoodHeight);
    g.setColour (Theme::face);
    g.fillRoundedRectangle (faceArea, kCorner);
    juce::Path hoodShape;
    hoodShape.addRoundedRectangle (hood.getX(), hood.getY(), hood.getWidth(), hood.getHeight(), kCorner, kCorner, false, false, true, true);
    g.setColour (Theme::hood);
    g.fillPath (hoodShape);

    // With the meter switched off the scale goes quiet, as an unlit meter does.
    const float scaleAlpha = mode == urei1176::MeterMode::off ? 0.45f : 1.0f;
    const auto centre = pivot();
    const float r = radius();

    // The arc: a thin line to 0, the red zone from 0 to +3.
    juce::Path line, red;
    line.addCentredArc (centre.x, centre.y, r, r, 0.0f, angleFor (urei1176::Range::meterMinDb), angleFor (0.0f), true);
    red.addCentredArc (centre.x, centre.y, r + 1.0f, r + 1.0f, 0.0f, angleFor (0.0f), angleFor (urei1176::Range::meterMaxDb), true);
    g.setColour (Theme::label.withAlpha (0.7f * scaleAlpha));
    g.strokePath (line, juce::PathStrokeType (1.0f));
    g.setColour (Theme::redZone.withMultipliedAlpha (scaleAlpha));
    g.strokePath (red, juce::PathStrokeType (3.0f, juce::PathStrokeType::mitered, juce::PathStrokeType::butt));

    // Marks and figures.
    g.setFont (Theme::valueFont (8.0f));
    for (const float mark : kMarks)
    {
        const float a = angleFor (mark);
        const bool plus = mark > 0.0f;
        g.setColour ((plus ? Theme::redZone : Theme::label).withMultipliedAlpha (scaleAlpha));
        g.drawLine ({ centre.getPointOnCircumference (r, a), centre.getPointOnCircumference (r + 5.0f, a) }, mark == 0.0f ? 1.5f : 1.0f);

        const auto text = mark < 0.0f ? minusSign() + juce::String ((int) -mark)
                                      : (plus ? "+" : "") + juce::String ((int) mark);
        const auto at = centre.getPointOnCircumference (r + 11.0f, a);
        g.drawText (text, juce::Rectangle<float> (22.0f, 10.0f).withCentre (at), juce::Justification::centred, false);
    }

    g.setColour (Theme::label.withAlpha (0.5f * scaleAlpha));
    g.setFont (Theme::titleFont (10.0f));
    g.drawText ("VU", juce::Rectangle<float> (faceArea.getX(), centre.y - r + 35.0f, faceArea.getWidth(), 12.0f),
                juce::Justification::centred, false);

    // The gain cell's reduction, as a number.
    auto readout = hood.reduced (8.0f, 0.0f);
    g.setColour (Theme::label);
    g.setFont (Theme::labelFont (8.0f));
    g.drawText ("GR", readout.removeFromLeft (18.0f), juce::Justification::centredLeft, false);
    g.setFont (Theme::valueFont (9.0f));
    g.drawText ((grDb >= 0.05f ? minusSign() : juce::String()) + juce::String (grDb, 1) + " dB", readout,
                juce::Justification::centredLeft, false);

    // The needle, from its pivot under the hood.
    juce::Graphics::ScopedSaveState state (g);
    g.reduceClipRegion (faceArea.withBottom (hood.getY()).toNearestInt());
    g.setColour (Theme::pale);
    g.drawLine ({ centre, centre.getPointOnCircumference (r + 2.0f, angleFor (needleDb)) }, 1.25f);
}

//==============================================================================
EditorView::EditorView (Urei1176Processor& p)
    : processor (p),
      input (p.apvts, ParamID::input, "Input",
             "How hard the signal drives the limiter. The threshold is fixed, so more Input means more gain reduction. "
             "The dial is the hardware's, in dB of loss: Input and Output both at 24 is about unity gain below the threshold.",
             64, 320.0f),
      output (p.apvts, ParamID::output, "Output",
              "The level after the gain cell, into the output amplifier and transformer. It does not change how hard the "
              "unit limits; turned far up it drives the output amplifier into clipping, as on the hardware.",
              64, 320.0f),
      attack (p.apvts, ParamID::attack, "Attack",
              "How quickly it clamps down: 7 (fully clockwise) is the fastest, about 0.25 ms; 1 about 4 ms. Turn past 1 to "
              "click it OFF: the side chain is cut and the unit does not limit, while the audio still runs through its "
              "amplifiers and transformers.",
              64),
      release (p.apvts, ParamID::release, "Release",
               "How quickly the gain recovers once the level drops: 7 (fully clockwise) is the fastest, about 90 ms; 1 about 1.3 s.",
               64),
      calibration (p.apvts, ParamID::calibration, "0 dBFS",
                   "The level in dBu that 0 dBFS (a full-scale sine) stands for, at the input and at the output. The 1176's "
                   "threshold is fixed in dBu, so a higher setting drives it less for the same track level. Input and Output "
                   "at 24 stay at about unity gain whatever this is.",
                   36),
      ratioRow (*p.apvts.getParameter (ParamID::ratio), "Ratio", { "4", "8", "12", "20", "All" },
                "4 to 20: how hard it limits above the threshold, which rises with the ratio. ALL is every button pressed in at "
                "once: the bias shifts and, on GR, the needle pins to the right at idle, as on a real unit. Click the lit "
                "button to let it out: with no button in, the unit does not limit.",
                (int) urei1176::Ratio::none),
      meterRow (*p.apvts.getParameter (ParamID::meter), "Meter", { "Off", "GR", "+4", "+8" },
                "What the needle shows; it never changes the sound. GR: gain reduction, resting at 0 and falling to the left "
                "(" + minusSign() + "10 is 10 dB). +4 / +8: output level, 0 VU = +4 or +8 dBu. OFF lets the needle rest (on the "
                "hardware it switches the unit off). In stereo, GR shows the channel reducing more, +4 / +8 the louder one.")
{
    const std::pair<juce::Component*, const char*> controls[] {
        { &input, ParamID::input }, { &output, ParamID::output }, { &attack, ParamID::attack }, { &release, ParamID::release },
        { &calibration, ParamID::calibration }, { &ratioRow, ParamID::ratio }, { &meterRow, ParamID::meter }
    };
    for (const auto& [component, id] : controls)
    {
        component->setComponentID (id);
        addAndMakeVisible (component);
    }
    addAndMakeVisible (vu);

    // The two rows share one label column, so their options start together.
    const int column = juce::jmax (ratioRow.labelTextWidth(), meterRow.labelTextWidth()) + kLabelGap;
    ratioRow.setLabelColumn (column);
    meterRow.setLabelColumn (column);

    showHelpFor (nullptr);
    updateMeters();
    setSize (kWidth, kHeight);
}

void EditorView::updateMeters()
{
    const auto mode = (urei1176::MeterMode) juce::jlimit (0, 3, juce::roundToInt (processor.apvts.getRawParameterValue (ParamID::meter)->load()));
    const auto meters = processor.getMeters();

    // One needle for both channels: on GR the one reducing more, on +4 / +8 the louder.
    float needle = meters.needleDb[0], gr = meters.gainReductionDb[0];
    for (int ch = 1; ch < meters.channels; ++ch)
    {
        needle = mode == urei1176::MeterMode::gainReduction ? juce::jmin (needle, meters.needleDb[(size_t) ch])
                                                             : juce::jmax (needle, meters.needleDb[(size_t) ch]);
        gr = juce::jmax (gr, meters.gainReductionDb[(size_t) ch]);
    }

    // The readout rises at once and falls at about 20 dB/s, so it can be read.
    shownGrDb = juce::jmax (gr, shownGrDb - 0.7f);
    vu.setReading (needle, shownGrDb, mode);
}

void EditorView::showHelpFor (juce::Component* component)
{
    juce::String title = kDefaultHelpTitle, text = kDefaultHelp;

    // The help belongs to the nearest ancestor that has any (a knob's slider has none itself).
    for (auto* c = component; c != nullptr && c != this; c = c->getParentComponent())
    {
        if (c->getProperties().contains (helpId))
        {
            title = c->getProperties()[helpTitleId].toString();
            text = c->getProperties()[helpId].toString();
            break;
        }
    }

    if (title != helpTitle || text != helpText)
    {
        helpTitle = title;
        helpText = text;
        repaint (helpArea);
    }
}

void EditorView::showHelpForParameter (const juce::String& paramId)
{
    for (auto* child : getChildren())
        if (child->getComponentID() == paramId)
            showHelpFor (child);
}

void EditorView::paint (juce::Graphics& g)
{
    g.fillAll (Theme::background);

    // Title: the model, and quieter, which one.
    const auto title = Theme::titleFont (15.0f), subtitle = Theme::titleFont (11.0f);
    const float baseline = 33.0f;
    g.setColour (Theme::pale);
    g.setFont (title);
    g.drawSingleLineText ("1167", 24, (int) baseline);
    g.setColour (Theme::label);
    g.setFont (subtitle);
    g.drawSingleLineText (juce::String::fromUTF8 ("LN \xC2\xB7 BLACKFACE REV F"), 24 + Theme::widthOf (title, "1167") + 12, (int) baseline);

    // Help, bottom left: what the control under the mouse does.
    auto help = helpArea;
    g.setColour (Theme::pale);
    g.setFont (Theme::labelFont());
    g.drawText (helpTitle, help.removeFromTop (16), juce::Justification::topLeft, false);
    g.setColour (Theme::label);
    g.setFont (Theme::bodyFont());
    g.drawFittedText (helpText, help, juce::Justification::topLeft, 3, 1.0f);
}

void EditorView::resized()
{
    constexpr int margin = 24, header = 50, knobCell = 104, faceWidth = 200, faceHeight = 104;
    constexpr int calibrationColumn = 64, laneGap = 24, laneToKnobs = 4;

    auto area = getLocalBounds().reduced (margin, 0);
    area.removeFromTop (header);

    // Bottom: what the control under the mouse does.
    area.removeFromBottom (10);
    helpArea = area.removeFromBottom (54);

    // Ratio and meter share one compact lane above the knob bank. To the right sit a two-thirds
    // size VU face and the calibration control; the help follows immediately below the unit.
    auto unit = area;
    auto knobArea = unit.removeFromLeft (4 * knobCell);

    const int laneWidth = ratioRow.preferredWidth() + laneGap + meterRow.preferredWidth();
    auto lane = juce::Rectangle<int> (knobArea.getCentreX() - laneWidth / 2, knobArea.getY(), laneWidth, kRowHeight);
    ratioRow.setBounds (lane.removeFromLeft (ratioRow.preferredWidth()));
    lane.removeFromLeft (laneGap);
    meterRow.setBounds (lane.removeFromLeft (meterRow.preferredWidth()));

    knobArea.removeFromTop (kRowHeight + laneToKnobs);
    for (auto* knob : { &input, &output, &attack, &release })
        knob->setBounds (knobArea.removeFromLeft (knobCell).withSizeKeepingCentre (knobCell, knob->preferredHeight()));

    auto calibrationArea = unit.removeFromRight (calibrationColumn);
    calibration.setBounds (calibrationArea.withSizeKeepingCentre (calibrationColumn, calibration.preferredHeight()));

    vu.setBounds (unit.withSizeKeepingCentre (faceWidth, faceHeight));
}

//==============================================================================
Urei1176Editor::Urei1176Editor (Urei1176Processor& p)
    : AudioProcessorEditor (p), owner (p), view (p)
{
    setLookAndFeel (&lookAndFeel);
    addAndMakeVisible (view);

    // Scales as a whole, 0.75x to 2x, keeping the design's aspect ratio.
    constexpr int w = EditorView::kWidth, h = EditorView::kHeight;
    setResizable (true, true);
    setResizeLimits (w * 3 / 4, h * 3 / 4, w * 2, h * 2);
    getConstrainer()->setFixedAspectRatio ((double) w / (double) h);

    const double scale = juce::jlimit (0.75, 2.0, (double) p.apvts.state.getProperty (editorScaleId, 1.0));
    setSize (juce::roundToInt (w * scale), juce::roundToInt (h * scale));
    startTimerHz (30);
}

Urei1176Editor::~Urei1176Editor()
{
    setLookAndFeel (nullptr);
}

void Urei1176Editor::paint (juce::Graphics& g)
{
    g.fillAll (Theme::background);
}

void Urei1176Editor::resized()
{
    const float scale = (float) getWidth() / (float) EditorView::kWidth;
    view.setTransform (juce::AffineTransform::scale (scale));
    view.setBounds (0, 0, EditorView::kWidth, EditorView::kHeight);

    // Remember a size the user chose (a window on screen), not the ones a host passes through
    // while opening or closing it.
    if (isShowing() && getPeer() != nullptr)
        owner.apvts.state.setProperty (editorScaleId, (double) scale, nullptr);
}

void Urei1176Editor::timerCallback()
{
    view.updateMeters();

    auto* under = juce::Desktop::getInstance().getMainMouseSource().getComponentUnderMouse();
    view.showHelpFor (under != nullptr && view.isParentOf (under) ? under : nullptr);
}
