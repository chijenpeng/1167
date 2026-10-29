#include "PluginEditor.h"

#include <BinaryData.h>

namespace
{
    // The analog-recreation / drum-gate language: flat, no borders, on a dark or a light face. The discs
    // are the things you turn; the pale cue colour (or white) is what is selected; the quieter label
    // colour names things. The colours come from the palette of the selected version (Palette.h), per
    // editor instance: `col (component, paletteLabel)` reads what the editor set on itself or on its
    // view. `Theme` below holds only the fonts and text measuring.
    juce::Colour col (const juce::Component& c, int id) { return c.findColour (id, true); }

    /** An option's ink: the selected one in `pale`, the hovered one in `text`, the others in `label` at 70 %. */
    juce::Colour optionInk (const juce::Component& c, bool selected, bool hovered)
    {
        return col (c, selected ? palettePale : (hovered ? paletteText : paletteLabel)).withMultipliedAlpha (selected || hovered ? 1.0f : 0.7f);
    }

    namespace Theme
    {
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
    constexpr int kOptionGap = 10, kLabelGap = 18, kRowHeight = 18, kRowGap = 2;

    const juce::Identifier helpId ("help"), helpTitleId ("helpTitle"), editorScaleId ("editorScale"), dimmedId ("dimmed");

    constexpr auto kDefaultHelpTitle = "1167";
    constexpr auto kHoverHint = "Hover over a control to see what it does; double-click a knob for its default, or its value to type one.";

    void setHelp (juce::Component& c, const juce::String& title, const juce::String& help)
    {
        c.getProperties().set (helpTitleId, title.toUpperCase());
        c.getProperties().set (helpId, help);
    }

    juce::String minusSign() { return juce::String::fromUTF8 ("\xE2\x88\x92"); }
    juce::String dot() { return juce::String::fromUTF8 (" \xC2\xB7 "); }

    /** The spec of a Version choice index (F G J; clamped). */
    const urei1176::VersionSpec& specAt (int index) { return urei1176::specFor (versionForChoice (index)); }
    bool isSilver (const urei1176::Palette& p) { return &p == &urei1176::silverFace; }

    /** Each version writes its own copy. */
    juce::String defaultHelpFor (int index)
    {
        const auto& spec = specAt (index);
        if (spec.version == urei1176::Version::F)
            return "1167 is an independent model inspired by the UREI 1176LN revision F (blackface). In stereo it runs "
                   "two units: independent, tied as with the 1176SA adapter, or on mid and side. " + juce::String (kHoverHint);
        if (spec.version == urei1176::Version::G)
            return "1167 is an independent model inspired by the UREI 1176LN revision G (blackface). G has a 5532 bridging input into a 10 k Input pot, a regulated "
                   "30.3 V supply and an output transformer loaded by R84. Input and output are trimmed so that its threshold and loudness match F's at the same dials "
                   "(a plug-in choice, not circuit). " + juce::String (kHoverHint);
        // J: the last of the choices (specAt gives F, G or J)
        return "1167 is an independent model inspired by the UREI 1176LN revision J (silverface), from the sheet titled H as revised through J: G's 5532 input, "
               "R33 and C13 as an AC ground, transformer 163-12614 (data assumed). Levels trimmed to F's (a plug-in choice); colours chosen for the plug-in. " + juce::String (kHoverHint);
    }

    juce::String versionRowHelp()
    {
        return "Which 1176LN revision to model: F, G or J. F is the revision this plug-in has modelled from the start; G and J are modelled too, each with its own "
               "circuit values. The colours are chosen for the plug-in after the hardware: black face for F and G, silver face for J.";
    }
}

namespace
{
    template <typename Target>
    void setPaletteColours (Target& target, const urei1176::Palette& p)
    {
        const std::pair<int, uint32_t> colours[] {
            { paletteBackground, p.background }, { paletteField, p.field }, { paletteLabel, p.label },
            { palettePale, p.pale }, { paletteDiscHover, p.discHover }, { paletteText, p.text }, { paletteAccent, p.accent },
            { paletteVuFace, p.vuFace }, { paletteVuHood, p.vuHood }, { paletteVuInk, p.vuInk },
            { paletteVuNeedle, p.vuNeedle }, { paletteRedZone, p.redZone }
        };
        for (const auto& [id, argb] : colours)
            target.setColour (id, juce::Colour (argb));
    }
}

void applyPalette (juce::Component& target, const urei1176::Palette& p)   { setPaletteColours (target, p); }
void applyPalette (juce::LookAndFeel& target, const urei1176::Palette& p) { setPaletteColours (target, p); }

const urei1176::Palette& paletteFor (const urei1176::VersionSpec& spec) noexcept
{
    return spec.version == urei1176::Version::J && ! spec.placeholderOfF ? urei1176::silverFace : urei1176::blackFace;
}

//==============================================================================
EditorFonts::EditorFonts()
    : inter (juce::Typeface::createSystemTypefaceFor (BinaryData::InterSemiBold_ttf, BinaryData::InterSemiBold_ttfSize)),
      mono (juce::Typeface::createSystemTypefaceFor (BinaryData::JetBrainsMonoMedium_ttf, BinaryData::JetBrainsMonoMedium_ttfSize))
{
}

//==============================================================================
FlatLookAndFeel::FlatLookAndFeel()
{
    // The resizer has no component to ask, so it reads this look-and-feel, which the editor keeps in step with its palette.
    applyPalette (*this, urei1176::blackFace);
}

void FlatLookAndFeel::drawRotarySlider (juce::Graphics& g, int x, int y, int width, int height, float position,
                                        float startAngle, float endAngle, juce::Slider& slider)
{
    const auto bounds = juce::Rectangle<int> (x, y, width, height).toFloat();
    const auto centre = bounds.getCentre();
    const float radius = juce::jmin (bounds.getWidth(), bounds.getHeight()) * 0.5f - 1.0f;
    const float angle = startAngle + position * (endAngle - startAngle);
    const float dot = juce::jmax (2.5f, radius * 0.11f);

    // A knob whose parameter has no effect at the moment (Attack while the limiter is switched off) is drawn at 45 %: it still turns.
    const float alpha = slider.getProperties().getWithDefault (dimmedId, false) ? 0.45f : 1.0f;
    g.setColour (col (slider, slider.isMouseOverOrDragging() ? paletteDiscHover : palettePale).withMultipliedAlpha (alpha));
    g.fillEllipse (juce::Rectangle<float> (radius * 2.0f, radius * 2.0f).withCentre (centre));

    g.setColour (col (slider, paletteBackground));
    g.fillEllipse (juce::Rectangle<float> (dot * 2.0f, dot * 2.0f).withCentre (centre.getPointOnCircumference (radius * 0.72f, angle)));
}

void FlatLookAndFeel::drawCornerResizer (juce::Graphics& g, int w, int h, bool isMouseOver, bool isMouseDragging)
{
    // Three short diagonals in the label grey, quiet until touched.
    g.setColour (findColour (paletteLabel).withAlpha (isMouseOver || isMouseDragging ? 0.8f : 0.3f));
    const float size = (float) juce::jmin (w, h);
    for (float i = 0.3f; i < 1.0f; i += 0.3f)
        g.drawLine ((float) w - size * i, (float) h - 2.0f, (float) w - 2.0f, (float) h - size * i, 1.0f);
}

//==============================================================================
Knob::Knob (juce::AudioProcessorValueTreeState& state, const char* paramId, const juce::String& nameToUse, const juce::String& help,
            int diameterToUse, float sweepDegrees)
    : Knob (state, paramId, nameToUse, help, diameterToUse, sweepDegrees, false)
{
}

std::unique_ptr<Knob> Knob::makeAttackKnob (juce::AudioProcessorValueTreeState& state, const juce::String& help, int diameter)
{
    return std::unique_ptr<Knob> (new Knob (state, ParamID::attack, "Attack", help, diameter, 270.0f, true));
}

Knob::Knob (juce::AudioProcessorValueTreeState& state, const char* paramId, const juce::String& nameToUse, const juce::String& help,
            int diameterToUse, float sweepDegrees, bool dialOnlyToUse)
    : name (nameToUse.toUpperCase()), diameter (diameterToUse), dialOnly (dialOnlyToUse)
{
    const float halfSweep = juce::degreesToRadians (sweepDegrees * 0.5f), twoPi = juce::MathConstants<float>::twoPi;
    slider.setRotaryParameters (twoPi - halfSweep, twoPi + halfSweep, true);
    slider.setVelocityModeParameters (0.15, 1, 0.0, true, juce::ModifierKeys::shiftModifier);   // Shift: fine

    if (dialOnly)
    {
        // The dial's own range, 1 to 7: the parameter's OFF stretch below it is not on this knob.
        auto& param = *state.getParameter (paramId);
        slider.setRange (urei1176::Range::attackMin, urei1176::Range::attackMax, 0.0);
        slider.setDoubleClickReturnValue (true, Default::attack);
        slider.textFromValueFunction = [] (double v) { return ParamText::attack ((float) v); };
        slider.valueFromTextFunction = [] (const juce::String& t)
        {
            return (double) juce::jlimit (urei1176::Range::attackMin, urei1176::Range::attackMax, t.getFloatValue());
        };
        dialAttachment = std::make_unique<juce::ParameterAttachment> (param, [this] (float v)
        {
            slider.setValue (juce::jmax (urei1176::Range::attackMin, v), juce::dontSendNotification);
            repaint();
        });
        slider.onDragStart = [this] { dragging = true; dialAttachment->beginGesture(); };
        slider.onDragEnd = [this] { dragging = false; dialAttachment->endGesture(); };
        slider.onValueChange = [this]
        {
            if (dragging) dialAttachment->setValueAsPartOfGesture ((float) slider.getValue());
            else dialAttachment->setValueAsCompleteGesture ((float) slider.getValue());
            repaint();
        };
        dialAttachment->sendInitialUpdate();
    }
    else
    {
        attachment = std::make_unique<juce::SliderParameterAttachment> (*state.getParameter (paramId), slider);
        slider.onValueChange = [this] { repaint(); };
    }
    addAndMakeVisible (slider);

    editor.setJustification (juce::Justification::centred);
    editor.setFont (Theme::valueFont (12.0f));
    editor.setIndents (0, 0);
    editor.setColour (juce::TextEditor::outlineColourId, juce::Colours::transparentBlack);
    editor.setColour (juce::TextEditor::focusedOutlineColourId, juce::Colours::transparentBlack);
    editor.onReturnKey = [this] { finishEditing (true); };
    editor.onEscapeKey = [this] { finishEditing (false); };
    editor.onFocusLost = [this] { finishEditing (true); };
    addChildComponent (editor);

    setHelp (*this, name, help);
}

void Knob::refreshColours()
{
    editor.setColour (juce::TextEditor::backgroundColourId, col (*this, paletteField));
    editor.setColour (juce::TextEditor::textColourId, col (*this, palettePale));
    editor.setColour (juce::TextEditor::highlightColourId, col (*this, paletteAccent).withAlpha (0.5f));
}

void Knob::setDimmed (bool shouldDim)
{
    if (dimmed == shouldDim)
        return;
    dimmed = shouldDim;
    slider.getProperties().set (dimmedId, dimmed);
    slider.repaint();
    repaint();
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

    const auto textColour = col (*this, paletteLabel).withMultipliedAlpha (dimmed ? 0.6f : 1.0f);
    g.setColour (textColour);
    g.setFont (Theme::labelFont());
    g.drawText (name, area.removeFromTop (kLabelHeight), juce::Justification::centred, false);

    // JetBrains Mono's infinity is tiny at this size; Inter's matches the figures.
    const auto value = slider.getTextFromValue (slider.getValue());
    const bool infinity = value == juce::String::fromUTF8 (urei1176::kDialLabels[0].text);
    g.setFont (infinity ? Theme::bodyFont (14.0f) : Theme::valueFont (12.0f));
    g.setColour (textColour);
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
    const bool meaningful = text.containsAnyOf ("0123456789")
                            || (! dialOnly && (text.startsWith ("off") || text.startsWith ("inf")
                                               || text.startsWith (juce::String::fromUTF8 (urei1176::kDialLabels[0].text))));
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
    g.setColour (col (*this, paletteLabel));
    g.drawText (label, 0, 0, labelColumn, getHeight(), juce::Justification::centredLeft, false);

    const auto ranges = optionRanges();
    for (int i = 0; i < options.size(); ++i)
    {
        const auto range = ranges[(size_t) i];
        g.setColour (optionInk (*this, i == selected, i == hovered));
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
AttackOffSwitch::AttackOffSwitch (juce::AudioProcessorValueTreeState& state)
    : offAttachment (*state.getParameter (ParamID::attackOff), [this] (float v) { switchOn = v >= 0.5f; changed(); }),
      // the attack parameter's own OFF stretch (old sessions, automation) lights the switch too
      attackAttachment (*state.getParameter (ParamID::attack), [this] (float v) { inOffSpan = AttackLaw::isOff (v); changed(); })
{
    offAttachment.sendInitialUpdate();
    attackAttachment.sendInitialUpdate();
    setHelp (*this, "Attack off",
             "Switches limiting off: the side chain is cut and the unit does not limit, while the audio still runs through its amplifiers and "
             "transformers. On the hardware this is the Attack knob's OFF position; here the knob keeps its setting and comes back when you "
             "click this again.");
}

void AttackOffSwitch::changed()
{
    repaint();
    if (wasLit != isLit())
    {
        wasLit = isLit();
        if (onLitChanged)
            onLitChanged (wasLit);
    }
}

int AttackOffSwitch::labelTextWidth() const
{
    return Theme::widthOf (Theme::labelFont(), "ATTACK");
}

juce::Rectangle<int> AttackOffSwitch::optionRange() const
{
    return { labelColumn, 0, Theme::widthOf (Theme::labelFont(), "OFF"), getHeight() };
}

int AttackOffSwitch::preferredWidth() const
{
    return optionRange().getRight();
}

void AttackOffSwitch::paint (juce::Graphics& g)
{
    g.setFont (Theme::labelFont());
    g.setColour (col (*this, paletteLabel));
    g.drawText ("ATTACK", 0, 0, labelColumn, getHeight(), juce::Justification::centredLeft, false);
    g.setColour (optionInk (*this, isLit(), hovered));
    g.drawText ("OFF", optionRange().withWidth (optionRange().getWidth() + 4), juce::Justification::centredLeft, false);
}

void AttackOffSwitch::mouseDown (const juce::MouseEvent& e)
{
    if (! optionRange().expanded (kOptionGap / 2, 0).contains (e.getPosition()))
        return;

    if (isLit())
    {
        // Release it: the switch, and the parameter's own OFF stretch, if that is what lit it (the pot goes back to 1).
        if (inOffSpan)
            attackAttachment.setValueAsCompleteGesture (urei1176::Range::attackMin);
        offAttachment.setValueAsCompleteGesture (0.0f);
    }
    else
        offAttachment.setValueAsCompleteGesture (1.0f);
}

void AttackOffSwitch::mouseMove (const juce::MouseEvent& e)
{
    const bool over = optionRange().expanded (kOptionGap / 2, 0).contains (e.getPosition());
    if (over != hovered)
    {
        hovered = over;
        repaint();
    }
}

void AttackOffSwitch::mouseExit (const juce::MouseEvent&)   { hovered = false; repaint(); }

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
             "falls to the left and over-reads fast peaks as the real meter does.");
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

void VuMeter::setReading (float newNeedleDb, urei1176::MeterMode newMode)
{
    const bool needleMoved = std::abs (angleFor (newNeedleDb) - angleFor (needleDb)) > 0.0005f;

    if (needleMoved || newMode != mode)
    {
        needleDb = newNeedleDb;
        mode = newMode;
        repaint();
    }
}

void VuMeter::paint (juce::Graphics& g)
{
    // The face, and at its foot the hood over the needle's pivot.
    const auto faceArea = getLocalBounds().toFloat();
    const auto hood = faceArea.withTop (faceArea.getBottom() - kHoodHeight);
    g.setColour (col (*this, paletteVuFace));
    g.fillRoundedRectangle (faceArea, kCorner);
    juce::Path hoodShape;
    hoodShape.addRoundedRectangle (hood.getX(), hood.getY(), hood.getWidth(), hood.getHeight(), kCorner, kCorner, false, false, true, true);
    g.setColour (col (*this, paletteVuHood));
    g.fillPath (hoodShape);

    // With the meter switched off the scale goes quiet, as an unlit meter does.
    const float scaleAlpha = mode == urei1176::MeterMode::off ? 0.45f : 1.0f;
    const auto centre = pivot();
    const float r = radius();

    // The arc: a thin line to 0, the red zone from 0 to +3.
    juce::Path line, red;
    line.addCentredArc (centre.x, centre.y, r, r, 0.0f, angleFor (urei1176::Range::meterMinDb), angleFor (0.0f), true);
    red.addCentredArc (centre.x, centre.y, r + 1.0f, r + 1.0f, 0.0f, angleFor (0.0f), angleFor (urei1176::Range::meterMaxDb), true);
    g.setColour (col (*this, paletteVuInk).withAlpha (0.7f * scaleAlpha));
    g.strokePath (line, juce::PathStrokeType (1.0f));
    g.setColour (col (*this, paletteRedZone).withMultipliedAlpha (scaleAlpha));
    g.strokePath (red, juce::PathStrokeType (3.0f, juce::PathStrokeType::mitered, juce::PathStrokeType::butt));

    // Marks and figures.
    g.setFont (Theme::valueFont (8.0f));
    for (const float mark : kMarks)
    {
        const float a = angleFor (mark);
        const bool plus = mark > 0.0f;
        g.setColour (col (*this, plus ? paletteRedZone : paletteVuInk).withMultipliedAlpha (scaleAlpha));
        g.drawLine ({ centre.getPointOnCircumference (r, a), centre.getPointOnCircumference (r + 5.0f, a) }, mark == 0.0f ? 1.5f : 1.0f);

        const auto text = mark < 0.0f ? minusSign() + juce::String ((int) -mark)
                                      : (plus ? "+" : "") + juce::String ((int) mark);
        const auto at = centre.getPointOnCircumference (r + 11.0f, a);
        g.drawText (text, juce::Rectangle<float> (22.0f, 10.0f).withCentre (at), juce::Justification::centred, false);
    }

    g.setColour (col (*this, paletteVuInk).withAlpha (0.7f * scaleAlpha));
    g.setFont (Theme::titleFont (10.0f));
    g.drawText ("VU", juce::Rectangle<float> (faceArea.getX(), centre.y - r + 35.0f, faceArea.getWidth(), 12.0f),
                juce::Justification::centred, false);

    // The needle, from its pivot under the hood.
    juce::Graphics::ScopedSaveState state (g);
    g.reduceClipRegion (faceArea.withBottom (hood.getY()).toNearestInt());
    g.setColour (col (*this, paletteVuNeedle));
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
      release (p.apvts, ParamID::release, "Release",
               "How quickly the gain recovers once the level drops: 7 (fully clockwise) is the fastest, 1 the slowest. Measured on this model, "
               "as the time to recover 63 % of a 6.7 dB reduction: 88 ms at 7, 1.28 s at 1 (UREI's manual gives 50 ms to 1.1 s).",
               64),
      calibration (p.apvts, ParamID::calibration, "0 dBFS",
                   "The level in dBu that 0 dBFS (a full-scale sine) stands for, at the input and at the output. The 1176's "
                   "threshold is fixed in dBu, so a higher setting drives it less for the same track level. Input and Output "
                   "at 24 stay at about unity gain whatever this is.",
                   36),
      attack (Knob::makeAttackKnob (p.apvts,
              "How quickly it clamps down: 7 (fully clockwise) is the fastest. On this model, 0.25 ms at 7 to 3.7 ms at 1, measured as the time to reach "
              "63 % of the reduction found 25 ms into a burst 10 dB over the threshold (UREI's manual: 20 to 800 microseconds). "
              "To switch limiting off use ATTACK OFF.",
              64)),
      ratioRow (*p.apvts.getParameter (ParamID::ratio), "Ratio", { "4", "8", "12", "20", "All" },
                "4 to 20: how hard it limits above the threshold, which rises with the ratio. ALL is every button pressed in at "
                "once: the bias shifts and, on GR, the needle pins to the right at idle, as on a real unit. Click the lit "
                "button to let it out: with no button in, the unit does not limit.",
                (int) urei1176::Ratio::none),
      meterRow (*p.apvts.getParameter (ParamID::meter), "Meter", { "Off", "GR", "+4", "+8" },
                "What the needle shows; it never changes the sound. GR: gain reduction, resting at 0 and falling to the left "
                "(" + minusSign() + "10 is 10 dB). +4 / +8: output level, 0 VU = +4 or +8 dBu. OFF lets the needle rest (on the "
                "hardware it switches the unit off). In stereo, GR shows the unit reducing more, +4 / +8 the louder one."),
      stereoRow (*p.apvts.getParameter (ParamID::stereo), "Stereo", { "Dual", "Link", "M/S" },
                 "DUAL: two independent units. LINK: the two units' control nodes are tied as in UREI's 1176SA adapter, so both channels "
                 "get the same gain reduction, set mostly by the louder one (derived from the circuit; the adapter's own values are not known). "
                 "M/S: the units see mid and side, M = (L + R) / 2 and S = (L " + minusSign() + " R) / 2, and the meter reads M and S. "
                 "Mono tracks ignore this."),
      sidechainRow (*p.apvts.getParameter (ParamID::sidechainFilter), "S/C HPF", { "OFF", "30", "60", "105", "125", "185" },
                    "A high-pass filter on the detector only; the audio you hear is not filtered. OFF, or 30 to 185 Hz (6 dB per octave): "
                    "the higher it is, the less low frequencies push the compressor. In M/S it acts on the mid and side detectors. "
                    "A plug-in addition, not on the original 1176."),
      versionRow (*p.apvts.getParameter (ParamID::model), "Rev", modelChoiceNames(), versionRowHelp()),
      attackOff (p.apvts),
      versionAttachment (*p.apvts.getParameter (ParamID::model), [this] (float v) { versionChanged (juce::roundToInt (v)); })
{
    const std::pair<juce::Component*, const char*> controls[] {
        { &input, ParamID::input }, { &output, ParamID::output }, { attack.get(), ParamID::attack }, { &release, ParamID::release },
        { &calibration, ParamID::calibration }, { &ratioRow, ParamID::ratio }, { &meterRow, ParamID::meter },
        { &stereoRow, ParamID::stereo }, { &sidechainRow, ParamID::sidechainFilter }, { &attackOff, ParamID::attackOff },
        { &versionRow, ParamID::model }
    };
    for (const auto& [component, id] : controls)
    {
        component->setComponentID (id);
        addAndMakeVisible (component);
    }
    addAndMakeVisible (vu);

    // Two columns (Ratio, Stereo and S/C HPF; Meter, Attack Off and Version); the rows of a column share one label column, so their options
    // start together.
    const int leftColumn = juce::jmax (ratioRow.labelTextWidth(), stereoRow.labelTextWidth(), sidechainRow.labelTextWidth()) + kLabelGap;
    const int rightColumn = juce::jmax (meterRow.labelTextWidth(), attackOff.labelTextWidth(), versionRow.labelTextWidth()) + kLabelGap;
    ratioRow.setLabelColumn (leftColumn);
    stereoRow.setLabelColumn (leftColumn);
    sidechainRow.setLabelColumn (leftColumn);
    meterRow.setLabelColumn (rightColumn);
    attackOff.setLabelColumn (rightColumn);
    versionRow.setLabelColumn (rightColumn);
    attackOff.onLitChanged = [this] (bool lit) { attack->setDimmed (lit); };
    attack->setDimmed (attackOff.isLit());

    versionAttachment.sendInitialUpdate();   // the palette, the subtitle and the default help of the saved version
    showHelpFor (nullptr);
    updateMeters();
    setSize (kWidth, kHeight);
}

void EditorView::versionChanged (int index)
{
    versionIndex = index;   // specAt clamps it
    palette = paletteOverride != nullptr ? paletteOverride : &paletteFor (specAt (versionIndex));

    applyPalette (*this, *palette);
    for (auto* knob : { &input, &output, &release, &calibration, attack.get() })
        knob->refreshColours();
    if (onPaletteChanged)
        onPaletteChanged();

    // The default help is the version's; a control under the mouse (a click on REV) keeps its own.
    auto* under = juce::Desktop::getInstance().getMainMouseSource().getComponentUnderMouse();
    showHelpFor (under != nullptr && isParentOf (under) ? under : nullptr);
    repaint();   // the subtitle may change while the colours do not
}

juce::String EditorView::subtitle() const
{
    const auto& spec = specAt (versionIndex);
    const auto letter = juce::String::charToString (spec.name);
    return "LN" + dot() + (isSilver (paletteFor (spec)) ? "SILVERFACE" : "BLACKFACE") + dot() + letter;
}

void EditorView::setPaletteOverride (const urei1176::Palette* p)
{
    paletteOverride = p;
    versionChanged (versionIndex);
}

juce::StringArray EditorView::controlsWithoutHelp() const
{
    juce::StringArray missing;
    for (auto* child : getChildren())
        if (child != &vu && (! child->getProperties().contains (helpId) || child->getProperties()[helpId].toString().isEmpty()))
            missing.add (child->getComponentID().isEmpty() ? "(unnamed)" : child->getComponentID());
    if (! vu.getProperties().contains (helpId))
        missing.add ("vu");
    return missing;
}

void EditorView::updateMeters()
{
    const auto mode = (urei1176::MeterMode) juce::jlimit (0, 3, juce::roundToInt (processor.apvts.getRawParameterValue (ParamID::meter)->load()));
    const auto meters = processor.getMeters();

    // One needle for both channels: on GR the one reducing more, on +4 / +8 the louder.
    float needle = meters.needleDb[0];
    for (int ch = 1; ch < meters.channels; ++ch)
    {
        needle = mode == urei1176::MeterMode::gainReduction ? juce::jmin (needle, meters.needleDb[(size_t) ch])
                                                             : juce::jmax (needle, meters.needleDb[(size_t) ch]);
    }

    vu.setReading (needle, mode);
}

void EditorView::showHelpFor (juce::Component* component)
{
    juce::String title = kDefaultHelpTitle, text = defaultHelpFor (versionIndex);

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
    g.fillAll (col (*this, paletteBackground));

    // Title: the model, and quieter, which one.
    const auto title = Theme::titleFont (15.0f), subtitle = Theme::titleFont (11.0f);
    const float baseline = 33.0f;
    g.setColour (col (*this, palettePale));
    g.setFont (title);
    g.drawSingleLineText ("1167", 24, (int) baseline);
    g.setColour (col (*this, paletteLabel));
    g.setFont (subtitle);
    g.drawSingleLineText (this->subtitle(), 24 + Theme::widthOf (title, "1167") + 12, (int) baseline);

    // Help, bottom left: what the control under the mouse does.
    auto help = helpArea;
    g.setColour (col (*this, palettePale));
    g.setFont (Theme::labelFont());
    g.drawText (helpTitle, help.removeFromTop (16), juce::Justification::topLeft, false);
    g.setColour (col (*this, paletteLabel));
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

    // Six option rows in two columns form one compact lane above the knob bank: Ratio, Stereo and S/C HPF; Meter, Attack Off and Version. To the right sit
    // a two-thirds size VU face and the calibration control; the help follows immediately below the unit.
    auto unit = area;
    auto knobArea = unit.removeFromLeft (4 * knobCell);

    const int leftWidth = juce::jmax (ratioRow.preferredWidth(), stereoRow.preferredWidth(), sidechainRow.preferredWidth());
    const int rightWidth = juce::jmax (meterRow.preferredWidth(), attackOff.preferredWidth(), versionRow.preferredWidth());
    const int laneWidth = leftWidth + laneGap + rightWidth;
    auto lane = juce::Rectangle<int> (knobArea.getCentreX() - laneWidth / 2, knobArea.getY(), laneWidth, 3 * kRowHeight + 2 * kRowGap);
    auto left = lane.removeFromLeft (leftWidth);
    lane.removeFromLeft (laneGap);
    auto right = lane;
    ratioRow.setBounds (left.removeFromTop (kRowHeight).withWidth (ratioRow.preferredWidth()));
    left.removeFromTop (kRowGap);
    stereoRow.setBounds (left.removeFromTop (kRowHeight).withWidth (stereoRow.preferredWidth()));
    left.removeFromTop (kRowGap);
    sidechainRow.setBounds (left.withWidth (sidechainRow.preferredWidth()));
    meterRow.setBounds (right.removeFromTop (kRowHeight).withWidth (meterRow.preferredWidth()));
    right.removeFromTop (kRowGap);
    attackOff.setBounds (right.removeFromTop (kRowHeight).withWidth (attackOff.preferredWidth()));
    right.removeFromTop (kRowGap);
    versionRow.setBounds (right.removeFromTop (kRowHeight).withWidth (versionRow.preferredWidth()));

    knobArea.removeFromTop (3 * kRowHeight + 2 * kRowGap + laneToKnobs);
    for (auto* knob : { &input, &output, attack.get(), &release })
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
    // The editor and its look-and-feel (the resize corner) take the view's palette too.
    view.onPaletteChanged = [this]
    {
        applyPalette (*this, view.currentPalette());
        applyPalette (lookAndFeel, view.currentPalette());
        repaint();
    };
    addAndMakeVisible (view);
    view.onPaletteChanged();

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
    g.fillAll (findColour (paletteBackground));
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
