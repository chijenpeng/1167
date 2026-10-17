#pragma once

#include "PluginProcessor.h"

/** The embedded typefaces. Shared by all open editors and released with the last one, so
    nothing outlives the host's JUCE shutdown. */
struct EditorFonts
{
    EditorFonts();
    juce::Typeface::Ptr inter, mono;
};

/** Flat drawing for the knobs and the resize corner; everything else paints itself. */
class FlatLookAndFeel final : public juce::LookAndFeel_V4
{
public:
    void drawRotarySlider (juce::Graphics&, int x, int y, int width, int height, float position,
                           float startAngle, float endAngle, juce::Slider&) override;
    void drawCornerResizer (juce::Graphics&, int w, int h, bool isMouseOver, bool isMouseDragging) override;
};

/** A flat disc with a dot, its name above and its value below. Double-click the disc for the
    default; double-click the value to type one. */
class Knob final : public juce::Component
{
public:
    /** `sweepDegrees`: the rotation from end to end, centred on 12 o'clock. */
    Knob (juce::AudioProcessorValueTreeState&, const char* paramId, const juce::String& name, const juce::String& help,
          int diameter, float sweepDegrees = 270.0f);

    void paint (juce::Graphics&) override;
    void resized() override;
    void mouseDoubleClick (const juce::MouseEvent&) override;

    static constexpr int kLabelHeight = 16, kGap = 8;
    /** Label, disc and value. */
    int preferredHeight() const noexcept { return 2 * (kLabelHeight + kGap) + diameter; }

private:
    void finishEditing (bool apply);

    juce::Slider slider { juce::Slider::RotaryHorizontalVerticalDrag, juce::Slider::NoTextBox };
    juce::String name;
    int diameter;
    juce::TextEditor editor;
    juce::SliderParameterAttachment attachment;
};

/** "LABEL  OPTION  OPTION ..." as plain text; the selected option is lit. With `releasedIndex`
    set, clicking the lit option selects that choice instead, which is not shown: a push button
    let out, leaving none lit. */
class OptionRow final : public juce::Component
{
public:
    OptionRow (juce::RangedAudioParameter&, const juce::String& label, juce::StringArray options,
               const juce::String& help, int releasedIndex = -1);

    void paint (juce::Graphics&) override;
    void mouseDown (const juce::MouseEvent&) override;
    void mouseMove (const juce::MouseEvent&) override;
    void mouseExit (const juce::MouseEvent&) override;

    /** The label's own width; rows stacked together share the widest plus the label gap. */
    int labelTextWidth() const;
    void setLabelColumn (int width) { labelColumn = width; repaint(); }
    int preferredWidth() const;

private:
    std::vector<juce::Range<int>> optionRanges() const;
    int optionAt (int x) const;

    juce::String label;
    juce::StringArray options;
    int releasedIndex, labelColumn = 0, selected = 0, hovered = -1;
    juce::ParameterAttachment attachment;
};

/** The VU meter as a flat scale: what the needle shows depends on the METER row. */
class VuMeter final : public juce::Component
{
public:
    VuMeter();

    /** The needle (dB on the VU scale), the gain cell's reduction (dB, >= 0) for the readout,
        and the meter mode. Repaints only when something visible changed. */
    void setReading (float needleDb, float gainReductionDb, urei1176::MeterMode mode);
    void paint (juce::Graphics&) override;

private:
    juce::Point<float> pivot() const;
    float radius() const;
    /** Angle (radians, clockwise from 12 o'clock) of a VU reading. */
    static float angleFor (float db) noexcept;

    float needleDb = urei1176::Range::meterMinDb, grDb = 0.0f;
    urei1176::MeterMode mode = urei1176::MeterMode::gainReduction;
};

/** Everything inside the window, laid out at the design size; the editor scales it. */
class EditorView final : public juce::Component
{
public:
    static constexpr int kWidth = 760, kHeight = 270;

    explicit EditorView (Urei1176Processor&);

    void paint (juce::Graphics&) override;
    void resized() override;

    /** Reads the processor's meters into the VU meter. */
    void updateMeters();
    /** Shows the help of the control under the mouse (nullptr: the default text). */
    void showHelpFor (juce::Component*);
    /** Shows the help of the control for this parameter ID. */
    void showHelpForParameter (const juce::String& paramId);

private:
    Urei1176Processor& processor;
    Knob input, output, attack, release, calibration;
    OptionRow ratioRow, meterRow;
    VuMeter vu;

    float shownGrDb = 0.0f;
    juce::String helpTitle, helpText;
    juce::Rectangle<int> helpArea;
};

class Urei1176Editor final : public juce::AudioProcessorEditor,
                             private juce::Timer
{
public:
    explicit Urei1176Editor (Urei1176Processor&);
    ~Urei1176Editor() override;

    void paint (juce::Graphics&) override;
    void resized() override;

    /** For the offline snapshot tool, which has no host message loop or mouse. */
    void updateMeters() { view.updateMeters(); }
    void showHelpForParameter (const juce::String& paramId) { view.showHelpForParameter (paramId); }

private:
    void timerCallback() override;

    Urei1176Processor& owner;
    juce::SharedResourcePointer<EditorFonts> fonts;
    FlatLookAndFeel lookAndFeel;
    EditorView view;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Urei1176Editor)
};
