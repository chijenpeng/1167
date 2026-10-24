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
    /** The Attack knob: the dial 1 to 7 only. Its parameter has a stretch below 1 that means OFF (AttackLaw), which the knob never enters:
        dragging, the default and typed values stay on the dial, and a parameter value inside that stretch shows as 1. */
    static std::unique_ptr<Knob> makeAttackKnob (juce::AudioProcessorValueTreeState&, const juce::String& help, int diameter);

    /** Draws the knob quieter (it still turns): its parameter has no effect at the moment. */
    void setDimmed (bool shouldDim);

    void paint (juce::Graphics&) override;
    void resized() override;
    void mouseDoubleClick (const juce::MouseEvent&) override;

    static constexpr int kLabelHeight = 16, kGap = 8;
    /** Label, disc and value. */
    int preferredHeight() const noexcept { return 2 * (kLabelHeight + kGap) + diameter; }

private:
    void finishEditing (bool apply);

    Knob (juce::AudioProcessorValueTreeState&, const char* paramId, const juce::String& name, const juce::String& help,
          int diameter, float sweepDegrees, bool dialOnly);

    juce::Slider slider { juce::Slider::RotaryHorizontalVerticalDrag, juce::Slider::NoTextBox };
    juce::String name;
    int diameter;
    bool dialOnly, dragging = false, dimmed = false;
    juce::TextEditor editor;
    std::unique_ptr<juce::SliderParameterAttachment> attachment;
    std::unique_ptr<juce::ParameterAttachment> dialAttachment;   // the dial-only knob writes the parameter itself
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

/** "ATTACK  OFF": the switch for the Attack Off parameter, in the option rows' grammar. OFF is lit while limiting is off, whether the switch is
    on or the Attack parameter sits in its OFF span (an old session or automation); clicking it then releases both. */
class AttackOffSwitch final : public juce::Component
{
public:
    explicit AttackOffSwitch (juce::AudioProcessorValueTreeState&);

    void paint (juce::Graphics&) override;
    void mouseDown (const juce::MouseEvent&) override;
    void mouseMove (const juce::MouseEvent&) override;
    void mouseExit (const juce::MouseEvent&) override;

    int labelTextWidth() const;
    void setLabelColumn (int width) { labelColumn = width; repaint(); }
    int preferredWidth() const;
    bool isLit() const noexcept { return switchOn || inOffSpan; }
    /** Called when lit changes (the editor dims the Attack knob). */
    std::function<void (bool)> onLitChanged;

private:
    juce::Rectangle<int> optionRange() const;
    void changed();

    int labelColumn = 0;
    bool switchOn = false, inOffSpan = false, hovered = false, wasLit = false;
    juce::ParameterAttachment offAttachment, attackAttachment;
};

/** The VU meter as a flat scale: what the needle shows depends on the METER row. */
class VuMeter final : public juce::Component
{
public:
    VuMeter();

    /** The needle (dB on the VU scale), the gain cell's reduction (dB, >= 0) for the readout,
        and the meter mode. Repaints only when something visible changed. */
    void setReading (float needleDb, float gainReductionDb, urei1176::MeterMode mode);
    /** Mid-side: the readout gives the two units' reductions, M and S, instead of one (the larger) figure. */
    void setMidSide (bool on, float midDb, float sideDb);
    void paint (juce::Graphics&) override;

private:
    juce::Point<float> pivot() const;
    float radius() const;
    /** Angle (radians, clockwise from 12 o'clock) of a VU reading. */
    static float angleFor (float db) noexcept;

    float needleDb = urei1176::Range::meterMinDb, grDb = 0.0f, midGrDb = 0.0f, sideGrDb = 0.0f;
    bool midSide = false;
    urei1176::MeterMode mode = urei1176::MeterMode::gainReduction;
};

/** Everything inside the window, laid out at the design size; the editor scales it. */
class EditorView final : public juce::Component
{
public:
    static constexpr int kWidth = 760, kHeight = 286;

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
    Knob input, output, release, calibration;
    std::unique_ptr<Knob> attack;
    OptionRow ratioRow, meterRow, stereoRow;
    AttackOffSwitch attackOff;
    VuMeter vu;

    float shownGrDb = 0.0f, shownMidDb = 0.0f, shownSideDb = 0.0f;
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
