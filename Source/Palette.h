#pragma once

#include <cstdint>

// The editor's colours, one palette per face of the hardware the selected version follows. No JUCE here, so tests/palette_contrast.cpp can
// compute the contrast ratios offline. Every value is chosen for the plug-in after the colour language of the hardware (docs/research/
// urei-1176-revisions.md, section 5.4: averaged from dealer photographs, then adjusted to hold the contrast rules of docs/ui-design.md);
// none is measured, and no logo or printed legend of the hardware is drawn.
namespace urei1176
{
/** 0xAARRGGBB, alpha opaque. */
struct Palette
{
    const char* name;
    uint32_t background, panel, field;
    uint32_t label;        // control names, the help body, unselected options (at 70 %)
    uint32_t pale;         // knob discs, selected options, the title, the help title
    uint32_t discHover;    // a disc under the mouse ("brighter" does nothing on a near-black disc, so it is its own token)
    uint32_t text;         // hovered options
    uint32_t accent;       // the value editor's selection
    uint32_t clipping;     // reserved for errors; the editor draws no error state today, so only the contrast test reads it
    // the VU meter: a light face with a dark hood, ink for the scale and needle
    uint32_t vuFace, vuHood, vuInk, vuNeedle, redZone;
};

/** F and G (and D, not exposed, which plays F): near-black panel, white legends (label is slightly dimmed), silver-capped knobs, cream lamp-lit VU. */
inline constexpr Palette blackFace {
    "black face",
    0xff1f2125, 0xff131416, 0xff363940,
    0xffc9c7c1, 0xffdcdedd, 0xffeef0ef, 0xffffffff, 0xffd8483a, 0xffff6a5a,
    0xffd5bc90, 0xff131416, 0xff1c1a14, 0xff1c1a14, 0xffb02a1d
};

/** J: silver face (a light theme), black knobs, red OFF, amber back-lit VU. */
inline constexpr Palette silverFace {
    "silver face",
    0xffbfc1be, 0xffd2d3d1, 0xffa9abab,
    0xff3a3b3b, 0xff1b1c1d, 0xff3c3d3e, 0xff101112, 0xffa6281b, 0xffa6281b,
    0xffb68f3d, 0xff1b1c1d, 0xff1c1a14, 0xff1c1a14, 0xff7a1a10
};
} // namespace urei1176
