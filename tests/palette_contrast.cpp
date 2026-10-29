// WCAG contrast of the editor's two palettes (Source/Palette.h), checked against the surface each colour is drawn on. No JUCE.
//
//   text                       >= 4.5 : 1 on background (and on panel)
//   label, label at 70 %       >= 3   : 1 on background and on panel (70 % is the unselected-option alpha the editor really uses)
//   pale, accent, clipping     >= 3   : 1 on background and on panel; pale also on field (the value editor's text)
//   ink on the VU face, and the 70 % ink of its arc and legend, red zone on the face >= 3 : 1
//
// Prints the table that docs/ui-design.md quotes. Exits 1 if a rule fails.

#include "Palette.h"

#include <cmath>
#include <cstdio>

namespace
{
struct Rgb { double r, g, b; };

Rgb rgbOf (uint32_t argb) { return { (double) ((argb >> 16) & 0xff), (double) ((argb >> 8) & 0xff), (double) (argb & 0xff) }; }

/** `fg` at `alpha` over the opaque `bg`, as the editor draws it. */
Rgb over (uint32_t fg, double alpha, uint32_t bg)
{
    const auto f = rgbOf (fg), b = rgbOf (bg);
    return { f.r * alpha + b.r * (1.0 - alpha), f.g * alpha + b.g * (1.0 - alpha), f.b * alpha + b.b * (1.0 - alpha) };
}

double linear (double c)
{
    c /= 255.0;
    return c <= 0.03928 ? c / 12.92 : std::pow ((c + 0.055) / 1.055, 2.4);
}

double luminance (Rgb c) { return 0.2126 * linear (c.r) + 0.7152 * linear (c.g) + 0.0722 * linear (c.b); }

double ratio (Rgb a, Rgb b)
{
    double x = luminance (a), y = luminance (b);
    if (x < y) { const double t = x; x = y; y = t; }
    return (x + 0.05) / (y + 0.05);
}

int failures = 0;

void check (const char* palette, const char* what, const char* surface, Rgb fg, uint32_t bg, double minimum)
{
    const double r = ratio (fg, rgbOf (bg));
    const bool ok = r >= minimum;
    std::printf ("%s  %-12s %-17s on %-10s %6.2f  (>= %.1f)\n", ok ? "ok  " : "FAIL", palette, what, surface, r, minimum);
    failures += ok ? 0 : 1;
}

void checkPalette (const urei1176::Palette& p)
{
    const auto solid = [] (uint32_t c) { return rgbOf (c); };

    check (p.name, "text", "background", solid (p.text), p.background, 4.5);
    check (p.name, "text", "panel", solid (p.text), p.panel, 4.5);
    check (p.name, "label", "background", solid (p.label), p.background, 3.0);
    check (p.name, "label", "panel", solid (p.label), p.panel, 3.0);
    check (p.name, "label at 70 %", "background", over (p.label, 0.7, p.background), p.background, 3.0);
    check (p.name, "label at 70 %", "panel", over (p.label, 0.7, p.panel), p.panel, 3.0);
    check (p.name, "pale", "background", solid (p.pale), p.background, 3.0);
    check (p.name, "pale", "panel", solid (p.pale), p.panel, 3.0);
    check (p.name, "pale", "field", solid (p.pale), p.field, 3.0);
    check (p.name, "accent", "background", solid (p.accent), p.background, 3.0);
    check (p.name, "accent", "panel", solid (p.accent), p.panel, 3.0);
    check (p.name, "clipping", "background", solid (p.clipping), p.background, 3.0);
    check (p.name, "clipping", "panel", solid (p.clipping), p.panel, 3.0);
    check (p.name, "disc hover", "background", solid (p.discHover), p.background, 3.0);
    check (p.name, "VU ink", "VU face", solid (p.vuInk), p.vuFace, 3.0);
    check (p.name, "VU ink at 70 %", "VU face", over (p.vuInk, 0.7, p.vuFace), p.vuFace, 3.0);
    check (p.name, "VU needle", "VU face", solid (p.vuNeedle), p.vuFace, 3.0);
    check (p.name, "red zone", "VU face", solid (p.redZone), p.vuFace, 3.0);
}
} // namespace

int main()
{
    checkPalette (urei1176::blackFace);
    checkPalette (urei1176::silverFace);
    std::printf (failures == 0 ? "all contrast checks passed\n" : "%d contrast check(s) failed\n", failures);
    return failures == 0 ? 0 : 1;
}
