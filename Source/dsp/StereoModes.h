#pragma once

// The plug-in's stereo modes around the fast engine: Dual Mono (two independent units), Link (the 1176SA tie of the two units' control
// nodes, FastChannel::processLinked) and Mid-Side (the units see M and S). No JUCE dependency, so the offline tests use it as the
// plug-in does. See docs/research/urei-1176.md, "Stereo link".

#include "Fast1176.h"

namespace urei1176
{
/** In the order of the plug-in's `stereo` parameter: append, never reorder. */
enum class StereoMode { dual = 0, link = 1, midSide = 2 };

/** Runs a stereo pair of FastChannels in a StereoMode. Nothing here allocates or locks.

    Mid-Side is a matrix around the two units: M = (L + R) / 2 and S = (L - R) / 2 in, L = M + S and R = M - S out. Moving into or out of
    it, the matrix is morphed from the identity over about 10 ms (both the encoder and the decoder, per sample), so the units' signals
    never jump from L / R to M / S and the output never jumps back; the units' own states continue throughout. Link engages and releases
    itself (FastChannel::processLinked), also without a step. */
class StereoRunner
{
public:
    void prepare (double sampleRate) noexcept
    {
        step = 1.0 / (0.010 * sampleRate);
        blend = 0.0;
    }
    /** Takes the mode at once, with no morph (for a host that sets the mode before it starts playing). */
    void snapTo (StereoMode mode) noexcept { blend = mode == StereoMode::midSide ? 1.0 : 0.0; }

    /** 0 in Dual and Link, 1 in a settled Mid-Side, between while the matrix morphs. */
    double midSideBlend() const noexcept { return blend; }

    void process (FastChannel& a, FastChannel& b, float* left, float* right, int numSamples, StereoMode mode) noexcept
    {
        const double target = mode == StereoMode::midSide ? 1.0 : 0.0;
        const bool linked = mode == StereoMode::link;
        if (blend == 0.0 && target == 0.0)
        {
            run (a, b, left, right, numSamples, linked);
            return;
        }
        const double start = blend;
        const double dir = target > start ? 1.0 : (target < start ? -1.0 : 0.0);
        const auto at = [&] (int i) { return clampToward (start + dir * step * (double) (i + 1), target, dir); };
        for (int i = 0; i < numSamples; ++i)
        {
            const double m = at (i);
            const double l = left[i], r = right[i];
            left[i] = (float) (l * (1.0 - 0.5 * m) + r * (0.5 * m));
            right[i] = (float) ((1.0 - m) * r + m * 0.5 * (l - r));
        }
        run (a, b, left, right, numSamples, linked);
        for (int i = 0; i < numSamples; ++i)
        {
            const double m = at (i);
            const double y0 = left[i], y1 = right[i];
            left[i] = (float) (y0 + m * y1);
            right[i] = (float) (y1 * (1.0 - 2.0 * m) + m * y0);
        }
        blend = at (numSamples - 1);
    }

private:
    static double clampToward (double v, double target, double dir) noexcept
    {
        return dir > 0.0 ? (v > target ? target : v) : (dir < 0.0 ? (v < target ? target : v) : target);
    }
    static void run (FastChannel& a, FastChannel& b, float* l, float* r, int n, bool linked) noexcept
    {
        if (linked) FastChannel::processLinked (a, b, l, r, n);
        else FastChannel::processPair (a, b, l, r, n);
    }

    double step = 0.0, blend = 0.0;
};
} // namespace urei1176
