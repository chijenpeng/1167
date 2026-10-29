#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

namespace ar::circuit::fast
{
/** exp(x) to ~2e-7 relative: 2^n from the exponent bits, 2^f from a polynomial. */
inline double exp (double x) noexcept
{
    if (x > 700.0) x = 700.0;
    if (x < -700.0) return 0.0;
    const double y = x * 1.4426950408889634;             // log2(e)
    const double n = std::floor (y);
    const double f = y - n;                                // 0..1
    // 2^f, minimax degree 6 on [0,1]
    const double p = 1.0 + f * (0.6931471805599453 + f * (0.2402265069591007 + f * (0.0555041086648216
                     + f * (0.0096181291076285 + f * (0.0013333558146428 + f * 0.0001540353039338)))));
    const int64_t bits = ((int64_t) n + 1023) << 52;
    double scale;
    std::memcpy (&scale, &bits, sizeof (scale));
    return p * scale;
}

/** log(x) for x > 0 to ~1e-9 relative: mantissa via atanh series, exponent from the bits. */
inline double log (double x) noexcept
{
    int64_t bits;
    std::memcpy (&bits, &x, sizeof (bits));
    int e = (int) ((bits >> 52) & 0x7ff) - 1022;          // x = m * 2^e with m in [0.5, 1)
    bits = (bits & 0x000fffffffffffffLL) | (1022LL << 52);
    double m;
    std::memcpy (&m, &bits, sizeof (m));
    if (m < 0.7071067811865476) { m *= 2.0; --e; }         // m in [0.707, 1.414)
    const double t = (m - 1.0) / (m + 1.0), t2 = t * t;
    const double s = 2.0 * t * (1.0 + t2 * (1.0 / 3.0 + t2 * (1.0 / 5.0 + t2 * (1.0 / 7.0 + t2 * (1.0 / 9.0 + t2 * (1.0 / 11.0 + t2 / 13.0))))));
    return s + (double) e * 0.6931471805599453;
}

/** softplus and its derivative (the logistic sigmoid), exactly. */
inline void softplusSigmoidExact (double x, double& sp, double& sg) noexcept
{
    if (x > 30.0)  { sp = x; sg = 1.0; return; }
    if (x < -14.0) { sp = sg = exp (x); return; }
    const double e = exp (x), d = 1.0 + e;
    sp = log (d);
    sg = e / d;
}

/** The same from a table with linear interpolation (64 points per unit on [-20, 40]): the JFET
    models (JfetDevice, the fast engine's Q1) and the fast engine's rectifier call it in their Newton iterations. */
struct SoftplusTable
{
    static constexpr double lo = -20.0, hi = 40.0;
    static constexpr int perUnit = 64;
    static constexpr int count = (int) ((hi - lo) * perUnit) + 2;
    double sp[count], sg[count];
    SoftplusTable() noexcept
    {
        for (int i = 0; i < count; ++i)
            softplusSigmoidExact (lo + (double) i / perUnit, sp[i], sg[i]);
    }
};
inline const SoftplusTable& softplusTable() noexcept { static const SoftplusTable t; return t; }

inline void softplusSigmoid (double x, double& sp, double& sg) noexcept
{
    if (x >= SoftplusTable::hi) { sp = x; sg = 1.0; return; }
    if (x <= SoftplusTable::lo) { sp = sg = exp (x); return; }
    const auto& T = softplusTable();
    const double f = (x - SoftplusTable::lo) * SoftplusTable::perUnit;
    const int i = (int) f;
    const double t = f - (double) i;
    sp = T.sp[i] + t * (T.sp[i + 1] - T.sp[i]);
    sg = T.sg[i] + t * (T.sg[i + 1] - T.sg[i]);
}

} // namespace ar::circuit::fast
