#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <vector>

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

inline double pow (double x, double y) noexcept { return x <= 0.0 ? 0.0 : exp (y * log (x)); }

/** softplus and its derivative (the logistic sigmoid), exactly. */
inline void softplusSigmoidExact (double x, double& sp, double& sg) noexcept
{
    if (x > 30.0)  { sp = x; sg = 1.0; return; }
    if (x < -14.0) { sp = sg = exp (x); return; }
    const double e = exp (x), d = 1.0 + e;
    sp = log (d);
    sg = e / d;
}

/** The same from a table with linear interpolation (64 points per unit on [-20, 40]): the
    tube models call this several times per Newton iteration and it is their main cost. */
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

/** x^y for a fixed exponent y (the Koren models' ex - 1), tabulated: fine steps below 4 where
    the curvature is, coarser above. Exact below 0.02 and above 64. */
class PowTable
{
public:
    static const PowTable& get (double exponent)
    {
        static std::mutex mutex;
        static std::vector<std::unique_ptr<PowTable>> tables;
        std::lock_guard<std::mutex> lock (mutex);
        for (const auto& t : tables)
            if (t->y == exponent)
                return *t;
        tables.push_back (std::unique_ptr<PowTable> (new PowTable (exponent)));
        return *tables.back();
    }
    double eval (double x) const noexcept
    {
        if (x < 0.02 || x >= 64.0)
            return pow (x, y);
        if (x < 4.0)
        {
            const double f = x * 256.0;
            const int i = (int) f;
            const double t = f - (double) i;
            return fine[i] + t * (fine[i + 1] - fine[i]);
        }
        const double f = (x - 4.0) * 32.0;
        const int i = (int) f;
        const double t = f - (double) i;
        return coarse[i] + t * (coarse[i + 1] - coarse[i]);
    }
    double exponent() const noexcept { return y; }
private:
    explicit PowTable (double exponent) : y (exponent)
    {
        for (int i = 0; i < 4 * 256 + 2; ++i) fine.push_back (std::pow ((double) i / 256.0, y));
        for (int i = 0; i < 60 * 32 + 2; ++i) coarse.push_back (std::pow (4.0 + (double) i / 32.0, y));
    }
    double y;
    std::vector<double> fine, coarse;
};

inline double softplus (double x) noexcept { double sp, sg; softplusSigmoid (x, sp, sg); return sp; }
inline double sigmoid (double x) noexcept  { double sp, sg; softplusSigmoid (x, sp, sg); return sg; }

/** atan(x) to ~1e-6 absolute, for the pentode plate-knee term. */
inline double atan (double x) noexcept
{
    const bool neg = x < 0.0;
    if (neg) x = -x;
    const bool inv = x > 1.0;
    if (inv) x = 1.0 / x;
    const double x2 = x * x;
    double r = x * (0.99997726 + x2 * (-0.33262347 + x2 * (0.19354346 + x2 * (-0.11643287 + x2 * (0.05265332 - x2 * 0.01172120)))));
    if (inv) r = 1.5707963267948966 - r;
    return neg ? -r : r;
}
} // namespace ar::circuit::fast
