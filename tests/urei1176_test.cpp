// Self-checking tests of the UREI 1176LN channel: operating point against the drawing, UREI's
// factory tests, ratio None and Attack OFF, timing, THD, levels, sample-rate independence, the
// VU meter, real-time safety and speed. Prints [ok] / [FAIL] per check; exits non-zero on any
// failure.

#include "dsp/Urei1176.h"
#include "dsp/Urei1176Circuit.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <new>
#include <string>
#include <vector>

//== allocation counter (real-time safety check) ===============================================
static std::atomic<long> gAllocations { 0 };
static std::atomic<bool> gCounting { false };
void* operator new (std::size_t n)
{
    if (gCounting.load (std::memory_order_relaxed)) gAllocations.fetch_add (1, std::memory_order_relaxed);
    if (void* p = std::malloc (n == 0 ? 1 : n)) return p;
    throw std::bad_alloc();
}
void* operator new[] (std::size_t n) { return operator new (n); }
void operator delete (void* p) noexcept { std::free (p); }
void operator delete[] (void* p) noexcept { std::free (p); }
void operator delete (void* p, std::size_t) noexcept { std::free (p); }
void operator delete[] (void* p, std::size_t) noexcept { std::free (p); }

using namespace urei1176;

namespace
{
constexpr double kPi = 3.14159265358979323846;
int failures = 0;

void check (bool ok, const std::string& what)
{
    std::printf ("  [%s] %s\n", ok ? "ok" : "FAIL", what.c_str());
    failures += ok ? 0 : 1;
}

double db (double x) { return 20.0 * std::log10 (std::max (x, 1e-30)); }
double fromDb (double d) { return std::pow (10.0, d / 20.0); }

/** dBFS (sine peak) of a level in dBu at the default calibration. */
double dbfsOf (double dbu, double cal = 18.0) { return dbu - cal; }

using Setup = std::function<void (Channel&)>;

struct Run
{
    std::vector<float> out;      // output samples
    std::vector<float> gr;       // getGainReductionDb() per block (the meter's reading)
    std::vector<float> cell;     // currentGainReductionDb() per block (the gain cell itself)
    std::vector<float> needle;   // getMeterNeedleDb() per block
    bool finite = true;
};

/** Runs `input` (a function of time in seconds, floats) through a fresh channel. */
Run run (double fs, double seconds, const std::function<double (double)>& input, const Setup& setup, int block = 32)
{
    Channel ch;
    setup (ch);
    ch.prepare (fs);
    const int n = (int) std::lround (seconds * fs);
    Run r;
    r.out.resize ((size_t) n);
    for (int i = 0; i < n; ++i) r.out[(size_t) i] = (float) input (i / fs);
    for (int start = 0; start < n; start += block)
    {
        const int count = std::min (block, n - start);
        ch.process (r.out.data() + start, count);
        r.gr.push_back (ch.getGainReductionDb());
        r.cell.push_back (ch.currentGainReductionDb());
        r.needle.push_back (ch.getMeterNeedleDb());
    }
    for (float v : r.out) r.finite = r.finite && std::isfinite (v);
    return r;
}

double rmsOf (const std::vector<float>& v, size_t from, size_t to)
{
    double s = 0.0;
    for (size_t i = from; i < to; ++i) s += (double) v[i] * v[i];
    return std::sqrt (s / (double) std::max<size_t> (1, to - from));
}

std::function<double (double)> sine (double dbfs, double hz)
{
    const double a = fromDb (dbfs);
    return [a, hz] (double t) { return a * std::sin (2.0 * kPi * hz * t); };
}

/** Steady-state level (dB re 1.0 rms) and gain reduction (meter and gain cell) of a tone. */
struct Steady { double outDb; double gr; double cell; bool finite; };
Steady steady (double fs, double dbfs, double hz, const Setup& setup, double seconds = 1.5)
{
    const auto r = run (fs, seconds, sine (dbfs, hz), setup, 256);
    const size_t n = r.out.size(), from = n - (size_t) (0.25 * fs);
    return { db (rmsOf (r.out, from, n)), r.gr.back(), r.cell.back(), r.finite };
}

/** Amplitude of harmonic k of f0 by a DFT over a whole number of cycles. */
double harmonic (const std::vector<float>& x, size_t from, size_t count, double fs, double f0, int k)
{
    double re = 0.0, im = 0.0;
    for (size_t i = 0; i < count; ++i)
    {
        const double ph = 2.0 * kPi * f0 * k * (double) i / fs;
        re += x[from + i] * std::cos (ph);
        im += x[from + i] * std::sin (ph);
    }
    return 2.0 * std::sqrt (re * re + im * im) / (double) count;
}


//==============================================================================================
void operatingPoint()
{
    std::printf ("operating point: the netlist with the rev K drawing's values against the voltages marked on it (gyraf.dk redraw)\n");
    Channel ch;
    ch.circuitModel().useRevisionKValues (true);
    ch.prepare (48000.0);
    auto& m = ch.circuitModel();
    struct Row { const char* net; const char* node; double drawing; double tol; const char* label; };
    // Tolerances: +-5 % (+-0.15 V minimum) where the drawing is consistent; the line amp's output
    // nodes +-0.5 V (the drawing's own figures disagree by 0.4 V there, see docs).
    const Row rows[] = {
        { "1176", "pre.B+", 25.8, 0.15, "preamp B+ (C7)" },
        { "1176", "q2.b", 1.02, 0.15, "Q2 base" },
        { "1176", "q2.c", 1.76, 0.15, "Q2 collector" },
        { "1176", "q2.e", 0.51, 0.15, "Q2 emitter" },
        { "1176", "q3.c", 11.98, 0.6, "Q3 collector" },
        { "1176", "q4.e", 11.5, 0.58, "Q4 emitter" },
        { "1176", "gr.B+", 27.8, 0.15, "GR amp B+ (C18)" },
        { "1176", "q12.b", 4.55, 0.23, "Q12 base" },
        { "1176", "q12.c", 15.1, 0.76, "Q12 collector" },
        { "1176", "q12.e", 4.03, 0.2, "Q12 emitter" },
        { "1176", "q13.e", 14.54, 0.73, "Q13 emitter" },
        { "1176", "q14.b", 3.51, 0.18, "Q14 base" },
        { "1176", "q14.c", 17.25, 0.86, "Q14 collector" },
        { "1176", "q15.e", 16.72, 0.84, "Q15 emitter" },
        { "line", "q5.b", 4.98, 0.25, "Q5 base" },
        { "line", "q5.e", 4.54, 0.25, "Q5 emitter" },
        { "line", "q5.c", 28.6, 0.5, "Q5 collector" },
        { "line", "q6.e", 29.1, 0.5, "Q6 emitter" },
        { "line", "q7.e", 29.7, 0.5, "Q7 emitter" },
        { "line", "q8.b", 14.06, 0.5, "Q8 base" },
        { "line", "q9.b", 12.77, 0.5, "Q9 base" },
        { "line", "q8.e", 13.47, 0.5, "Q8 emitter" },
        { "line", "q9.e", 13.35, 0.5, "Q9 emitter" },
        { "line", "line.out", 13.40, 0.5, "line amp output" },
    };
    for (const auto& r : rows)
    {
        const double v = m.nodeVolts (r.net, r.node);
        char what[160];
        std::snprintf (what, sizeof (what), "%-16s drawing %6.2f V, model %6.2f V (tolerance %.2f V)", r.label, r.drawing, v, r.tol);
        check (std::abs (v - r.drawing) <= r.tol, what);
    }
    char what[200];
    std::snprintf (what, sizeof (what), "Q1 gate at idle %.3f V (factory bias: 1.0 dB idle reduction; R81 wiper at %.3f), adjustable range 0 to -2.9 V",
                   m.nodeVolts ("1176", "q1.g"), m.biasPotPosition());
    check (m.nodeVolts ("1176", "q1.g") < 0.0 && m.nodeVolts ("1176", "q1.g") > -2.9, what);

    // The modelled unit (rev F values): no voltages are marked on that drawing. The parts it shares
    // with rev K must stay within the same tolerances (scaled to its 30.0 V rail); the line amp,
    // whose bias network differs, must sit near half the rail for symmetrical clipping, every
    // transistor in its active region.
    std::printf ("operating point of the rev F build (its drawing marks no voltages)\n");
    Channel f;
    f.prepare (48000.0);
    auto& mf = f.circuitModel();
    const double scale = 30.0 / 30.3;
    for (const auto& r : rows)
    {
        if (std::string (r.net) != "1176" || std::string (r.node) == "q3.c" || std::string (r.node) == "q4.e" || std::string (r.node) == "pre.B+") continue;
        const double v = mf.nodeVolts (r.net, r.node);
        std::snprintf (what, sizeof (what), "%-16s rev K drawing x 30.0/30.3: %6.2f V, rev F model %6.2f V (tolerance %.2f V)", r.label, r.drawing * scale, v, r.tol);
        check (std::abs (v - r.drawing * scale) <= r.tol, what);
    }
    const double q4e = mf.nodeVolts ("1176", "q4.e"), out = mf.nodeVolts ("line", "line.out");
    const double q5c = mf.nodeVolts ("line", "q5.c"), q5e = mf.nodeVolts ("line", "q5.e"), q5b = mf.nodeVolts ("line", "q5.b");
    std::snprintf (what, sizeof (what), "preamp: Q4 emitter %.2f V (R21 6.8k: rev K 11.5 V with 7.5k), Q3 collector %.2f V, B+ %.2f V",
                   q4e, mf.nodeVolts ("1176", "q3.c"), mf.nodeVolts ("1176", "pre.B+"));
    check (q4e > 9.5 && q4e < 12.5, what);
    std::snprintf (what, sizeof (what), "line amp: output %.2f V (half rail 15.0), Q5 b/e/c %.2f / %.2f / %.2f V, Q8 / Q9 emitters %.2f / %.2f V",
                   out, q5b, q5e, q5c, mf.nodeVolts ("line", "q8.e"), mf.nodeVolts ("line", "q9.e"));
    check (std::abs (out - 15.0) < 1.5 && q5b - q5e > 0.5 && q5c > q5b + 1.0, what);
}

//==============================================================================================
/** Input level (dBFS) at which a 1 kHz tone gives `targetGr` dB of reduction (the gain cell's):
    a coarse search on 1.25 s tones, then a settled one on `settle` s tones (C5 via R7 + R9 and C2
    via R8 settle with ~1 s time constants; UREI's tests read settled levels). */
double levelForReduction (double fs, double targetGr, const Setup& setup, double settle = 0.0)
{
    double lo = -80.0, hi = 20.0;
    for (int i = 0; i < 12; ++i)
    {
        const double mid = 0.5 * (lo + hi);
        (steady (fs, mid, 1000.0, setup, 1.25).cell < targetGr ? lo : hi) = mid;
    }
    if (settle > 0.0)
    {
        lo -= 2.0;
        hi += 2.0;
        for (int i = 0; i < 6; ++i)
        {
            const double mid = 0.5 * (lo + hi);
            (steady (fs, mid, 1000.0, setup, settle).cell < targetGr ? lo : hi) = mid;
        }
    }
    return 0.5 * (lo + hi);
}

const char* ratioName (Ratio r)
{
    switch (r)
    {
        case Ratio::r4: return "4:1"; case Ratio::r8: return "8:1"; case Ratio::r12: return "12:1";
        case Ratio::r20: return "20:1"; case Ratio::all: return "All"; case Ratio::none: return "None";
    }
    return "?";
}

void ratioTest()
{
    std::printf ("UREI factory test 11 (ratio): from 1 dB of limiting, +20 dB in; Input full, 1 kHz, settled (5 s tones)\n");
    constexpr double target[] { 5.0, 2.5, 1.66, 1.0 };
    for (int k = 0; k < 4; ++k)
    {
        const Ratio ratio = (Ratio) k;
        const Setup setup = [ratio] (Channel& c) { c.setInput (Range::inputMax); c.setRatio (ratio); };
        const double base = levelForReduction (48000.0, 1.0, setup, 5.0);
        const auto a = steady (48000.0, base, 1000.0, setup, 5.0), b = steady (48000.0, base + 20.0, 1000.0, setup, 5.0);
        const double rise = b.outDb - a.outDb;
        char what[200];
        std::snprintf (what, sizeof (what), "%-4s output rises %.2f dB (UREI: %.2f +-20 %%); 1 dB of limiting at %.1f dBFS in, %.1f dB GR at +20 dB",
                       ratioName (ratio), rise, target[k], base, b.gr);
        check (a.finite && b.finite && std::abs (rise / target[k] - 1.0) < 0.2, what);
    }
}

/** Thresholds per ratio at the default settings: the input (dBFS) for 1 dB of reduction. */
void thresholds()
{
    std::printf ("thresholds at the default settings (Input and Output at 24, calibration 18 dBu): 1 dB of reduction, 1 kHz, settled\n");
    double previous = -100.0;
    for (int k = 0; k < 5; ++k)
    {
        const Ratio ratio = (Ratio) k;
        const Setup setup = [ratio] (Channel& c) { c.setRatio (ratio); };
        const double t = levelForReduction (48000.0, 1.0, setup, 5.0);
        const auto out = steady (48000.0, t, 1000.0, setup, 5.0);
        char what[200];
        std::snprintf (what, sizeof (what), "%-4s input %6.1f dBFS (%5.1f dBu), output %5.1f dBFS (%5.1f dBu)", ratioName (ratio), t, t + 18.0,
                       out.outDb + db (std::sqrt (2.0)), out.outDb + db (std::sqrt (2.0)) + 18.0);
        // The thresholds rise with the ratio (UREI: 4:1 lowest, 20:1 highest); All sits near 8:1.
        check (t > -60.0 && t < 20.0 && (k == 4 || t > previous), what);
        if (k < 4) previous = t;
    }
}

void noReduction()
{
    std::printf ("ratio None and Attack OFF: no gain reduction\n");
    // 20 dB above the 4:1 threshold, Input full: deep reduction at 4:1 and All.
    const double level = levelForReduction (48000.0, 1.0, [] (Channel& c) { c.setInput (Range::inputMax); c.setRatio (Ratio::r4); }) + 20.0;
    auto at = [level] (Ratio ratio, bool off)
    {
        return steady (48000.0, level, 1000.0, [ratio, off] (Channel& c) { c.setInput (Range::inputMax); c.setRatio (ratio); c.setLimitOff (off); });
    };
    const auto ref = at (Ratio::r4, false), all = at (Ratio::all, false), none = at (Ratio::none, false);
    const auto off4 = at (Ratio::r4, true), offAll = at (Ratio::all, true);
    char what[220];
    std::snprintf (what, sizeof (what), "at %.1f dBFS, Input full: 4:1 %.1f dB GR, All %.1f dB; None %.2f dB, Attack OFF %.2f dB (4:1) / %.2f dB (All)",
                   level, ref.cell, all.cell, none.cell, off4.cell, offAll.cell);
    // All lowers the bias (the shorted threshold chain draws more current), so with limiting
    // off Q1 conducts less than at the calibrated idle: a small negative reduction.
    check (ref.cell > 10.0 && all.cell > 10.0 && std::abs (none.cell) < 0.1 && std::abs (off4.cell) < 0.1 && offAll.cell < 0.1 && offAll.cell > -1.1
               && none.finite && off4.finite && offAll.finite, what);
    // Without limiting the output is the uncompressed one, ~GR dB louder than with it.
    std::snprintf (what, sizeof (what), "None vs 4:1 output: +%.1f dB (gain reduction %.1f dB); Attack OFF vs None: %.2f dB",
                   none.outDb - ref.outDb, ref.cell, off4.outDb - none.outDb);
    check (std::abs ((none.outDb - ref.outDb) - ref.cell) < 1.5 && std::abs (off4.outDb - none.outDb) < 0.05, what);
}

/** Attack and release times on a tone burst, from the gain cell's own reduction (no meter):
    attack = onset to 63 % of the reduction reached 25 ms later; release = burst end to 63 % of
    the way back. The gate takes half of a fast control change at once and the rest over C5's
    ~1 s, so the attack is read on the fast part. */
void timing()
{
    std::printf ("attack / release on a 1 kHz burst (Input full, 4:1, 10 dB above the 1 dB threshold; gain cell reduction)\n");
    const double fs = 96000.0;
    const Setup base = [] (Channel& c) { c.setInput (Range::inputMax); c.setRatio (Ratio::r4); };
    const double t1 = levelForReduction (48000.0, 1.0, base);
    const double quiet = t1 - 20.0, loud = t1 + 10.0;
    auto burst = [&] (double tOn, double tOff)
    {
        const double aq = fromDb (quiet), al = fromDb (loud);
        return [=] (double t) { return (t >= tOn && t < tOff ? al : aq) * std::sin (2.0 * kPi * 1000.0 * t); };
    };
    struct Times { double attackMs, releaseMs, grEnd; };
    auto measure = [&] (float attack, float release)
    {
        const double tOn = 0.5, tOff = 1.0, total = 4.0;
        const int block = 2;
        const auto r = run (fs, total, burst (tOn, tOff), [&] (Channel& c) { base (c); c.setAttack (attack); c.setRelease (release); }, block);
        const double dtBlock = block / fs;
        const size_t on = (size_t) (tOn / dtBlock), off = (size_t) (tOff / dtBlock), settle = on + (size_t) (0.025 / dtBlock);
        const double gr0 = r.cell[on - 1], grSettle = r.cell[settle], grEnd = r.cell[off - 1];
        Times t { 1e9, 1e9, grEnd };
        for (size_t i = on; i < settle; ++i)
            if (r.cell[i] >= gr0 + 0.632 * (grSettle - gr0)) { t.attackMs = (double) (i - on + 1) * dtBlock * 1e3; break; }
        const double floorGr = r.cell.back();
        for (size_t i = off; i < r.cell.size(); ++i)
            if (r.cell[i] <= floorGr + 0.368 * (grEnd - floorGr)) { t.releaseMs = (double) (i - off + 1) * dtBlock * 1e3; break; }
        return t;
    };
    const auto fast = measure (7.0f, 7.0f), slow = measure (1.0f, 1.0f);
    char what[260];
    std::snprintf (what, sizeof (what), "attack 7 (fastest) %.3f ms, attack 1 (slowest) %.3f ms (UREI: 0.02-0.8 ms; checked: < 0.5 ms and 0.2-5 ms)",
                   fast.attackMs, slow.attackMs);
    check (fast.attackMs < 0.5 && slow.attackMs > 0.2 && slow.attackMs < 5.0 && slow.attackMs > 2.0 * fast.attackMs, what);
    std::snprintf (what, sizeof (what), "release 7 (fastest) %.0f ms, release 1 (slowest) %.0f ms after %.1f dB GR (UREI: 50-1100 ms; checked: 20-300 ms and 0.3-5 s)",
                   fast.releaseMs, slow.releaseMs, fast.grEnd);
    check (fast.releaseMs > 20.0 && fast.releaseMs < 300.0 && slow.releaseMs > 300.0 && slow.releaseMs < 5000.0, what);
}

void thd()
{
    std::printf ("THD at UREI's R16 setting point (5.3.5): 500 Hz, -30 dBu, Input full, Output at 18, Attack OFF, Release full, 20:1 (spec 0.5 %%)\n");
    const double fs = 96000.0, f0 = 500.0;
    auto measure = [&] (double dbu, bool limiting, double& gr)
    {
        const Setup setup = [limiting] (Channel& c) {
            c.setInput (Range::inputMax); c.setOutput (rotationForMarking (18.0f)); c.setRatio (Ratio::r20);
            c.setRelease (7.0f); c.setLimitOff (! limiting); };
        const auto r = run (fs, 3.0, sine (dbfsOf (dbu), f0), setup, 256);
        const size_t count = (size_t) (fs / f0) * 200, from = r.out.size() - count;
        const double h1 = harmonic (r.out, from, count, fs, f0, 1);
        double h = 0.0;
        for (int k = 2; k <= 10; ++k) { const double a = harmonic (r.out, from, count, fs, f0, k); h += a * a; }
        gr = r.cell.back();
        return r.finite ? 100.0 * std::sqrt (h) / h1 : 1e9;
    };
    double gr = 0.0;
    const double atSpec = measure (-30.0, false, gr);
    char what[200];
    std::snprintf (what, sizeof (what), "THD %.3f %% (limiting off)", atSpec);
    check (atSpec < 0.5, what);
    // With limiting (the spec's "50 Hz to 15 kHz with limiting"): 500 Hz, 10 and 20 dB of reduction.
    for (double over : { 10.0, 20.0 })
    {
        const double t = levelForReduction (48000.0, 1.0, [] (Channel& c) { c.setInput (Range::inputMax); c.setRatio (Ratio::r20); });
        const double withGr = measure (t + 18.0 + over, true, gr);
        std::snprintf (what, sizeof (what), "THD %.3f %% with %.1f dB of reduction (20:1, attack 4, release 7)", withGr, gr);
        check (withGr < 0.5, what);
    }
}

/** Gain (dB) from the plug-in's input to its output for a tone well below threshold. */
double gainAt (double hz, const Setup& setup, double dbfs = -60.0)
{
    const double fs = 96000.0;
    const double seconds = std::max (1.0, 40.0 / hz);
    const auto r = run (fs, seconds, sine (dbfs, hz), setup, 512);
    const size_t count = (size_t) std::lround (std::floor (0.5 * seconds * hz) * fs / hz), from = r.out.size() - count;
    return db (harmonic (r.out, from, count, fs, hz, 1) / fromDb (dbfs));
}

void unityGain()
{
    std::printf ("gain structure (below threshold, limiting on, 4:1)\n");
    const double unity = gainAt (1000.0, [] (Channel&) {});
    char what[200];
    std::snprintf (what, sizeof (what), "Input and Output at 24 (UREI / UA: unity gain): %+.2f dB at 1 kHz", unity);
    check (std::abs (unity) < 1.0, what);
    const double full = gainAt (2000.0, [] (Channel& c) { c.setInput (Range::inputMax); c.setOutput (Range::outputMax); }, -70.0);
    std::snprintf (what, sizeof (what), "maximum gain (both full) %.1f dB at 2 kHz (rev H / UA: 45 +-1; rev F ~6 dB less per UREI's revision notes)", full);
    check (full > 37.0 && full < 41.0, what);
    // UREI factory test 8 window (+0.5 / -1 dB, 20 Hz - 20 kHz) at the unity setting.
    double worstHi = -100.0, worstLo = 100.0;
    std::string list;
    for (double hz : { 20.0, 50.0, 100.0, 5000.0, 10000.0, 15000.0, 20000.0 })
    {
        const double g = gainAt (hz, [] (Channel&) {}) - unity;
        worstHi = std::max (worstHi, g);
        worstLo = std::min (worstLo, g);
        char item[40];
        std::snprintf (item, sizeof (item), " %g Hz %+.2f", hz, g);
        list += item;
    }
    // Test 8's window (+0.5 / -1 dB) is from the rev H-era procedure; rev F's line amp adds a
    // small HF shelf (R33 + C13), so +0.6 dB is allowed at the top.
    std::snprintf (what, sizeof (what), "response re 1 kHz (UREI test 8, rev H era: +0.5 / -1 dB; rev F C13 shelf, checked +0.6 / -1):%s", list.c_str());
    check (worstHi < 0.6 && worstLo > -1.0, what);
}

/** UREI test 10 on the model: limiting off, 20:1, release full; R81 sets Q1 to cost exactly 1.0 dB
    against R81 fully counter-clockwise (Q1 pinched off). */
void biasTest()
{
    std::printf ("UREI factory test 10 (Q bias): R81 at its factory setting vs fully counter-clockwise\n");
    auto level = [] (bool ccw)
    {
        Channel ch;
        ch.setInput (Range::inputMax); ch.setOutput (Range::outputMax); ch.setRatio (Ratio::r20); ch.setRelease (7.0f); ch.setLimitOff (true);
        ch.prepare (96000.0);
        if (ccw)
        {
            ch.circuitModel().setBiasPot (1.0);
            ch.reset();   // settles the operating point with R81 there (reset() keeps the pot)
        }
        std::vector<float> x ((size_t) (96000 * 0.5));
        const double a = fromDb (dbfsOf (-30.0));
        for (size_t i = 0; i < x.size(); ++i) x[i] = (float) (a * std::sin (2.0 * kPi * 2000.0 * (double) i / 96000.0));
        ch.process (x.data(), (int) x.size());
        const size_t count = 96000 / 4;
        return db (harmonic (x, x.size() - count, count, 96000.0, 2000.0, 1));
    };
    const double off = level (true), biased = level (false);
    char what[160];
    std::snprintf (what, sizeof (what), "output drops %.2f dB when R81 is set (UREI: exactly 1.0 dB)", off - biased);
    check (std::abs (off - biased - 1.0) < 0.15, what);
}

/** The GR meter: zero at idle (R55), -10 VU at 10 dB of settled reduction (R54), and its law
    elsewhere (linear in the control voltage). */
void meterTracking()
{
    std::printf ("GR meter tracking (UREI test 11 calibrates 0 and -10 VU), settled 1 kHz tones, Input full, 4:1\n");
    const Setup setup = [] (Channel& c) { c.setInput (Range::inputMax); c.setRatio (Ratio::r4); };
    for (double target : { 5.0, 10.0, 20.0 })
    {
        const double level = levelForReduction (48000.0, target, setup, 6.0);
        const auto r = run (48000.0, 10.0, sine (level, 1000.0), setup, 256);
        char what[160];
        std::snprintf (what, sizeof (what), "%.1f dB of reduction: needle %.2f VU", r.cell.back(), r.needle.back());
        check (target != 10.0 || std::abs (r.needle.back() + 10.0) < 0.5, what);
    }
}

void sampleRates()
{
    std::printf ("sample-rate independence (no oversampling)\n");
    // 1 kHz at the 4:1 threshold + 15 dB, Input full, default attack / release.
    const Setup setup = [] (Channel& c) { c.setInput (Range::inputMax); c.setRatio (Ratio::r4); };
    const double level = levelForReduction (48000.0, 1.0, setup) + 15.0;
    const auto a = steady (48000.0, level, 1000.0, setup), b = steady (192000.0, level, 1000.0, setup);
    const auto c = steady (44100.0, level, 1000.0, setup), d = steady (96000.0, level, 1000.0, setup);
    char what[200];
    std::snprintf (what, sizeof (what), "gain reduction at 44.1 / 48 / 96 / 192 kHz: %.2f / %.2f / %.2f / %.2f dB (48 vs 192 within 0.15 dB)",
                   c.cell, a.cell, d.cell, b.cell);
    check (a.finite && b.finite && c.finite && d.finite && std::abs (a.cell - b.cell) < 0.15, what);
    std::snprintf (what, sizeof (what), "output level at 48 / 192 kHz: %.2f / %.2f dB", a.outDb, b.outDb);
    check (std::abs (a.outDb - b.outDb) < 0.15, what);
}

void hotInput()
{
    std::printf ("+24 dBu-equivalent input (beyond the hardware's maximum input level)\n");
    for (Ratio ratio : { Ratio::r4, Ratio::all, Ratio::none })
    {
        const auto r = run (48000.0, 1.0, sine (dbfsOf (24.0), 1000.0),
                            [ratio] (Channel& c) { c.setInput (Range::inputMax); c.setOutput (Range::outputMax); c.setRatio (ratio); c.setAttack (7.0f); }, 256);
        float peak = 0.0f;
        for (float v : r.out) peak = std::max (peak, std::abs (v));
        char what[160];
        std::snprintf (what, sizeof (what), "%-4s Input and Output full, attack fastest: finite, output peak %.1f dBFS", ratioName (ratio), db (peak));
        check (r.finite && peak < 100.0f, what);
    }
}

void idleMeter()
{
    std::printf ("meter at idle\n");
    for (Ratio ratio : { Ratio::r4, Ratio::r20, Ratio::none })
    {
        const auto r = run (48000.0, 0.5, [] (double) { return 0.0; }, [ratio] (Channel& c) { c.setRatio (ratio); }, 256);
        char what[160];
        std::snprintf (what, sizeof (what), "%-4s silence: GR %.3f dB, needle %.2f dB (0 VU)", ratioName (ratio), r.gr.back(), r.needle.back());
        check (std::abs (r.gr.back()) < 0.05 && std::abs (r.needle.back()) < 0.05, what);
    }
}

/** The VU scale: a +4 / +8 dBu tone applied suddenly takes the needle to 99 % of its deflection
    in 300 ms (+-10 %), overshooting 1-1.5 %, and back the same. */
void vuBallistics()
{
    std::printf ("VU meter ballistics (+4 scale; ANSI C16.5 / IEC 60268-17: 99 %% in 300 ms, 1-1.5 %% overshoot)\n");
    const double fs = 48000.0;
    // No limiting (ratio None) so the output follows the input; find the input for 0 VU first.
    const Setup setup = [] (Channel& c) { c.setRatio (Ratio::none); c.setMeterMode (MeterMode::plus4); };
    const auto probe = run (fs, 2.0, sine (-20.0, 1000.0), setup, 64);
    const double gainDb = probe.needle.back() - (-20.0 + 18.0 - 4.0);   // needle reads output dBu - 4
    const double inDbfs = 4.0 - 18.0 - gainDb;                           // input for 0 VU
    const double tOn = 0.5, tOff = 1.5;
    const int block = 16;
    const double amp = fromDb (inDbfs);
    const auto r = run (fs, 2.5, [=] (double t) { return t >= tOn && t < tOff ? amp * std::sin (2.0 * kPi * 1000.0 * t) : 0.0; }, setup, block);
    const double dtBlock = block / fs;
    const size_t on = (size_t) (tOn / dtBlock), off = (size_t) (tOff / dtBlock);
    const double finalDb = r.needle[off - 1];
    double t99 = -1.0, peakDb = -100.0, tFall = -1.0;
    for (size_t i = on; i < off; ++i)
    {
        if (t99 < 0.0 && r.needle[i] >= finalDb + db (0.99)) t99 = (double) (i - on + 1) * dtBlock;
        peakDb = std::max (peakDb, (double) r.needle[i]);
    }
    // Falling: the deflection to 1 % of the reference (-40 dB, below the scale) is not readable
    // on the -20 dB scale; the movement is linear, so the fall to -20 dB (10 %) is checked against
    // the same second-order movement: 10 % remaining at 0.2 s.
    for (size_t i = off; i < r.needle.size(); ++i)
        if (r.needle[i] <= -19.99f) { tFall = (double) (i - off + 1) * dtBlock; break; }
    const double overshootPct = 100.0 * (fromDb (peakDb - finalDb) - 1.0);
    char what[220];
    std::snprintf (what, sizeof (what), "rise to 99 %% in %.0f ms (300 +-30), overshoot %.2f %% (1-1.5), steady needle %.2f VU (0 expected)",
                   t99 * 1e3, overshootPct, finalDb);
    check (std::abs (t99 - 0.3) < 0.03 && overshootPct > 0.9 && overshootPct < 1.6 && std::abs (finalDb) < 0.1, what);
    std::snprintf (what, sizeof (what), "fall to -20 VU (10 %% deflection) in %.0f ms (the same second-order movement: 224 ms +-10 %%)", tFall * 1e3);
    check (std::abs (tFall - 0.224) < 0.0224, what);
}

void realtimeSafety()
{
    std::printf ("real-time safety\n");
    Channel ch;
    ch.prepare (48000.0);
    std::vector<float> x (512);
    long allocations = 0;
    for (int b = 0; b < 400; ++b)
    {
        for (size_t i = 0; i < x.size(); ++i) x[i] = (float) (0.3 * std::sin (2.0 * kPi * 440.0 * (double) (b * 512 + (int) i) / 48000.0));
        gCounting = true;
        // Every control moves while processing.
        ch.setInput ((float) (b % 11));
        ch.setOutput ((float) ((b * 3) % 11));
        ch.setAttack (1.0f + (float) (b % 7));
        ch.setRelease (1.0f + (float) ((b * 5) % 7));
        ch.setRatio ((Ratio) (b % 6));
        ch.setLimitOff (b % 13 == 0);
        ch.setMeterMode ((MeterMode) (b % 4));
        ch.setCalibrationDbu (10.0f + (float) (b % 17));
        ch.process (x.data(), (int) x.size());
        gCounting = false;
    }
    allocations = gAllocations.load();
    char what[160];
    std::snprintf (what, sizeof (what), "no allocation in process() or the setters while every control moves: %ld allocations", allocations);
    check (allocations == 0, what);
}

void performance()
{
    std::printf ("performance (one channel, this machine, music-like signal with 5-15 dB GR)\n");
    for (double fs : { 48000.0, 96000.0 })
    {
        Channel ch;
        ch.setInput (Range::inputMax);
        ch.prepare (fs);
        const int n = (int) (4.0 * fs);
        std::vector<float> x ((size_t) n);
        for (int i = 0; i < n; ++i)
        {
            const double t = i / fs;
            x[(size_t) i] = (float) (fromDb (-40.0) * (0.5 * std::sin (2 * kPi * 110 * t) + 0.3 * std::sin (2 * kPi * 440 * t) + 0.2 * std::sin (2 * kPi * 3000 * t))
                                     * (1.0 + 0.5 * std::sin (2 * kPi * 2 * t)));
        }
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < n; i += 256) ch.process (x.data() + i, std::min (256, n - i));
        const double secs = std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();
        const double rate = n / secs;
        char what[200];
        std::snprintf (what, sizeof (what), "%.0f kHz: %.0f samples/s per channel = %.2fx real time mono, %.2fx stereo (GR %.1f dB)",
                       fs / 1000.0, rate, rate / fs, rate / (2.0 * fs), ch.getGainReductionDb());
        check (rate > fs, what);
    }
}
} // namespace

int main (int argc, char** argv)
{
    // Optional arguments: names of the sections to run (default: all).
    struct Section { const char* name; void (*fn)(); };
    const Section sections[] = {
        { "dc", operatingPoint }, { "ratio", ratioTest }, { "threshold", thresholds }, { "none", noReduction },
        { "timing", timing }, { "thd", thd }, { "unity", unityGain }, { "bias", biasTest }, { "meter", meterTracking },
        { "rates", sampleRates }, { "hot", hotInput }, { "idle", idleMeter }, { "vu", vuBallistics }, { "rt", realtimeSafety },
        { "perf", performance },
    };
    std::setvbuf (stdout, nullptr, _IOLBF, 0);
    std::printf ("UREI 1176LN channel tests\n");
    for (const auto& sec : sections)
    {
        bool wanted = argc < 2;
        for (int i = 1; i < argc; ++i) wanted = wanted || std::string (argv[i]) == sec.name;
        if (wanted) sec.fn();
    }
    std::printf (failures == 0 ? "all checks passed\n" : "%d check(s) failed\n", failures);
    return failures == 0 ? 0 : 1;
}
