// Self-checking tests of the stereo modes around the fast engine: the 1176SA tie of the control nodes (Link) against two independent
// units (Dual), Mid-Side, the transitions between modes while playing, robustness at +24 dBu, real-time safety and speed.
// Prints [ok] / [FAIL] per check and the measured numbers; exits non-zero on any failure.

#include "dsp/Fast1176.h"
#include "dsp/StereoModes.h"
#include "dsp/Urei1176Circuit.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
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
const double kPi = 3.14159265358979323846;
const double kFs = 48000.0;
int failures = 0;

void check (bool ok, const std::string& what)
{
    std::printf ("  [%s] %s\n", ok ? "ok" : "FAIL", what.c_str());
    failures += ok ? 0 : 1;
}

std::string fmt (const char* f, double a = 0.0, double b = 0.0, double c = 0.0, double d = 0.0)
{
    char buf[400];
    std::snprintf (buf, sizeof (buf), f, a, b, c, d);
    return buf;
}

double db (double x) { return 20.0 * std::log10 (std::max (x, 1e-30)); }
double fromDb (double d) { return std::pow (10.0, d / 20.0); }

struct Settings
{
    float input = Range::inputMax;
    float attack = 4.0f, release = 4.0f;
    Ratio ratio = Ratio::r4;
    bool offA = false, offB = false;
};

struct Pair
{
    FastChannel a, b;
    explicit Pair (const Settings& s, double fs = kFs)
    {
        for (FastChannel* c : { &a, &b })
        {
            c->setInput (s.input);
            c->setAttack (s.attack);
            c->setRelease (s.release);
            c->setRatio (s.ratio);
            c->setMeterMode (MeterMode::gainReduction);
        }
        a.setLimitOff (s.offA);
        b.setLimitOff (s.offB);
        a.prepare (fs);
        b.prepare (fs);
    }
};

typedef std::vector<float> Signal;

Signal tone (double seconds, double dbfs, double hz, double fromSecond = 0.0, double toSecond = 1.0e9)
{
    const int n = (int) std::lround (seconds * kFs);
    Signal x ((size_t) n, 0.0f);
    const double amp = fromDb (dbfs);
    for (int i = 0; i < n; ++i)
    {
        const double t = i / kFs;
        if (t >= fromSecond && t < toSecond) x[(size_t) i] = (float) (amp * std::sin (2.0 * kPi * hz * t));
    }
    return x;
}

Signal ampTone (double seconds, double amp, double hz)
{
    Signal x = tone (seconds, 0.0, hz);
    for (auto& v : x) v = (float) (v * amp);
    return x;
}

double rms (const Signal& x, double fromSecond, double toSecond)
{
    const size_t from = (size_t) (fromSecond * kFs), to = std::min (x.size(), (size_t) (toSecond * kFs));
    double s = 0.0;
    for (size_t i = from; i < to; ++i) s += (double) x[i] * x[i];
    return std::sqrt (s / (double) std::max<size_t> (1, to - from));
}

double peak (const Signal& x)
{
    double p = 0.0;
    for (float v : x) p = std::max (p, (double) std::abs (v));
    return p;
}

bool finite (const Signal& x)
{
    for (float v : x) if (! std::isfinite (v)) return false;
    return true;
}

enum class Engine { dual, link };

/** Runs the pair through the engine in blocks; `each` is called after every block with the sample index reached. */
void render (Pair& p, Engine e, Signal& l, Signal& r, int block, const std::function<void (int)>& each = {})
{
    const int n = (int) std::min (l.size(), r.size());
    for (int i = 0; i < n; i += block)
    {
        const int c = std::min (block, n - i);
        if (e == Engine::link) FastChannel::processLinked (p.a, p.b, l.data() + i, r.data() + i, c);
        else FastChannel::processPair (p.a, p.b, l.data() + i, r.data() + i, c);
        if (each) each (i + c);
    }
}

//== 1. identical L and R: Link is Dual ==========================================================
void identicalInputs()
{
    std::printf ("link with identical L and R equals dual\n");
    const struct { Ratio ratio; double dbfs; double hz; const char* name; } cases[] = {
        { Ratio::r4, -20.0, 1000.0, "4:1, -20 dBFS" }, { Ratio::r20, -8.0, 220.0, "20:1, -8 dBFS" },
        { Ratio::all, -3.0, 3000.0, "All, -3 dBFS" }, { Ratio::r12, -30.0, 1000.0, "12:1, -30 dBFS (little reduction)" },
        { Ratio::none, -6.0, 1000.0, "None, -6 dBFS" } };
    for (const auto& c : cases)
    {
        Settings s;
        s.ratio = c.ratio;
        const Signal burst = tone (1.5, c.dbfs, c.hz, 0.2, 1.0);
        Pair pd (s), pl (s);
        Signal dl = burst, dr = burst, ll = burst, lr = burst;
        // single-sample blocks: the control voltage (double) is compared after every sample
        double ctlErr = 0.0, ctlMax = 1e-30, outErr = 0.0, outMax = 1e-30;
        for (size_t i = 0; i < burst.size(); ++i)
        {
            FastChannel::processPair (pd.a, pd.b, dl.data() + i, dr.data() + i, 1);
            FastChannel::processLinked (pl.a, pl.b, ll.data() + i, lr.data() + i, 1);
            for (int ch = 0; ch < 2; ++ch)
            {
                const double cd = ch == 0 ? pd.a.controlVolts() : pd.b.controlVolts(), cl = ch == 0 ? pl.a.controlVolts() : pl.b.controlVolts();
                ctlErr = std::max (ctlErr, std::abs (cd - cl));
                ctlMax = std::max (ctlMax, std::abs (cd));
            }
            for (const float* pair : { (const float*) nullptr })
                (void) pair;
            outErr = std::max (outErr, (double) std::max (std::abs (dl[i] - ll[i]), std::abs (dr[i] - lr[i])));
            outMax = std::max (outMax, (double) std::abs (dl[i]));
        }
        const double e = ctlErr / ctlMax;
        // the output is a float: two engines that agree to double rounding can still differ by one float ulp where a result sits on a rounding edge
        const double o = outErr / outMax;
        check (e < 1.0e-9 && o < 2.0e-7 && std::abs (pd.a.currentGainReductionDb() - pl.a.currentGainReductionDb()) < 1.0e-6,
               std::string (c.name) + fmt (": control voltage differs by %.1e (relative, double); output by %.1e of its peak (a float ulp at the peak is 6e-8)", e, o));
    }
}

//== 2. signal on L only ==========================================================================
void oneSidedDrive()
{
    std::printf ("link with signal on L only\n");
    for (Ratio ratio : { Ratio::r4, Ratio::all })
    {
        Settings s;
        s.ratio = ratio;
        const Signal loud = tone (2.0, -14.0, 1000.0, 0.1), quiet = tone (2.0, -55.0, 3000.0);
        Pair link (s);
        Signal l = loud, r = quiet;
        render (link, Engine::link, l, r, 64);
        const double grL = link.a.currentGainReductionDb(), grR = link.b.currentGainReductionDb();
        // R alone, no reduction (the quiet tone alone does not compress): the reference for R's gain path
        Pair alone (s);
        Signal silent ((size_t) loud.size(), 0.0f), r2 = quiet;
        render (alone, Engine::dual, silent, r2, 64);
        const double gainDb = db (rms (r, 1.5, 2.0)) - db (rms (r2, 1.5, 2.0));
        // and L alone in dual, for the record
        Pair dual (s);
        Signal l3 = loud, r3 = quiet;
        render (dual, Engine::dual, l3, r3, 64);
        const std::string name = ratio == Ratio::r4 ? "4:1" : "All";
        check (std::abs (grL - grR) < 0.02 && grL > 3.0,
               name + fmt (": both channels reduce by the same amount: L %.3f dB, R %.3f dB", grL, grR));
        const double grAlone = alone.b.currentGainReductionDb();
        check (std::abs (gainDb + (grR - grAlone)) < 0.3,
               name + fmt (": R's output over R alone is %.2f dB; R's gain reduction beyond R alone's (%.2f dB) is %.2f dB", gainDb, grAlone, grR - grAlone));
        std::printf ("        (dual, same input: L reduces by %.2f dB, R by %.2f dB; linked L %.2f dB)\n", (double) dual.a.currentGainReductionDb(),
                     (double) dual.b.currentGainReductionDb(), grL);
    }
}

//== 3. attack and release times ==================================================================
struct Times { double swing, attack63, release63; };

/** 63 % times of the control voltage (C27's node) from a burst: 0.3 s quiet, 1.2 s tone, then silence. `driveA` / `driveB` say which of
    L / R carry it. The swing is measured from the voltage before the burst to its value at the burst's end. */
Times measure (const Settings& s, Engine e, bool driveA, bool driveB, double dbfs)
{
    const double total = 6.0, on = 0.3, off = 1.5;
    const Signal burst = tone (total, dbfs, 1000.0, on, off), none ((size_t) std::lround (total * kFs), 0.0f);
    Signal l = driveA ? burst : none, r = driveB ? burst : none;
    Pair p (s);
    const int block = 2;
    std::vector<double> ctl;
    render (p, e, l, r, block, [&] (int) { ctl.push_back (p.a.controlVolts()); });
    const size_t iOn = (size_t) (on * kFs / block), iOff = (size_t) (off * kFs / block);
    const double idle = ctl[iOn - 1], fin = ctl[iOff - 1];   // the control voltage rises with gain reduction
    Times t { fin - idle, 0.0, 0.0 };
    for (size_t i = iOn; i < iOff; ++i)
        if ((ctl[i] - idle) >= 0.632 * (fin - idle)) { t.attack63 = (double) (i - iOn) * block / kFs; break; }
    for (size_t i = iOff; i < ctl.size(); ++i)
        if ((ctl[i] - idle) <= 0.368 * (fin - idle)) { t.release63 = (double) (i - iOff) * block / kFs; break; }
    return t;
}

void timesTest()
{
    std::printf ("attack and release 63 %% times (2 sample blocks; tone 1 kHz, ratio 4:1)\n");
    const struct { float attack, release; } dials[] = { { 4.0f, 7.0f }, { 1.0f, 5.0f } };
    for (const auto& d : dials)
    {
        Settings s;
        s.attack = d.attack;
        s.release = d.release;
        const double level = -14.0;
        const Times dual = measure (s, Engine::dual, true, false, level);
        const Times one = measure (s, Engine::link, true, false, level);
        const Times both = measure (s, Engine::link, true, true, level);
        const Times dualBoth = measure (s, Engine::dual, true, true, level);
        std::printf ("        attack %.0f release %.0f   control-voltage swing / attack 63%% / release 63%%\n", (double) d.attack, (double) d.release);
        std::printf ("          dual (either)       %5.2f V  %7.3f ms  %7.1f ms\n", dual.swing, dual.attack63 * 1e3, dual.release63 * 1e3);
        std::printf ("          link, one side      %5.2f V  %7.3f ms  %7.1f ms\n", one.swing, one.attack63 * 1e3, one.release63 * 1e3);
        std::printf ("          link, both sides    %5.2f V  %7.3f ms  %7.1f ms\n", both.swing, both.attack63 * 1e3, both.release63 * 1e3);
        check (std::abs (dualBoth.attack63 - dual.attack63) < 1e-9, "dual: the other channel does not matter");
        check (std::abs (both.attack63 - dual.attack63) < 2.0 * 4.0 / kFs && std::abs (both.release63 - dual.release63) < 0.02 * dual.release63 + 8.0 / kFs,
               fmt ("link with both sides driven times like dual (attack %.3f vs %.3f ms)", both.attack63 * 1e3, dual.attack63 * 1e3));
        const double ratio = one.attack63 / std::max (both.attack63, 1e-9);
        check (ratio > 1.4 && ratio < 2.6,
               fmt ("link, one side driven: attack %.2f times as slow as with both sides driven", ratio));
        const double rratio = one.release63 / std::max (both.release63, 1e-9);
        check (rratio > 0.5 && rratio < 1.5,
               fmt ("link, one side driven: release %.2f times as long as with both sides driven", rratio));
    }
}

//== 4. robustness at +24 dBu =====================================================================
void robustness()
{
    std::printf ("link at +24 dBu (calibration 18 dBu: +6 dBFS), every ratio, drive and Attack OFF cases\n");
    int cases = 0, bad = 0;
    double worstPeak = 0.0, worstDev = 0.0;
    const Signal loud = ampTone (0.6, 2.0, 1000.0), none ((size_t) loud.size(), 0.0f);
    for (int ratioIndex = 0; ratioIndex < 6; ++ratioIndex)
        for (int drive = 0; drive < 3; ++drive)         // L only, R only, both
            for (int off = 0; off < 3; ++off)           // neither, R only, L only
            {
                Settings s;
                s.ratio = (Ratio) ratioIndex;
                s.offA = off == 2;
                s.offB = off == 1;
                const bool dA = drive != 1, dB = drive != 0;
                Signal l = dA ? loud : none, r = dB ? loud : none;
                Pair link (s);
                render (link, Engine::link, l, r, 50 + 7 * ratioIndex);
                Signal l2 = dA ? loud : none, r2 = dB ? loud : none;
                Pair dual (s);
                render (dual, Engine::dual, l2, r2, 50);
                ++cases;
                bool ok = finite (l) && finite (r) && peak (l) < 40.0 && peak (r) < 40.0;
                worstPeak = std::max (worstPeak, std::max (peak (l), peak (r)));
                // a reset after a failed solve would leave silence: the driven channels still have to be about as loud as dual's
                for (int ch = 0; ch < 2; ++ch)
                {
                    if (! (ch == 0 ? dA : dB)) continue;
                    const double got = rms (ch == 0 ? l : r, 0.4, 0.6), want = rms (ch == 0 ? l2 : r2, 0.4, 0.6);
                    const double dev = std::abs (db (got) - db (want));
                    worstDev = std::max (worstDev, dev);
                    ok = ok && got > 1.0e-3 && dev < 10.0;
                }
                if (! ok)
                {
                    ++bad;
                    std::printf ("        failed: ratio %d drive %d off %d\n", ratioIndex, drive, off);
                }
            }
    check (bad == 0, fmt ("%.0f cases, none non-finite, silent or over 40 (worst peak %.2f, worst level against dual %.1f dB)", cases, worstPeak, worstDev));
}

//== 5. real-time safety ==========================================================================
void realtimeSafety()
{
    std::printf ("real-time safety\n");
    Settings s;
    Pair p (s);
    std::vector<float> l (512), r (512);
    StereoRunner runner;
    runner.prepare (kFs);
    long total = 0;
    for (int b = 0; b < 300; ++b)
    {
        for (size_t i = 0; i < l.size(); ++i)
        {
            l[i] = (float) (0.3 * std::sin (2.0 * kPi * 440.0 * (double) (b * 512 + (int) i) / kFs));
            r[i] = (float) (0.1 * std::sin (2.0 * kPi * 660.0 * (double) (b * 512 + (int) i) / kFs));
        }
        gAllocations = 0;
        gCounting = true;
        for (FastChannel* c : { &p.a, &p.b })
        {
            c->setAttack (1.0f + (float) (b % 7));
            c->setRatio ((Ratio) (b % 6));
            c->setLimitOff (b % 11 == 0);
        }
        runner.process (p.a, p.b, l.data(), r.data(), 512, (StereoMode) ((b / 3) % 3));
        gCounting = false;
        total += gAllocations.load();
    }
    check (total == 0, fmt ("no allocation while the mode changes every 3 blocks and the controls move: %.0f allocations", (double) total));
}

//== 6. Mid-Side ==================================================================================
void midSide()
{
    std::printf ("mid-side\n");
    Settings s;
    const Signal t = tone (1.5, -16.0, 1000.0, 0.1);
    Pair dual (s), ms (s);
    Signal dl = t, dr = t, ml = t, mr = t;
    render (dual, Engine::dual, dl, dr, 128);
    StereoRunner runner;
    runner.prepare (kFs);
    runner.process (ms.a, ms.b, ml.data(), mr.data(), (int) ml.size(), StereoMode::midSide);
    // the morph from the identity takes 10 ms at the start; look after it
    const size_t from = (size_t) (0.05 * kFs);
    double e = 0.0, m = 1e-30;
    for (size_t i = from; i < dl.size(); ++i)
    {
        e = std::max (e, std::max ((double) std::abs (dl[i] - ml[i]), (double) std::abs (dr[i] - mr[i])));
        m = std::max (m, (double) std::abs (dl[i]));
    }
    check (e / m < 1.0e-6, fmt ("L == R: mid-side equals dual on both channels, largest difference %.2e of the peak (the S unit's idle output is the rest)", e / m));
    // The same after a morph: Dual for a while, then M/S
    Pair p2 (s);
    StereoRunner r2;
    r2.prepare (kFs);
    Signal a = t, b = t;
    r2.process (p2.a, p2.b, a.data(), b.data(), (int) (0.5 * kFs), StereoMode::dual);
    r2.process (p2.a, p2.b, a.data() + (int) (0.5 * kFs), b.data() + (int) (0.5 * kFs), (int) (kFs), StereoMode::midSide);
    double e2 = 0.0;
    for (size_t i = (size_t) (1.2 * kFs); i < dl.size(); ++i)
        e2 = std::max (e2, std::max ((double) std::abs (dl[i] - a[i]), (double) std::abs (dr[i] - b[i])));
    check (r2.midSideBlend() == 1.0 && e2 / m < 1.0e-3,
           fmt ("L == R: after switching from dual while playing, mid-side follows dual to %.2e of the peak once the S unit's input has faded out (>0.7 s)", e2 / m));

    // M / S of a stereo signal: L only gives M = S = L / 2, decoded sum
    Signal l = tone (1.0, -30.0, 1000.0), r ((size_t) l.size(), 0.0f);
    Settings lin;
    lin.ratio = Ratio::none;
    Pair p3 (lin);
    StereoRunner r3;
    r3.prepare (kFs);
    Signal lo = l, ro = r;
    r3.process (p3.a, p3.b, lo.data(), ro.data(), (int) lo.size(), StereoMode::midSide);
    Pair p4 (lin);
    Signal ld = l, rd = r;
    render (p4, Engine::dual, ld, rd, 128);
    check (std::abs (db (rms (lo, 0.5, 1.0)) - db (rms (ld, 0.5, 1.0))) < 0.5 && rms (ro, 0.5, 1.0) < 0.05 * rms (lo, 0.5, 1.0),
           fmt ("L only, no reduction: mid-side passes it to L (%.2f dB against dual) and R stays quiet (%.1f dB)",
                db (rms (lo, 0.5, 1.0)) - db (rms (ld, 0.5, 1.0)), db (rms (ro, 0.5, 1.0)) - db (rms (lo, 0.5, 1.0))));
}

//== 7. switching modes while playing ===============================================================
void transitions()
{
    std::printf ("switching modes while playing (L -12 dBFS 1 kHz, R -26 dBFS 1.7 kHz, ratio 4:1, input max)\n");
    const struct { StereoMode from, to; const char* name; } cases[] = {
        { StereoMode::dual, StereoMode::link, "dual -> link" }, { StereoMode::link, StereoMode::dual, "link -> dual" },
        { StereoMode::dual, StereoMode::midSide, "dual -> mid-side" }, { StereoMode::midSide, StereoMode::dual, "mid-side -> dual" },
        { StereoMode::link, StereoMode::midSide, "link -> mid-side" }, { StereoMode::midSide, StereoMode::link, "mid-side -> link" } };
    for (const auto& c : cases)
    {
        Settings s;
        const double total = 1.6, at = 0.8;
        const Signal l0 = tone (total, -12.0, 1000.0), r0 = tone (total, -26.0, 1700.0);
        auto run = [&] (bool switchMode, Signal& l, Signal& r, std::vector<double>& grA, std::vector<double>& grB)
        {
            Pair p (s);
            StereoRunner runner;
            runner.prepare (kFs);
            l = l0;
            r = r0;
            const int block = 16, n = (int) l.size();
            for (int i = 0; i < n; i += block)
            {
                const StereoMode mode = (switchMode && i >= (int) (at * kFs)) ? c.to : c.from;
                runner.process (p.a, p.b, l.data() + i, r.data() + i, std::min (block, n - i), mode);
                grA.push_back (p.a.currentGainReductionDb());
                grB.push_back (p.b.currentGainReductionDb());
            }
        };
        Signal l, r, lr, rr;
        std::vector<double> a1, b1, a0, b0;
        run (true, l, r, a1, b1);
        run (false, lr, rr, a0, b0);
        // sample-to-sample steps in the 5 ms after the switch against the largest in the 5 ms before
        const size_t s0 = (size_t) (at * kFs), w = (size_t) (0.005 * kFs);
        double before = 0.0, after = 0.0;
        for (size_t i = s0 - w; i < s0; ++i) before = std::max (before, std::max ((double) std::abs (l[i] - l[i - 1]), (double) std::abs (r[i] - r[i - 1])));
        for (size_t i = s0; i < s0 + w; ++i) after = std::max (after, std::max ((double) std::abs (l[i] - l[i - 1]), (double) std::abs (r[i] - r[i - 1])));
        double stepGr = 0.0;
        const size_t b0i = s0 / 16;
        for (size_t i = b0i - 4; i < b0i + 200 && i + 1 < a1.size(); ++i)
            stepGr = std::max (stepGr, std::max (std::abs (a1[i + 1] - a1[i]), std::abs (b1[i + 1] - b1[i])));
        double refStepGr = 0.0;
        for (size_t i = b0i - 204; i < b0i - 4; ++i)
            refStepGr = std::max (refStepGr, std::max (std::abs (a1[i + 1] - a1[i]), std::abs (b1[i + 1] - b1[i])));
        const size_t late = a1.size() - 1;
        std::printf ("        %-17s GR before %.2f / %.2f dB, 100 ms later %.2f / %.2f dB; largest 16-sample GR change %.3f dB (before the switch: %.3f dB); largest sample step %.4f (before: %.4f)\n",
                     c.name, a1[b0i - 1], b1[b0i - 1], a1[b0i + 300], b1[b0i + 300], stepGr, refStepGr, after, before);
        (void) late;
        check (finite (l) && finite (r) && after < 1.5 * before + 0.002 && stepGr < 0.6,
               std::string (c.name) + ": no click (sample steps stay near their steady size, gain moves under 0.6 dB per 16 samples)");
    }
}

//== 8. speed =====================================================================================
void performance()
{
    std::printf ("performance (stereo pair, 48 kHz, this machine, music-like signal with 5-15 dB GR, best of five)\n");
    const int n = (int) (4.0 * kFs);
    Signal xl ((size_t) n), xr ((size_t) n);
    for (int i = 0; i < n; ++i)
    {
        const double t = i / kFs, env = 1.0 + 0.5 * std::sin (2 * kPi * 2 * t);
        xl[(size_t) i] = (float) (fromDb (-40.0) * (0.5 * std::sin (2 * kPi * 110 * t) + 0.3 * std::sin (2 * kPi * 440 * t) + 0.2 * std::sin (2 * kPi * 3000 * t)) * env);
        xr[(size_t) i] = (float) (fromDb (-43.0) * (0.4 * std::sin (2 * kPi * 130 * t) + 0.4 * std::sin (2 * kPi * 520 * t) + 0.2 * std::sin (2 * kPi * 2500 * t)) * (2.0 - env));
    }
    double rates[3] {};
    const StereoMode modes[3] = { StereoMode::dual, StereoMode::link, StereoMode::midSide };
    const char* names[3] = { "dual", "link", "mid-side" };
    for (int m = 0; m < 3; ++m)
    {
        Settings s;
        Pair p (s);
        StereoRunner runner;
        runner.prepare (kFs);
        double secs = 1e30;
        for (int rep = 0; rep < 5; ++rep)
        {
            p.a.reset();
            p.b.reset();
            Signal l = xl, r = xr;
            const std::clock_t t0 = std::clock();
            for (int i = 0; i < n; i += 256)
                runner.process (p.a, p.b, l.data() + i, r.data() + i, std::min (256, n - i), modes[m]);
            secs = std::min (secs, (double) (std::clock() - t0) / (double) CLOCKS_PER_SEC);
        }
        rates[m] = n / secs;
        std::printf ("        %-8s %9.0f samples/s per channel pair = %.1fx real time stereo (GR %.1f / %.1f dB)\n", names[m], rates[m], rates[m] / kFs,
                     (double) p.a.getGainReductionDb(), (double) p.b.getGainReductionDb());
    }
    std::printf ("        cost against dual: link %.2fx, mid-side %.2fx\n", rates[0] / rates[1], rates[0] / rates[2]);
    check (rates[0] / rates[1] < 1.5, fmt ("link costs %.2fx dual", rates[0] / rates[1]));
    check (rates[0] / rates[2] < 1.15, fmt ("mid-side costs %.2fx dual", rates[0] / rates[2]));
}
} // namespace

int main (int argc, char** argv)
{
    // MSVC's _IOLBF validates the buffer size as 2..INT_MAX even when buffer is null, so 0
    // invokes the invalid-parameter handler and fast-fails before the first test output.
    if (std::setvbuf (stdout, nullptr, _IONBF, 0) != 0)
    {
        std::fprintf (stderr, "failed to disable stdout buffering\n");
        return 1;
    }
    std::vector<std::string> wanted;
    for (int i = 1; i < argc; ++i) wanted.push_back (argv[i]);
    struct Section { const char* name; void (*fn)(); };
    const Section sections[] = { { "identical", identicalInputs }, { "oneside", oneSidedDrive }, { "times", timesTest }, { "robust", robustness },
                                 { "rt", realtimeSafety }, { "ms", midSide }, { "switch", transitions }, { "perf", performance } };
    std::printf ("stereo modes (fast engine)\n");
    for (const auto& sec : sections)
    {
        // perf compares timings, which a loaded machine or CI runner can upset: run it only when asked
        bool want = wanted.empty() && std::string (sec.name) != "perf";
        for (const auto& w : wanted) want = want || w == sec.name;
        if (want) sec.fn();
    }
    std::printf (failures == 0 ? "all checks passed\n" : "%d check(s) failed\n", failures);
    return failures == 0 ? 0 : 1;
}
