// Self-checking tests of the stereo modes around the fast engine: the 1176SA tie of the control nodes (Link) against two independent
// units (Dual), Mid-Side, the transitions between modes while playing, robustness at +24 dBu, real-time safety and speed.
// Prints [ok] / [FAIL] per check and the measured numbers; exits non-zero on any failure.

#include "dsp/Fast1176.h"
#include "dsp/StereoModes.h"
#include "dsp/Urei1176Circuit.h"
#include "dsp/VersionSwitch.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <functional>
#include <map>
#include <memory>
#include <new>
#include <string>
#include <thread>
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

/** The revision the single-version sections (1 to 8) run as (--version D|F|G|J; the sections of the switching machinery name their own). */
Version gVersion = Version::F;
/** --trims off takes the version's level trims off in the single-version sections (diagnostic; the plug-in plays with them on). */
bool gTrims = true;

struct Settings
{
    float input = Range::inputMax;
    float attack = 4.0f, release = 4.0f;
    Ratio ratio = Ratio::r4;
    bool offA = false, offB = false;
    int hpf = 0;   // S/C HPF choice, both units
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
            c->setSidechainFilter (s.hpf);
        }
        a.setLimitOff (s.offA);
        b.setLimitOff (s.offB);
        a.setVersionTrims (gTrims);
        b.setVersionTrims (gTrims);
        a.prepare (fs, gVersion);
        b.prepare (fs, gVersion);
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
    for (const int hpf : { 0, 5 })
    for (const auto& c : cases)
    {
        Settings s;
        s.ratio = c.ratio;
        s.hpf = hpf;
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
               std::string (hpf ? "S/C HPF 185, " : "") + c.name + fmt (": control voltage differs by %.1e (relative, double); output by %.1e of its peak (a float ulp at the peak is 6e-8)", e, o));
    }
}

//== 2. signal on L only ==========================================================================
void oneSidedDrive()
{
    std::printf ("link with signal on L only\n");
    for (const int hpf : { 0, 5 })
    for (Ratio ratio : { Ratio::r4, Ratio::all })
    {
        Settings s;
        s.ratio = ratio;
        s.hpf = hpf;
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
        const std::string name = std::string (hpf ? "S/C HPF 185, " : "") + (ratio == Ratio::r4 ? "4:1" : "All");
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
    // The deviation of the unit with limiting OFF (driven +6 dBFS, clipping in Dual) when Link ties it to the other unit's reduction is the
    // gap between an unlimited and a fully limited output at this drive. F: heuristic 10 dB. G: the reference engine's own gap
    // (urei1176::Channel, 0.6 s tone of amplitude 2, limiting OFF against ON) is 11.4 dB at 4:1 and 12.0 dB at All, F's 10.7 and 10.7, since
    // G's unlimited output is 0.8 dB higher (and its All-ratio limited output 0.4 dB lower); Link's shared control node adds up to 0.6 dB.
    // The fast G (11.2 dB at 4:1, 13.4 dB at All) follows the reference; F's 8 to 9.7 dB comes from its fast engine reading about 3 dB hot
    // there. So G's bound is 14 dB (the reference's 12.0 + 0.6 + margin). J has G's input, cell and line amp: its reference gap is 11.5 dB at 4:1 and
    // 12.0 dB at All (the same measurement, 11.5 / 12.0 with J's trims on), so it has the same 14 dB; the fast J reads 13.0 dB.
    const double devBound = gVersion == Version::G || gVersion == Version::J ? 14.0 : 10.0;
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
                    ok = ok && got > 1.0e-3 && dev < devBound;
                    if (dev >= devBound) std::printf ("        (ratio %d drive %d off %d, channel %d: linked %.1f dBFS rms, dual %.1f dBFS rms)\n", ratioIndex, drive, off, ch, db (got), db (want));
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
            c->setSidechainFilter ((b / 2) % 6);
        }
        runner.process (p.a, p.b, l.data(), r.data(), 512, (StereoMode) ((b / 3) % 3));
        gCounting = false;
        total += gAllocations.load();
    }
    check (total == 0, fmt ("no allocation while the mode changes every 3 blocks, the S/C HPF every 2 and the controls move: %.0f allocations", (double) total));
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
    // With the S/C HPF on, the M and S detectors are filtered and L == R still gives the same as Dual (S is silent)
    {
        Settings h;
        h.hpf = 5;
        Pair dualH (h), msH (h);
        Signal hl = tone (1.5, -16.0, 1000.0, 0.1), hr = hl, ql = hl, qr = hl;
        for (size_t i = 0; i < hl.size(); ++i) { const float bass = (float) (0.05 * std::sin (2.0 * kPi * 50.0 * (double) i / kFs)); hl[i] += bass; hr[i] += bass; ql[i] += bass; qr[i] += bass; }
        render (dualH, Engine::dual, hl, hr, 128);
        StereoRunner runnerH;
        runnerH.prepare (kFs);
        runnerH.snapTo (StereoMode::midSide);   // (the morph from the identity would leave S a fading copy of the bass)
        runnerH.process (msH.a, msH.b, ql.data(), qr.data(), (int) ql.size(), StereoMode::midSide);
        double eh = 0.0, mh = 1e-30;
        for (size_t i = from; i < hl.size(); ++i)
        {
            eh = std::max (eh, std::max ((double) std::abs (hl[i] - ql[i]), (double) std::abs (hr[i] - qr[i])));
            mh = std::max (mh, (double) std::abs (hl[i]));
        }
        check (eh / mh < 1.0e-6, fmt ("S/C HPF 185, L == R with bass: mid-side equals dual, largest difference %.2e of the peak", eh / mh));
    }
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

//== 7b. the S/C HPF ===============================================================================
/** Steady-state gain reduction (dB) of one unit, driven by a tone, with an S/C HPF choice, at a sample rate. */
double grWith (int hpf, double hz, double dbfs, double fs, Ratio ratio = Ratio::r4)
{
    FastChannel c;
    c.setInput (Range::inputMax);
    c.setRatio (ratio);
    c.setSidechainFilter (hpf);
    c.prepare (fs, gVersion);
    const int n = (int) std::lround (2.0 * fs);
    std::vector<float> x ((size_t) n);
    for (int i = 0; i < n; ++i) x[(size_t) i] = (float) (fromDb (dbfs) * std::sin (2.0 * kPi * hz * (double) i / fs));
    // the GR is read from the control voltage's own state: the mean over the last 0.25 s of per-block values
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < n; i += 64)
    {
        c.process (x.data() + i, std::min (64, n - i));
        if (i > n - (int) (0.25 * fs)) { sum += c.currentGainReductionDb(); ++count; }
    }
    return sum / count;
}

void sidechainFilterResponse()
{
    std::printf ("S/C HPF: detector response (4:1, input max, steady state)\n");
    // 40 Hz, several dB of reduction: the higher the corner, the less it reduces
    double gr[6];
    bool monotonic = true;
    std::printf ("        40 Hz, -12 dBFS, GR by choice (OFF 30 60 105 125 185):");
    for (int i = 0; i < 6; ++i)
    {
        gr[i] = grWith (i, 40.0, -12.0, kFs);
        std::printf (" %.2f", gr[i]);
        if (i > 0 && ! (gr[i] < gr[i - 1] - 0.05)) monotonic = false;
    }
    std::printf (" dB\n");
    check (gr[0] > 4.0 && monotonic, fmt ("40 Hz drives %.1f dB of reduction with the filter off and the reduction falls with every higher corner (185 Hz: %.2f dB)", gr[0], gr[5]));
    // |H| of a first-order high-pass at 1 kHz is -0.15 dB for 185 Hz: the reduction moves by about that much (the loop gain is below one)
    const double offK = grWith (0, 1000.0, -12.0, kFs), onK = grWith (5, 1000.0, -12.0, kFs);
    const double predicted = -db (1.0 / std::sqrt (1.0 + std::pow (185.0 / 1000.0, 2.0)));
    check (offK - onK > 0.0 && offK - onK < predicted + 0.03 && offK > 3.0,
           fmt ("1 kHz: reduction %.3f dB off, %.3f dB at 185 Hz; the change %.3f dB stays within the first-order magnitude, %.3f dB", offK, onK, offK - onK, predicted));
    // the corner at the sample rate in use: the same reduction change at 185 Hz (a tone at the corner) at every rate
    double change[4];
    const double rates[4] = { 44100.0, 48000.0, 96000.0, 192000.0 };
    std::printf ("        reduction change, 185 Hz tone at -12 dBFS, filter 185 Hz against off, at 44.1 / 48 / 96 / 192 kHz:");
    for (int i = 0; i < 4; ++i)
    {
        change[i] = grWith (0, 185.0, -12.0, rates[i]) - grWith (5, 185.0, -12.0, rates[i]);
        std::printf (" %.3f", change[i]);
    }
    std::printf (" dB\n");
    double spread = 0.0;
    for (int i = 1; i < 4; ++i) spread = std::max (spread, std::abs (change[i] - change[0]));
    check (change[1] > 0.5 && spread < 0.08, fmt ("the corner stays at 185 Hz across sample rates: the changes agree within %.3f dB (and exceed the 1 kHz one)", spread));
    // and a corner is a corner: the 105 Hz filter at 105 Hz acts like the 185 Hz one at 185 Hz, at 44.1 and 192 kHz
    const double c105 = grWith (0, 105.0, -12.0, 44100.0) - grWith (3, 105.0, -12.0, 44100.0), c105h = grWith (0, 105.0, -12.0, 192000.0) - grWith (3, 105.0, -12.0, 192000.0);
    check (std::abs (c105 - c105h) < 0.08 && c105 > 0.5, fmt ("105 Hz corner: change %.3f dB at 44.1 kHz, %.3f dB at 192 kHz", c105, c105h));
}

/** The filter off is the filter absent. A unit that is told "off" every block is bit for bit one that is never told (the arithmetic is exact: the
    feedthrough is 1 and the history term -0). A unit that had the filter on and then off again, 0.3 s of silence each, differs from one that
    never did only by what the glide left in its state (the idle preamp output is not exactly zero, so the low-pass holds a sub-microvolt
    value that the glide briefly let through): far below a float ulp of the signal, which the test bounds. */
void sidechainFilterOff()
{
    std::printf ("S/C HPF: off is the filter absent\n");
    for (const auto mode : { StereoMode::dual, StereoMode::link, StereoMode::midSide })
    {
        const std::string name = mode == StereoMode::dual ? "dual" : mode == StereoMode::link ? "link" : "mid-side";
        Settings s;
        Pair never (s), told (s), toggled (s);
        StereoRunner r0, r1, r2;
        for (auto* r : { &r0, &r1, &r2 }) { r->prepare (kFs); r->snapTo (mode); }
        Signal sl ((size_t) (0.3 * kFs), 0.0f), sr = sl;
        for (FastChannel* c : { &toggled.a, &toggled.b }) c->setSidechainFilter (4);
        r2.process (toggled.a, toggled.b, sl.data(), sr.data(), (int) sl.size(), mode);
        for (FastChannel* c : { &toggled.a, &toggled.b }) c->setSidechainFilter (0);
        std::fill (sl.begin(), sl.end(), 0.0f);
        std::fill (sr.begin(), sr.end(), 0.0f);
        r2.process (toggled.a, toggled.b, sl.data(), sr.data(), (int) sl.size(), mode);   // the 10 ms glide ends here
        const Signal x = tone (1.0, -14.0, 440.0), y = tone (1.0, -20.0, 1300.0);
        Signal a0 = x, b0 = y, a1 = x, b1 = y, a2 = x, b2 = y;
        Signal z0 ((size_t) (0.6 * kFs), 0.0f), z1 = z0, z2 = z0, z3 = z0;   // (a unit's output for silence is not exactly zero: each gets its own)
        r0.process (never.a, never.b, z0.data(), z1.data(), (int) z0.size(), mode);
        r1.process (told.a, told.b, z2.data(), z3.data(), (int) z2.size(), mode);
        for (int i = 0; i < (int) a0.size(); i += 64)
        {
            const int c = std::min (64, (int) a0.size() - i);
            for (FastChannel* f : { &told.a, &told.b }) f->setSidechainFilter (0);
            r0.process (never.a, never.b, a0.data() + i, b0.data() + i, c, mode);
            r1.process (told.a, told.b, a1.data() + i, b1.data() + i, c, mode);
        }
        r2.process (toggled.a, toggled.b, a2.data(), b2.data(), (int) a2.size(), mode);
        const bool same = std::memcmp (a0.data(), a1.data(), a0.size() * sizeof (float)) == 0 && std::memcmp (b0.data(), b1.data(), b0.size() * sizeof (float)) == 0;
        double d = 0.0;
        for (size_t i = 0; i < a0.size(); ++i) d = std::max (d, std::max ((double) std::abs (a0[i] - a2[i]), (double) std::abs (b0[i] - b2[i])));
        check (same, name + ": told 'off' every block: output bit-identical to a unit that is never told");
        check (d < 1.0e-6 * peak (a0), fmt ("... and after on, then off with the glide finished, the output differs from it by %.1e (the peak is %.3f)", d, peak (a0)));
    }
}

/** Switching the filter while driven (a 1 kHz tone with 45 Hz bass on L, other tones on R): no step beyond the compressor's own. The criterion
    is the one of the stereo-mode switch test: the largest sample-to-sample step of the output in the 5 ms after the switch stays under 1.5 times
    the largest in the 5 ms before it, and the gain reduction never moves by more than 0.6 dB in 16 samples. */
void sidechainFilterSwitching()
{
    std::printf ("S/C HPF: switching while playing (L 1 kHz + 45 Hz, R 700 Hz + 60 Hz, each -16 dBFS, ratio 4:1, input max)\n");
    const struct { int from, to; const char* name; } changes[] = { { 0, 5, "off -> 185" }, { 5, 0, "185 -> off" }, { 1, 5, "30 -> 185" }, { 5, 2, "185 -> 60" } };
    const struct { StereoMode mode; const char* name; } modes[] = { { StereoMode::dual, "dual" }, { StereoMode::link, "link" }, { StereoMode::midSide, "mid-side" } };
    for (const auto& m : modes)
        for (const auto& c : changes)
        {
            Settings s;
            s.hpf = c.from;
            Pair p (s);
            StereoRunner runner;
            runner.prepare (kFs);
            runner.snapTo (m.mode);
            const double total = 1.6, at = 0.8;
            Signal l = tone (total, -16.0, 1000.0), r = tone (total, -16.0, 700.0);
            const Signal bl = tone (total, -16.0, 45.0), br = tone (total, -16.0, 60.0);
            for (size_t i = 0; i < l.size(); ++i) { l[i] += bl[i]; r[i] += br[i]; }
            std::vector<double> gr;
            const int block = 16, n = (int) l.size();
            for (int i = 0; i < n; i += block)
            {
                if (i == (int) (at * kFs) / block * block) { p.a.setSidechainFilter (c.to); p.b.setSidechainFilter (c.to); }
                runner.process (p.a, p.b, l.data() + i, r.data() + i, std::min (block, n - i), m.mode);
                gr.push_back (std::max (p.a.currentGainReductionDb(), p.b.currentGainReductionDb()));
            }
            const size_t s0 = (size_t) (at * kFs), w = (size_t) (0.005 * kFs);
            double before = 0.0, after = 0.0, stepGr = 0.0;
            for (size_t i = s0 - w; i < s0; ++i) before = std::max (before, std::max ((double) std::abs (l[i] - l[i - 1]), (double) std::abs (r[i] - r[i - 1])));
            for (size_t i = s0; i < s0 + w; ++i) after = std::max (after, std::max ((double) std::abs (l[i] - l[i - 1]), (double) std::abs (r[i] - r[i - 1])));
            for (size_t i = s0 / (size_t) block - 4; i < s0 / (size_t) block + 300 && i + 1 < gr.size(); ++i) stepGr = std::max (stepGr, std::abs (gr[i + 1] - gr[i]));
            std::printf ("        %-9s %-12s GR before %.2f dB, 150 ms later %.2f dB; largest 16-sample GR change %.3f dB; largest sample step %.4f (before: %.4f)\n", m.name, c.name,
                         gr[s0 / (size_t) block - 1], gr[s0 / (size_t) block + 450], stepGr, after, before);
            check (finite (l) && finite (r) && after < 1.5 * before + 0.002 && stepGr < 0.6,
                   std::string (m.name) + ", " + c.name + ": no click (sample steps stay near their steady size, gain moves under 0.6 dB per 16 samples)");
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
//== 9. version switching ==========================================================================
// The switching machinery (dsp/VersionSwitch.h) between F, G and J (the exposed Version choices; D is deferred, see docs/research/rev-d-data-search.md).
// Each is a real engine (G and J have their own input, cell and line-amp values, level-trimmed to F). A switch is a real engine swap (a freshly prepared engine
// adopting the outgoing one's gain-reduction memory): the tests look for clicks, level steps, NaN, wrong fade lengths and allocation.
struct Rig
{
    VersionSwitch sw;
    std::atomic<float> input { Range::inputMax }, attack { 4.0f }, release { 4.0f };
    std::atomic<int> ratio { (int) Ratio::r4 }, hpf { 0 };
    double fs = kFs;

    static void push (void* self, FastChannel& c) noexcept
    {
        const auto& r = *static_cast<const Rig*> (self);
        c.setInput (r.input.load());
        c.setAttack (r.attack.load());
        c.setRelease (r.release.load());
        c.setRatio ((Ratio) r.ratio.load());
        c.setMeterMode (MeterMode::gainReduction);
        c.setSidechainFilter (r.hpf.load());
    }
    explicit Rig (double rate = kFs, int maxBlock = 512, Version v = Version::F, StereoMode m = StereoMode::dual) : fs (rate)
    {
        sw.setControlSource (&Rig::push, this);
        sw.prepare (rate, maxBlock, v, m);
    }
};

/** The settle worker: VersionSwitch::pollSettle every 2 ms, as the processor's thread does. */
struct Poller
{
    std::atomic<bool> run { true };
    std::thread t;
    explicit Poller (VersionSwitch& s) : t ([this, &s] { while (run.load()) { s.pollSettle(); std::this_thread::sleep_for (std::chrono::milliseconds (2)); } }) {}
    ~Poller() { run = false; t.join(); }
};

/** The input of every switching test: two different tones (so that L != R), continuing across calls through `phase`. */
struct Feed
{
    double phase = 0.0, ampL, ampR;
    double fs;
    Feed (double level, double rate = kFs) : ampL (level), ampR (0.7 * level), fs (rate) {}
    void fill (float* l, float* r, int n)
    {
        for (int i = 0; i < n; ++i)
        {
            l[i] = (float) (ampL * std::sin (phase));
            r[i] = (float) (ampR * std::sin (1.5 * phase));
            phase += 2.0 * kPi * 1000.0 / fs;
        }
    }
};

/** Runs `count` samples through the rig in blocks of `block`, appending the output to `outL` / `outR`; `each(blockStartIndex)` is called
    before every block (to move controls or the wanted version). */
void drive (Rig& rig, Feed& feed, int count, int block, Version& want, StereoMode mode, bool offline, Signal& outL, Signal& outR,
            const std::function<void (int)>& each = {}, int channels = 2)
{
    std::vector<float> l ((size_t) block), r ((size_t) block);
    for (int done = 0; done < count; done += block)
    {
        const int n = std::min (block, count - done);
        if (each) each ((int) outL.size());
        feed.fill (l.data(), r.data(), n);
        float* ptrs[2] = { l.data(), r.data() };
        rig.sw.process (ptrs, channels, n, want, mode, offline);
        outL.insert (outL.end(), l.begin(), l.begin() + n);
        if (channels == 2) outR.insert (outR.end(), r.begin(), r.begin() + n);
    }
}

double maxStep (const Signal& x, size_t from, size_t to)
{
    double m = 0.0;
    for (size_t i = std::max<size_t> (from, 1); i < std::min (to, x.size()); ++i) m = std::max (m, (double) std::abs (x[i] - x[i - 1]));
    return m;
}

/** The RMS of x over 10 ms windows (ten cycles of the 1 kHz tone); `index` is the window's start in samples. */
double windowDb (const Signal& x, size_t start, double rate = kFs)
{
    const size_t n = (size_t) std::lround (0.010 * rate);
    double s = 0.0;
    for (size_t i = start; i < std::min (x.size(), start + n); ++i) s += (double) x[i] * x[i];
    return db (std::sqrt (s / (double) n));
}

Version all3 (int i) { static const Version v[3] = { Version::F, Version::G, Version::J }; return v[i % 3]; }

/** The level (dBFS, a 1 kHz tone on the left) that gives `targetGr` dB of gain reduction at Input full, 4:1 (found once per target). */
double levelForGr (double targetGr)
{
    static std::map<int, double> cache;
    const int key = (int) std::lround (targetGr * 10.0);
    const auto found = cache.find (key);
    if (found != cache.end()) return found->second;
    if (targetGr <= 0.0) return cache[key] = -60.0;
    double lo = -60.0, hi = 0.0;
    for (int it = 0; it < 9; ++it)
    {
        const double mid = 0.5 * (lo + hi);
        Rig rig;
        Feed feed (fromDb (mid));
        Version want = Version::F;
        Signal l, r;
        drive (rig, feed, (int) (1.0 * kFs), 512, want, StereoMode::dual, true, l, r);
        (rig.sw.channel (0).currentGainReductionDb() < targetGr ? lo : hi) = mid;
    }
    return cache[key] = 0.5 * (lo + hi);
}

void switching()
{
    std::printf ("version switching (F, G, J)\n");
    const Version all[3] = { Version::F, Version::G, Version::J };
    const double targets[3] = { 0.0, 6.0, 12.0 };

    // --- all 6 ordered pairs at 0, 6 and 12 dB of gain reduction ---------------------------------------------------------------
    for (double target : targets)
    {
        const double level = levelForGr(target);
        double worstRatio = 0.0, worstEnv = 0.0;
        bool finiteAll = true, endsRight = true, fadeOk = true;
        double measuredGr = 0.0;
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b)
            {
                if (a == b) continue;
                const int pre = (int) (0.8 * kFs), post = (int) (0.3 * kFs);
                Signal l, r, bl, br;
                {
                    Rig rig (kFs, 512, all[a]);
                    Feed feed (fromDb (level));
                    Version want = all[a];
                    drive (rig, feed, pre, 128, want, StereoMode::dual, true, l, r);
                    measuredGr = rig.sw.channel (0).currentGainReductionDb();
                    want = all[b];
                    drive (rig, feed, post, 128, want, StereoMode::dual, true, l, r);
                    finiteAll = finiteAll && finite (l) && finite (r);
                    endsRight = endsRight && rig.sw.activeVersion() == all[b] && ! rig.sw.isFading();
                }
                {
                    // the same input without a switch: the engines' own inter-sample change
                    Rig rig (kFs, 512, all[a]);
                    Feed feed (fromDb (level));
                    Version want = all[a];
                    drive (rig, feed, pre + post, 128, want, StereoMode::dual, true, bl, br);
                }
                // and through the incoming engine alone: the fade mixes two different waveforms, so the bound is the larger of
                // the two engines' own steps (plan 7.4)
                Signal cl, cr;
                {
                    Rig rig (kFs, 512, all[b]);
                    Feed feed (fromDb (level));
                    Version want = all[b];
                    drive (rig, feed, pre + post, 128, want, StereoMode::dual, true, cl, cr);
                }
                const size_t from = (size_t) pre, to = (size_t) pre + (size_t) std::lround (0.020 * kFs) + 64;
                const double ref = std::max ({ maxStep (bl, from, to), maxStep (br, from, to), maxStep (cl, from, to), maxStep (cr, from, to) });
                const double got = std::max (maxStep (l, from, to), maxStep (r, from, to));
                worstRatio = std::max (worstRatio, got / ref);
                // the level after the fade against the level before (the same program through an engine that adopted the gain reduction)
                double env = 0.0;
                for (size_t w = from + (size_t) (0.030 * kFs); w + 480 < l.size(); w += 480)
                    env = std::max (env, std::abs (windowDb (l, w) - windowDb (bl, w)));
                worstEnv = std::max (worstEnv, env);
                fadeOk = fadeOk && ! (got > 1.01 * ref + 1e-6);
            }
        char what[300];
        std::snprintf (what, sizeof (what), "6 ordered pairs at %.0f dB GR (measured %.1f dB): no NaN, ends on the new version, no sample step above the larger of the two engines' own (worst %.4fx), level within %.2f dB of no switch",
                       target, measuredGr, worstRatio, worstEnv);
        check (finiteAll && endsRight && fadeOk && worstEnv < 0.5, what);
    }

    // --- fade length: 20 ms +-1 sample, at every rate ---------------------------------------------------------------------------
    for (double rate : { 44100.0, 48000.0, 88200.0, 96000.0, 176400.0, 192000.0 })
    {
        Rig rig (rate, 256, Version::F);
        Feed feed (fromDb (-30.0), rate);
        Version want = Version::F;
        Signal l, r;
        drive (rig, feed, (int) (0.3 * rate), 256, want, StereoMode::dual, true, l, r);
        want = Version::G;
        int calls = 0;
        std::vector<float> a (1), b (1);
        bool finiteOut = true;
        do
        {
            feed.fill (a.data(), b.data(), 1);
            float* ptrs[2] = { a.data(), b.data() };
            rig.sw.process (ptrs, 2, 1, want, StereoMode::dual, true);
            finiteOut = finiteOut && std::isfinite (a[0]) && std::isfinite (b[0]);
            ++calls;
        } while (rig.sw.isFading() && calls < 100000);
        const double expected = 0.020 * rate;
        check (std::abs(calls - expected) <= 1.0 && calls == rig.sw.fadeLengthSamples() && finiteOut && rig.sw.activeVersion() == Version::G,
               fmt ("%.0f Hz: the fade lasts %.0f samples (20 ms = %.1f)", rate, (double) calls, expected));
    }

    // --- the same fade, in blocks that end it mid-block, and blocks longer than the scratch ------------------------------------
    {
        Rig rig (kFs, 512, Version::F);
        Feed feed (fromDb (-30.0));
        Version want = Version::F;
        Signal l, r;
        drive (rig, feed, (int) (0.3 * kFs), 512, want, StereoMode::dual, true, l, r);
        want = Version::J;
        drive (rig, feed, 20000, 20000, want, StereoMode::dual, true, l, r);   // one block of 20000 samples, over the 8192-sample scratch
        check (finite (l) && finite (r) && rig.sw.activeVersion() == Version::J && ! rig.sw.isFading(),
               "a block longer than the scratch (20000 samples) holds the whole fade, in chunks, and ends on the new version");
    }

    // --- rapid automation: F G J every 3 blocks, with the worker (the real-time path) and without (offline) ------------------
    for (const bool offline : { true, false })
    {
        Rig rig (kFs, 128, Version::F);
        std::unique_ptr<Poller> poller;
        if (! offline) poller = std::make_unique<Poller> (rig.sw);
        Feed feed (fromDb (levelForGr (6.0)));
        Version want = Version::F;
        Signal l, r;
        int fades = 0;
        bool wasFading = false;
        drive (rig, feed, (int) (0.3 * kFs), 128, want, StereoMode::dual, offline, l, r);
        drive (rig, feed, 128 * 3 * 40, 128, want, StereoMode::dual, offline, l, r, [&] (int index)
        {
            want = all[(index / (128 * 3)) % 3];
            if (! offline) std::this_thread::sleep_for (std::chrono::microseconds (2000));   // about real time for a block of 128
            const bool f = rig.sw.isFading();
            if (f && ! wasFading) ++fades;
            wasFading = f;
        });
        want = Version::J;
        drive (rig, feed, (int) (0.4 * kFs), 128, want, StereoMode::dual, offline, l, r, [&] (int) { if (! offline) std::this_thread::sleep_for (std::chrono::microseconds (2000)); });
        const size_t from = (size_t) (0.3 * kFs);
        char what[200];
        std::snprintf (what, sizeof (what), "%s: automation F G J every 3 blocks of 128 for 120 blocks: %d fades, no NaN, bounded (peak %.3f), settles on J",
                       offline ? "offline" : "worker", fades, std::max (peak (l), peak (r)));
        check (finite (l) && finite (r) && std::max (peak (l), peak (r)) < 1.5 && rig.sw.activeVersion() == Version::J && ! rig.sw.isFading() && fades >= 3
                   && maxStep (l, from, l.size()) < 0.3,
               what);
    }

    // --- a request for the version already active does nothing; one withdrawn before the settle is dropped (no worker thread: exact) ---
    {
        Rig rig (kFs, 128, Version::F);
        Feed feed (fromDb (-30.0));
        Version want = Version::G;
        Signal l, r;
        drive (rig, feed, 128, 128, want, StereoMode::dual, false, l, r);   // G is asked for; nothing settles it (no worker)
        want = Version::F;
        drive (rig, feed, 128, 128, want, StereoMode::dual, false, l, r);   // ... and withdrawn
        const bool built = rig.sw.pollSettle();                             // the request now names F, the active version: nothing to build
        check (! built && rig.sw.standbyStateForTest() == 0 && rig.sw.activeVersion() == Version::F && ! rig.sw.isFading(),
               "a version asked for and withdrawn before it settled: the worker builds nothing, the active set plays on");
        want = Version::G;
        drive (rig, feed, 128, 128, want, StereoMode::dual, false, l, r);
        const bool builtG = rig.sw.pollSettle();                            // settled (state ready) ...
        const int readyState = rig.sw.standbyStateForTest();
        want = Version::F;
        drive (rig, feed, 128, 128, want, StereoMode::dual, false, l, r);   // ... and withdrawn: the ready result is discarded
        check (builtG && readyState == 2 && rig.sw.standbyStateForTest() == 0 && ! rig.sw.isFading() && rig.sw.activeVersion() == Version::F && finite (l),
               "a settled result nobody wants any more is discarded, no fade starts");
        want = Version::J;
        drive (rig, feed, 128, 128, want, StereoMode::dual, false, l, r);
        want = Version::G;                                                  // changes its mind before the worker looks: only the newest request is built
        drive (rig, feed, 128, 128, want, StereoMode::dual, false, l, r);
        rig.sw.pollSettle();
        drive (rig, feed, 128, 128, want, StereoMode::dual, false, l, r);
        check (rig.sw.isFading() && rig.sw.targetVersion() == Version::G, "switch again before the settle: only the newest request (G) is built and faded in");
        // the same version again: nothing happens
        Rig same (kFs, 128, Version::G);
        Version wantG = Version::G;
        Signal sl, sr;
        bool fadeSeen = false;
        drive (same, feed, (int) (0.3 * kFs), 128, wantG, StereoMode::dual, true, sl, sr, [&] (int) { fadeSeen = fadeSeen || same.sw.isFading(); });
        check (! fadeSeen && same.sw.standbyStateForTest() == 0, "a request for the version already active starts no fade and builds nothing");
    }

    // --- switch again mid-fade: the running fade completes, then the newest request is settled and faded in -------------------------
    {
        Rig rig (kFs, 128, Version::F);
        Feed feed (fromDb (levelForGr (6.0)));
        Version want = Version::F;
        Signal l, r;
        drive (rig, feed, (int) (0.3 * kFs), 128, want, StereoMode::dual, true, l, r);
        want = Version::G;
        drive (rig, feed, 256, 128, want, StereoMode::dual, true, l, r);
        const bool fadingG = rig.sw.isFading() && rig.sw.targetVersion() == Version::G;
        want = Version::J;
        drive (rig, feed, 128, 128, want, StereoMode::dual, true, l, r);   // J asked for mid-fade: ignored until the G fade ends
        const bool stillG = rig.sw.targetVersion() == Version::G || rig.sw.activeVersion() == Version::G;
        drive (rig, feed, (int) (0.3 * kFs), 128, want, StereoMode::dual, true, l, r);
        check (finite (l) && fadingG && stillG && rig.sw.activeVersion() == Version::J && ! rig.sw.isFading(),
               "a switch asked for mid-fade waits for the running fade, then runs: G, then J");
    }

    // --- a switch during Link, and during the Mid-Side morph --------------------------------------------------------------------
    for (int variant = 0; variant < 3; ++variant)
    {
        const char* label[3] = { "Link", "a Mid-Side morph", "Link engaging while the mode morphs back from Mid-Side" };
        Signal l, r, bl, br;
        const int pre = (int) (0.5 * kFs), post = (int) (0.25 * kFs);
        const StereoMode before = variant == 2 ? StereoMode::midSide : (variant == 0 ? StereoMode::link : StereoMode::dual);
        const StereoMode after = variant == 0 ? StereoMode::link : (variant == 1 ? StereoMode::midSide : StereoMode::link);
        for (int pass = 0; pass < 2; ++pass)
        {
            Rig rig (kFs, 256, Version::F, before);
            Feed feed (fromDb (levelForGr (6.0)));
            Version want = Version::F;
            Signal& ol = pass == 0 ? l : bl;
            Signal& orr = pass == 0 ? r : br;
            drive (rig, feed, pre, 128, want, before, true, ol, orr);
            // the mode change and the switch land together (the morph and the glide are 10 and 15 ms, the fade 20 ms)
            if (pass == 0) want = Version::G;
            drive (rig, feed, post, 128, want, after, true, ol, orr);
        }
        const size_t from = (size_t) pre, to = (size_t) pre + (size_t) (0.030 * kFs);
        const double ref = std::max (maxStep (bl, from, to), maxStep (br, from, to));
        const double got = std::max (maxStep (l, from, to), maxStep (r, from, to));
        check (finite (l) && finite (r) && got <= 1.05 * ref + 1e-6,
               std::string ("switch during ") + label[variant] + fmt (": no NaN, steps %.5f vs %.5f without the switch", got, ref));
    }

    // --- mono ------------------------------------------------------------------------------------------------------------------
    {
        Rig rig (kFs, 256, Version::F);
        Feed feed (fromDb (levelForGr (6.0)));
        Version want = Version::F;
        Signal l, r;
        drive (rig, feed, (int) (0.5 * kFs), 128, want, StereoMode::dual, true, l, r, {}, 1);
        want = Version::J;
        drive (rig, feed, (int) (0.2 * kFs), 128, want, StereoMode::dual, true, l, r, {}, 1);
        check (finite (l) && rig.sw.activeVersion() == Version::J && ! rig.sw.isFading() && maxStep (l, (size_t) (0.5 * kFs), l.size()) < 0.2,
               "mono: a switch fades one channel and ends on the new version");
    }

    // --- 44.1 to 192 kHz, with the worker -----------------------------------------------------------------------------------------
    for (double rate : { 44100.0, 96000.0, 192000.0 })
    {
        Rig rig (rate, 512, Version::F);
        Poller poller (rig.sw);
        Feed feed (fromDb (levelForGr (6.0)), rate);
        Version want = Version::F;
        Signal l, r;
        drive (rig, feed, (int) (0.4 * rate), 256, want, StereoMode::dual, false, l, r);
        want = Version::G;
        int blocks = 0;
        while ((rig.sw.activeVersion() != Version::G || rig.sw.isFading()) && blocks < 4000)
        {
            drive (rig, feed, 256, 256, want, StereoMode::dual, false, l, r);
            std::this_thread::sleep_for (std::chrono::milliseconds (1));
            ++blocks;
        }
        check (finite (l) && finite (r) && rig.sw.activeVersion() == Version::G && blocks < 4000,
               fmt ("%.0f Hz, with the settle worker: switches to G", rate) + fmt (" after %.0f blocks of 256", (double) blocks));
    }

    // --- a state load while running: the parameter changes first, then settleNow() runs on another thread while the audio goes on -----
    {
        Rig rig (kFs, 256, Version::F);
        Poller poller (rig.sw);
        Feed feed (fromDb (levelForGr (6.0)));
        Version want = Version::F;
        Signal l, r;
        drive (rig, feed, (int) (0.4 * kFs), 256, want, StereoMode::dual, false, l, r);
        want = Version::J;   // setStateInformation: replaceState has set the parameter ...
        std::atomic<bool> loaded { false };
        std::thread loader ([&] { rig.sw.settleNow (Version::J); loaded = true; });   // ... and the load settles the version on its own thread
        int blocks = 0;
        bool fadeAfterReady = true, sawReady = false;
        while (! loaded.load() && blocks < 4000)
        {
            const bool readyBefore = rig.sw.standbyStateForTest() == 2;
            drive (rig, feed, 256, 256, want, StereoMode::dual, false, l, r);
            sawReady = sawReady || readyBefore;
            if (readyBefore) fadeAfterReady = fadeAfterReady && (rig.sw.isFading() || rig.sw.activeVersion() == Version::J);
            std::this_thread::sleep_for (std::chrono::milliseconds (1));
            ++blocks;
        }
        loader.join();
        drive (rig, feed, (int) (0.1 * kFs), 256, want, StereoMode::dual, false, l, r);
        check (finite (l) && fadeAfterReady && rig.sw.activeVersion() == Version::J && ! rig.sw.isFading(),
               "state load while running: settleNow() on another thread; the first block that finds the settled set fades it in, and J is reached");
        (void) sawReady;
    }

    // --- prepare() between switches -------------------------------------------------------------------------------------------------
    {
        Rig rig (kFs, 256, Version::F);
        Feed feed (fromDb (levelForGr (6.0)));
        Version want = Version::F;
        Signal l, r;
        bool ok = true;
        const double rates[3] = { 48000.0, 96000.0, 44100.0 };
        for (int k = 0; k < 3; ++k)
        {
            // a switch under way is abandoned by prepare(); the new set is the requested one and no fade runs
            want = all[(k + 1) % 3];
            drive (rig, feed, 300, 100, want, StereoMode::dual, true, l, r);
            const bool fadingBefore = rig.sw.isFading();
            rig.fs = rates[k];
            rig.sw.prepare (rates[k], 256, all[k], StereoMode::dual);
            Feed fresh (fromDb (-30.0), rates[k]);
            Signal pl, pr;
            want = all[k];
            drive (rig, fresh, (int) (0.1 * rates[k]), 256, want, StereoMode::dual, true, pl, pr);
            ok = ok && fadingBefore && ! rig.sw.isFading() && rig.sw.activeVersion() == all[k] && finite (pl) && finite (pr)
                 && rig.sw.fadeLengthSamples() == (int) std::lround (0.020 * rates[k]);
        }
        check (ok, "prepare() between switches (rate changes, a fade under way): the requested set is active, no fade, finite output");
    }
}

//== 10. the continuity of the gain reduction at a switch ============================================================================
void continuity()
{
    std::printf ("continuity at a switch\n");
    const double level = levelForGr (12.0);
    // F, G and J are real engine swaps (G and J have their own cell, input and line amp). Gate: with the copy the envelope moves by less than 0.5 dB.
    const struct { Version from, to; } pairs[] = { { Version::F, Version::G }, { Version::G, Version::F }, { Version::F, Version::J },
                                                   { Version::J, Version::F }, { Version::G, Version::J }, { Version::J, Version::G } };
    for (const auto& pair : pairs)
        for (const bool adopt : { true, false })
        {
            Rig rig (kFs, 256, pair.from);
            rig.sw.setAdoptControlStateForTest (adopt);
            Feed feed (fromDb (level));
            Version want = pair.from;
            Signal l, r;
            drive (rig, feed, (int) (1.0 * kFs), 256, want, StereoMode::dual, true, l, r);
            const double gr = rig.sw.channel (0).currentGainReductionDb();
            const size_t at = l.size();
            want = pair.to;
            drive (rig, feed, (int) (0.3 * kFs), 256, want, StereoMode::dual, true, l, r);
            double worst = 0.0;
            const double ref = windowDb (l, at - 480);
            for (size_t w = at - 480; w + 480 < l.size(); w += 48)
                worst = std::max (worst, std::abs (windowDb (l, w) - ref));
            char what[300];
            std::snprintf (what, sizeof (what), "%c to %c at %.1f dB GR, %s the control-state copy: the envelope moves by up to %.2f dB (10 ms windows)",
                           specFor (pair.from).name, specFor (pair.to).name, gr, adopt ? "with" : "without", worst);
            if (adopt) check (worst < 0.5, what);
            else { std::printf ("        %s  (recorded; not a gate)\n", what); }
        }
}

//== 11. no allocation, with the worker running ======================================================================================
void switchAllocation()
{
    std::printf ("switching: real-time safety\n");
    Rig rig (kFs, 512, Version::F);
    Poller poller (rig.sw);
    std::vector<float> l (512), r (512);
    Feed feed (fromDb (levelForGr (6.0)));
    long total = 0;
    int fades = 0;
    bool wasFading = false;
    Version want = Version::F;
    for (int b = 0; b < 600; ++b)
    {
        feed.fill (l.data(), r.data(), 512);
        want = all3((b / 15) % 3);
        float* ptrs[2] = { l.data(), r.data() };
        gAllocations = 0;
        gCounting = true;
        rig.sw.process (ptrs, 2, 512, want, (StereoMode) ((b / 7) % 3), false);
        gCounting = false;
        total += gAllocations.load();
        const bool f = rig.sw.isFading();
        if (f && ! wasFading) ++fades;
        wasFading = f;
        std::this_thread::sleep_for (std::chrono::milliseconds (2));   // gives the worker time; its settles are counted too (process-wide counter)
    }
    // the worker's own settles, counted separately and over the whole span
    int settles = 0;
    wasFading = false;
    gAllocations = 0;
    gCounting = true;
    for (int b = 0; b < 400; ++b)
    {
        feed.fill (l.data(), r.data(), 512);
        want = all3(((b / 10) + 1) % 3);
        float* ptrs[2] = { l.data(), r.data() };
        rig.sw.process (ptrs, 2, 512, want, StereoMode::dual, false);
        const bool f = rig.sw.isFading();
        if (f && ! wasFading) ++settles;
        wasFading = f;
        std::this_thread::sleep_for (std::chrono::milliseconds (2));
    }
    gCounting = false;
    const long overall = gAllocations.load();
    check (total == 0 && fades >= 8, fmt ("no allocation on the audio thread over 600 blocks with a version change every 15 blocks (%.0f allocations, %.0f fades)", (double) total, (double) fades));
    check (overall == 0 && settles >= 8, fmt ("none in the worker's settles either, counted process-wide over 400 blocks (%.0f allocations, ", (double) overall) + fmt ("%.0f settles and fades)", (double) settles));
    Rig offlineRig (kFs, 512, Version::F);
    long inlineTotal = 0;
    for (int b = 0; b < 200; ++b)
    {
        feed.fill (l.data(), r.data(), 512);
        float* ptrs[2] = { l.data(), r.data() };
        gAllocations = 0;
        gCounting = true;
        offlineRig.sw.process (ptrs, 2, 512, all3((b / 12) % 3), StereoMode::dual, true);
        gCounting = false;
        inlineTotal += gAllocations.load();
    }
    check (inlineTotal == 0, fmt ("none in an offline render's inline settles (%.0f allocations)", (double) inlineTotal));
}

//== 12. offline determinism =========================================================================================================
void determinism()
{
    std::printf ("offline determinism\n");
    const double level = levelForGr (6.0);
    const auto render = [level] (int block)
    {
        Rig rig (kFs, block, Version::F);
        Poller poller (rig.sw);   // running, as in a host: the offline path must not depend on it
        Feed feed (fromDb (level));
        Signal l, r;
        Version want = Version::F;
        const int period = 4096;   // a multiple of both block sizes: the automation lands on block boundaries
        drive (rig, feed, period * 12, block, want, StereoMode::dual, true, l, r, [&] (int index)
        {
            const int step = index / period;
            static const Version order[6] = { Version::F, Version::G, Version::J, Version::G, Version::F, Version::J };
            want = order[(step / 2) % 6];
            rig.input = (step % 3 == 2) ? 0.8f : Range::inputMax;   // a control moves too
        });
        Signal out = l;
        out.insert (out.end(), r.begin(), r.end());
        return out;
    };
    const Signal a1 = render(128), a2 = render(128), b1 = render(512), b2 = render(512);
    check (a1 == a2, "two offline renders with automated versions, block 128: bit-identical");
    check (b1 == b2, "two offline renders with automated versions, block 512: bit-identical");
    std::printf ("        (block 128 against block 512: %s)\n", a1 == b1 ? "also bit-identical" : "differ");
    // a freshly prepared engine behaves as a first-time one: prepare() twice gives the same render
    Rig rig1 (kFs, 256, Version::F), rig2 (kFs, 256, Version::F);
    rig2.sw.prepare (kFs, 256, Version::F, StereoMode::dual);
    Feed f1 (fromDb (levelForGr (6.0))), f2 (fromDb (levelForGr (6.0)));
    Version w = Version::F;
    Signal x1, y1, x2, y2;
    drive (rig1, f1, 20000, 256, w, StereoMode::dual, true, x1, y1);
    drive (rig2, f2, 20000, 256, w, StereoMode::dual, true, x2, y2);
    check (x1 == x2 && y1 == y2, "an engine prepared a second time renders exactly as a first-time one");
}
//== 13. the cost of a fade (timing: run on request, like perf) ======================================================================
void fadePerformance()
{
    std::printf("cost of a version fade (48 kHz, blocks of 128, stereo, ~6 dB GR; budget per block %.3f ms)\n", 1000.0 * 128 / kFs);
    Rig rig (kFs, 128, Version::F);
    Feed feed (fromDb (levelForGr (6.0)));
    std::vector<float> l (128), r (128);
    const auto timed = [&] (Version want, bool offline)
    {
        feed.fill (l.data(), r.data(), 128);
        float* ptrs[2] = { l.data(), r.data() };
        const auto t0 = std::chrono::steady_clock::now();
        rig.sw.process (ptrs, 2, 128, want, StereoMode::dual, offline);
        return std::chrono::duration<double, std::milli> (std::chrono::steady_clock::now() - t0).count();
    };
    for (int i = 0; i < 400; ++i) timed (Version::F, true);
    std::vector<double> steady, fading;
    double worstSettle = 0.0;
    for (int i = 0; i < 400; ++i) steady.push_back (timed (Version::F, true));
    Version v = Version::F;
    for (int rep = 0; rep < 20; ++rep)
    {
        v = (v == Version::F) ? Version::G : Version::F;
        // the settle as the worker does it (not on the audio thread), then the fade blocks
        timed (v, false);
        while (! rig.sw.pollSettle() && rig.sw.standbyStateForTest() != 2) {}
        while (rig.sw.activeVersion() != v || rig.sw.isFading())
            fading.push_back (timed (v, false));
        for (int i = 0; i < 40; ++i) timed (v, true);
    }
    for (int rep = 0; rep < 10; ++rep)   // offline: the inline settle lands on one block
    {
        v = (v == Version::F) ? Version::G : Version::F;
        worstSettle = std::max (worstSettle, timed (v, true));
        while (rig.sw.isFading()) timed (v, true);
    }
    std::sort (steady.begin(), steady.end());
    std::sort (fading.begin(), fading.end());
    const double budget = 1000.0 * 128 / kFs;
    std::printf ("        steady block: median %.3f ms, worst %.3f ms; fade block: median %.3f ms, worst %.3f ms (%zu blocks); offline settle block: worst %.3f ms\n",
                 steady[steady.size() / 2], steady.back(), fading[fading.size() / 2], fading.back(), fading.size(), worstSettle);
    check (fading.back() < 0.5 * budget, fmt ("every fade block is under 50%% of its real-time budget (worst %.3f ms of %.3f ms)", fading.back(), budget));
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
    for (int i = 1; i < argc; ++i)
    {
        if (std::string (argv[i]) == "--version" && i + 1 < argc)
        {
            const std::string v = argv[++i];
            if (v == "F") gVersion = Version::F; else if (v == "G") gVersion = Version::G; else if (v == "J") gVersion = Version::J;
            else { std::fprintf (stderr, "--version takes F, G or J\n"); return 2; }
        }
        else if (std::string (argv[i]) == "--trims" && i + 1 < argc) gTrims = std::string (argv[++i]) != "off";
        else wanted.push_back (argv[i]);
    }
    struct Section { const char* name; void (*fn)(); };
    const Section sections[] = { { "hpf", sidechainFilterResponse }, { "hpfoff", sidechainFilterOff }, { "hpfswitch", sidechainFilterSwitching }, { "identical", identicalInputs }, { "oneside", oneSidedDrive }, { "times", timesTest }, { "robust", robustness },
                                 { "rt", realtimeSafety }, { "ms", midSide }, { "switch", transitions }, { "switching", switching }, { "continuity", continuity },
                                 { "switchrt", switchAllocation }, { "determinism", determinism }, { "perf", performance }, { "fadeperf", fadePerformance } };
    std::printf ("stereo modes (fast engine), rev %c\n", specFor (gVersion).name);
    for (const auto& sec : sections)
    {
        // perf compares timings, which a loaded machine or CI runner can upset: run it only when asked
        bool want = wanted.empty() && std::string (sec.name) != "perf" && std::string (sec.name) != "fadeperf";
        for (const auto& w : wanted) want = want || w == sec.name;
        if (want) sec.fn();
    }
    std::printf (failures == 0 ? "all checks passed\n" : "%d check(s) failed\n", failures);
    return failures == 0 ? 0 : 1;
}
