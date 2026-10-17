// Offline runner for one 1176LN channel.
//
//   urei1176_eval [--wav in.wav] [--out out.wav] [--rate 48000] [--channel 0] [--block 512]
//                 [--set name=value ...]
//
// Without --wav it runs a built-in test signal: 1 s of a 1 kHz tone at -30 dBFS, 1 s at -6 dBFS,
// 1 s at -30 dBFS again (gain reduction, recovery). With --wav it processes one channel of the
// file at the file's rate (16/24/32-bit PCM or 32-bit float). It prints the gain reduction per
// 100 ms, the output peak, the time taken and the real-time factor, and writes a 32-bit float
// WAV when --out is given.
//
// --set names: input, output (knob rotation 0..1, 0.5 = "24"; or a printed dial value with a
// "d" prefix, e.g. input=d18), attack, release (dial 1..7), ratio (4, 8, 12, 20, all, none),
// limitoff (0/1), meter (off, gr, +4, +8), cal (dBu at 0 dBFS, 10..26).

#include "dsp/Urei1176.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::vector<float> readWavChannel (const std::string& path, double& rate, int channel)
{
    std::ifstream f (path, std::ios::binary);
    std::vector<char> bytes ((std::istreambuf_iterator<char> (f)), std::istreambuf_iterator<char>());
    if (bytes.size() < 12 || std::memcmp (bytes.data(), "RIFF", 4) != 0 || std::memcmp (bytes.data() + 8, "WAVE", 4) != 0)
        return {};
    int channels = 0, bits = 0, format = 0;
    for (size_t pos = 12; pos + 8 <= bytes.size();)
    {
        uint32_t size;
        std::memcpy (&size, bytes.data() + pos + 4, 4);
        const size_t bodyPos = pos + 8;
        if (size > bytes.size() - bodyPos) return {};
        const auto* body = (const unsigned char*) bytes.data() + bodyPos;
        if (std::memcmp (bytes.data() + pos, "fmt ", 4) == 0 && size >= 16)
        {
            uint16_t f16, c16, b16;
            uint32_t r32;
            std::memcpy (&f16, body, 2); std::memcpy (&c16, body + 2, 2);
            std::memcpy (&r32, body + 4, 4); std::memcpy (&b16, body + 14, 2);
            format = f16; channels = c16; rate = r32; bits = b16;
            if (format == 0xFFFE && size >= 26) { uint16_t sub; std::memcpy (&sub, body + 24, 2); format = sub; }   // WAVE_FORMAT_EXTENSIBLE
        }
        else if (std::memcmp (bytes.data() + pos, "data", 4) == 0 && channels > 0 && rate > 0.0)
        {
            if (channel >= channels) return {};
            if (! ((format == 1 && (bits == 16 || bits == 24 || bits == 32)) || (format == 3 && bits == 32)))
                return {};
            const int width = bits / 8, frame = width * channels;
            std::vector<float> out;
            out.reserve (size / (size_t) frame);
            for (size_t i = 0; i < size / (size_t) frame; ++i)
            {
                const auto* p = body + i * (size_t) frame + (size_t) (channel * width);
                if (format == 3) { float v; std::memcpy (&v, p, 4); out.push_back (v); }
                else if (bits == 16) out.push_back ((float) (int16_t) (p[0] | p[1] << 8) / 32768.0f);
                else if (bits == 24) out.push_back ((float) ((int32_t) ((uint32_t) p[0] << 8 | (uint32_t) p[1] << 16 | (uint32_t) p[2] << 24) >> 8) / 8388608.0f);
                else out.push_back ((float) ((int32_t) ((uint32_t) p[0] | (uint32_t) p[1] << 8 | (uint32_t) p[2] << 16 | (uint32_t) p[3] << 24) / 2147483648.0));
            }
            return out;
        }
        pos = bodyPos + size + (size & 1);
    }
    return {};
}

void writeFloatWav (const std::string& path, const std::vector<float>& data, int rate)
{
    std::ofstream f (path, std::ios::binary);
    auto put32 = [&] (uint32_t v) { f.write ((const char*) &v, 4); };
    auto put16 = [&] (uint16_t v) { f.write ((const char*) &v, 2); };
    const uint32_t bytes = (uint32_t) data.size() * 4;
    f.write ("RIFF", 4); put32 (36 + bytes); f.write ("WAVE", 4);
    f.write ("fmt ", 4); put32 (16); put16 (3); put16 (1); put32 ((uint32_t) rate); put32 ((uint32_t) rate * 4); put16 (4); put16 (32);
    f.write ("data", 4); put32 (bytes);
    f.write ((const char*) data.data(), (std::streamsize) bytes);
}

double db (double x) { return 20.0 * std::log10 (std::max (x, 1e-12)); }

bool apply (urei1176::Channel& ch, const std::string& name, const std::string& value)
{
    using namespace urei1176;
    const float v = (float) std::atof (value.c_str());
    const float rotation = ! value.empty() && value[0] == 'd' ? rotationForMarking ((float) std::atof (value.c_str() + 1)) : v;
    if (name == "input") ch.setInput (rotation);
    else if (name == "output") ch.setOutput (rotation);
    else if (name == "attack") ch.setAttack (v);
    else if (name == "release") ch.setRelease (v);
    else if (name == "limitoff") ch.setLimitOff (v > 0.5f);
    else if (name == "cal") ch.setCalibrationDbu (v);
    else if (name == "ratio")
    {
        if (value == "4") ch.setRatio (Ratio::r4);
        else if (value == "8") ch.setRatio (Ratio::r8);
        else if (value == "12") ch.setRatio (Ratio::r12);
        else if (value == "20") ch.setRatio (Ratio::r20);
        else if (value == "all") ch.setRatio (Ratio::all);
        else if (value == "none") ch.setRatio (Ratio::none);
        else return false;
    }
    else if (name == "meter")
    {
        if (value == "off") ch.setMeterMode (MeterMode::off);
        else if (value == "gr") ch.setMeterMode (MeterMode::gainReduction);
        else if (value == "+4" || value == "4") ch.setMeterMode (MeterMode::plus4);
        else if (value == "+8" || value == "8") ch.setMeterMode (MeterMode::plus8);
        else return false;
    }
    else return false;
    return true;
}
} // namespace

int main (int argc, char** argv)
{
    std::string inPath, outPath;
    double rate = 48000.0;
    int channel = 0, block = 512;
    std::vector<std::pair<std::string, std::string>> sets;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--wav" && i + 1 < argc) inPath = argv[++i];
        else if (a == "--out" && i + 1 < argc) outPath = argv[++i];
        else if (a == "--rate" && i + 1 < argc) rate = std::atof (argv[++i]);
        else if (a == "--channel" && i + 1 < argc) channel = std::atoi (argv[++i]);
        else if (a == "--block" && i + 1 < argc) block = std::max (1, std::atoi (argv[++i]));
        else if (a == "--set" && i + 1 < argc)
        {
            const std::string kv = argv[++i];
            const auto eq = kv.find ('=');
            if (eq == std::string::npos) { std::fprintf (stderr, "--set needs name=value\n"); return 1; }
            sets.push_back ({ kv.substr (0, eq), kv.substr (eq + 1) });
        }
        else
        {
            std::fprintf (stderr, "usage: urei1176_eval [--wav in.wav] [--out out.wav] [--rate 48000] [--channel 0] [--block 512] [--set name=value ...]\n");
            return 1;
        }
    }

    std::vector<float> x;
    if (! inPath.empty())
    {
        x = readWavChannel (inPath, rate, channel);
        if (x.empty()) { std::fprintf (stderr, "cannot read %s (channel %d)\n", inPath.c_str(), channel); return 1; }
        std::printf ("input %s: %.2f s at %.0f Hz, channel %d\n", inPath.c_str(), (double) x.size() / rate, rate, channel);
    }
    else
    {
        const int n = (int) (3.0 * rate);
        x.resize ((size_t) n);
        for (int i = 0; i < n; ++i)
        {
            const double t = i / rate;
            const double level = (t >= 1.0 && t < 2.0) ? std::pow (10.0, -6.0 / 20.0) : std::pow (10.0, -30.0 / 20.0);
            x[(size_t) i] = (float) (level * std::sin (2.0 * kPi * 1000.0 * t));
        }
        std::printf ("built-in signal: 1 kHz, 1 s at -30 dBFS, 1 s at -6 dBFS, 1 s at -30 dBFS, %.0f Hz\n", rate);
    }

    urei1176::Channel ch;
    for (const auto& s : sets)
        if (! apply (ch, s.first, s.second)) { std::fprintf (stderr, "unknown setting %s=%s\n", s.first.c_str(), s.second.c_str()); return 1; }
    const auto t0 = std::chrono::steady_clock::now();
    ch.prepare (rate);
    const double prepMs = std::chrono::duration<double, std::milli> (std::chrono::steady_clock::now() - t0).count();
    for (const auto& s : sets)
        std::printf ("  %s = %s\n", s.first.c_str(), s.second.c_str());

    std::vector<float> y = x;
    const int report = std::max (block, (int) (0.1 * rate) / block * block);
    float grPeak = 0.0f, grReport = 0.0f, peakOut = 0.0f, peakIn = 0.0f;
    double procSeconds = 0.0;
    std::printf ("  time     GR peak  needle   out peak\n");
    for (size_t i = 0; i < y.size(); i += (size_t) block)
    {
        const int count = (int) std::min<size_t> ((size_t) block, y.size() - i);
        float blockPeakOut = 0.0f;
        for (int k = 0; k < count; ++k) peakIn = std::max (peakIn, std::fabs (y[i + (size_t) k]));
        const auto t1 = std::chrono::steady_clock::now();
        ch.process (y.data() + i, count);
        procSeconds += std::chrono::duration<double> (std::chrono::steady_clock::now() - t1).count();
        for (int k = 0; k < count; ++k) blockPeakOut = std::max (blockPeakOut, std::fabs (y[i + (size_t) k]));
        peakOut = std::max (peakOut, blockPeakOut);
        grPeak = std::max (grPeak, ch.getGainReductionDb());
        grReport = std::max (grReport, ch.getGainReductionDb());
        if (((i / (size_t) block) + 1) % (size_t) (report / block) == 0)
        {
            std::printf ("  %6.2f s  %5.2f dB  %6.2f   %6.2f dBFS\n", (double) (i + (size_t) count) / rate, grReport, ch.getMeterNeedleDb(), db (blockPeakOut));
            grReport = 0.0f;
        }
    }
    bool finite = true;
    for (float v : y) finite = finite && std::isfinite (v);
    const double audioSeconds = (double) y.size() / rate;
    std::printf ("prepare %.0f ms; processed %.2f s in %.3f s: %.2fx real time, %.0f samples/s\n", prepMs, audioSeconds, procSeconds,
                 audioSeconds / procSeconds, (double) y.size() / procSeconds);
    std::printf ("input peak %.2f dBFS, output peak %.2f dBFS, largest GR %.2f dB, output %s\n", db (peakIn), db (peakOut), grPeak,
                 finite ? "finite" : "NOT FINITE");
    if (! outPath.empty())
    {
        writeFloatWav (outPath, y, (int) std::lround (rate));
        std::printf ("wrote %s\n", outPath.c_str());
    }
    return finite ? 0 : 1;
}
