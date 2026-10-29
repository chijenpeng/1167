#include "VersionSwitch.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace urei1176
{
VersionSwitch::VersionSwitch() = default;

void VersionSwitch::setControlSource (PushControls fn, void* context) noexcept
{
    pushFn = fn;
    pushContext = context;
}

void VersionSwitch::prepare (double sampleRate, int maxBlock, Version version, StereoMode snapMode)
{
    const std::lock_guard<std::mutex> guard (lifecycle);
    fs = sampleRate;
    fadeLength = std::max (1, (int) std::lround (kFadeSeconds * sampleRate));
    fadePos = 0;
    fadingNow.store (false);
    activeIndex.store (0);
    activeVersionValue.store ((int) version);
    incomingVersionValue.store ((int) version);

    Set& a = sets[0];
    for (auto& channel : a.ch)
    {
        push (channel);
        channel.prepare (sampleRate, version);
    }
    a.runner.prepare (sampleRate);
    a.runner.snapTo (snapMode);   // a mode set before playback starts needs no morph

    // The standby is rebuilt on demand (at this sample rate) by the worker; whatever it held is void.
    readyVersion.store (-1);
    requestWord.store (encode (sequence.fetch_add (1) + 1, (int) version));
    standbyState.store (stFree);

    const int capacity = std::max(maxBlock, kMinScratch);
    if (capacity != scratchCapacity)
    {
        for (auto& s : scratch) s.assign ((size_t) capacity, 0.0f);
        scratchCapacity = capacity;
    }
    prepared = true;
}

//==============================================================================================
void VersionSwitch::request (int version) noexcept
{
    if (versionOf (requestWord.load()) != version)
        requestWord.store (encode (sequence.fetch_add (1) + 1, version));
}

bool VersionSwitch::takeStandbyForBuild (int version) noexcept
{
    int expected = stFree;
    if (standbyState.compare_exchange_strong (expected, stBuilding))
        return true;
    // A ready standby that holds another version is void: take it back.
    expected = stReady;
    if (readyVersion.load() != version && standbyState.compare_exchange_strong (expected, stBuilding))
        return true;
    return false;
}

void VersionSwitch::build (int version) noexcept
{
    Set& st = sets[1 - activeIndex.load (std::memory_order_acquire)];
    for (auto& channel : st.ch)
    {
        push (channel);
        channel.prepare (fs, (Version) version);
    }
    st.runner.prepare (fs);
    // Honour the newest request only: a result nobody wants any more is dropped.
    if (versionOf (requestWord.load()) == version && version != activeVersionValue.load())
    {
        readyVersion.store (version);
        standbyState.store (stReady, std::memory_order_release);
    }
    else
    {
        readyVersion.store (-1);
        standbyState.store (stFree, std::memory_order_release);
    }
}

bool VersionSwitch::pollSettle() noexcept
{
    if (! prepared || fadingNow.load (std::memory_order_acquire))
        return false;
    const int wanted = versionOf (requestWord.load());
    if (wanted == activeVersionValue.load() || wanted < 0 || wanted >= kNumVersions)
        return false;
    if (! takeStandbyForBuild (wanted))   // also false when the standby is already ready as `wanted`
        return false;
    build (wanted);
    return true;
}

void VersionSwitch::settleNow (Version want)
{
    const std::lock_guard<std::mutex> guard (lifecycle);
    if (! prepared)
        return;
    const int version = (int) want;
    request (version);   // whatever was pending for another version is void
    if (version == activeVersionValue.load())
        return;
    for (int attempt = 0; attempt < 400; ++attempt)   // at most about 400 ms for a worker that is mid-build
    {
        const int s = standbyState.load (std::memory_order_acquire);
        if (s == stInUse)
            return;
        if (s == stReady && readyVersion.load() == version)
            return;
        if (takeStandbyForBuild (version))   // fails while the worker is building
        {
            build (version);
            return;
        }
        std::this_thread::sleep_for (std::chrono::milliseconds (1));
    }
}

//==============================================================================================
bool VersionSwitch::tryStartFade (int version) noexcept
{
    if (readyVersion.load() != version)
        return false;
    int expected = stReady;
    if (! standbyState.compare_exchange_strong (expected, stInUse))
        return false;
    // The worker may have rebuilt the standby as another version between the check above and the exchange (ready -> building -> ready):
    // ask the engine itself, and hand a mismatch back as it was.
    if (sets[1 - activeIndex.load (std::memory_order_acquire)].ch[0].version() != (Version) version)
    {
        standbyState.store (stReady, std::memory_order_release);
        return false;
    }
    beginFade (version);
    return true;
}

void VersionSwitch::beginFade (int version) noexcept
{
    const int a = activeIndex.load (std::memory_order_acquire);
    Set& out = sets[a];
    Set& in = sets[1 - a];
    for (int c = 0; c < kMaxChannels; ++c)
    {
        push (in.ch[c]);
        if (adoptControlState)
            in.ch[c].adoptControlState (out.ch[c]);
    }
    in.runner = out.runner;   // the Mid-Side morph carries on from where it is
    fadePos = 0;
    incomingVersionValue.store (version);
    fadingNow.store (true, std::memory_order_release);
}

void VersionSwitch::endFade() noexcept
{
    const int a = activeIndex.load (std::memory_order_acquire);
    activeVersionValue.store (incomingVersionValue.load());
    activeIndex.store (1 - a, std::memory_order_release);
    fadingNow.store (false, std::memory_order_release);
    readyVersion.store (-1);
    standbyState.store (stFree, std::memory_order_release);   // the outgoing set is the new standby
}

void VersionSwitch::runSet (Set& set, float* const* channels, int offset, int numChannels, int count, StereoMode mode) noexcept
{
    if (numChannels == 2)
        set.runner.process (set.ch[0], set.ch[1], channels[0] + offset, channels[1] + offset, count, mode);
    else
        for (int c = 0; c < numChannels; ++c)
            set.ch[c].process (channels[c] + offset, count);
}

void VersionSwitch::processFading (float* const* channels, int numChannels, int numSamples, StereoMode mode) noexcept
{
    const int a = activeIndex.load (std::memory_order_acquire);
    Set& out = sets[a];
    Set& in = sets[1 - a];
    float* scratchPtr[kMaxChannels] = { scratch[0].data(), scratch[1].data() };
    int pos = 0;
    while (pos < numSamples && fadingNow.load (std::memory_order_relaxed))
    {
        const int chunk = std::min (numSamples - pos, scratchCapacity);
        const int mixed = std::min (chunk, fadeLength - fadePos);   // samples of this chunk that are still fading
        for (int c = 0; c < numChannels; ++c)
            std::copy (channels[c] + pos, channels[c] + pos + chunk, scratchPtr[c]);
        runSet (out, channels, pos, numChannels, mixed, mode);
        runSet (in, scratchPtr, 0, numChannels, chunk, mode);
        // A linear ramp: the two outputs are the same program, so they are correlated and a linear ramp sums to unity (an equal-power
        // one would bump +3 dB at the middle).
        const float invLength = 1.0f / (float) fadeLength;
        for (int c = 0; c < numChannels; ++c)
        {
            float* dst = channels[c] + pos;
            const float* src = scratchPtr[c];
            for (int i = 0; i < mixed; ++i)
            {
                const float g = (float) (fadePos + i + 1) * invLength;
                dst[i] = dst[i] * (1.0f - g) + src[i] * g;
            }
            for (int i = mixed; i < chunk; ++i)
                dst[i] = src[i];
        }
        fadePos += mixed;
        pos += chunk;
        if (fadePos >= fadeLength)
            endFade();
    }
    if (pos < numSamples)
        runSet (sets[activeIndex.load (std::memory_order_acquire)], channels, pos, numChannels, numSamples - pos, mode);
}

void VersionSwitch::process (float* const* channels, int numChannels, int numSamples, Version wantVersion, StereoMode mode, bool nonRealtime) noexcept
{
    const int want = (int) wantVersion;
    if (! fadingNow.load (std::memory_order_relaxed))
    {
        if (want == activeVersionValue.load (std::memory_order_relaxed))
        {
            // Back at (or still at) the active version: any request or ready result for another one is void.
            request (want);
            int expected = stReady;
            standbyState.compare_exchange_strong (expected, stFree);
        }
        else
        {
            bool started = tryStartFade (want);
            if (! started && nonRealtime && takeStandbyForBuild (want))
            {
                // Offline: settle here, on the block where the parameter changed, so the render does not depend on thread timing.
                request (want);
                build (want);
                started = tryStartFade (want);
            }
            if (! started)
                request (want);   // the worker settles it; the active set plays on
        }
    }

    // The controls reach every engine that runs.
    Set& act = sets[activeIndex.load (std::memory_order_acquire)];
    for (int c = 0; c < numChannels; ++c) push (act.ch[c]);
    if (! fadingNow.load (std::memory_order_relaxed))
    {
        runSet (act, channels, 0, numChannels, numSamples, mode);
        return;
    }
    Set& in = sets[1 - activeIndex.load (std::memory_order_acquire)];
    for (int c = 0; c < numChannels; ++c) push (in.ch[c]);
    processFading (channels, numChannels, numSamples, mode);
}
} // namespace urei1176
