#pragma once

// The plug-in's version switching around the fast engine: two engine sets (active and standby), a settle step that builds the standby as
// the requested Version off the audio thread, and a 20 ms linear crossfade from the old set to the new one. No JUCE dependency, so the
// offline tests drive it as the plug-in does. See docs/research/urei-1176-versions-plan.md, section 2.
//
// Threads. The audio thread calls process(); one settle worker calls pollSettle() every few milliseconds (the audio thread never signals
// it); the host's state-load thread may call settleNow(). They share only atomics:
//   standbyState   free -> building -> ready -> in use -> free   (each arrow a compare-exchange; the audio thread takes ready -> in use)
//   requestWord    (sequence << 8 | version) the newest version wanted, written by the audio thread and by settleNow()
//   readyVersion   the version the standby holds while it is ready
// Nothing on the audio thread allocates, locks or waits.
//
// Continuity. The incoming set is a freshly prepared engine; at the hand-off it adopts the outgoing engine's slow gain-reduction memory
// (FastChannel::adoptControlState), so a switch while compressing does not fade a jump in level. Both sets get the controls every block.
//
// Meters. During a fade the meters keep reading the outgoing set (channel() is the active set): the incoming engine adopted its needle
// and gain-reduction values, and the outgoing one is in steady state, so nothing flickers; the read moves to the new set when the fade ends.

#include "Fast1176.h"
#include "StereoModes.h"
#include "Versions.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

namespace urei1176
{
class VersionSwitch
{
public:
    static constexpr int kMaxChannels = 2;
    static constexpr double kFadeSeconds = 0.020;
    /** The scratch holds at least this many samples per channel; longer blocks are processed in chunks. */
    static constexpr int kMinScratch = 8192;

    /** Sets every control of an engine from the live parameters (the processor's pushControls). Called on the audio thread, on the settle
        worker and on the state-load thread; it must only read. */
    using PushControls = void (*) (void* context, FastChannel&) noexcept;

    VersionSwitch();
    void setControlSource (PushControls fn, void* context) noexcept;

    /** Prepares the active set synchronously as `version` (no fade), invalidates the standby, sizes the scratch for blocks of up to
        `maxBlock` samples. Allocates. Never concurrently with process(), pollSettle() or settleNow(): stop the worker first. */
    void prepare (double sampleRate, int maxBlock, Version version, StereoMode snapMode);

    //== audio thread ============================================================================
    /** Processes `numChannels` (1 or 2) channels in place. `want` is the version the parameter asks for now. With `nonRealtime` the
        standby is settled right here on the block where `want` changed, so an offline render does not depend on thread timing. */
    void process (float* const* channels, int numChannels, int numSamples, Version want, StereoMode mode, bool nonRealtime) noexcept;

    //== settle worker / state-load thread =======================================================
    /** One pass of the worker: if a version other than the active one is requested and the standby is free, builds it. Returns whether it
        built. Allocates nothing, so it may run beside the audio thread. */
    bool pollSettle() noexcept;
    /** Settles `want` now, on the calling (non-audio) thread: for a state load, so that the next block already finds it ready. Waits (a
        bounded time) for the worker if it is mid-build. Does nothing before prepare(), or while a fade is running (the audio thread
        takes the request afterwards). */
    void settleNow (Version want);

    //== any thread ==============================================================================
    /** The set currently audible as the main one (during a fade: the outgoing). */
    Version activeVersion() const noexcept { return (Version) activeVersionValue.load (std::memory_order_acquire); }
    /** Where the engine is heading: the incoming version during a fade, else the active one. */
    Version targetVersion() const noexcept { return fadingNow.load (std::memory_order_acquire) ? (Version) incomingVersionValue.load (std::memory_order_acquire) : activeVersion(); }
    bool isFading() const noexcept { return fadingNow.load (std::memory_order_acquire); }
    int fadeLengthSamples() const noexcept { return fadeLength; }
    const FastChannel& channel (int index) const noexcept { return sets[activeIndex.load (std::memory_order_acquire)].ch[index]; }
    /** Test hook: the standby's state (0 free, 1 building, 2 ready, 3 in use). */
    int standbyStateForTest() const noexcept { return standbyState.load (std::memory_order_acquire); }
    /** Test hook: false skips FastChannel::adoptControlState at the hand-off, to record what the continuity copy is worth. Set it before
        process() runs. */
    void setAdoptControlStateForTest (bool on) noexcept { adoptControlState = on; }

private:
    struct Set
    {
        FastChannel ch[kMaxChannels];
        StereoRunner runner;
    };
    enum State : int { stFree = 0, stBuilding = 1, stReady = 2, stInUse = 3 };

    static std::uint32_t encode (std::uint32_t seq, int version) noexcept { return (seq << 8) | (std::uint32_t) version; }
    static int versionOf (std::uint32_t word) noexcept { return (int) (word & 0xffu); }

    void push (FastChannel& channel) noexcept { if (pushFn != nullptr) pushFn (pushContext, channel); }
    void request (int version) noexcept;
    /** Builds the standby (already taken: state is `building`) as `version`, and publishes it if it is still wanted. */
    void build (int version) noexcept;
    bool takeStandbyForBuild (int version) noexcept;
    /** Audio thread: takes a standby that is ready as `version` (ready -> in use) and starts the fade into it. */
    bool tryStartFade (int version) noexcept;
    void beginFade (int version) noexcept;
    void endFade() noexcept;
    void runSet (Set& set, float* const* channels, int offset, int numChannels, int count, StereoMode mode) noexcept;
    void processFading (float* const* channels, int numChannels, int numSamples, StereoMode mode) noexcept;

    Set sets[2];
    std::atomic<int> activeIndex { 0 };
    std::atomic<int> activeVersionValue { (int) Version::F };
    std::atomic<int> incomingVersionValue { (int) Version::F };
    std::atomic<bool> fadingNow { false };
    std::atomic<int> standbyState { stFree };
    std::atomic<int> readyVersion { -1 };
    std::atomic<std::uint32_t> requestWord { 0 };
    std::atomic<std::uint32_t> sequence { 0 };

    std::mutex lifecycle;   // prepare() against settleNow(), both off the audio thread (the audio thread never takes it)
    PushControls pushFn = nullptr;
    void* pushContext = nullptr;
    double fs = 0.0;
    bool prepared = false;
    bool adoptControlState = true;
    int fadePos = 0, fadeLength = 1;
    std::vector<float> scratch[kMaxChannels];
    int scratchCapacity = 0;
};
} // namespace urei1176
