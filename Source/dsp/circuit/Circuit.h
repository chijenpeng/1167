#pragma once

#include "Elements.h"

#include <memory>
#include <vector>

namespace ar::circuit
{
/** A chain of linear networks and non-linear elements that together make an amplifier.

    Networks are processed in the order they were added. An element may span two consecutive
    networks (some of its ports in one, the rest in the next). A buffer copies a
    probe voltage of one network into a voltage source of a later one, for places where the
    real amp has a buffer (an effects loop) or where loading is negligible. */
class Circuit
{
public:
    LinearNetwork& addNetwork (const std::string& name);
    int indexOf (const LinearNetwork&) const;
    PortRef port (LinearNetwork& net, int node);
    PortRef port (LinearNetwork& net, const std::string& nodeName);

    /** Adds a group of devices that are solved together. */
    DeviceGroup& addGroup (std::vector<std::unique_ptr<Device>> devices);

    template <typename DeviceType, typename... Args>
    DeviceGroup& addDevice (Args&&... args)
    {
        std::vector<std::unique_ptr<Device>> d;
        d.push_back (std::make_unique<DeviceType> (std::forward<Args> (args)...));
        return addGroup (std::move (d));
    }

    /** A voltage source standing in for a large supply capacitor (a reservoir or screen
        filter cap) to ground: the source is moved at a low rate by the average current the
        network draws from it, integrated on `farads` (backward Euler on the network's own
        resistive admittance, so stiff supplies stay stable). */
    void addSlowSupply (LinearNetwork& net, int sourceIndex, Real farads);
    static constexpr int kSlowSupplyInterval = 32;

    void addBuffer (LinearNetwork& from, int fromNode, LinearNetwork& to, int toSource, Real gain = 1.0);
    void setInput (LinearNetwork& net, int sourceIndex);
    void setOutput (LinearNetwork& net, int node);

    /** Builds the processing order. Call once after all parts are added. */
    void finalize();
    /** Gauss-Seidel passes over the groups of each network per sample (1 = one solve each).
        Groups in one network see each other's currents one sample late on the first pass;
        more passes make strongly coupled ones (op-amp stages loading each other) consistent. */
    void setGroupSweeps (int n) noexcept { groupSweeps = n < 1 ? 1 : n; }
    /** Passes for one network only, when a feedback loop runs through several of its groups:
        the one-sample lag between them adds phase that can make the loop oscillate at low
        sample rates (SVT-VR at 48 kHz). */
    void setNetworkSweeps (LinearNetwork& net, int n);
    /** At prepare(), merge groups of one network that load each other strongly (coupling above
        `threshold`, see mergeCoupledGroups) into jointly solved groups. Group pointers taken
        before prepare() become invalid; device pointers stay valid. */
    void setMergeGroups (bool b, Real threshold = 0.0) noexcept { mergeGroups = b; mergeThreshold = threshold; }
    void prepare (Real sampleRate);
    /** Settles the DC operating point with the input silent. */
    void findOperatingPoint (int iterations = 400);
    void refactorIfNeeded();
    Real process (Real inputVolts) noexcept;

    std::vector<std::unique_ptr<LinearNetwork>>& networks() noexcept { return nets; }
    std::vector<std::unique_ptr<DeviceGroup>>& groups() noexcept { return elements; }

private:
    struct Buffer { int fromNet, fromProbe, toNet, toSource; Real gain; };
    struct Op { enum Type { begin, solve, end, buffer, sweep } type; int index; };
    std::vector<std::vector<int>> sweepLists;   // per sweep op: the single-network groups to re-solve
    std::vector<int> sweepPasses;               // per sweep op: total passes
    std::vector<int> netSweeps;                 // per network: passes asked for (0: groupSweeps)
    int groupSweeps = 1;
    bool mergeGroups = false, merged = false;
    Real mergeThreshold = 0.0;
    void mergeCoupledGroups();

    std::vector<std::unique_ptr<LinearNetwork>> nets;
    std::vector<std::unique_ptr<DeviceGroup>> elements;
    std::vector<Buffer> buffers;

    struct SlowSupply { int net, source; Real farads; Real state = 0, applied = 0, ramp = 0, sumI = 0, sumV = 0; };
    std::vector<SlowSupply> slow;
    std::vector<Real> slowJ, slowA, slowB;   // S x S admittance, system, rhs
    unsigned slowRefactorSum = ~0u;
    int slowCount = 0;
    bool slowDc = false;
    Real slowStepSeconds = 0;
    void resetSlowSupplies() noexcept;
    void stepSlowSupplies() noexcept;
    bool isSlowSource (int net, int source) const noexcept;
    std::vector<Op> ops;
    int inputNet = -1, inputSource = -1, outputNet = -1, outputProbe = -1;
    bool finalized = false;
};
} // namespace ar::circuit
