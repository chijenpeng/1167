#pragma once

// Parameter smoothing at a control interval and the prepare / reset pattern for this plug-in's
// schematic-built circuit. Only the controls and lifecycle needed by the 1176 are kept here.

#include "circuit/Circuit.h"

#include <functional>
#include <vector>

namespace urei1176
{
using ar::circuit::Real;

/** A schematic-driven circuit with smoothed controls.

    A subclass builds its networks and device groups in build() and declares each control with
    addParam(): a target value, a smoothing step and the function that applies a value to the
    circuit (moving a pot, closing a switch). The audio thread sets targets with setTarget();
    controlTick() moves every control one step towards its target and refactors the networks
    whose pots or switches changed. Call it every kControlInterval samples.

    Real-time contract: setTarget(), controlTick() and the circuit's process() neither allocate
    nor lock (the apply functions must not either). prepare() and reset() do both and belong
    off the audio thread (or in the host's prepare call). */
class CircuitModule
{
public:
    virtual ~CircuitModule() = default;

    /** Samples between control updates (parameter steps and refactoring). */
    static constexpr int kControlInterval = 32;

    /** Builds the circuit for this rate and settles its operating point. Allocates. */
    void prepare (double sampleRate);
    /** Jumps every control to its target and settles the operating point again. */
    void reset();

    bool isPrepared() const noexcept { return built; }

protected:
    /** Adds the circuit's networks and groups and declares the controls. */
    virtual void build (double sampleRate) = 0;

    /** Declares a control. `step` is the largest change per control tick (0: it jumps). The
        target persists across prepare() calls (a sample-rate change keeps the knobs). */
    int addParam (float initial, float step, std::function<void (float)> apply);
    void setTarget (int index, float value) noexcept;

    /** Moves the controls one step and refactors the networks when anything changed. */
    void controlTick() noexcept;

    ar::circuit::Circuit circ;

private:
    struct Param
    {
        float step;
        std::function<void (float)> apply;
        float target, current;
    };
    std::vector<Param> params;
    bool built = false;
};
} // namespace urei1176
