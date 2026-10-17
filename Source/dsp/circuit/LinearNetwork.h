#pragma once

#include <string>
#include <vector>

namespace ar::circuit
{
using Real = double;

/** Potentiometer tapers. Position 0..1 is the knob; the taper maps it to the fraction of the
    track between the wiper and the "b" end. */
enum class Taper { linear, audio, reverseAudio };

Real taperFraction (Taper taper, Real position) noexcept;

/** A linear sub-circuit solved by modified nodal analysis with the trapezoidal rule.

    Nodes are created by name (node 0 is ground). Components stamp conductances; capacitors and
    inductors keep their own history. Potentiometers, switches and voltage sources can be
    changed at run time; a pot or switch change refactors the matrix, a source change only
    touches the right-hand side.

    Non-linear devices connect through *ports*: a port is a node into which an external current
    is injected. The network exposes, for the current sample, the affine relation
        v[port] = v0[port] + sum_q R[port][q] * i[q]
    so a device can solve for its own currents with Newton's method, then hand them back. */
class LinearNetwork
{
public:
    explicit LinearNetwork (std::string name = {});

    //== Building ===============================================================
    int node (const std::string& name);                  // get or create; "gnd" / "0" is ground
    int numNodes() const noexcept { return (int) nodeNames.size(); }

    void resistor (int a, int b, Real ohms);
    void capacitor (int a, int b, Real farads);
    /** A capacitor whose trapezoidal companion can be pre-warped (see setCapacitorWarp).
        Returns an index for setCapacitorWarp(). */
    int warpedCapacitor (int a, int b, Real farads);
    void inductor (int a, int b, Real henries);
    /** Two magnetically coupled windings (a transformer): branch 1 from a1 to b1, branch 2
        from a2 to b2, self inductances L1, L2 and mutual inductance M (positive when current
        entering a1 and a2 produce aiding flux). */
    void coupledInductors (int a1, int b1, int a2, int b2, Real L1, Real L2, Real M);
    /** A pot between a and b with the wiper at w. Returns an index for setPot(). */
    int potentiometer (int a, int w, int b, Real ohms, Taper taper, Real initialPosition = 0.5);
    /** A resistor that can be opened. Returns an index for setSwitch(). */
    int switchResistor (int a, int b, Real ohms, bool closed);
    /** A capacitor that can be disconnected. Returns an index for setSwitch(). */
    int switchCapacitor (int a, int b, Real farads, bool closed);
    /** An ideal voltage source from ground to the node. Returns an index for setSource(). */
    int voltageSource (int node, Real volts);
    /** An ideal voltage-controlled voltage source: v[out] = gain * (v[inPlus] - v[inMinus]),
        referenced to ground. Used for transformer secondaries. */
    void vcvs (int out, int inPlus, int inMinus, Real gain);
    /** A node where an external device injects current. Returns the port index. */
    int port (int node);
    /** A node whose voltage is read after each sample. Returns the probe index. */
    int probe (int node);

    //== Run-time control ========================================================
    void setPot (int index, Real position01);
    void setSwitch (int index, bool closed);
    void setSource (int index, Real volts);
    /** Pre-warps a capacitor from warpedCapacitor() for `hz`: its companion conductance is
        scaled by x / tan x (x = pi hz T), so its admittance at hz is the analog one instead of
        the trapezoidal rule's (which is tan x / x times too large there). A sub-circuit whose
        capacitors all share one warp frequency then has the analog response exactly at that
        frequency, as a bilinear transform pre-warped there. 0 is the plain rule. The voltage
        and current of the capacitor carry over, so it can move while running (a refactor
        follows, as for a pot). Opt-in: plain capacitors are never warped. */
    void setCapacitorWarp (int index, Real hz);
    Real getSource (int index) const noexcept { return sources[(size_t) index].volts; }
    int numSources() const noexcept { return (int) sources.size(); }

    //== Simulation ==============================================================
    void prepare (Real sampleRate);
    /** DC mode opens the capacitors and shorts the inductors, to find the operating point. */
    void setDcMode (bool dc);
    /** Refactors the matrix if a pot or switch changed. Call between samples. */
    void refactorIfNeeded();

    void beginSample();
    /** The node voltage with every port current at zero (valid between beginSample and endSample). */
    Real openNodeVoltage (int node) const noexcept
    {
        if (node <= 0) return 0.0;
        const int p = reducedIndex (node);
        return p >= 0 ? xh[(size_t) p] : sources[(size_t) nodeSource[(size_t) node - 1]].volts;
    }
    Real portResistance (int p, int q) const noexcept { return portR[(size_t) p * ports.size() + (size_t) q]; }
    void setPortCurrent (int port, Real amps) noexcept { portI[(size_t) port] = amps; }
    Real portCurrent (int port) const noexcept { return portI[(size_t) port]; }
    int  portNode (int port) const noexcept { return ports[(size_t) port]; }
    int  numPorts() const noexcept { return (int) ports.size(); }
    /** Increments on every refactor; devices use it to cache port resistances. */
    unsigned refactorCount() const noexcept { return refactors; }
    void endSample();

    /** The current flowing from a voltage source's node into the source (valid after endSample). */
    Real sourceCurrent (int index) const noexcept;
    /** d sourceCurrent(m) / d volts(index) for every source m, from the factored matrix (not
        for the audio thread's every sample: it solves the system once). */
    void sourceAdmittance (int index, std::vector<Real>& out) const;
    Real probeVoltage (int probe) const noexcept { return probeV[(size_t) probe]; }
    Real nodeVoltage (int node) const noexcept { return node == 0 ? 0.0 : x[(size_t) node - 1]; }
    const std::string& nodeName (int node) const { return nodeNames[(size_t) node]; }
    const std::string& getName() const noexcept { return name; }
    /** Debug: prints every reactive element's voltage and companion current. */
    void dumpReactive (const char* label) const;
    void setComputeAllNodes (bool) noexcept {}
    std::string describe() const;

private:
    struct Resistor { int a, b; Real ohms; };
    struct Capacitor { int a, b; Real farads, g = 0, hist = 0, vPrev = 0; bool enabled = true; Real warpHz = 0; };
    struct Inductor { int a, b; Real henries, g = 0, hist = 0; };
    struct CoupledPair { int a1, b1, a2, b2; Real L1, L2, M; Real g11 = 0, g12 = 0, g22 = 0, hist1 = 0, hist2 = 0; };
    struct Pot { int a, w, b; Real ohms; Taper taper; Real position; };
    struct Switch { int component; bool isCapacitor; bool closed; };
    struct Source { int node; Real volts; };
    struct Vcvs { int out, inPlus, inMinus; Real gain; };

    void stampConductance (int a, int b, Real g) noexcept;
    /** The trapezoidal companion conductance of a capacitor at the running rate, pre-warped. */
    Real companionConductance (const Capacitor& c) const noexcept;
    void factorise();
    void solveInPlace (std::vector<Real>& rhs) const noexcept;
    void rebuildPortResponses();

    std::string name;
    std::vector<std::string> nodeNames;
    std::vector<Resistor> resistors;
    std::vector<Capacitor> capacitors;
    std::vector<Inductor> inductors;
    std::vector<CoupledPair> coupled;
    std::vector<Pot> pots;
    std::vector<Switch> switches;
    std::vector<Source> sources;
    std::vector<Vcvs> vcvsList;
    std::vector<int> ports, probes;

    Real T = 1.0 / 44100.0;
    bool dcMode = false, needsRefactor = true, prepared = false;
    unsigned refactors = 0;
    int dim = 0;    // unknowns of the reduced system (free nodes, kept source rows, VCVS rows)
    int full = 0;   // unknowns of the plain MNA system

    std::vector<Real> A;          // full x full, row-major
    std::vector<Real> lu;         // dim x dim, row-major
    std::vector<int> pivot;
    // The LU factors as compressed rows (only non-zeros), for the per-sample solves.
    std::vector<int> lRowStart, lCol, uRowStart, uCol;
    std::vector<Real> lVal, uVal, uDiag;
    // Fixed-node elimination: a node driven by a grounded source is not an unknown.
    struct Coupling { int row, source; Real g; };
    std::vector<int> freeIdx, posOf, nodeSource;     // reduced -> full, full -> reduced (-1), node-1 -> source (-1)
    /** The node's index in the reduced system, or -1 for ground and fixed nodes. */
    int reducedIndex (int node) const noexcept { return node > 0 ? posOf[(size_t) node - 1] : -1; }
    std::vector<char> srcEliminated;
    std::vector<Coupling> couplings;                 // A[free row][fixed node column]
    std::vector<std::pair<int, int>> fixedNodes;     // (full index, source)
    std::vector<int> srcRowStart, srcRowCol;         // per eliminated source: its node's row of A
    std::vector<Real> srcRowVal;
    std::vector<std::vector<int>> srcPorts;          // ports on each fixed node
    std::vector<Real> rhsFull, x;                    // full layout: right-hand side, solution
    std::vector<Real> xh, xr;                        // reduced: ports open, final
    mutable std::vector<Real> scratch;
    /** rebuildPortResponses()' right-hand side, sized in prepare(): a refactor (a pot or switch
        moved, on the audio thread) then allocates nothing. (urei1176 change, see docs.) */
    std::vector<Real> unitRhs;
    // Ports on one node share a response (urei1176 change, see docs): per port its slot (-1 on
    // a fixed node or ground, which no injected current moves), per slot the node's reduced
    // index, its dim-vector response to a unit injected current, and the slot's total current.
    std::vector<int> portSlot, slotPos;
    std::vector<Real> slotZ, slotI;
    std::vector<Real> portR;      // numPorts x numPorts
    std::vector<Real> portI, probeV;
};
} // namespace ar::circuit
