// UREI 1176LN limiting amplifier, one channel: see Urei1176.h, docs/reference/schematic.md and
// docs/research/urei-1176.md.
//
// Two networks, as in analog-recreation's Fet1176 (the verified reference this extends):
//   "1176": the 600 ohm source, the bridged-T Input attenuator, input transformer T1, the Q1
//           gain cell, the Q2-Q4 preamp and the whole side chain (ratio sections A and B, the
//           Q12-Q15 GR amp, CR3 / CR4, attack, release, Q bias, the Attack-off switch). Every
//           semiconductor of it is one Newton group, so the compression loop closes within the
//           sample.
//   "line": the Output pot R27 and the discrete class AB line amp Q5-Q9, C15, output transformer
//           T2 and the bridging load, fed by an ideal copy of the preamp output (R27's 100k load
//           stays in the first network).

#include "Urei1176Circuit.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace urei1176
{
using namespace ar::circuit;

namespace
{
using DeviceList = std::vector<std::unique_ptr<Device>>;

/** R81 "Q bias": UREI's factory test 10 turns it until the output drops exactly 1.0 dB with the
    limiting switched off, which puts Q1 slightly into conduction at idle. */
constexpr Real kIdleReductionDb = 1.0;
/** R16, the 100 ohm trimmer setting how much of the drain swing reaches the gate, set as UREI's
    manual says (5.3.5): minimum THD for 500 Hz at -30 dB, Input full, Output at 18, Attack OFF,
    Release full, 20:1. With the rev F values the minimum is still at 25-33 ohm. */
constexpr Real kR16 = 33.0;
/** The values that differ between UREI's rev F drawing (s/n 2611-7052, the one modelled) and its
    rev K drawing (the one gyraf.dk redrew with static voltages), in the part both share: from
    T1's secondary (rev K: C33 and the Input pot) to C15. The rev K set exists to check the
    netlist against the voltages marked on that drawing. */
struct Values
{
    Real supply;                // +30 V rail: rev F a 30 V shunt zener (CR5, R84 75R 3W); rev K a 7824 lifted to 30.3 V
    Real c2, r21, c9;           // preamp
    Real c10, r29, r30, r32, r33, c13, r34, c12;   // line amp
};
constexpr Values kRevF { 30.0, 68e-6, 6.8e3, 1.0e-6, 0.15e-6, 6.8e6, 1e6, 1e3, 4.7e3, 2.2e-9, 3.9e3, 12e-12 };
constexpr Values kRevK { 30.3, 100e-6, 7.5e3, 47e-6, 1.0e-6, 1e6, 200e3, 2e3, 11e3, 100e-6, 4.3e3, 10e-12 };
/** Fraction of the drain swing across Q1's channel: the source follows the rest (C2 to the
    R13 / R14 junction). Used for the meter and the factory bias only, not in the circuit. */
constexpr Real kChannelShare = 0.48;

/** Q1, UREI's selected 2N5457 (13-0027): Vp -1.5 V, beta 1.2 mA/V^2 (Idss 2.7 mA), the value
    that passes UREI's ratio test (analog-recreation's Fet1176). */
constexpr JfetSpec kQ1 { 1.2e-3, -1.5, 0.01 };

/** Q1's channel conductance at vds = 0. */
Real channelConductance (const JfetDevice& fet, Real vgs) noexcept
{
    Real v[2] = { vgs, 0.0 }, u[1] = { 0.0 }, j[2] = { 0.0, 0.0 };
    fet.evaluate (v, u, j);
    return j[1];
}
} // namespace

//==============================================================================================
Real Circuit1176::gateSourceVolts() const noexcept
{
    return main->nodeVoltage (gateNode) - main->nodeVoltage (sourceNode);
}

Real Circuit1176::controlVolts() const noexcept
{
    return main->nodeVoltage (controlNode);
}

void Circuit1176::zeroMeter() noexcept
{
    feedbackAverage = main->nodeVoltage (feedbackNode);
    sourceAverage = sourceIdle = main->nodeVoltage (sourceNode);
    controlIdle = main->nodeVoltage (controlNode);
    vgsControl = gateSourceVolts();
    averageCoeff = 1.0 - std::exp (-1.0 / (0.05 * fs));
    idleConductanceMeter = channelConductance (*gainFet, vgsControl);
    // R54 "tracking": -10 VU at 10 dB of real reduction.
    meterSlope = (1.0 - std::pow (10.0, -10.0 / 20.0)) / (controlForReduction (10.0) - controlIdle);
}

Real Circuit1176::controlForReduction (Real dB) const noexcept
{
    // At the calibration setting (Input full), steady state: the gate's DC is the control node's.
    auto reduction = [this] (Real vc)
    {
        const Real g = channelConductance (*gainFet, vc - sourceIdle);
        return 20.0 * std::log10 ((1.0 + calibrationOhms * g) / (1.0 + calibrationOhms * idleConductanceMeter));
    };
    Real lo = controlIdle, hi = 0.5;
    for (int i = 0; i < 60; ++i)
    {
        const Real mid = 0.5 * (lo + hi);
        (reduction (mid) < dB ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

void Circuit1176::setBiasPot (Real position) noexcept
{
    main->setPot (biasPot, position);
    main->refactorIfNeeded();
}

void Circuit1176::startTracking() noexcept
{
    feedbackAverage = main->nodeVoltage (feedbackNode);
    sourceAverage = main->nodeVoltage (sourceNode);
    vgsControl = gateSourceVolts();
}

Real Circuit1176::reductionDbAt (Real vgs) const noexcept
{
    // Against the meter's zero (R55, set at idle): negative when Q1 conducts less than at the
    // calibrated idle (the All buttons' lower bias), as the real needle then rises above 0.
    const Real g = channelConductance (*gainFet, vgs);
    return 20.0 * std::log10 ((1.0 + seriesOhms * g) / (1.0 + seriesOhms * idleConductanceMeter));
}

Real Circuit1176::nodeVolts (const std::string& network, const std::string& node)
{
    for (auto& net : circ.networks())
        if (net->getName() == network)
            for (int k = 1; k < net->numNodes(); ++k)
                if (net->nodeName (k) == node)
                    return net->nodeVoltage (k);
    throw std::invalid_argument ("no node " + network + "/" + node);
}


//==============================================================================================
void Circuit1176::build (double sampleRate)
{
    fs = sampleRate;
    const Values& v = revK ? kRevK : kRevF;
    // Q1's gate junctions (to source and drain): they clamp the gate near +0.5 V when the
    // control voltage overshoots at extreme input levels.
    const DiodeSpec gateJunction { 1.0e-14, 1.0, 0.0, 1.0e-12 };
    const BjtSpec npn { 1.0e-14, 300.0, 4.0 };         // BC107 / 2N3391A / 2N3707 class
    // CR3 / CR4 FD333: the control node sits behind up to 5.3M of release network, where a
    // 1N4148's nanoamps of leakage would pull Q1's idle gate down by 25 mV.
    const DiodeSpec fd333 = DiodeSpec::fd333();
    auto bjt = [this, npn] (LinearNetwork& net, int b, int c, int e)
    {
        return std::make_unique<BjtDevice> (npn, circ.port (net, b), circ.port (net, c), circ.port (net, e));
    };
    auto diode = [this, fd333] (LinearNetwork& net, int anode, int cathode)
    {
        return std::make_unique<DiodePairDevice> (fd333, DiodeSpec::none(), circ.port (net, anode), circ.port (net, cathode));
    };

    auto& n = circ.addNetwork ("1176");
    main = &n;
    const int v30 = n.node ("+30V"), vNeg = n.node ("-10V");
    n.voltageSource (v30, v.supply);        // rev F: CR5 30 V shunt zener behind R84 75R
    n.voltageSource (vNeg, -10.0);          // CR6 10 V zener
    const int vPre = n.node ("pre.B+"), vGr = n.node ("gr.B+");
    n.resistor (v30, vPre, 6.8e3);          // R17
    n.capacitor (vPre, 0, 100e-6);          // C7
    n.resistor (v30, vGr, 4.7e3);           // R70
    n.capacitor (vGr, 0, 47e-6);            // C18

    //-- input: 600 ohm source, Input attenuator, T1 -------------------------------------------
    const int r6In = buildInput (n);

    //-- gain cell and preamp -------------------------------------------------------------------
    // Q1 shunts the signal after R6 27k. Its source returns through C2 to the middle of Q2's
    // emitter resistors, so about half the drain swing appears across the channel (the LN
    // circuit), and R9 / C5 feed half the preamp's feedback node to the gate.
    const int d = n.node ("q1.d"), g = n.node ("q1.g"), s = n.node ("q1.s");
    const int c1 = n.node ("c1"), b2 = n.node ("q2.b"), c2 = n.node ("q2.c"), e2 = n.node ("q2.e"), e2m = n.node ("r13.r14");
    const int r15 = n.node ("r15.r16"), fb = n.node ("fb"), c3 = n.node ("q3.c"), e3 = n.node ("q3.e"), e3m = n.node ("r19.r20");
    const int out1 = n.node ("q4.e"), pre = n.node ("pre.out");
    n.resistor (r6In, d, 27e3);             // R6
    n.resistor (s, 0, 10e3);                // R8
    n.capacitor (s, e2m, v.c2);             // C2
    n.capacitor (d, c1, 1.0e-6);            // C1
    n.resistor (c1, b2, 1e3);               // R10
    n.capacitor (b2, 0, 27e-12);            // C6
    n.resistor (b2, e3m, 560e3);            // R11 (DC feedback from Q3's emitter)
    n.resistor (vPre, c2, 1e6);             // R12
    n.resistor (e2, e2m, 180.0);            // R13
    n.resistor (e2m, 0, 180.0);             // R14
    n.resistor (e2, r15, 150.0);            // R15
    n.resistor (r15, fb, kR16);             // R16 (100 ohm trimmer, factory set)
    n.resistor (fb, out1, v.r21);           // R21
    n.resistor (vPre, c3, 22e3);            // R18
    n.resistor (e3, e3m, 82.0);             // R19
    n.resistor (e3m, 0, 1.8e3);             // R20
    n.capacitor (e3m, 0, 100e-6);           // C8
    n.capacitor (out1, pre, v.c9);          // C9
    n.resistor (pre, 0, 100e3);             // R27's track (the pot itself is in "line")

    //-- gate: half the control voltage and half the feedback node ------------------------------
    const int ctl = n.node ("ctl"), c5 = n.node ("c5");
    n.resistor (g, ctl, 2.2e6);             // R7
    n.capacitor (g, ctl, 200e-12);          // C3
    n.resistor (g, c5, 2.2e6);              // R9
    n.capacitor (g, c5, 200e-12);           // C4
    n.capacitor (c5, fb, 0.22e-6);          // C5
    n.capacitor (ctl, 0, 0.22e-6);          // C27, the timing capacitor

    DeviceList loop;   // the gain cell, the preamp and the side chain: one Newton group
    {
        auto fet = std::make_unique<JfetDevice> (kQ1, circ.port (n, g), circ.port (n, d), circ.port (n, s));
        gainFet = fet.get();
        loop.push_back (std::move (fet));            // Q1
        loop.push_back (std::make_unique<DiodePairDevice> (gateJunction, DiodeSpec::none(), circ.port (n, g), circ.port (n, s)));
        loop.push_back (std::make_unique<DiodePairDevice> (gateJunction, DiodeSpec::none(), circ.port (n, g), circ.port (n, d)));
    }
    loop.push_back (bjt (n, b2, c2, e2));            // Q2
    loop.push_back (bjt (n, c2, c3, e3));            // Q3
    loop.push_back (bjt (n, c3, v30, out1));         // Q4
    gateNode = g;
    sourceNode = s;
    feedbackNode = fb;
    controlNode = ctl;

    //-- ratio, section B: how much of the preamp output reaches the GR amp ---------------------
    // The four B sections connect a tap to the ratio bus; S9 (the Attack knob's OFF switch)
    // passes the bus to node 22 and C17, or grounds node 22 (limiting OFF).
    const int rb4 = n.node ("ratio.b4"), rb3 = n.node ("ratio.b3"), rb2 = n.node ("ratio.b2"), rb1 = n.node ("ratio.b1"), bus = n.node ("ratio.bus");
    const int grIn = n.node ("gr.in");   // node 22
    n.resistor (pre, rb4, 56e3);            // R26
    n.resistor (rb4, rb3, 68e3);            // R25
    n.resistor (rb3, rb2, 56e3);            // R24
    n.resistor (rb2, rb1, 56e3);            // R22
    n.resistor (rb1, 0, 47e3);              // R23
    int sw = 0;
    for (int tap : { rb1, rb2, rb3, rb4 })  // S1B 4:1 ... S4B 20:1
        ratioSwitches[sw++] = n.switchResistor (tap, bus, 1.0, tap == rb1);
    limitSwitches[0] = n.switchResistor (bus, grIn, 1.0, true);    // S9, limiting ON
    limitSwitches[1] = n.switchResistor (grIn, 0, 1.0, false);     // S9, OFF: node 22 to ground

    //-- GR control amp: Q12 / Q13 give the inverted phase, Q14 / Q15 the direct one -----------
    const int b12 = n.node ("q12.b"), c12 = n.node ("q12.c"), e12 = n.node ("q12.e"), r64 = n.node ("r64.r65"), e13 = n.node ("q13.e");
    const int b14 = n.node ("q14.b"), c14 = n.node ("q14.c"), e14 = n.node ("q14.e"), r72 = n.node ("r72.r73"), e15 = n.node ("q15.e");
    n.capacitor (grIn, b12, 1.0e-6);        // C17
    n.resistor (c12, b12, 1e6);             // R60
    n.resistor (b12, 0, 470e3);             // R61
    n.resistor (vGr, c12, 47e3);            // R62
    n.resistor (e12, r64, 4.7e3);           // R63
    n.resistor (r64, 0, 270.0);             // R65
    n.resistor (e13, r64, 2.4e3);           // R64
    n.resistor (e12, e15, 47e3);            // R68 (overall feedback)
    n.resistor (e13, b14, 182e3);           // R66
    n.resistor (b14, r72, 38.3e3);          // R67
    n.resistor (vGr, c14, 44.2e3);          // R69
    n.resistor (e14, r72, 7.68e3);          // R71
    n.resistor (e15, r72, 2.4e3);           // R72
    n.resistor (r72, 0, 180.0);             // R73

    //-- rectifier, attack, release --------------------------------------------------------------
    // CR3 / CR4 charge C27 through R76 || C22 and the Attack pot; C27 discharges through the
    // Release pot (|| R79 on part of it) into the bias. The diodes' anodes return to the ratio
    // switch's tap (section A) through R74 / R75: the threshold.
    const int ra = n.node ("cr3.a"), rbn = n.node ("cr4.a"), ref = n.node ("ratio.ref"), k = n.node ("cr.k"), aw = n.node ("attack.w");
    const int rw = n.node ("release.w"), bias = n.node ("bias"), bp = n.node ("bias.p");
    n.capacitor (e15, ra, 6.8e-6);          // C19
    n.capacitor (e13, rbn, 6.8e-6);         // C20
    n.resistor (ra, ref, 47e3);             // R74
    n.resistor (rbn, ref, 47e3);            // R75
    n.capacitor (ref, 0, 100e-6);           // C21
    n.resistor (k, aw, 470.0);              // R76
    n.capacitor (k, aw, 22e-9);             // C22
    const int attackPot = n.potentiometer (ctl, aw, k, 25e3, Taper::linear);      // R77, position 1 = fastest
    const int releasePot = n.potentiometer (ctl, rw, bias, 5e6, Taper::linear);   // R78, position 1 = fastest
    n.resistor (rw, bias, 270e3);           // R79
    n.resistor (vNeg, bp, 10e3);            // R82
    n.resistor (bp, 0, 3.9e3);              // R83
    seriesOhms = calibrationOhms = gainCellOhms (inputPosition (Range::inputMax));
    gateAtIdle = idleGate();
    biasPos = biasPosition (gateAtIdle);
    biasPot = n.potentiometer (bp, bias, 0, 2e3, Taper::linear, biasPos);   // R81 "Q bias"

    //-- ratio, section A: the rectifier's reference below the bias -----------------------------
    const int ra1 = n.node ("ratio.a1"), ra2 = n.node ("ratio.a2"), ra3 = n.node ("ratio.a3"), ra4 = n.node ("ratio.a4");
    n.resistor (bias, ra1, 150.0);          // R87
    n.resistor (ra1, ra2, 470.0);           // R42
    n.resistor (ra2, ra3, 560.0);           // R43
    n.resistor (ra3, ra4, 1.5e3);           // R44
    n.resistor (ra4, vNeg, 2.2e3);          // R45
    n.resistor (ra1, ref, 10e6);            // R46
    for (int tap : { ra1, ra2, ra3, ra4 })  // S1A ... S4A
        ratioSwitches[sw++] = n.switchResistor (tap, ref, 1.0, tap == ra1);

    // Solved with the gain cell: as separate groups the cell would see the rectifier's current a
    // sample late (at 48 kHz up to 1 dB off a 1 ms envelope in All mode).
    loop.push_back (bjt (n, b12, c12, e12));         // Q12
    loop.push_back (bjt (n, c12, v30, e13));         // Q13
    loop.push_back (bjt (n, b14, c14, e14));         // Q14
    loop.push_back (bjt (n, c14, v30, e15));         // Q15
    loop.push_back (diode (n, ra, k));               // CR3
    loop.push_back (diode (n, rbn, k));              // CR4
    circ.addGroup (std::move (loop));

    //== line amp ===================================================================================
    // Q5 drives the Q6 / Q7 PNP Darlington, whose collector current runs through the CR1 / CR2 bias
    // diodes and R38 / R39 (bootstrapped by C14) into the MPS-U05 / MPS-U55 class AB pair;
    // feedback R34 over R32 and R33 + C13, a non-inverting gain of 3.54.
    auto& l = circ.addNetwork ("line");
    const int lv30 = l.node ("+30V"), lin = l.node ("pre.out");
    l.voltageSource (lv30, v.supply);
    circ.addBuffer (n, pre, l, l.voltageSource (lin, 0.0));
    const int ow = l.node ("output.w"), b5 = l.node ("q5.b"), b5r = l.node ("r28.r29"), e5 = l.node ("q5.e"), c13 = l.node ("c13");
    const int q5c = l.node ("q5.c"), e6 = l.node ("q6.e"), e7 = l.node ("q7.e"), x = l.node ("q8.b"), cr = l.node ("cr1.cr2");
    const int y = l.node ("q9.b"), z = l.node ("r38.r39"), r37 = l.node ("c12.r37"), e8 = l.node ("q8.e"), e9 = l.node ("q9.e");
    const int lo = l.node ("line.out");
    const int outputPot = l.potentiometer (lin, ow, 0, 100e3, Taper::audio);   // R27
    l.capacitor (ow, b5, v.c10);            // C10
    l.resistor (lv30, b5r, 10e3);           // R28
    l.capacitor (b5r, 0, 47e-6);            // C11
    l.resistor (b5r, b5, v.r29);            // R29
    l.resistor (b5, 0, v.r30);              // R30
    l.resistor (lv30, q5c, 68e3);           // R31
    l.resistor (e5, 0, v.r32);              // R32
    l.resistor (e5, c13, v.r33);            // R33
    l.capacitor (c13, 0, v.c13);            // C13 (rev F: a small HF shelf; rev K: AC ground)
    l.resistor (lo, e5, v.r34);             // R34 (feedback)
    l.resistor (lv30, e6, 10e3);            // R35
    l.resistor (lv30, e7, 100.0);           // R36
    l.capacitor (q5c, r37, v.c12);          // C12
    l.resistor (r37, y, 1e3);               // R37
    l.resistor (y, z, 1.2e3);               // R38
    l.resistor (z, 0, 1.2e3);               // R39
    l.capacitor (z, lo, 22e-6);             // C14 (bootstrap)
    l.capacitor (y, 0, 47e-12);             // C29
    l.resistor (e8, lo, 4.3);               // R40
    l.resistor (lo, e9, 4.3);               // R41
    l.resistor (x, 0, 1e9);                 // leaks for the nodes that only junctions touch
    l.resistor (cr, 0, 1e9);
    {
        // Saturation currents of the power pair from the drawing's 0.59 V at about 15 mA.
        const BjtSpec q5Spec { 1.0e-14, 300.0, 4.0 };          // Q5 (5117; gyraf BC107)
        const BjtSpec q67Spec { 6.7e-15, 250.0, 4.0, true };   // Q6, Q7 2N5087
        const BjtSpec q8Spec { 2.0e-12, 100.0, 4.0 };          // Q8 MPS-U05
        const BjtSpec q9Spec { 2.0e-12, 100.0, 4.0, true };    // Q9 MPS-U55
        const DiodeSpec n4003 { 14.1e-9, 1.984, 0.034 };       // CR1, CR2 1N4003
        DeviceList amp;
        amp.push_back (std::make_unique<BjtDevice> (q5Spec, circ.port (l, b5), circ.port (l, q5c), circ.port (l, e5)));
        amp.push_back (std::make_unique<BjtDevice> (q67Spec, circ.port (l, q5c), circ.port (l, x), circ.port (l, e6)));
        amp.push_back (std::make_unique<BjtDevice> (q67Spec, circ.port (l, e6), circ.port (l, x), circ.port (l, e7)));
        amp.push_back (std::make_unique<DiodePairDevice> (n4003, DiodeSpec::none(), circ.port (l, x), circ.port (l, cr)));
        amp.push_back (std::make_unique<DiodePairDevice> (n4003, DiodeSpec::none(), circ.port (l, cr), circ.port (l, y)));
        amp.push_back (std::make_unique<BjtDevice> (q8Spec, circ.port (l, x), circ.port (l, lv30), circ.port (l, e8)));
        amp.push_back (std::make_unique<BjtDevice> (q9Spec, circ.port (l, y), circ.port (l, 0), circ.port (l, e9)));
        circ.addGroup (std::move (amp));
    }
    circ.setOutput (l, buildOutput (l, lo));

    //-- controls (pot positions 0..1 clockwise, switch states) ----------------------------------
    // Pots move at most 2 % of their rotation per control tick, as analog-recreation does.
    addParam ((float) inputPosition (Range::inputDefault), 0.02f, [this] (float v) { applyInput (v); });
    addParam ((float) outputPosition (Range::outputDefault), 0.02f, [&l, outputPot] (float v) { l.setPot (outputPot, v); });
    addParam ((float) attackPosition (Range::attackDefault), 0.02f, [&n, attackPot] (float v) { n.setPot (attackPot, v); });
    addParam ((float) releasePosition (Range::releaseDefault), 0.02f, [&n, releasePot] (float v) { n.setPot (releasePot, v); });
    addParam ((float) Ratio::r4, 0.0f, [this] (float v) { applyRatio ((int) std::lround (v)); });
    addParam (0.0f, 0.0f, [this] (float v) { applyLimitOff (v > 0.5f); });
}

void Circuit1176::applyRatio (int sel)
{
    // Section B switches 0..3, section A switches 4..7; All closes every one, None none.
    for (int i = 0; i < 8; ++i)
        main->setSwitch (ratioSwitches[i], sel == (int) Ratio::all || (sel != (int) Ratio::none && (i % 4) == sel));
}

/** Position of R81 (2k between ground and R82 / R83's junction) that puts the bias node at
    `volts` with the 4:1 threshold chain (R87 ... R45, 4.88k to -10 V) hanging on it. */
Real Circuit1176::biasPosition (Real volts)
{
    auto biasAt = [] (Real f)
    {
        // nodes P (R82 / R83 / pot end) and B (wiper): 2 x 2 nodal equations
        const Real rLow = std::max (1.0, 2e3 * f), rHigh = std::max (1.0, 2e3 * (1.0 - f));
        const Real gChain = 1.0 / 4880.0;
        const Real a11 = 1.0 / 10e3 + 1.0 / 3.9e3 + 1.0 / rHigh, a12 = -1.0 / rHigh, b1 = -10.0 / 10e3;
        const Real a21 = -1.0 / rHigh, a22 = 1.0 / rHigh + 1.0 / rLow + gChain, b2 = -10.0 * gChain;
        return (a11 * b2 - a21 * b1) / (a11 * a22 - a12 * a21);
    };
    Real lo = 0.0, hi = 1.0;   // the bias falls as the wiper moves towards P
    for (int i = 0; i < 60; ++i)
    {
        const Real mid = 0.5 * (lo + hi);
        (biasAt (mid) > volts ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

/** The idle gate voltage that costs kIdleReductionDb with the Input fully up, as the factory set
    it (UREI test 10). */
Real Circuit1176::idleGate() const
{
    Real lo = kQ1.vp - 0.5, hi = 0.0;
    for (int i = 0; i < 60; ++i)
    {
        const Real mid = 0.5 * (lo + hi);
        const Real dB = 20.0 * std::log10 (1.0 + seriesOhms * channelConductance (*gainFet, mid));
        (dB < kIdleReductionDb ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}


//==============================================================================================
// Input section and output transformer

namespace
{
/** Adds a transformer: coupled windings (a1-b1 primary, a2-b2 secondary) behind their winding
    resistances, with winding capacitances across each and between the hot ends. */
void transformer (LinearNetwork& n, const std::string& name, const TransformerSpec& t, int a1, int b1, int a2, int b2)
{
    const int pw = n.node (name + ".pw"), sw = n.node (name + ".sw");
    n.resistor (a1, pw, t.primaryOhms);
    n.resistor (a2, sw, t.secondaryOhms);
    const Real l1 = t.primaryHenries, l2 = l1 * t.turnsRatio * t.turnsRatio;
    n.coupledInductors (pw, b1, sw, b2, l1, l2, t.coupling * std::sqrt (l1 * l2));
    if (t.primaryFarads > 0.0) n.capacitor (a1, b1, t.primaryFarads);
    if (t.secondaryFarads > 0.0) n.capacitor (a2, b2, t.secondaryFarads);
    if (t.interwindingFarads > 0.0) n.capacitor (a1, a2, t.interwindingFarads);
    if (t.coreOhms > 0.0) n.resistor (pw, b1, t.coreOhms);
}

/** The bridged-T's fixed arms R2 = R3 (620 ohm): its design impedance. */
constexpr Real kZ0 = 620.0;
/** R1A, the bridging deck, at the counter-clockwise stop (one measured unit: 300-330k); R1B, the
    shunt deck, at the clockwise stop (35k measured). */
constexpr Real kBridgeMax = 330e3, kShuntMax = 35e3;
/** The rheostats are modelled as linear pots whose position is set to the wanted resistance. */
constexpr Real kDeckOhms = 1e6;
} // namespace

const TransformerSpec& Circuit1176::inputTransformer()
{
    // UTC O-12 wired 500 ohm (pins 1-5) to 200 ohm (pins 6-8): 1.58:1 down. The original's data is
    // not published; Sowter's 1260 replacement (same ratio) gives the inductance and resistance.
    static const TransformerSpec t {
        1.0 / 1.58,     // turns: 200 / 500 ohm
        9.0,            // primary inductance (Sowter 1260: ~9 H)
        0.99983,        // leakage ~3 mH referred to the primary (approximation)
        55.0, 14.4,     // DCR: 91 ohm total referred to the primary (Sowter 1260), split 55 / 36 x 0.4
        100e-12, 150e-12, 0.0 };   // winding capacitances (approximation; shielded: no interwinding term)
    return t;
}

const TransformerSpec& Circuit1176::outputTransformer()
{
    // B11148 (rev F/G), both primaries and both secondaries in parallel. Sowter's replacement
    // (1486) is "50+50 / 150+150 ohm": 1:1.73 per winding pair. No data on the original: the
    // inductance keeps C15's series resonance below the factory response limits, the winding
    // resistances give the ~39 ohm output impedance UREI's test 7 implies; the leakage is
    // LL5402-like (0.2-0.5 mH). Winding capacitance is left out (see docs: its ~400 kHz
    // resonance would fold to ~23 kHz without oversampling, its in-band effect is < 0.05 dB).
    static const TransformerSpec t {
        1.7320508,      // turns: 50 : 150 ohm
        2.5,            // primary inductance (paralleled 50 ohm windings), approximation
        0.99992,        // leakage ~0.4 mH referred to the primary, approximation
        4.0, 27.0,      // DCR: 27 + 3 x 4 = 39 ohm output impedance (test 7: 600 ohm load costs 0.55 dB)
        0.0, 0.0, 0.0,
        1.5e3 };        // core loss across the primary, approximation (30 x the 50 ohm winding)
    return t;
}

namespace
{
/** The printed values of the Input and Output dial (UA's reissue panel; UREI's not verified),
    40 degrees apart from fully counter-clockwise to fully clockwise. The first label is "inf";
    60 stands for it where a number is needed (the next step of the 36 / 48 spacing). */
constexpr float kDialValues[] { 60.0f, 48.0f, 36.0f, 30.0f, 24.0f, 18.0f, 12.0f, 6.0f, 0.0f };
} // namespace

Real Circuit1176::inputPosition (float rotation) noexcept { return std::clamp ((Real) rotation, 0.0, 1.0); }
Real Circuit1176::outputPosition (float rotation) noexcept { return std::clamp ((Real) rotation, 0.0, 1.0); }
Real Circuit1176::attackPosition (float dial) noexcept { return std::clamp (((Real) dial - 1.0) / 6.0, 0.0, 1.0); }
Real Circuit1176::releasePosition (float dial) noexcept { return std::clamp (((Real) dial - 1.0) / 6.0, 0.0, 1.0); }

Real Circuit1176::inputLossDb (Real rotation) noexcept
{
    // The attenuation follows the printed dial (dB of loss, interpolated in angle between the
    // labels) down to 48; from there to the counter-clockwise stop it rises to what R1A's full
    // track gives (330k: 54.5 dB): the pad never reaches infinity.
    rotation = std::clamp (rotation, 0.0, 1.0);
    const Real pos = rotation * 8.0;
    const int i = std::min (7, (int) pos);
    const Real f = pos - i;
    const Real endDb = 20.0 * std::log10 (1.0 + kBridgeMax / kZ0);
    const Real a = i == 0 ? endDb : (Real) kDialValues[i], b = (Real) kDialValues[i + 1];
    return a + (b - a) * f;
}

void Circuit1176::bridgedTArms (Real rotation, Real& bridge, Real& shunt) noexcept
{
    // Constant impedance: bridge = Z0 (K - 1), shunt = Z0 / (K - 1), each deck within its track.
    const Real k = std::pow (10.0, inputLossDb (rotation) / 20.0);
    bridge = std::clamp (kZ0 * (k - 1.0), 0.0, kBridgeMax);
    shunt = std::min (kShuntMax, k > 1.0 ? kZ0 / (k - 1.0) : kShuntMax);
}

Real Circuit1176::gainCellOhms (Real rotation)
{
    // R6 plus what T1's secondary presents (R5 270 || the reflected source and pad, plus the
    // windings), times the share of the drain swing across the channel.
    Real bridge, shunt;
    bridgedTArms (rotation, bridge, shunt);
    // The pad's output resistance with the 600 ohm source on its input (nodal, 3 nodes).
    const Real gS = 1.0 / kSourceOhms, gB = 1.0 / std::max (bridge, 1.0), gA = 1.0 / kZ0, gSh = 1.0 / std::max (shunt, 1.0);
    // nodes: in (1), mid (2); output driven by a 1 A test current, solve for v_out.
    // [gS+gB+gA, -gA, -gB; -gA, 2gA+gSh, -gA; -gB, -gA, gB+gA] v = [0, 0, 1]
    const Real a11 = gS + gB + gA, a12 = -gA, a13 = -gB, a22 = 2.0 * gA + gSh, a23 = -gA, a33 = gB + gA;
    const Real det = a11 * (a22 * a33 - a23 * a23) - a12 * (a12 * a33 - a23 * a13) + a13 * (a12 * a23 - a22 * a13);
    const Real rPad = (a11 * a22 - a12 * a12) / det;
    const auto& t = inputTransformer();
    const Real n2 = t.turnsRatio * t.turnsRatio;
    const Real rSec = t.secondaryOhms + n2 * (t.primaryOhms + rPad);
    const Real rSource = rSec * 270.0 / (rSec + 270.0);
    return (27e3 + rSource) * kChannelShare;
}

int Circuit1176::buildInput (LinearNetwork& n)
{
    // The 600 ohm source drives TB1 "+-" against "C" (the input floats; C is the reference here).
    // Bridged-T: R2, R3 620 in series, R1A across both, R1B from their junction to C. T1 pins 1-5,
    // secondary pins 6-8 (8 to ground), R5 270 across it, then R6 27k (build()).
    const int gen = n.node ("gen"), in = n.node ("in"), mid = n.node ("pad.mid"), pri = n.node ("t1.pri"), sec = n.node ("t1.sec");
    circ.setInput (n, n.voltageSource (gen, 0.0));
    n.resistor (gen, in, kSourceOhms);
    n.resistor (in, mid, kZ0);                                                   // R2
    n.resistor (mid, pri, kZ0);                                                  // R3
    inputBridge = n.potentiometer (in, pri, pri, kDeckOhms, Taper::linear, 0.0);   // R1A (rheostat)
    // R1B (rheostat). It must reach ~1.2 ohm at the counter-clockwise stop and the engine keeps
    // every pot section at 5 ohm or more, so it is four parallel sections set to 4 x the value.
    for (int k = 0; k < kShuntSections; ++k)
        inputShunt[k] = n.potentiometer (mid, 0, 0, kDeckOhms, Taper::linear, 0.0);
    transformer (n, "t1", inputTransformer(), pri, 0, sec, 0);                     // T1
    n.resistor (sec, 0, 270.0);                                                  // R5
    return sec;
}

void Circuit1176::applyInput (Real rotation)
{
    Real bridge, shunt;
    bridgedTArms (rotation, bridge, shunt);
    // potentiometer (a, w, b): a-w is (1 - position) of the track.
    main->setPot (inputBridge, 1.0 - bridge / kDeckOhms);
    for (int k = 0; k < kShuntSections; ++k)
        main->setPot (inputShunt[k], 1.0 - kShuntSections * shunt / kDeckOhms);
    seriesOhms = gainCellOhms (rotation);
}

int Circuit1176::buildOutput (LinearNetwork& l, int lineOut)
{
    // C15 into T2's paralleled primaries (rev F: B11148, nothing across the secondary), the
    // secondary into the bridging load.
    const int c15 = l.node ("c15"), sec = l.node ("t2.sec");
    l.capacitor (lineOut, c15, 470e-6);     // C15
    transformer (l, "t2", outputTransformer(), c15, 0, sec, 0);
    l.resistor (sec, 0, kLoadOhms);
    return sec;
}

void Circuit1176::applyLimitOff (bool off)
{
    // S9 on R77: a changeover at the GR amp's input (see build()).
    main->setSwitch (limitSwitches[0], ! off);
    main->setSwitch (limitSwitches[1], off);
}

//==============================================================================================
// Channel: levels, controls, meters.

namespace
{
constexpr double kDbuVolts = 0.7745966692;   // 0 dBu, volts rms (1 mW into 600 ohms)
/** VU movement (ANSI C16.5-1954, IEC 60268-17): a sine applied suddenly takes the needle to
    99 % of its steady deflection in 300 ms, overshooting by 1 to 1.5 %; the same falling. As a
    second-order movement with 1.25 % overshoot: zeta 0.8127, and 99 % first reached at 0.3 s
    for wn 13.512 rad/s (2.15 Hz). */
constexpr double kVuZeta = 0.8127, kVuOmega = 13.512;
/** The needle's stops, as deflections (1 = 0 VU; +3 VU = 1.413; 0 = rest, left of -20). */
constexpr double kVuPin = 1.5;
/** A VU meter's full-wave rectifier reads the average: mean |sin| / rms. */
constexpr double kSineAverageOverRms = 0.9003163162;

double dbuToVolts (double dbu) noexcept { return kDbuVolts * std::pow (10.0, dbu / 20.0); }
} // namespace

float dialMarking (float rotation) noexcept
{
    const auto& values = kDialValues;
    rotation = std::clamp (rotation, 0.0f, 1.0f);
    if (rotation <= 0.0f)
        return std::numeric_limits<float>::infinity();
    const float pos = rotation * 8.0f;
    const int i = std::min (7, (int) pos);
    return values[i] + (values[i + 1] - values[i]) * (pos - (float) i);
}

float rotationForMarking (float marking) noexcept
{
    const auto& values = kDialValues;
    if (! (marking < 48.0f))
        return 0.0f;
    marking = std::max (0.0f, marking);
    for (int i = 1; i < 8; ++i)
        if (marking >= values[i + 1])
            return ((float) i + (values[i] - marking) / (values[i] - values[i + 1])) / 8.0f;
    return 1.0f;
}

Channel::Channel() : model (std::make_unique<Circuit1176>()) {}
Channel::~Channel() = default;

void Channel::prepare (double sampleRate)
{
    fs = sampleRate;
    calCurrent = ctl.calibrationDbu;
    updateScale();
    pushControls();
    model->prepare (sampleRate);
    reset();
}

void Channel::reset()
{
    if (! model->isPrepared())
        return;
    // R55, the meter zero: set at idle with a ratio button in (4:1 .. 20:1 idle alike), as the
    // factory does, then the operating point for the present controls.
    pushControls();
    model->setControl (Circuit1176::pRatio, (float) Ratio::r20);
    model->setControl (Circuit1176::pLimitOff, 1.0f);
    model->reset();
    model->zeroMeter();
    pushControls();
    model->reset();
    model->startTracking();
    calCurrent = ctl.calibrationDbu;
    updateScale();
    countdown = 0;
    const float gr = (float) model->reductionDbAt (model->controlGateSourceVolts());
    grNow.store (gr, std::memory_order_relaxed);
    blockGrDb.store (std::max (0.0f, gr), std::memory_order_relaxed);
    needle = ctl.meter == MeterMode::gainReduction ? model->meterDeflection() : 0.0;
    needleSpeed = 0.0;
    needleDb.store ((float) std::clamp (20.0 * std::log10 (std::max (needle, 1e-6)), (double) Range::meterMinDb, (double) Range::meterMaxDb),
                    std::memory_order_relaxed);
}

void Channel::pushControls() noexcept
{
    model->setControl (Circuit1176::pInput, (float) Circuit1176::inputPosition (ctl.input));
    model->setControl (Circuit1176::pOutput, (float) Circuit1176::outputPosition (ctl.output));
    model->setControl (Circuit1176::pAttack, (float) Circuit1176::attackPosition (ctl.attack));
    model->setControl (Circuit1176::pRelease, (float) Circuit1176::releasePosition (ctl.release));
    model->setControl (Circuit1176::pRatio, (float) (int) ctl.ratio);
    model->setControl (Circuit1176::pLimitOff, ctl.limitOff ? 1.0f : 0.0f);
}

void Channel::setInput (float rotation) noexcept
{
    ctl.input = std::clamp (rotation, Range::inputMin, Range::inputMax);
    model->setControl (Circuit1176::pInput, (float) Circuit1176::inputPosition (ctl.input));
}

void Channel::setOutput (float rotation) noexcept
{
    ctl.output = std::clamp (rotation, Range::outputMin, Range::outputMax);
    model->setControl (Circuit1176::pOutput, (float) Circuit1176::outputPosition (ctl.output));
}

void Channel::setAttack (float dial) noexcept
{
    ctl.attack = std::clamp (dial, Range::attackMin, Range::attackMax);
    model->setControl (Circuit1176::pAttack, (float) Circuit1176::attackPosition (ctl.attack));
}

void Channel::setRelease (float dial) noexcept
{
    ctl.release = std::clamp (dial, Range::releaseMin, Range::releaseMax);
    model->setControl (Circuit1176::pRelease, (float) Circuit1176::releasePosition (ctl.release));
}

void Channel::setLimitOff (bool off) noexcept
{
    ctl.limitOff = off;
    model->setControl (Circuit1176::pLimitOff, off ? 1.0f : 0.0f);
}

void Channel::setRatio (Ratio ratio) noexcept
{
    ctl.ratio = ratio;
    model->setControl (Circuit1176::pRatio, (float) (int) ratio);
}

void Channel::setMeterMode (MeterMode mode) noexcept
{
    ctl.meter = mode;
    updateScale();
}

void Channel::setCalibrationDbu (float dbu) noexcept
{
    ctl.calibrationDbu = std::clamp (dbu, Range::calibrationMin, Range::calibrationMax);
}

void Channel::updateScale() noexcept
{
    // 1.0 is the peak of a sine at calibrationDbu. The input is a 600 ohm source calibrated as a
    // generator is, into a matched load: its EMF is twice the level it delivers to 600 ohms. The
    // output is the voltage across the load on T2's secondary.
    unitVolts = dbuToVolts (calCurrent) * std::sqrt (2.0);
    inGain = 2.0 * unitVolts;
    outGain = 1.0 / unitVolts;
    // The VU movement (3.9k with its rectifier) across T2's secondary through R51 3.6k (+4: the
    // standard 0 VU = 1.228 V) or R50 8.2k (+8: 12.1k / 7.5k, 4.16 dB less sensitive).
    const double ref = ctl.meter == MeterMode::plus8 ? 4.0 + 20.0 * std::log10 ((3.9e3 + 8.2e3) / (3.9e3 + 3.6e3)) : 4.0;
    vuReference = kSineAverageOverRms * dbuToVolts (ref);
}

void Channel::controlTick() noexcept
{
    model->tick();
    if (calCurrent != ctl.calibrationDbu)
    {
        // Ramp the calibration (both ends together) by at most 0.25 dB per tick.
        const float step = 0.25f, diff = ctl.calibrationDbu - calCurrent;
        calCurrent = std::abs (diff) <= step ? ctl.calibrationDbu : calCurrent + (diff > 0.0f ? step : -step);
        updateScale();
    }
}

void Channel::process (float* samples, int numSamples) noexcept
{
    if (! model->isPrepared())
        return;
    const double dt = 1.0 / fs, w2 = kVuOmega * kVuOmega, damp = 2.0 * kVuZeta * kVuOmega;
    const MeterMode mode = ctl.meter;
    float grPeak = 0.0f, gr = grNow.load (std::memory_order_relaxed);
    for (int i = 0; i < numSamples; ++i)
    {
        if (countdown <= 0)
        {
            controlTick();
            countdown = CircuitModule::kControlInterval;
        }
        --countdown;

        const double out = model->processSample ((double) samples[i] * inGain);
        samples[i] = (float) (out * outGain);

        // Gain reduction from the gain cell's state (its control voltage, without the audio).
        model->trackControl();
        gr = (float) model->reductionDbAt (model->controlGateSourceVolts());
        grPeak = std::max (grPeak, gr);   // starts at 0: the bar shows no negative reduction

        // The meter movement: what drives it depends on the switch.
        double drive = 0.0;
        if (mode == MeterMode::gainReduction)
            drive = model->meterDeflection();
        else if (mode == MeterMode::plus4 || mode == MeterMode::plus8)
            drive = std::abs (out) / vuReference;
        needleSpeed += dt * (w2 * (drive - needle) - damp * needleSpeed);
        needle += dt * needleSpeed;
        if (needle > kVuPin) { needle = kVuPin; needleSpeed = std::min (needleSpeed, 0.0); }
        if (needle < 0.0)    { needle = 0.0;    needleSpeed = std::max (needleSpeed, 0.0); }
    }
    grNow.store (gr, std::memory_order_relaxed);
    blockGrDb.store (grPeak, std::memory_order_relaxed);
    needleDb.store ((float) std::clamp (20.0 * std::log10 (std::max (needle, 1e-6)), (double) Range::meterMinDb, (double) Range::meterMaxDb),
                    std::memory_order_relaxed);
}

} // namespace urei1176
