#pragma once

// The 1176LN revisions the plug-in can switch between, and the values that differ from one to the next.
// See docs/research/urei-1176-versions-plan.md (sections 1.1 and 1.2).
//
// F, G and J are real. D is a placeholder copy of F (placeholderOfF = true), deferred (docs/research/rev-d-data-search.md) and not exposed
// by the plug-in. A VersionSpec is plain data and every field is constexpr; the engine reads it in prepare() and in the build functions,
// never in the sample loop.
//
// Provenance of every field (AGENTS.md: Circuit, Trace, Derived, Approximation, Capture or Custom) and its source is in the comment
// beside it. "schematic.md" is docs/reference/schematic.md (rev F component values), "reference" is the circuit engine
// (Urei1176.cpp, urei1176::Channel).

namespace urei1176
{
/** The engine's revisions, in chronological order: the index of kVersionSpecs (specFor). Not the plug-in's `model` parameter order,
    which kModelChoices (PluginProcessor.h) maps to these. */
enum class Version { D = 0, F = 1, G = 2, J = 3 };
inline constexpr int kNumVersions = 4;

/** The fitted overload constants (docs/research/urei-1176.md, "The overload fit"), in the units of Fast1176.cpp:
    the preamp output stage's rails, knees and operating-point slide, then the line amp's.
    Provenance: Approximation, fitted to the reference circuit engine (not to hardware). */
struct OverloadParams
{
    double o1Lo, o1Hi, o1KneeHi, o1KneeLo, o1ShiftPerVolt, o1ShiftStart, o1EnvSeconds;
    double lineLo, lineHi, lineKneeHi, lineKneeLo, lineShiftPerVolt, lineShiftStart, lineEnvSeconds;
    static constexpr int kCount = 14;
    double* data() noexcept { return &o1Lo; }
    const double* data() const noexcept { return &o1Lo; }
};
static_assert (sizeof (OverloadParams) == OverloadParams::kCount * sizeof (double), "data() walks the fields as one array");

/** The bridging input of rev G (and J): IC2, a 5532 balanced receiver, with C30 / C31 / C33 / C32 and the 10 k resistors, then the Input pot R5.
    It replaces F's bridged-T pad and T1. All zero in a spec without it (inputBridging = false). Sources: docs/reference/schematic-revisions.md
    2.1 (leaf 54) and schematic.md "Revision G in the reference engine"; the op-amp is the reference engine's macro-model. */
struct BridgingInput
{
    double r1Ohms;                       // R1-A to R1-E, five 10 k: Circuit, Legible
    double r2Ohms;                       // R2 "CMR ADJ.", 500 ohm, wiper centred: Approximation (a factory null that moves only the CMR)
    double c30Farads, c31Farads, c33Farads;   // C30, C31, C33: Circuit, 10 uF (the polarity marks of C30 / C31 are ambiguous, not modelled)
    double c32Farads;                    // C32: Circuit, 100 pF (G); 220 pF (the sheet titled H, J)
    double rfFarads;                     // the RF capacitor from INPUT + to ground: Circuit, 100 pF
    double potOhms;                      // R5, the Input pot: Circuit, 10 k. Its law follows F's printed dial (Circuit1176::inputPotFraction): Approximation
    double opAmpGain, opAmpGbwHz, opAmpDiffOhms, opAmpOutOhms, opAmpPoleOhms;   // the 5532 macro-model: Approximation, NE5532 data sheet typicals
    /** IC2's output clamps, as the reference engine's: two diodes (a forward spec Is, n = 1) from each op-amp output to a node `headroom + on` volts
        inside each supply rail. The outputs are then stopped about `headroom` (2 V) from the rails: +-13 V around the 15.15 V bias. Approximation
        (the 5532's output swing, a data-sheet typical); the diode drops (`on`: the drop at about 1 mA) set the knee. */
    double clampHeadroomVolts, clampOnVolts, clampIs;
};

struct VersionSpec
{
    Version version;
    char name;                  // 'D', 'F', 'G' or 'J'
    bool placeholderOfF;        // true until the version has its own values (D, deferred)

    // --- supply -------------------------------------------------------------------------------------------------------------
    /** The +30 V rail (F: a 30 V shunt zener; G and J: a 7824 lifted to 30.3 V). Circuit, schematic.md "Supplies". In the fast engine the
        operating-point currents below carry its effect, except for IC2's output clamps (G and J), which buildReceiver places from it. */
    double supplyVolts;

    // --- input (buildInput) --------------------------------------------------------------------------------------------------
    /** false: F's bridged-T pad, T1 and R5 270 (D too). true: the bridging receiver below (input impedance 10 k from the receiver's R1-D and
        R1-E, the 600 ohm source barely loaded). */
    bool inputBridging;
    BridgingInput bridging;

    // --- preamp cell (buildCell) ---------------------------------------------------------------------------------------------
    double c2Farads;            // Circuit, schematic.md "C2" (F, G, J 68 uF; D 50 uF)
    double r21Ohms;             // Circuit, "R21" (F 6.8 k)
    double r16Ohms;             // "R16", the distortion trimmer, factory set per unit for minimum THD (manual 5.3.5): F 33 ohm (Circuit); G and J
                                // 30 ohm (Derived: the null of the factory procedure on the reference model, urei1176_test --version G / J r16)
    double r20Ohms, c8Farads;   // Circuit, "R20", "C8" (F 1.8 k, 100 uF; J: R20 is 1.3 k or 1.8 k on the sheet, 1.8 k used)
    /** DC emitter currents of Q2, Q3, Q4 (amps). Derived: the reference circuit's DC operating point, regenerated per version with
        urei1176_opdump. */
    double ieQ2, ieQ3, ieQ4;

    // --- C9 into the Output pot (buildOutBlock) ------------------------------------------------------------------------------
    double c9Farads;            // Circuit, "C9" (F 1 uF; G, J 47 uF)
    double outputPotOhms;       // Circuit, "R27" (F 100 k)

    // --- GR amp (buildGr) ----------------------------------------------------------------------------------------------------
    /** F's R66, R67, R69 and R71 (D draws them as R42, R43, R45 and R48 with other values). Circuit, schematic.md "Side chain". */
    double grR66Ohms, grR67Ohms, grR69Ohms, grR71Ohms;
    /** DC emitter currents of Q12 to Q15 (amps). Derived, as ieQ2. The GR amp's IF0 and IF1 follow from ieQ12 and ieQ14. */
    double ieQ12, ieQ13, ieQ14, ieQ15;

    // --- line amp Q5 to Q9 (buildLineA) --------------------------------------------------------------------------------------
    double c10Farads;           // Circuit, "C10" (F 0.15 uF)
    double r29Ohms, r30Ohms;    // Circuit, "R29", "R30" (F 6.8 M, 1 M: Q5's base bias divider)
    double r31Ohms;             // Circuit, "R31" (F 68 k)
    double r32Ohms;             // Circuit, "R32" (F 1 k)
    double r33Ohms, c13Farads;  // Circuit, "R33", "C13" (F 4.7 k, 2.2 nF; G 10 k, 0.001 uF; J 11 k, 100 uF: an AC ground, R33 Ambiguous)
    double r34Ohms;             // Circuit, "R34" (F 3.9 k)
    double c12Farads;           // Circuit, "C12" (F 12 pF; the F scan lacks the leading digit, see schematic.md)
    /** DC collector currents of Q5 to Q9 and the bias diodes' current (amps). Derived, as ieQ2. */
    double icQ5, icQ6, icQ7, icQ8, icQ9, idBias;

    // --- T2 and C15 (buildLineB) ---------------------------------------------------------------------------------------------
    /** C15 (F 470 uF). Circuit. T2's winding data, and how each version straps it (F: both primaries and both secondaries paralleled;
        G: one primary; J: one primary and one secondary; Approximation, no winding data exists), live in the reference engine's
        Circuit1176::outputTransformer (Version), which buildLineB reads. */
    double c15Farads;
    /** R84, a resistor across T2's secondary, 0 = none (F: none; G and J: 1 k, Circuit, Legible). */
    double r84Ohms;

    // --- meter (updateScale) -------------------------------------------------------------------------------------------------
    /** The +8 mode's reference: the level that reads 0 VU with the +8 meter resistors in is 4 dBu raised by
        20 log10 ((R59 + plus8Series) / (R59 + plus8Reference)). Circuit, schematic.md "Meter" (F: R59 3.9 k; G, J and K: 3.6 k). */
    double meterR59Ohms, meterPlus8SeriesOhms, meterPlus8ReferenceOhms;

    // --- level trims (updateScale, in the plug-in's dBFS-to-circuit calibration; both engines) ---------------------------------
    /** Custom (a plug-in choice, not circuit; the hardware has none). Rev G's front end is 9.5 dB hotter than F's and its line amp gives back
        3.85 dB, so at the same dials its thresholds are 10.3 dB lower and its overall gain 5.4 dB higher (Phase 3b-1, urei1176_levels).
        inputTrimDb is applied ahead of the circuit and makes G's thresholds equal F's across the dial; outputTrimDb is applied after it and
        brings G's overall gain at the printed 24 / 24 back to F's. The VU meter in +4 / +8 reads the trimmed level, so the same plug-in
        output reads the same on every version. 0 on F. FastChannel::setVersionTrims / Channel::setVersionTrims (false) takes both off, for
        the tests that measure the circuit itself. */
    double inputTrimDb, outputTrimDb;

    // --- overload ---------------------------------------------------------------------------------------------------------
    OverloadParams overload;
};

namespace detail
{
/** Rev F, the version the plug-in shipped first. Every value is the literal the engine used before the specs existed; the parts F does not
    have (the bridging input, R84, the level trims) are zero. */
inline constexpr VersionSpec kSpecF {
    Version::F, 'F', false,
    30.0,                                                              // supply: Circuit
    false, {},                                                         // input: the bridged-T pad and T1 (no bridging receiver)
    68e-6, 6.8e3, 33.0, 1.8e3, 100e-6,                                 // C2, R21, R16, R20, C8: Circuit
    2.166262e-05, 6.319734e-04, 1.482238e-03,                          // Q2..Q4: Derived (reference DC solve, ratio 4:1 idle)
    1.0e-6, 100e3,                                                     // C9, R27: Circuit
    182e3, 38.3e3, 44.2e3, 7.68e3,                                     // R66, R67, R69, R71: Circuit
    2.392212e-04, 5.398527e-03, 2.156952e-04, 6.624131e-03,            // Q12..Q15: Derived
    0.15e-6, 6.8e6, 1e6, 68e3, 1e3, 4.7e3, 2.2e-9, 3.9e3, 12e-12,     // C10, R29, R30, R31, R32, R33, C13, R34, C12: Circuit
    2.913120e-05, 1.558209e-04, 6.073487e-03, 1.866136e-02, 1.552901e-02, 6.042694e-03,   // Q5..Q9, bias diodes: Derived
    470e-6,                                                            // C15: Circuit
    0.0,                                                               // no R84
    3.9e3, 8.2e3, 3.6e3,                                               // R59, +8 series, +8 reference: Circuit
    0.0, 0.0,                                                          // no level trims
    // overload: Approximation, fitted to the reference (docs/research/urei-1176.md, "The overload fit")
    { -9.889, 12.454, 0.05, 0.05, 0.923, 12.249, 0.285,
      -15.526, 12.557, 0.627, 1.195, 0.425, 13.78, 0.0548 }
};

/** Rev G (UREI leaf 54, schematic-revisions.md 2.1; the reference engine's netlist, Urei1176.cpp kRevG). The operating-point currents are Derived
    from the reference (urei1176_opdump --version G, ratio 4:1 idle). The overload constants start from F's and are refitted for the circuitry
    that differs (docs/research/urei-1176-versions-plan.md, "Phase 3b-2 results"). */
inline constexpr VersionSpec kSpecG {
    Version::G, 'G', false,
    30.3,                                                              // supply: a 7824 lifted to 30.3 V: Circuit
    true, { 10e3, 500.0, 10e-6, 10e-6, 10e-6, 100e-12, 100e-12, 10e3,  // R1-A..E, R2, C30, C31, C33, C32, RF cap, R5: see BridgingInput
            1.0e5, 10.0e6, 300e3, 10.0, 1e3,                           // 5532 macro-model: Approximation
            2.0, 0.54, 1e-12 },                                         // IC2 output clamps: Approximation (NE5532 typical swing)
    68e-6, 7.5e3, 30.0, 1.8e3, 100e-6,                                 // C2, R21, R16, R20, C8: Circuit (R16 Derived)
    2.208435e-05, 6.218485e-04, 1.427628e-03,                          // Q2..Q4: Derived
    47e-6, 100e3,                                                      // C9, R27: Circuit
    182e3, 38.3e3, 44.2e3, 7.68e3,                                     // R66, R67, R69, R71: Circuit (as F)
    2.420021e-04, 5.447296e-03, 2.183189e-04, 6.683615e-03,            // Q12..Q15: Derived
    1.0e-6, 1e6, 200e3, 68e3, 2e3, 10e3, 0.001e-6, 4.3e3, 12e-12,     // C10, R29, R30, R31, R32, R33, C13, R34, C12: Circuit
    2.797576e-05, 1.456469e-04, 5.368659e-03, 1.709718e-02, 1.493174e-02, 5.343334e-03,   // Q5..Q9, bias diodes: Derived
    470e-6,                                                            // C15: Circuit
    1e3,                                                               // R84 1 k (Circuit)
    3.6e3, 8.2e3, 3.6e3,                                               // R59 (G: 3.6 k), +8 series, +8 reference: Circuit
    -10.2, 4.97,                                                       // input and output trims: Custom (4.97: urei1176_levels --makeup)
    // overload: Approximation, fitted to the reference with urei1176_fit --version G from F's constants: the eight rails, slides and limits free,
    // then the line amp's four again with the hot-input level limit at 0.8 (data/fit/G-subset-D; the knees and followers stay F's). Objective data cost
    // 21362 (F's constants: 23927), hot-input level worst 0.41 dB. The three 4:1 hot-input H3 points at circuit +12 / +13.8 / +16 dBu are not
    // reachable by any of these constants (Q1's gate junction, plan "Phase 3b-2 results").
    { -9.4651, 11.8511, 0.05, 0.05, 0.8854, 13.4121, 0.285,
      -15.3205, 12.9601, 0.627, 1.195, 0.2937, 14.4010, 0.0548 }
};

/** Rev J: the sheet titled H as revised through J (leaf 54's neighbour, schematic-revisions.md 3.1 and 3.2). G's circuit except C32 220 pF,
    R33 11 k with C13 100 uF (the shelf becomes an AC ground) and T2 163-12614 (one primary and one secondary). R33 (11 k, 1.1 k or 10 k) and
    R20 (1.3 k or 1.8 k) are Ambiguous: 11 k and 1.8 k are used (the reference sweep, schematic-revisions.md 3.4: R33 1.1 k would add 6 dB of
    gain against the manual's 45 +-1 dB maximum; R33 10 k and R20 1.3 k move gain and threshold by 0.1 dB or less). The operating-point
    currents are Derived from the reference (urei1176_opdump --version J) and equal G's: none of J's differences enters the DC solution. The
    meter amp (LF356H), R49 and the lamps have no audio effect; R50 is kept at 8.2 k (the other reading is 3.2 k). Trims (Custom) and
    overload constants (fitted): plan, "Phase 4 results". */
inline constexpr VersionSpec kSpecJ {
    Version::J, 'J', false,
    30.3,                                                              // supply: a 7824 lifted to 30.3 V: Circuit
    true, { 10e3, 500.0, 10e-6, 10e-6, 10e-6, 220e-12, 100e-12, 10e3,  // R1-A..E, R2, C30, C31, C33, C32 (220 pF), RF cap, R5: see BridgingInput
            1.0e5, 10.0e6, 300e3, 10.0, 1e3,                           // 5532 macro-model: Approximation
            2.0, 0.54, 1e-12 },                                         // IC2 output clamps: Approximation (NE5532 typical swing)
    68e-6, 7.5e3, 30.0, 1.8e3, 100e-6,                                 // C2, R21, R16, R20, C8: Circuit (R16 Derived, R20 Ambiguous)
    2.208435e-05, 6.218485e-04, 1.427628e-03,                          // Q2..Q4: Derived
    47e-6, 100e3,                                                      // C9, R27: Circuit
    182e3, 38.3e3, 44.2e3, 7.68e3,                                     // R66, R67, R69, R71: Circuit (as F)
    2.420021e-04, 5.447296e-03, 2.183189e-04, 6.683615e-03,            // Q12..Q15: Derived
    1.0e-6, 1e6, 200e3, 68e3, 2e3, 11e3, 100e-6, 4.3e3, 12e-12,       // C10, R29, R30, R31, R32, R33, C13, R34, C12: Circuit (R33 Ambiguous)
    2.797576e-05, 1.456469e-04, 5.368659e-03, 1.709718e-02, 1.493174e-02, 5.343334e-03,   // Q5..Q9, bias diodes: Derived
    470e-6,                                                            // C15: Circuit
    1e3,                                                               // R84 1 k (Circuit)
    3.6e3, 8.2e3, 3.6e3,                                               // R59 (3.6 k), +8 series (R50 8.2 k, Ambiguous), +8 reference: Circuit
    -10.2, 4.21,                                                       // input and output trims: Custom (4.21: urei1176_levels --version J --makeup)
    // overload: Approximation, fitted to the reference with urei1176_fit --version J from G's constants: the line amp's four rails and slide constants free
    // (J's line amp has R33 / C13 as an AC ground, gain 3.54 against G's 3.15), the preamp's, knees and followers held at G's (data/fit/J-subset-A). Objective
    // data cost 22445 (G's constants: 23067), hot-input level worst 0.40 dB. The 4:1 hot-input H3 points are not reachable by any constant (Q1's gate
    // junction, plan "Phase 3b-2 results"; confirmed for J in "Phase 4 results").
    { -9.4651, 11.8511, 0.05, 0.05, 0.8854, 13.4121, 0.285,
      -15.8312, 12.4179, 0.627, 1.195, 0.3156, 15.7370, 0.0548 }
};

/** A placeholder for a version that has no values of its own yet: F's, labelled. D's only (deferred). */
constexpr VersionSpec placeholderOfF (Version v, char name) noexcept
{
    VersionSpec s = kSpecF;
    s.version = v;
    s.name = name;
    s.placeholderOfF = true;
    return s;
}
} // namespace detail

/** One entry per Version, in its order. D is a placeholder (F's values) and is not exposed: deferred, see docs/research/rev-d-data-search.md. */
inline constexpr VersionSpec kVersionSpecs[kNumVersions] = {
    detail::placeholderOfF (Version::D, 'D'),
    detail::kSpecF,
    detail::kSpecG,
    detail::kSpecJ,
};

constexpr const VersionSpec& specFor (Version v) noexcept { return kVersionSpecs[(int) v]; }
} // namespace urei1176
