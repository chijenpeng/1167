# UREI 1176LN, revision F: component values

The model (`Source/dsp/Urei1176.cpp`) is UREI's drawing R11914 "Solid state limiting amplifier
model 1176LN", issue "effective for S/N suffix F" (serials 2611 to 7052, drawing rev D of 1978,
"per ECO-1176-015"), component for component, with the 600 ohm source of the plug-in's input and a
10 kohm bridging load on the output. Every value below is read from that drawing unless the
"Source" column says otherwise. `‖` means in parallel. Design notes, sources and measurements:
[research notes](../research/urei-1176.md).

Sources of the values:

- **F**: UREI drawing, rev F issue (archive.org item `Urei_1176LN_owners_manual`, leaf 53; full
  resolution at
  `https://iiif.archive.org/image/iiif/2/Urei_1176LN_owners_manual%2fUrei_1176LN_owners_manual_jp2.zip%2fUrei_1176LN_owners_manual_jp2%2fUrei_1176LN_owners_manual_0053.jp2/full/full/0/default.jpg`).
- **K**: UREI drawing, rev K issue (same item, leaf 56) and gyraf.dk's redraw of it with static
  voltages; analog-recreation's verified model of it (`docs/reference/schematics.md` §14 there).
- **G**: UREI drawing, rev G issue (leaf 54), used where the rev F scan is unclear.
- **App.**: an approximation where no drawing gives the value (see the research notes).

## Signal path

| Ref | Value | Function | Source |
|---|---|---|---|
| (source) | 600 Ω | The plug-in's input: a 600 Ω generator, EMF = 2 x the calibrated level | design choice |
| TB1 ±, C | floating | Input terminals; C is the pad's and T1 primary's return (not tied to GRD) | F |
| R2, R3 | 620 Ω ¼ W each | Bridged-T series arms (input → R2 → mid → R3 → T1 pin 1), Z0 = 620 Ω | F |
| R1A | rheostat, ≈0 → 330 kΩ | Bridged-T bridging deck, input node to T1 pin 1 (front panel "Input", deck A) | F (topology); 330 k: one measured rev B unit (GroupDIY), App. |
| R1B | rheostat, 35 kΩ → ≈1 Ω | Bridged-T shunt deck, mid node to C (deck B of the same control) | F (topology); 35 k: same measurement, App. |
| Input law | loss = printed dial (dB), 0.15 dB at full CW, 54.5 dB at the CCW stop | R1A = Z0 (K − 1), R1B = Z0 / (K − 1), K = 10^(loss/20) | dial art: UA reissue; App. |
| T1 | UTC O-12 "or equivalent", pins 1-5 (500 Ω) to 6-8 (200 Ω), 1.58:1 step-down; 7 (CT) and 2-4 unused; pin 8 to ground | Input transformer | F (part, pins); UTC terminal chart (impedances) |
| T1 data | Lp 9 H, DCR 55 Ω / 14.4 Ω (91 Ω total referred to the primary), k = 0.99983 (≈3 mH leakage), 100 pF / 150 pF winding capacitance | | Sowter 1260 (O-12 replacement) for Lp and DCR; leakage and C App. |
| R5 | 270 Ω | Across T1's secondary (reflects ≈675 Ω to the pad) | F |
| R6 | 27 kΩ | Series element of the gain cell | F = K |
| Q1 | UREI 13-0027 (selected 2N5457) | Gain-cell FET. Model: Vp −1.5 V, β 1.2 mA/V² (Idss 2.7 mA), λ 0.01; gate junctions to source and drain (Is 10 fA, 1 pA leakage) | F = K; β from UREI's ratio test (analog-recreation) |
| R8, C2 | 10 kΩ, 68 µF 6.3 V | Q1 source to ground; C2 to the R13 / R14 junction (the LN connection) | F (rev K: C2 100 µ) |
| R7 ‖ C3, R9 ‖ C4 | 2.2 MΩ ‖ 200 pF each | Gate to control node (C27), gate to C5 | F = K |
| C5 | 0.22 µF | To the preamp feedback node (R16 / R21) | F = K |
| C1, R10, C6 | 1.0 µF, 1 kΩ, 27 pF | Drain to Q2 base; C6 base to ground | F = K |
| R11 | 560 kΩ | Q2 base bias from the R19 / R20 junction | F = K |
| Q2, Q3, Q4 | 2N3391A | Preamp | F = K |
| R12, R17, C7 | 1 MΩ, 6.8 kΩ, 100 µF | Q2 collector load; preamp B+ filter | F = K |
| R13, R14, R15 | 180, 180, 150 Ω | Q2 emitter | F = K |
| R16 | 100 Ω trimmer, set to 33 Ω | Distortion trim (UREI §5.3.5: minimum THD at 500 Hz, −30 dB, Input full, Output 18, Attack OFF, 20:1) | F = K; setting: analog-recreation, rechecked here (THD minimum 25-33 Ω) |
| R21 | 6.8 kΩ | Feedback node to Q4 emitter | F (rev K: 7.5 k) |
| R18, R19, R20, C8 | 22 kΩ, 82 Ω, 1.8 kΩ, 100 µF | Q3 collector and emitter | F = K |
| C9 | 1.0 µF 100 V | Q4 emitter to the Output pot | F (rev K: 47 µ) |
| R27 | 100 kΩ 2 W, audio | Output pot (front panel); model taper (20^p − 1) / 19 (17 % at 12 o'clock) | F; taper App. |
| C10 | 0.15 µF | R27 wiper to Q5 base | F (rev K: 1.0 µ) |
| R28, C11 | 10 kΩ, 47 µF 40 V | Q5 bias supply filter | F = K |
| R29, R30 | 6.8 MΩ, 1 MΩ | Q5 base bias divider | F (rev K: 1 M, 200 k) |
| Q5 | "5117" (gyraf: BC107) | Line amp input | F = K |
| R31 | 68 kΩ | Q5 collector | F = K |
| R32, R33, C13 | 1 kΩ, 4.7 kΩ, 2200 pF | Feedback return: R32 to ground, R33 + C13 a small HF shelf (+0.5 dB at 15-20 kHz) | F (rev K: 2 k, 11 k, 100 µF) |
| R34 | 3.9 kΩ | Feedback from the amp output (before C15): gain 1 + 3.9k / 1k = 4.9 (13.8 dB) | F (rev K: 4.3 k) |
| Q6, Q7 | 2N5087 | PNP Darlington driver; R35 10 kΩ, R36 100 Ω | F = K |
| C12, R37 | 12 pF, 1 kΩ | Compensation, Q5 collector to Q9 base | F/G read "_2 PF"; rev K log "REV T PER ECN 0090: C12 WAS 12PF NOW 10PF" |
| CR1, CR2 | 1N4003 | Output-stage bias | F = K |
| R38, R39, C14 | 1.2 kΩ, 1.2 kΩ, 22 µF | Bias string, bootstrapped by C14 | F = K |
| C29 | 47 pF | Q9 base to ground | F = K |
| Q8, Q9 | MPS-U05, MPS-U55 | Class AB output pair, R40 / R41 4.3 Ω | F = K |
| C15 | 470 µF 40 V | To T2's primary | F = K |
| T2 | B11148 (drawn "311148"), both primaries (RED-BRN ‖ YEL-ORG) and both secondaries (GRN-BLU ‖ VIO-GRY) in parallel; nothing across the secondary | Output transformer | F; rev G: 16-11148; rev H on: 163-12614 with R84 1 k across the secondary |
| T2 data | 1:1.732 (50 Ω : 150 Ω windings), Lp 2.5 H, DCR 4 Ω / 27 Ω, k = 0.99992 (≈0.4 mH leakage), core loss 1.5 kΩ across the primary; no winding capacitance | | ratio: Sowter 1486 ("50+50/150+150", replacement for B11148), confirmed by the gain structure; the rest App. (output impedance 39 Ω from UREI test 7) |
| (load) | 10 kΩ | Bridging load across T2's secondary; the plug-in's output | design choice |

## Side chain

| Ref | Value | Function | Source |
|---|---|---|---|
| R26, R25, R24, R22, R23 | 56 k, 68 k, 56 k, 56 k, 47 kΩ | Ratio section B ladder from R27's top to ground; taps b4 (20:1) to b1 (4:1) | F = K (gyraf reads R22 as "R2") |
| S1B-S4B | ratio buttons 4, 8, 12, 20 | Connect a tap to the ratio bus; All: every button; None: no button | F = K |
| "UA4706 T+C *" | factory-set | A box at S4B; not identified, not modelled | F = K |
| S9 | changeover on R77 ("G.R. disable switch located on R77") | Limiting ON: ratio bus → node 22; OFF (Attack fully CCW, click): node 22 to ground, the bus cut | F = K; function: UREI manual §1.4, factory test 10 |
| C17 | 1.0 µF 100 V | Node 22 to Q12 base | F = K |
| Q12-Q15 | 2N3707 (hFE ≥ 250) | GR control amp | F = K |
| R60, R61, R62 | 1 MΩ, 470 kΩ, 47 kΩ | Q12 bias and collector | F = K |
| R63, R64, R65 | 4.7 kΩ, 2.4 kΩ, 270 Ω | Q12 / Q13 emitters | F = K |
| R66, R67, R69, R71 | 182 k, 38.3 k, 44.2 k, 7.68 kΩ (1 %) | Q14 stage | F = K |
| R68, R72, R73 | 47 kΩ, 2.4 kΩ, 180 Ω | Overall feedback; Q15 emitter | F = K |
| R70, C18 | 4.7 kΩ, 47 µF 40 V | GR amp B+ filter | F = K |
| C19, C20 | 6.8 µF 35 V | Q15 / Q13 emitters to the rectifier | F = K |
| CR3, CR4 | FD333 | Full-wave rectifier; model: 1N4148 forward fit, 0.1 nA leakage | F = K |
| R74, R75, C21 | 47 kΩ, 47 kΩ, 100 µF 25 V | Anode returns to the threshold reference (node 21) | F = K |
| R76 ‖ C22 | 470 Ω ‖ 0.022 µF | Rectifier to the Attack pot | F = K |
| R77 | 25 kΩ ½ W, linear | Attack ("1" slowest CCW to "7" fastest CW), carries S9 | F = K |
| R78, R79 | 5 MΩ ½ W linear, 270 kΩ | Release ("1" slowest to "7" fastest), R79 from the wiper to the bias end | F = K |
| C27 | 0.22 µF | Timing capacitor (control node); the "stereo interconnect" J1 also lands here | F = K |
| R82, R83 | 10 kΩ, 3.9 kΩ | −10 V divider | F = K |
| R81 | 2 kΩ trimmer, "Q bias" | Bias for Q1; set as UREI test 10 (1.0 dB drop with limiting off): wiper 742 Ω from ground (0.371 of the track) in the model, Q1 gate −1.52 V | F = K; setting computed |
| R87, R42, R43, R44, R45 | 150, 470, 560, 1.5 k, 2.2 kΩ | Ratio section A chain, bias to −10 V; taps a1-a4 | F (F-75 issue: R45 1.5 k) |
| S1A-S4A | ratio buttons | Connect a tap to the rectifier reference | F = K |
| R46 | 10 MΩ | a1 to the reference (no "pop" between buttons) | F = K |

## Meter (approximated in the model, values for reference)

| Ref | Value | Function | Source |
|---|---|---|---|
| R48 | 3.9 MΩ | Control node to Q10's gate | F = K |
| Q10 | UREI 13-0027, selected with Q1 | Meter FET, source follower | F = K |
| R52 | 10 kΩ | Q10 source to −10 V | F = K |
| R53, R54 | 15 kΩ, 100 kΩ trimmer "tracking adj." | Q10 source to IC1 pin 2 | F = K |
| R55, R56 | 2 kΩ trimmer "zero adj." (front panel), 10 kΩ | IC1 pin 3 reference | F = K |
| R57, R58 | 470 kΩ, 100 kΩ | IC1 pin 2 to −10 V, feedback after CR11 | F = K |
| IC1, C16 | "846" (F; LM301 on F-75, TL071 on K), 33 pF | Meter amp | F, K |
| R49, C28 | 5.6 kΩ (K: 4.3 k), 0.1 µF | IC1 / Q10 supply from +30 V | F, K |
| CR11, R59 | diode, 3.9 kΩ (K: 3.6 k) | IC1 output to the meter (GR) | F, K |
| S5-S7 | GR, +8, +4 (S8 "A.C. on") | Meter function | F = K |
| R51 | 3.6 kΩ | +4: VU meter across T2's secondary (0 VU = 1.228 V, +4 dBu) | F = K |
| R50 | 8.2 kΩ | +8: (3.9 k + 8.2 k) / (3.9 k + 3.6 k): 0 VU = +8.16 dBu | F = K |
| M | VU movement (3.9 kΩ with its rectifier) | 99 % in 300 ms, 1-1.5 % overshoot | standard |

Model of the meter: the GR needle drive is linear in the control-node voltage (Q10 a source
follower, IC1 linear), zeroed at idle with limiting off (R55) and scaled for −10 VU at 10 dB of
settled reduction (R54), as UREI's test 11 calibrates it; CR11 passes no reverse current. +4 / +8:
the full-wave average of the load voltage against the resistor law above. One second-order VU
movement (ζ 0.8127, ωn 13.512 rad/s) carries the needle in every mode.

## Supplies

| Ref | Value | Source |
|---|---|---|
| +30 V | CR5 30 V 10 W shunt zener (1N2989A) fed through R84 75 Ω 3 W; modelled as an ideal 30.0 V | F (rev G-K: 7824 lifted to 30.3 V) |
| −10 V | CR6 1N4740 (10 V) through R85 / R86 1 kΩ; modelled as ideal −10 V | F = K |
| Preamp B+ | R17 6.8 kΩ, C7 100 µF | F = K |
| GR amp B+ | R70 4.7 kΩ, C18 47 µF | F = K |

## Values that differ between rev F and rev K (from T1's secondary to C15)

| Ref | Rev F (modelled) | Rev K | Effect |
|---|---|---|---|
| +30 V | 30.0 V (shunt zener) | 30.3 V (7824) | all bias points ≈1 % lower |
| C2 | 68 µF | 100 µF | none in band |
| R21 | 6.8 kΩ | 7.5 kΩ | preamp gain 13.5 vs 14.8 (−0.8 dB); ratio test unchanged when settled |
| C9 | 1.0 µF | 47 µF | 1.6 Hz high-pass into R27 |
| C10 | 0.15 µF | 1.0 µF | 1.2 Hz high-pass into Q5 |
| R29, R30 | 6.8 MΩ, 1 MΩ | 1 MΩ, 200 kΩ | Q5 bias; line output at 15.5 V (rev K 13.4 V) |
| R32, R33, C13 | 1 kΩ, 4.7 kΩ, 2200 pF | 2 kΩ, 11 kΩ, 100 µF | line-amp gain 13.8 dB (rev K 11.0 dB), small HF shelf |
| R34 | 3.9 kΩ | 4.3 kΩ | as above |
| C12 | 12 pF | 10 pF | compensation |
| T2 | B11148, 1:1.73, no load resistor | 163-12614, R84 1 k across the secondary | output level and impedance |
| Input | bridged-T + T1 + R5 270 Ω | IC2 5532 differential receiver, C33, R5 10 k Input pot | the rev F input is the subject of this model |

The F-75 issue of the same drawing (waltzingbear.com) also differs in C2 (50 µF), C7 / C8 / C21
(80 µF), C11 / C18 (50 µF), C14 (25 µF), C15 (400 µF), R45 (1.5 kΩ) and IC1 (LM301). The model
follows the later F-78 issue.

## DC operating point

The only drawing with marked voltages is rev K (gyraf.dk). The test builds the netlist with the rev
K values (`Circuit1176::useRevisionKValues`) and compares; the rev F build is checked where the
two share parts (drawing voltages scaled by 30.0 / 30.3) and for a sane line-amp bias. Values from
`urei1176_test dc`:

| Node | Rev K drawing | Model, rev K values | Model, rev F (modelled) |
|---|---|---|---|
| Preamp B+ (C7) | 25.8 | 25.89 | 25.52 |
| Q2 base / collector / emitter | 1.02 / 1.76 / 0.51 | 1.08 / 1.81 / 0.52 | 1.10 / 1.83 / 0.54 |
| Q3 collector (Q4 base) | 11.98 | 12.15 | 11.56 |
| Q4 emitter | 11.5 | 11.49 | 10.89 |
| GR amp B+ (C18) | 27.8 | 27.90 | 27.63 |
| Q12 base / collector / emitter | 4.55 / 15.1 / 4.03 | 4.61 / 15.22 / 3.99 | 4.57 / 15.09 / 3.95 |
| Q13 emitter | 14.54 | 14.52 | 14.39 |
| Q14 base / collector | 3.51 / 17.25 | 3.50 / 17.31 | 3.47 / 17.16 |
| Q15 emitter | 16.72 | 16.60 | 16.45 |
| Q5 base / emitter / collector | 4.98 / 4.54 / 28.6 | 4.99 / 4.43 / 28.44 | 3.76 / 3.19 / 28.06 |
| Q6 / Q7 emitters | 29.1 / 29.7 | 29.05 / 29.76 | 28.68 / 29.39 |
| Q8 / Q9 bases | 14.06 / 12.77 | 14.50 / 13.18 | 16.21 / 14.88 |
| Q8 / Q9 emitters, output | 13.47 / 13.35, 13.40 | 13.91 / 13.77, 13.84 | 15.61 / 15.46, 15.53 |
| Q1 gate (bias, not marked) | adjustable 0 to −2.9 | −1.52 | −1.52 (4:1-20:1, None), −2.44 (All) |

The rev K line amp's marked voltages disagree with each other by about 0.4 V (through R32 and
R34 they put 0.2 mA through Q5, whose collector resistor carries 25 µA), so that residual is not
taken as a model error.
