# 1167 release notes

## v0.3.0

Three selectable revisions, a detector high-pass filter and new Attack and Release defaults.

- **Rev F, G and J (new parameter `model`, host name Version, default F).**
  - **F** is the circuit of v0.2.x, bit-identical with the S/C HPF off.
  - **G** (provenance: Circuit, with Approximation where the sheets are silent): a 5532 bridging input
    with a 10 k Input pot in place of the bridged-T pad and T1, R84 (1 k) across T2, a 30.3 V supply
    and its own preamp and line-amp values. Its Input dial reads as F's (Approximation).
  - **J** (Circuit, with Approximation): the sheet titled H as revised through J. G's 5532 input,
    supply and preamp, R33 11 k and C13 100 uF in the line amp, and output transformer 163-12614 with
    no winding data (F's data with both winding resistances doubled: Approximation).
  - **Level trims** (Custom, a plug-in choice; the hardware has none): G and J are trimmed so that
    their thresholds and their gain at 24 / 24 match F's.
  - **Panels:** black face for F and G, silver face for J, with colours chosen for the plug-in.
  - **Switching** crossfades over 20 ms, the new engine starting from the old one's gain reduction.
  - Revision D is not included: there is no measured data for its output transformer and no printed
    class A bias.
- **S/C HPF (new parameter `sidechainFilter`).** OFF, 30, 60, 105, 125 or 185 Hz, first order, on the
  detector only (provenance: Custom; the 1176 has no such filter). OFF is exactly the unfiltered
  detector. This control was not in v0.2.1.
- **New defaults.** A new instance starts with Attack at 1 (the slowest) and Release at 7 (the
  fastest). Saved sessions keep their own values.
- **Help text.** The Attack and Release hover help now give the model's measured times with the
  definition used, beside the manual's figures.
- **Editor.** The option rows are now STEREO, ATTACK OFF, S/C HPF and REV; the face colours follow the
  selected revision. A new image, `docs/editor-J.png`, shows revision J.
- **Known limit.** At 4:1 with a very hot input on G and J (+22 to +26 dBu at the plug-in's input),
  the reference circuit has -53 to -61 dBc of third harmonic from Q1's gate junctions that the fast
  engine does not produce. On F the line amplifier clips first at those levels, and the fast engine
  matches the reference there.
- **CPU (Apple M3, hosted stereo VST3, 48 kHz, 512-sample buffer):** F about 118x real time, G about
  112x. J was not timed on the wall clock; it is about the same as G (same instruction count).
- **Compatibility.** Same identity (`com.c0dypeng.legacy1167`, `Cdpg` / `U167`). Two parameters are
  appended (`sidechainFilter`, `model`); existing parameter IDs, order and ranges are unchanged.
  Sessions saved by v0.2.x load as revision F with the S/C HPF off and their own Attack, Release and
  other values, and with F selected and the S/C HPF off the engine is bit-identical to v0.2.x at the same settings. Sessions saved by v0.1.0
  keep the v0.2.0 migration (Dual; Attack OFF becomes the Attack Off switch with Attack at 1).
  Latency is 0 samples.
- macOS Universal: AU and VST3, arm64 + x86_64, ad-hoc signed, not notarized.
- Windows x64: VST3, built natively with MSVC on windows-2022, unsigned. Windows CTest 4/4
  ([run 40870013432](https://github.com/chijenpeng/1167/actions/runs/40870013432)).
- macOS CTest 4/4 (`plugin1167_test`, `plugin1167_test_fast`, `plugin1167_stereo_test`,
  `plugin1167_palette_contrast`).

## v0.2.1

- **Meter readout removed.** The numeric gain-reduction readout under the VU meter (`GR ... dB`, or
  `M` and `S` in M/S) is gone. The meter, its needle, and the meter's hover help are otherwise
  unchanged. The hood under the needle stays.
- DSP, parameters, state and plug-in identity are unchanged from v0.2.0.

## v0.2.0

A faster engine, two new controls, and the gain-reduction amplifier solved as transistors.

- **Engine.** The plug-in now runs `urei1176::FastChannel`, a reduced-circuit model: the same
  component values as before, with the linear blocks solved exactly by the trapezoidal rule as small
  state-space systems, and only Q1, the gain-reduction amplifier's Q12 and Q14 (solved as
  transistors, Ebers-Moll, 2 x 2 Newton), the CR3/CR4 rectifier and the control-voltage loop solved
  per sample; the loop still closes within the sample. Overload of the preamp and the line amplifier
  is a fitted approximation of the full circuit solver, refitted in this release. The full circuit
  solver stays in the source tree as the reference and is still tested (`plugin1167_test`);
  `plugin1167_test --fast` tests the new engine.
- **Overload refit.** The fitted overload was refitted to the full solver. In a development
  comparison against the circuit engine, 29 of 29 acceptance rows now pass (28 of 29 with the
  first reduced-circuit engine; the miss was the third harmonic at ratio None with +20 and +24 dBu
  input).
- **Stereo modes (new parameter `stereo`).** Dual (two independent units, as before), Link and M/S.
  Link ties the two units' control nodes as UREI's 1176SA adapter does and solves the shared node
  jointly within each sample; both channels get the same gain reduction, set mostly by the louder
  side, and with one side driven the attack is about twice as slow. Its provenance is Derived from
  the 1176SA tie; the adapter's offset source is not modelled because its values are unverified.
  M/S runs the units on (L+R)/2 and (L-R)/2. Mode changes glide without clicks. Mono tracks ignore it.
- **Attack Off switch (new parameter `attackOff`).** Replaces the Attack knob's OFF click, so the
  knob covers 1 to 7 and keeps its setting while limiting is off. The editor gained a second option
  row (STEREO, ATTACK OFF).
- **CPU (Apple M3, one core).** Hosted as a stereo VST3 at 48 kHz with a 512-sample buffer: about 3x
  real time in v0.1.0; about 119x in Dual and in M/S and about 112x in Link in v0.2.0 (about 160x for
  the first reduced-circuit engine, before the Q12/Q14 solve, which costs about 20 %). One mode costs
  about the same as another: Link about 1.1x the cost of Dual, M/S the same.
- **Sound.** The output is not bit-identical to v0.1.0. Against the full circuit solver, program
  material (three program files, at 4:1 and All) nulls at about -42 to -65 dB, and gain,
  frequency response, gain-reduction curves and timing agree within the tolerances fixed
  beforehand (for example 0.002 dB gain, 0.010 dB static gain reduction against a 0.5 dB limit,
  0.10 dB RMS and 0.26 dB maximum gain-reduction error on program material against 0.5 and 2 dB,
  attack and release times to 0.0 % against a 10 % limit).
- **Known deviations from the full solver.**
  - A subsonic tail in quiet gaps (-60 to -70 dBFS) after loud passages in All and 20:1: the 50 ms
    envelope there is off by about 0.7 to 0.8 dB (p99). The tail is below 12 Hz.
  - Second harmonic too high at extreme gain reduction (Input full, +18 to +24 dBu, 33 to 35 dB of
    gain reduction): about -40 against -62 dBc.
  - At 5 kHz with ratio None and Output at 0.75 or more, the line amplifier clips harder than the
    full solver (second harmonic -5 against -10 dBc).
- **Compatibility.** Same identity (`com.c0dypeng.legacy1167`, `Cdpg` / `U167`). Two parameters are
  appended (`stereo`, `attackOff`); existing parameter IDs, order and ranges are unchanged. Sessions
  saved with v0.1.0 load as Dual. A v0.1.0 session with Attack at OFF loads with the Attack Off switch
  on and Attack at 1. Output is not bit-identical to v0.1.0. Latency is 0 samples.
- macOS Universal: AU and VST3, arm64 + x86_64, ad-hoc signed, not notarized.
- Windows x64: VST3, built natively with MSVC on windows-2022, unsigned. CTest 3/3 on windows-2022
  (`plugin1167_test` 151 s, `plugin1167_test_fast` 7 s, `plugin1167_stereo_test` 3 s); the VST3 is
  PE32+ x86-64 and exports `GetPluginFactory`. Requires the Microsoft Visual C++ x64
  Redistributable.
- Checked before release on macOS: CTest 3/3 (`plugin1167_test`, `plugin1167_test_fast`,
  `plugin1167_stereo_test`); snapshot tool checks passed and both editor images rendered; AU and VST3
  slices arm64 + x86_64; ad-hoc signatures, ZIP integrity and SHA-256 verified by the packaging
  script.

## v0.1.0

First release of 1167, a UREI 1176LN revision F model solved component by component with a
per-sample nonlinear circuit solver. AU and VST3 for macOS Universal, VST3 for Windows x64.
It was accurate but heavy: about 3x real time as a hosted stereo plug-in at 48 kHz on an Apple M3.
