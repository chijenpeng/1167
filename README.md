# 1167

Inspired by the UREI 1176LN, revision F.<br>
Part of the codycodie legacy release collection. A reduced-circuit model of the original component
network: about 120x real time as a hosted stereo plug-in at 48 kHz on an Apple M3, with Dual, Link and
Mid/Side stereo modes.

1167 is a schematic-derived AU and VST3 compressor model. Its signal path, side chain, gain-reduction FET, discrete line amplifier, transformers, meter behaviour, and front-panel control laws follow the documented revision F circuit rather than a conventional compressor transfer curve.

Since v0.2.0 the plug-in runs a reduced-circuit engine, `urei1176::FastChannel`. It uses the same component values as the full circuit model. The linear blocks (input network, gain-reduction network, ratio ladder, side-chain amplifier, line amplifier and transformers) are solved exactly by the trapezoidal rule as small state-space systems, so they keep the bilinear response of the full solver at every sample rate. The gain-reduction FET (Q1), the gain-reduction amplifier's input and second-stage transistors (Q12 and Q14, solved as transistors), the CR3/CR4 rectifier and the control-voltage loop are solved per sample, and the loop still closes within the sample. The overload behaviour of the preamp and the line amplifier is a fitted approximation of the full circuit solver, not a circuit solution. The engine is about 35 times cheaper than the full solver in stereo: the hosted stereo VST3 runs at about 120x real time at 48 kHz with a 512-sample buffer on an Apple M3 (Dual and M/S about 119x, Link about 112x; v0.1.0: about 3x). The full solver, `urei1176::Channel`, stays in the source tree as the reference the fast engine is fitted and tested against.

![1167 editor](docs/editor.png)

1167 is an independent product identity. It keeps the original parameter IDs, order, ranges and defaults, and appends two parameters (`stereo`, `attackOff`), so existing 1176 automation and state data remain compatible. Its bundle identifier is `com.c0dypeng.legacy1167`.

## Downloads

Get the latest packages from the [1167 releases page](https://github.com/chijenpeng/1167/releases/latest). The macOS release contains Universal AU and VST3 bundles for Apple Silicon and Intel. The Windows release contains a native x64 VST3. macOS bundles are ad-hoc signed and are not notarized; the Windows plug-in is unsigned.

1167 has its own plug-in identity, so existing DAW instances continue to refer to the original 1176 plug-in and must be replaced manually. The original parameter IDs, order, ranges and defaults are preserved, and state saved without the two appended parameters loads as Dual with the Attack Off switch set from the old Attack value, for compatible host state and preset transfer.

## Controls

- **Input** and **Output** use the hardware's attenuation markings. Both at 24 are approximately unity gain below the limiting threshold.
- **Attack** and **Release** run from 1 to 7, clockwise faster. The **Attack Off** switch (below the
  Meter row) switches limiting off while leaving the audio path active: it is the hardware's S9 (the
  Attack knob's OFF click), kept as a switch so the knob keeps its setting; the knob is dimmed while
  it is on. Sessions saved by v0.1.0 with Attack in its OFF stretch load with the switch on and
  Attack at 1.
- **Ratio** provides 4:1, 8:1, 12:1, 20:1, all-buttons-in, and no button selected.
- **Meter** switches between off, gain reduction, +4 VU, and +8 VU.
- **0 dBFS** sets the analogue calibration from 10 to 26 dBu.
- **Stereo** (Dual, Link, M/S; mono tracks ignore it):
  - **Dual**: each channel has its own complete engine and side chain.
  - **Link** (provenance: Derived): the 1176SA tie of the two units' control nodes. Both rectifiers
    charge one shared control node (both C27s, release and bias returns in parallel, each gain
    cell drawing from it), solved jointly inside every sample; both channels get the same gain
    reduction, driven by whichever side is louder. With one side driven the attack is about twice as
    slow (the node has twice the capacitance) and the release is unchanged. The real adapter's
    floating offset source, which nulls two units' idle difference, is not modelled: its values are
    unverified, and two identical units have none to null. With identical L and R, Link is Dual.
  - **M/S**: the two units see M = (L + R) / 2 and S = (L - R) / 2 and the output is L = M + S,
    R = M - S. A mono-compatible source sounds as in Dual.

  Switching mode while playing does not click: Link glides the two units' control voltages together
  over about 15 ms, M/S morphs its matrix in or out over 10 ms.

## Build

The project requires CMake 3.22 or later and a C++17 compiler. For AU and VST3 builds, use macOS 11 or later; the Windows VST3 release uses native x64 MSVC from Visual Studio 2022. It uses JUCE 8.0.9. Put a local checkout at `third_party/JUCE` to avoid a fetch; otherwise CMake downloads that tagged release while configuring.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

On Windows, configure with `cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCOPY_PLUGIN_AFTER_BUILD=OFF`, then build with `cmake --build build --config Release --parallel 2` and test with `ctest --test-dir build -C Release --output-on-failure`.

Plug-in copying is disabled by default, so builds stay in the build tree and do not touch installed plug-ins. To copy AU and VST3 bundles into the current user's Audio Plug-Ins folders, explicitly configure with `-DCOPY_PLUGIN_AFTER_BUILD=ON`. Build products remain under `build/Plugin1167_artefacts/Release`.

## Validation and snapshots

`plugin1167_test` checks operating points, factory ratio and bias tests, attack and release, distortion, frequency response, meter ballistics, sample-rate independence, real-time allocation safety, and processing speed on the full circuit engine. `plugin1167_test --fast` runs the same checks on the fast engine that the plug-in uses (the DC-mode section excepted); CTest runs both and the stereo test. `plugin1167_eval` measures a rendered audio file, and `plugin1167_eval --fast` does so through the fast engine. `plugin1167_stereo_test` checks the stereo modes (Link against Dual, one-sided drive, attack and release times, +24 dBu robustness, M/S, mode switching, allocation); `plugin1167_stereo_test perf` adds the speed comparison of the modes, which CTest leaves out because timings vary with machine load.

During development the fast engine was compared with the full circuit engine on 29 fixed acceptance rows (gain, response, static curves, timing, program material, harmonics, hot input, meters, sample rates); all 29 pass after the gain-reduction amplifier's transistors were solved and the overload fit was redone. Program material nulls against the full solver at about -42 to -66 dB. Measured results and CPU figures are in the research notes below. The comparison tool is not part of this repository.

The snapshot utility additionally checks plug-in parameter laws, the Attack Off switch, old-state migration and state restoration, then renders the two maintained editor images (a second directory argument also renders the editor in Link and M/S there):

```sh
cmake --build build --target plugin1167_snapshot
./build/plugin1167_snapshot_artefacts/Release/plugin1167_snapshot docs
```

The hardware circuit sources and modelling boundaries are documented in [`docs/reference/schematic.md`](docs/reference/schematic.md). Research sources, measured results, and known limitations are in [`docs/research/urei-1176.md`](docs/research/urei-1176.md).
