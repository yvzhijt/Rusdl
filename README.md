# Rusdl —— 简略电路仿真软件

> **Rusdl** 是参考开源项目 [circuitjs1](https://github.com/pfalstad/circuitjs1)（Paul Falstad 的
> Java/GWT 电路仿真器）自定义实现的一个简化版电路仿真软件。仅保留了基础元件（电阻、电容、电感、
> 二极管、三极管、运放等）的直流/交流仿真能力，定位为中学物理电路实验教学工具。
>
> 与原始的 circuitjs1 相比，Rusdl 做了大量精简：去掉了逻辑门、数字芯片、子电路、
> 示波器高级功能等，专注于模拟电路的基础教学场景。

`circuit-sim` is a headless circuit solver —— the core engine of Rusdl. Its intended first use is ordinary school
physics: DC and AC supplies, wires, ground, resistors, inductors, capacitors, diodes and
transistors for basic charging, rectifying and biasing experiments. It does not need a full
device model library for that scope.

The solver internals remain available for circuitjs1 netlist compatibility, but a simple
circuit can be assembled directly without building a UI or a larger component framework:

```rust
use circuit_sim::{Ammeter, Circuit, Ground, Point, PowerSupply, Resistor, Voltmeter};

let (positive, after_meter, ground) = (
    Point::new(0, 0),
    Point::new(0, 40),
    Point::new(0, 80),
);
let mut circuit = Circuit::new();
circuit.add(Box::new(PowerSupply::dc_between(ground, positive, 6.0)));
circuit.add(Box::new(Ammeter::between(positive, after_meter)));
circuit.add(Box::new(Resistor::between(after_meter, ground, 2_000.0)));
circuit.add(Box::new(Ground::at_point(ground)));
circuit.analyze()?;
circuit.step()?;

let current = circuit.elements()[1]
    .as_any()
    .downcast_ref::<Ammeter>()
    .expect("ammeter")
    .amperes();
let voltage = Voltmeter::between(after_meter, ground).read(&circuit)?;
assert!((current - 0.003).abs() < 1e-9);
assert!((voltage - 6.0).abs() < 1e-9);
# Ok::<(), Box<dyn std::error::Error>>(())
```

An `Ammeter` is an ideal zero-resistance series element. A `Voltmeter` is an ideal
non-loading measurement. There is no oscilloscope in the engine: a caller that wants a trace
reads the meters in a loop, which is what the editor does, so the sampling policy stays with
the caller instead of being fixed in the engine. The editor samples on a fixed interval of
*simulated* time rather than once per solver step, because the solver's adaptive timestep
moves over six orders of magnitude and a per-step sample would make the window anywhere from
nanoseconds to milliseconds wide.

Run the Rust engine tests from this directory with `cargo test -p circuit-sim`.

## Desktop editor

The `circuit-desktop` crate uses egui/eframe for a small circuit editor. The crate keeps a
descriptive name but the program it builds is the project's own, so a release build produces
`Rusdl.exe`. Start it with:

```text
cargo run -p circuit-desktop
```

A release build is a Windows GUI-subsystem binary, so launching it does not flash up a console
window behind the app; a debug build keeps its console, which is where `cargo run`'s panics and
messages go. Since a console-less process has nowhere to print, the two things that used to
report to stderr now report on screen instead: a missing Chinese font goes into the status
line as the window appears, and a panic or a failure to start opens a message box.

Select or drag a component from the left palette onto the grid. Components snap to grid, can
be moved or rotated, and their values are edited in a floating properties window opened by
right-clicking a part; placing a function signal source opens its expression editor immediately.
The editor includes a source's amplitude and frequency, waveform picker, a switch's position,
and rotate and delete controls. Double-clicking a part selects it, or opens its waveform if it
is an instrument. The palette is searchable and groups parts into four shelves. Use the mouse wheel
or toolbar zoom buttons to zoom around the pointer/canvas center; click **适应画布** to fit all
placed parts, and left-drag blank canvas space to pan. To connect components, click **连线**,
click the starting terminal, click any blank-canvas positions to add wire bends, then click the
destination terminal. The live preview follows this manually placed route; after each
connection, wire mode remains active for another connection. Click **连线** again or
right-click to exit/cancel wire mode. Shared junctions are visibly marked. The toolbar provides
select/place/wire tools, run/pause, single-step, open, save, zoom, fit, and clear actions.
Clicking a switch on the canvas throws it, and the run keeps going. The voltage probe measures
its single connected node relative to ground; the ammeter's positive direction runs from its
first terminal to its second. Readouts are drawn on the instruments themselves rather than in a list
in the corner, and every instrument records a trace of its own readings while the simulation
runs: double-click one to open its waveform in a floating window, with both axes labelled and
scaled to the data. The plot shows the latest, minimum and maximum readings; hovering it adds a
crosshair and the nearest sampled value and time. The component library can be resized and
scrolled independently, while the toolbar wraps on narrow windows and status
stays in its own bar. This is what replaced the separate oscilloscope part and the strip of
canvas it used to occupy at the bottom of the window. The editor covers a fixed millisecond of
*simulated* time per frame rather than taking a fixed number of steps, and samples each trace
every 100 µs of that simulated time, with denser 10 µs sampling during the first 10 ms to
capture RC startup transients. A 60 ms window holds three cycles of a 50 Hz signal whatever
the engine's adaptive timestep happens to be doing — which varies over six orders of
magnitude, from a nanosecond on a stiff circuit to five microseconds on a trivial one. While
running, the waveform follows the latest 60 ms; while paused, it shows the complete captured
history. Drag horizontally on the plot to pan and use the mouse wheel to zoom around the
pointer, so you can inspect transients at any point in the run. Current-carrying parts are
tinted along their leads and carry small
markers; the markers are spaced a fixed distance apart
and a phase advances by that run's *own* current every frame, so parts in series move in
lockstep, a branch carrying twice the current visibly runs twice as fast, and reversing a
current reverses the march. A part with more than one run of current gets a separate phase for
each — a transformer's two windings, a transistor's three terminals, where `Ie = -(Ib + Ic)`
is what makes the picture agree with Kirchhoff's current law. The markers freeze when the
simulation is paused. Saved `.sdl` files use a small versioned text format; a file written
before the oscilloscope was folded into the meters still opens, without that one part. On
Windows the app uses an installed DengXian, SimHei, or SimSun font for Chinese UI text; it
falls back to the egui default font and logs a warning if none is available.

Saving updates the currently opened `.sdl` file; **另存为** chooses a different path. The
window title marks unsaved edits with `*`, and closing with edits prompts to save, discard, or
cancel.

Every part is drawn as leads either side of a symbol rather than a line through it, and a
d.c. source is marked `+` and `−` on the terminals they actually belong to, so it is obvious
which wire is live.

The drawing code is checked by rendering it: `cargo test -p circuit-desktop
render_symbol_sheet -- --nocapture` rasterises every symbol, and a small common-emitter stage
to show the marker flow including the transistor's three phases, to
`circuit-desktop/target/symbols.png`, and the waveform window to
`circuit-desktop/target/waveforms.png`, which draws one panel per source waveform. It uses a
software rasteriser in `render.rs` because egui paints through a GPU backend and there is
otherwise no headless way to look at the result.

## Components

The palette is four shelves — 无源器件, 有源器件, 开关与调节, 仪表 — grouped by what a part *is*
rather than alphabetically, because a flat list of nineteen is a list nobody scans. A shelf
with nothing in it disappears while a search is running, and the shelf the selected part is on
opens itself.

| Shelf | Parts | Values | Notes |
| --- | --- | --- | --- |
| 无源器件 | 电阻 / 电容 / 电感 | R in Ω, C in F, L in H | |
| | 变压器 | primary and secondary turns | Ideal transformer. The secondary winding is placed automatically beside the primary. |
| | 二极管 | forward drop at 1 A | Series diode, pointing from the first terminal to the second. |
| | 导线 / 接地 | — | A wire is a connection, not a component, and gets no caption. |
| 有源器件 | 直流电源 / 交流电源 | amplitude, frequency (a.c. only) | The a.c. source offers sine, square, triangle, sawtooth, and pulse. |
| | 函数信号源 | voltage expression as a function of `t` | Enter the expression when placing the source or by right-clicking it. |
| | 恒流源 | current in A | Drawn as a circle with an arrow, the international symbol. |
| | 三极管 | β, the forward current gain | NPN. Collector, emitter and a base lead offset to one side. |
| | 运放 / 比较器 | open-loop gain | The same hollow triangle with `−` and `+` on the inputs; a schematic does not distinguish the two any other way. The output saturates at ±10 V. |
| 开关与调节 | 开关 | open or closed | Click the switch on the canvas to throw it, or use the button in the property dialog. Either way the run keeps going: changing a switch re-analyses the circuit but does not stop it, since being able to operate a switch while the circuit runs is the point of the part. A switch starts closed, so one you have just dropped into a circuit conducts. |
| | 双掷开关 | throw 1 or throw 2 | The pole and two derived throws. |
| | 压控开关 | on- and off-resistance | The control terminal is derived from the body's midpoint. |
| | 滑动变阻器 | maximum resistance, slider position 0–1 | Drag the slider on its symbol; the effective series resistance changes with its position. |
| 仪表 | 电压探针 / 电流表 | — | The single-terminal voltage probe reads to ground; the ammeter's positive direction is its first-to-second terminal. Both keep a rolling trace that a double click opens as a waveform. |

A multi-terminal part's extra terminals are **derived**, and the editor asks the engine element
for them rather than working them out a second time —
`SpdtSwitch::throw_post_layout`, `AnalogSwitch::control_post`, `OpAmp::input_posts`. Each of
those is a port of the corresponding `setPoints` in circuitjs1, so the terminal a user clicks
is the terminal the solver will join. A test pins the two lists together for every kind.

That has one consequence worth knowing: because the engine joins nodes by coordinate
equality, a terminal that landed between two grid cells could never be wired to. The editor
therefore only ever places parts on whole cells, axis-aligned, and the voltage-controlled
switch gets an even-length body so that its control terminal at the body's midpoint is a cell.
The rheostat's slider adjusts the resistance of its two-terminal series element; a minimum
resistance is retained to avoid a zero-ohm branch in the matrix.

An ideal transformer is d.c.-transparent: an applied d.c. level reappears on the secondary
scaled by the turns ratio. That is the idealisation, and it is what makes a resistive
secondary solvable without any iteration. A secondary shorted straight back onto the primary
is degenerate and is reported as a singular matrix rather than a meaningless answer.

### Custom source waveforms

The function signal source lets the user write an expression and that expression *is* the
source's voltage in volts. `3 - sin(t - pi/2)` is a
sine riding on a 3 V offset; `5*sin(2*pi*50*t)` is a 5 V, 50 Hz sine. `t` is the simulation
time in seconds and angles are radians. Amplitude and frequency controls do not apply to this
source: the expression already fixes both the size of the signal and how fast it repeats. The
ordinary a.c. source remains a separate component with built-in waveform choices.

The language is deliberately not a scripting language: no assignment, no state, no loops, and
no way to reach outside the arithmetic. A source is re-evaluated every timestep, so anything
with state would make the result depend on how the solver happened to iterate rather than on
the time. It supports `+ - * / % ^`, parentheses, the constants `pi`, `e` and `tau`, and
`sin cos tan asin acos atan sinh cosh tanh exp ln log log10 sqrt cbrt abs sign floor ceil round
trunc sqr min max pow atan2 hypot mod`. `^` is right-associative and binds tighter than a
leading minus, so `-2^2` is `-4` and `2^-1` is `0.5`, as they are written in a textbook.
Arithmetic follows IEEE 754, so `1/0` is an infinity rather than a syntax error — which means
an expression can be perfectly good and still be undefined at one particular time. When that
happens the source flags it and the run stops with a message naming the expression, rather than
stamping a NaN into the matrix and surfacing much later as a singular matrix.

The parser and evaluator live in `circuit-sim/src/expression.rs` and have nothing to do with
circuits; they are exported as `circuit_sim::Expr` so the editor can check an expression as it
is typed instead of at the moment the run starts.

The transistor is the SPICE Gummel-Poon transport-factor model with the junction
capacitances left out, so `β` is the common-emitter current gain you get in forward active.
Both junctions are solved together rather than as two independent diodes, which is what lets
a saturated stage settle rather than oscillate.

Desktop code is separated by responsibility: `app.rs` owns application state and UI,
`canvas.rs` owns drawing and canvas interactions, `model.rs` defines placed parts, and
`persistence.rs` reads and writes circuit files. `render.rs` is a test-only software renderer
used to look at the drawing code.