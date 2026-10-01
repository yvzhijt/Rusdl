//! # Rusdl —— 简略电路仿真引擎
//!
//! Rusdl 是一个无头（headless）电路仿真引擎，参考开源项目
//! [circuitjs1](https://github.com/pfalstad/circuitjs1) 的 Java/GWT 实现，
//! 使用 Rust 语言重新编写并做了简化，仅保留基础电路元件的直流/交流仿真能力，
//! 定位于中学物理实验场景。
//!
//! 参考实现的 Java 代码位于 `circuit-js/src/com/lushprojects/circuitjs1/client/`，
//! 核心求解器 `CirSim.java` 按模块对等移植如下：
//!
//! | Rust | Java |
//! | --- | --- |
//! | [`lu`] | `lu_factor` / `lu_solve` |
//! | [`stamper`] | `stampMatrix` and the rest of the `stamp*` helpers, `simplifyMatrix`, `RowInfo` |
//! | [`element`] | the simulation half of `CircuitElm` |
//! | [`circuit`] | `analyzeCircuit`, `calculateWireClosure`, `calcWireInfo`, `findPath`, `runCircuit` |
//! | [`elements`] | the concrete `CircuitElm` subclasses |
//! | [`netlist`] | the text format consumed by `readCircuit` |
//!
//! # Model
//!
//! Modified nodal analysis. Node voltages are the unknowns, one row per node, plus one row
//! per voltage source enforcing `V(n1) - V(n2) = v`. Time-varying elements are handled with
//! companion models: a capacitor becomes a conductance plus a current source, an inductor a
//! resistance plus a current source, both refreshed in `do_step`.
//!
//! # Indexing
//!
//! Node numbers are **1-based with 0 meaning ground**, inherited from the original. Ground is
//! never stamped, so every `stamp_matrix`/`stamp_right_side` call drops `i == 0 || j == 0`.
//! Voltage-source rows are addressed as `node_count + vs` where `node_count` *includes*
//! ground. Keeping this convention rather than a tidier zero-based one means element code
//! translates directly from the Java.
//!
//! # Typical use
//!
//! Elements sharing a post coordinate become the same node, so a divider needs no wires:
//! the source's two ends and the resistor chain's ends sit on the same three points.
//!
//! ```
//! use circuit_sim::{Circuit, Ground, Point, Resistor, VoltageSource};
//!
//! // `dc_between(p1, p2, v)` drives `p2` positive, so the 10V rail is the *second* argument.
//! let mut c = Circuit::new();
//! c.add(Box::new(VoltageSource::dc_between(
//!     Point::new(0, 200), Point::new(0, 0), 10.0)));
//! c.add(Box::new(Resistor::between(
//!     Point::new(0, 0), Point::new(0, 100), 1_000.0)));
//! c.add(Box::new(Resistor::between(
//!     Point::new(0, 100), Point::new(0, 200), 1_000.0)));
//! c.add(Box::new(Ground::at(0, 200)));
//!
//! c.analyze().unwrap();
//! c.step().unwrap();
//!
//! // Node 2 is the +10V rail and node 3 the midpoint, which an equal divider puts at
//! // half the supply. Node 0 exists but nothing lands on it: with an explicit ground
//! // present, the engine allocates it as a spare reference rather than reusing it.
//! assert!((c.node_voltage(2) - 10.0).abs() < 1e-6);
//! assert!((c.node_voltage(3) - 5.0).abs() < 1e-6);
//! ```

pub mod circuit;
pub mod element;
pub mod elements;
pub mod expression;
pub mod lu;
pub mod netlist;
pub mod school;
pub mod stamper;

pub use circuit::{Circuit, SimError};
pub use element::{Element, Point, SimContext};
pub use elements::{
    builtin_model, AnalogSwitch, Base, Bjt, Capacitor, Comparator, CurrentSource, Diode, DiodeKind,
    DiodeModel, Ground, Inductor, OpAmp, Pot, Resistor, SpdtSwitch, Switch, Transformer,
    VoltageSource, Waveform, Wire,
};
pub use expression::{Expr, ExprError};
pub use lu::{lu_factor, lu_solve, Matrix};
pub use netlist::{parse as parse_netlist, LoadedCircuit, NetlistError};
pub use school::{Ammeter, InstrumentError, PowerSupply, Voltmeter};
pub use stamper::{RowInfo, RowType, Stamper, StamperError};
