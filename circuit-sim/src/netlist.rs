//! Netlist parser. Port of the text format read by `CirSim.readCircuit`.
//!
//! The format is one element per line, whitespace separated:
//!
//! ```text
//! $ <flags> <maxTimeStep> <speed> <currentBar> <voltageRange> [<powerBar> <minTimeStep>]
//! v <x1> <y1> <x2> <y2> <flags> <waveform> <frequency> <maxVoltage> <bias> <phaseShift> <dutyCycle>
//! w <x1> <y1> <x2> <y2> <flags>
//! r <x1> <y1> <x2> <y2> <flags> <resistance>
//! g <x1> <y1> <x2> <y2> <flags> [<symbolType>]
//! c <x1> <y1> <x2> <y2> <flags> <capacitance> <voltdiff> [<initialVoltage>]
//! l <x1> <y1> <x2> <y2> <flags> <inductance> <current>
//! i <x1> <y1> <x2> <y2> <flags> <current>
//! t <x1> <y1> <x2> <y2> <flags> <primaryTurns> [<secondaryTurns>]
//! q <x1> <y1> <x2> <y2> <ex> <ey> <flags> [<beta> | <area>]
//! d <x1> <y1> <x2> <y2> <flags> [<fwdrop> | <modelName>]
//! z <x1> <y1> <x2> <y2> <flags> [<fwdrop>] <zvoltage> | <modelName>
//! 162 <x1> <y1> <x2> <y2> <flags> [<fwdrop>] <r> <g> <b> [<maxBrightnessCurrent>]
//! s <x1> <y1> <x2> <y2> <flags> <position> <momentary>
//! S <x1> <y1> <x2> <y2> <flags> <position> <momentary> <link> [<throwCount>]
//! 174 <x1> <y1> <x2> <y2> <flags> <maxResistance> <position> [<sliderText>...]
//! 159 <x1> <y1> <x2> <y2> <flags> [<r_on> <r_off>]
//! ```
//!
//! The linear elements, the three diode types, the two switches, the potentiometer and the
//! analog switch are understood. Anything else — scopes (`o`), logic outputs (`O`), subcircuit
//! models (`.`) and the remaining numeric element types — is skipped and reported through
//! [`LoadedCircuit::warnings`] rather than treated as an error, so a netlist that merely
//! *contains* an unsupported element still loads.
//!
//! # Coordinates are connectivity
//!
//! Post coordinates are not decoration. The engine keys nodes off the exact `(x, y)` of a
//! post, so two elements sharing a coordinate are electrically joined with no wire needed.
//! That is why parsing has to preserve the integers verbatim.

use std::fmt;

use crate::builtin_model;
use crate::circuit::Circuit;
use crate::element::Point;
use crate::elements::{
    AnalogSwitch, Base, Bjt, Capacitor, Comparator, CurrentSource, Diode, DiodeKind, DiodeModel,
    Ground, Inductor, OpAmp, Pot, Resistor, SpdtSwitch, Switch, Transformer, VoltageSource,
    Waveform, Wire, FLAG_FWDROP, FLAG_MODEL,
};

/// A parsed netlist and everything in it that could not be represented.
pub struct LoadedCircuit {
    pub circuit: Circuit,
    /// One message per line that was recognised but skipped, in file order.
    pub warnings: Vec<String>,
}

impl fmt::Debug for LoadedCircuit {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LoadedCircuit")
            .field("elements", &self.circuit.elements().len())
            .field("warnings", &self.warnings)
            .finish_non_exhaustive()
    }
}

/// A line that could not be parsed at all. Skippable-but-unknown elements do not land here;
/// they become warnings instead.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NetlistError {
    /// The line has fewer whitespace-separated tokens than its type requires.
    MissingToken {
        line: usize,
        wanted: &'static str,
        text: String,
    },
    /// A numeric field did not parse.
    BadNumber {
        line: usize,
        token: String,
        text: String,
    },
    /// A `v` line named a waveform code outside the `WF_*` range.
    UnknownWaveform {
        line: usize,
        code: i32,
        text: String,
    },
}

impl fmt::Display for NetlistError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingToken { line, wanted, text } => {
                write!(f, "line {line}: expected {wanted}, got `{text}`")
            }
            Self::BadNumber { line, token, text } => {
                write!(f, "line {line}: `{token}` is not a number in `{text}`")
            }
            Self::UnknownWaveform { line, code, text } => {
                write!(f, "line {line}: unknown waveform code {code} in `{text}`")
            }
        }
    }
}

impl std::error::Error for NetlistError {}

/// One line's tokens, with the parse errors able to name the offending line.
struct Fields<'a> {
    line: usize,
    text: &'a str,
    toks: Vec<&'a str>,
}

impl<'a> Fields<'a> {
    fn get(&self, i: usize, what: &'static str) -> Result<&'a str, NetlistError> {
        self.toks
            .get(i)
            .copied()
            .ok_or_else(|| NetlistError::MissingToken {
                line: self.line,
                wanted: what,
                text: self.text.to_string(),
            })
    }

    fn int(&self, i: usize, what: &'static str) -> Result<i32, NetlistError> {
        let t = self.get(i, what)?;
        t.parse::<i32>().map_err(|_| NetlistError::BadNumber {
            line: self.line,
            token: t.to_string(),
            text: self.text.to_string(),
        })
    }

    fn num(&self, i: usize, what: &'static str) -> Result<f64, NetlistError> {
        let t = self.get(i, what)?;
        t.parse::<f64>().map_err(|_| NetlistError::BadNumber {
            line: self.line,
            token: t.to_string(),
            text: self.text.to_string(),
        })
    }

    /// Absolute token index of the nth field after the flag word, or `None` if absent.
    fn extra(&self, n: usize) -> Option<f64> {
        self.toks.get(6 + n).and_then(|t| t.parse::<f64>().ok())
    }

    fn extra_int(&self, n: usize) -> Option<i32> {
        self.toks.get(6 + n).and_then(|t| t.parse::<i32>().ok())
    }

    /// The five fields every element line shares, as a [`Base`].
    fn base(&self, node_slots: usize) -> Result<Base, NetlistError> {
        let x1 = self.int(1, "x1")?;
        let y1 = self.int(2, "y1")?;
        let x2 = self.int(3, "x2")?;
        let y2 = self.int(4, "y2")?;
        let flags = self.int(5, "flags")?;
        Ok(Base::from_points(
            Point::new(x1, y1),
            Point::new(x2, y2),
            flags,
            node_slots,
        ))
    }
}

/// Parses a netlist. Unsupported elements are skipped with a warning; malformed lines are
/// errors. The returned [`Circuit`] has *not* been analyzed yet.
pub fn parse(text: &str) -> Result<LoadedCircuit, NetlistError> {
    let mut circuit = Circuit::new();
    let mut warnings = Vec::new();

    for (idx, raw) in text.lines().enumerate() {
        let trimmed = raw.trim();
        if trimmed.is_empty() {
            continue;
        }
        let toks: Vec<&str> = trimmed.split_whitespace().collect();
        let f = Fields {
            line: idx + 1,
            text: trimmed,
            toks,
        };
        let kind = f.toks[0];

        match kind.chars().next().unwrap() {
            // Transient-analysis options: timestep, integration mode, and display knobs.
            '$' => read_options(&mut circuit, &f),
            // Hints, custom-model definitions and filter bookkeeping carry no geometry.
            'h' | '!' | '%' | '?' | 'B' | '.' => {}
            'o' => warnings.push(format!("line {}: scope `{kind}` ignored", f.line)),
            // `162`, `174` and `159` are numeric dump types. They have to be matched out here
            // rather than in the table below, because a leading digit would otherwise route
            // them straight to the "not supported" warning.
            _ if kind == "162" => add_diode(&mut circuit, &f, DiodeKind::Led)?,
            _ if kind == "174" => {
                let max_resistance = f.num(6, "maxResistance")?;
                let position = f.num(7, "position")?;
                circuit.add(Box::new(Pot::from_base(
                    f.base(3)?,
                    max_resistance,
                    position,
                )));
            }
            _ if kind == "159" => {
                let r_on = f.extra(0).unwrap_or(20.0);
                let r_off = f.extra(1).unwrap_or(1e10);
                circuit.add(Box::new(AnalogSwitch::from_base(f.base(3)?, r_on, r_off)));
            }
            // `401` is the comparator, another numeric dump type.
            _ if kind == "401" => {
                circuit.add(Box::new(Comparator::from_base(f.base(3)?)));
            }
            c if c.is_ascii_digit() => warnings.push(format!(
                "line {}: element type `{kind}` not supported",
                f.line
            )),
            _ => match kind {
                "r" => {
                    let r = f.num(6, "resistance")?;
                    circuit.add(Box::new(Resistor::from_base(f.base(2)?, r)));
                }
                "w" => circuit.add(Box::new(Wire::from_base(f.base(2)?))),
                "g" => circuit.add(Box::new(Ground::from_base(f.base(1)?))),
                "v" => {
                    // `VoltageElm`'s constructor reads this whole tail inside one try/catch,
                    // so a short line falls back to the field defaults rather than failing.
                    // `voltdivide.txt` relies on that: its source omits phaseShift and
                    // dutyCycle. Defaults mirror the Java constructor.
                    let code = f.extra_int(0).unwrap_or(0);
                    let waveform =
                        Waveform::from_code(code).ok_or_else(|| NetlistError::UnknownWaveform {
                            line: f.line,
                            code,
                            text: f.text.to_string(),
                        })?;
                    circuit.add(Box::new(VoltageSource::from_base(
                        f.base(2)?,
                        waveform,
                        f.extra(1).unwrap_or(40.0),
                        f.extra(2).unwrap_or(5.0),
                        f.extra(3).unwrap_or(0.0),
                        f.extra(4).unwrap_or(0.0),
                        f.extra(5).unwrap_or(0.5),
                        false,
                    )));
                }
                "c" => {
                    // capacitance and voltdiff are required; the reset voltage may be absent.
                    let capacitance = f.num(6, "capacitance")?;
                    let _voltdiff = f.num(7, "voltdiff")?;
                    let mut cap = Capacitor::from_base(f.base(2)?, capacitance);
                    if let Some(initial) = f.extra(2) {
                        cap.set_initial_voltage(initial);
                    }
                    circuit.add(Box::new(cap));
                }
                "l" => {
                    let inductance = f.num(6, "inductance")?;
                    let current = f.num(7, "current")?;
                    circuit.add(Box::new(Inductor::from_base(
                        f.base(2)?,
                        inductance,
                        current,
                    )));
                }
                "i" => {
                    // `CurrentElm` defaults to 10mA when the value is missing.
                    let current = f.extra(0).unwrap_or(0.01);
                    circuit.add(Box::new(CurrentSource::from_base(f.base(2)?, current)));
                }
                // `t` is the transformer: the two windings' terminals, then their turn
                // counts. The secondary pair is derived from the primary's geometry the
                // same way the desktop editor derives it, so a netlist and a saved canvas
                // describe the same four posts.
                "t" => {
                    let p1 = Point::new(f.int(1, "x1")?, f.int(2, "y1")?);
                    let p2 = Point::new(f.int(3, "x2")?, f.int(4, "y2")?);
                    let primary_turns = f.num(6, "primary turns")?;
                    let secondary_turns = f.extra(0).unwrap_or(1.0);
                    circuit.add(Box::new(Transformer::with_derived_secondary(
                        p1,
                        p2,
                        primary_turns,
                        secondary_turns,
                    )));
                }
                // `q` is the NPN transistor. circuitjs1's `BJTElm` only models the NPN case,
                // so a netlist cannot ask for anything else either.
                "q" => {
                    let base = Point::new(f.int(1, "x1")?, f.int(2, "y1")?);
                    let collector = Point::new(f.int(3, "x2")?, f.int(4, "y2")?);
                    let emitter = Point::new(f.int(5, "emitter x")?, f.int(6, "emitter y")?);
                    // `BJTElm`'s trailing token is the area, which the model does not use.
                    let beta = f.extra(0).unwrap_or(100.0);
                    circuit.add(Box::new(Bjt::new(base, collector, emitter, 0, beta)));
                }
                // `a` is the op-amp. The Java constructor reads six trailing fields in one
                // try/catch, so a short line keeps the defaults; being lenient here instead
                // just means each field falls back on its own.
                "a" => {
                    let max_out = f.extra(0).unwrap_or(15.0);
                    let min_out = f.extra(1).unwrap_or(-15.0);
                    // Without an explicit gain, CircuitJS derives it from FLAG_LOWGAIN.
                    let gain = match f.extra(5) {
                        Some(g) => g,
                        None => {
                            let flags = f.int(5, "flags").unwrap_or(0);
                            if flags & 4 != 0 {
                                1000.0
                            } else {
                                100_000.0
                            }
                        }
                    };
                    circuit.add(Box::new(OpAmp::from_base(
                        f.base(3)?,
                        max_out,
                        min_out,
                        gain,
                    )));
                }
                "s" => {
                    let position = switch_position(&f)?;
                    // `SwitchElm` reads this with `new Boolean(...)`, so a missing token is
                    // a malformed line; defaulting keeps hand-written netlists working.
                    let momentary = f.get(7, "momentary").is_ok_and(|t| t == "true");
                    circuit.add(Box::new(Switch::from_base(f.base(2)?, position, momentary)));
                }

                "S" => {
                    let position = switch_position(&f)?;
                    let momentary = f.get(7, "momentary").is_ok_and(|t| t == "true");
                    // `link` is required; `throwCount` defaults to 2.
                    let link = f.int(8, "link")?;
                    let throw_count = f.extra_int(3).unwrap_or(2).max(2) as usize;
                    circuit.add(Box::new(SpdtSwitch::from_base(
                        f.base(1 + throw_count)?,
                        position,
                        momentary,
                        link,
                        throw_count,
                    )));
                }
                // `d` and `z` are the same physics with different default models, so they
                // share one constructor and differ only in the kind tag.
                "d" | "z" => {
                    let kind = if kind == "z" {
                        DiodeKind::Zener
                    } else {
                        DiodeKind::Plain
                    };
                    add_diode(&mut circuit, &f, kind)?;
                }
                _ => warnings.push(format!(
                    "line {}: element type `{kind}` not supported",
                    f.line
                )),
            },
        }
    }

    Ok(LoadedCircuit { circuit, warnings })
}

/// Reads a switch's position token. Port of the `SwitchElm` netlist constructor, where
/// `"true"` means open and `"false"` means closed for everything except `LogicInputElm`.
fn switch_position(f: &Fields<'_>) -> Result<u8, NetlistError> {
    let t = f.get(6, "position")?;
    match t {
        "true" => Ok(1),
        "false" => Ok(0),
        // A numeric position is clamped: `toggle` wraps modulo 2, so anything outside
        // 0/1 is a malformed line rather than a third state.
        _ => Ok(f.int(6, "position")?.clamp(0, 1) as u8),
    }
}

/// Adds one diode-family element. Shared by the `d`, `z` and `162` lines, which differ only
/// in the default model and the kind tag.
fn add_diode(circuit: &mut Circuit, f: &Fields<'_>, kind: DiodeKind) -> Result<(), NetlistError> {
    let model = diode_model(f, kind)?;
    circuit.add(Box::new(Diode::from_base(
        f.base(model.node_slots())?,
        &model,
        kind,
    )));
    Ok(())
}

/// Works out which [`DiodeModel`] a `d`/`z`/`162` line asks for. Port of the `DiodeElm`,
/// `ZenerElm` and `LEDElm` netlist constructors.
///
/// A line that names a model (`flags & FLAG_MODEL`) uses that model verbatim. Otherwise the
/// curve is back-fitted to a forward voltage drop, which is what
/// `DiodeModel.getModelWithParameters` does — and what makes a plain `d` line and a `z` line
/// with no extra tokens behave like the defaults a user would expect.
///
/// One deliberate deviation: a netlist LED with no model name gets `default-led`, not the
/// `2.1024259` V drop the Java constructor asks for. That drop back-solves to a saturation
/// current of ~2e-18 A, which the Java source itself flags as "way too small (causes
/// numerical errors)"; `default-led` is the model CircuitJS switched to for exactly that
/// reason, and its forward drop (2.23 V) is close to the old one.
fn diode_model(f: &Fields<'_>, kind: DiodeKind) -> Result<DiodeModel, NetlistError> {
    let flags = f.int(5, "flags")?;

    if flags & FLAG_MODEL != 0 {
        let name = f.get(6, "model name")?;
        if let Some(model) = builtin_model(name) {
            return Ok(model);
        }
        // `getModelWithNameOrCopy` falls back to the default model when the name is unknown.
        return Ok(builtin_model("default").expect("built-in model"));
    }

    if kind == DiodeKind::Led && flags & FLAG_FWDROP == 0 {
        return Ok(builtin_model("default-led").expect("built-in model"));
    }

    // The forward drop the element starts from, before any explicit override.
    let default_drop = match kind {
        DiodeKind::Led => 2.1024259,
        _ => 0.805904783,
    };
    let mut idx = 6;
    let mut drop = default_drop;
    if flags & FLAG_FWDROP != 0 {
        drop = f.num(idx, "forward voltage drop")?;
        idx += 1;
    }
    let mut model = DiodeModel::with_forward_voltage_drop(drop, 2.0);
    if kind == DiodeKind::Zener {
        model.breakdown_voltage = f.num(idx, "zener voltage")?;
    }
    Ok(model)
}

/// Applies a `$` line. Port of `CirSim.readOptions`.
///
/// `flags` bit 64 is `adjustTimeStep`. The fields after the timestep are display settings
/// and a vestigial speed-bar value, none of which affect the simulation; the trailing
/// minimum timestep is optional and the power-bar value before it is ignored.
fn read_options(circuit: &mut Circuit, f: &Fields<'_>) {
    let Ok(flags) = f.int(1, "flags") else { return };
    let Ok(time_step) = f.num(2, "maxTimeStep") else {
        return;
    };
    circuit.max_time_step = time_step;
    circuit.time_step = time_step;
    circuit.adjust_time_step = flags & 64 != 0;
    // Token 6 is the power bar; token 7, when present, is the minimum timestep.
    if let Some(min) = f.toks.get(7).and_then(|t| t.parse::<f64>().ok()) {
        circuit.min_time_step = min;
    }
}
