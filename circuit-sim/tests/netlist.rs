//! Netlist parser tests, including a regression against a real circuitjs1 netlist.

use circuit_sim::{parse_netlist, NetlistError, Point, Waveform};

/// The divider shipped with circuitjs1. It exercises the format's awkward parts: a `$`
/// transient header, vertical wires, a source with *no* explicit ground (so the source's
/// first terminal becomes ground), and trailing fields the source line omits.
const VOLTDIVIDE: &str = include_str!("data/voltdivide.txt");

#[test]
fn voltdivide_parses_with_only_probe_warnings() {
    let loaded = parse_netlist(VOLTDIVIDE).expect("voltdivide.txt should parse");
    // The file has four `O` output/probe elements (lines 7, 14, 15, 16), which we do not
    // model. They are skipped, not fatal.
    assert_eq!(loaded.warnings.len(), 4, "{:?}", loaded.warnings);
    assert!(loaded.warnings.iter().all(|w| w.contains("`O`")));
    // 1 source + 4 wires + 6 resistors = 11 elements.
    assert_eq!(loaded.circuit.elements().len(), 11);
}

#[test]
fn voltdivide_dc_operating_point() {
    let mut loaded = parse_netlist(VOLTDIVIDE).expect("parse");
    // The `$` header pins the timestep and bit 64 (adjustTimeStep) is clear.
    assert!((loaded.circuit.max_time_step - 5.0e-6).abs() < 1e-15);
    assert!((loaded.circuit.time_step - 5.0e-6).abs() < 1e-15);
    assert!(!loaded.circuit.adjust_time_step);

    loaded.circuit.analyze().expect("analyze");
    loaded.circuit.step().expect("step");
    let c = &loaded.circuit;

    let at = |x, y| {
        c.node_at_point(Point::new(x, y))
            .unwrap_or_else(|| panic!("no node at ({x},{y})"))
    };
    let v = |x, y| c.node_voltage(at(x, y));

    // The source spans (112,368)-(112,48) and post 1 is positive, so the y=48 rail is +10V
    // and y=368 is ground.
    assert!(
        (v(112, 48) - 10.0).abs() < 1e-6,
        "top rail = {}",
        v(112, 48)
    );
    assert!(v(112, 368).abs() < 1e-6, "bottom rail = {}", v(112, 368));

    // Left branch: two equal 10k resistors, so the midpoint is half the supply.
    assert!(
        (v(240, 208) - 5.0).abs() < 1e-6,
        "left tap = {}",
        v(240, 208)
    );

    // Right branch: four equal 10k resistors, giving 7.5 / 5 / 2.5 volts.
    assert!((v(432, 128) - 7.5).abs() < 1e-6, "rtap 1 = {}", v(432, 128));
    assert!((v(432, 208) - 5.0).abs() < 1e-6, "rtap 2 = {}", v(432, 208));
    assert!((v(432, 288) - 2.5).abs() < 1e-6, "rtap 3 = {}", v(432, 288));

    // Both branch midpoints are the same potential, so a wire between them would carry no
    // current; they are only distinct because nothing joins them.
    assert_ne!(at(240, 208), at(432, 208));
}

#[test]
fn source_without_ground_uses_its_first_terminal() {
    // No `g` element: setGroundNode anchors on the voltage source's post 0.
    let text = "v 0 100 0 0 0 0 0 5 0\nr 0 0 0 100 0 100\n";
    let mut loaded = parse_netlist(text).expect("parse");
    assert!(loaded.warnings.is_empty());
    loaded.circuit.analyze().expect("analyze");
    loaded.circuit.step().expect("step");

    let c = &loaded.circuit;
    let bottom = c.node_at_point(Point::new(0, 100)).unwrap();
    let top = c.node_at_point(Point::new(0, 0)).unwrap();
    assert!(c.node_voltage(bottom).abs() < 1e-9, "anchor must be ground");
    assert!((c.node_voltage(top) - 5.0).abs() < 1e-9);
}

#[test]
fn transient_options_are_read() {
    // flags = 64 selects adjustTimeStep; the trailing minimum timestep is optional.
    let text = "$ 64 1.0E-4 10 50 5.0 30 1.0E-9\n";
    let loaded = parse_netlist(text).expect("parse");
    assert!((loaded.circuit.max_time_step - 1.0e-4).abs() < 1e-15);
    assert!((loaded.circuit.time_step - 1.0e-4).abs() < 1e-15);
    assert!(loaded.circuit.adjust_time_step);
    assert!((loaded.circuit.min_time_step - 1.0e-9).abs() < 1e-21);

    // Without the optional pair, adjustTimeStep is false and minTimeStep keeps its default.
    let loaded = parse_netlist("$ 1 5.0E-6 10 63 10.0\n").expect("parse");
    assert!(!loaded.circuit.adjust_time_step);
    assert!(loaded.circuit.min_time_step > 0.0);
}

#[test]
fn waveform_and_rc_fields_round_trip() {
    let text = "v 0 0 0 40 0 1 60 3 0.5 0 0.25\n\
                r 0 0 0 20 0 1000\n\
                c 0 20 0 40 0 1e-6 2.5 2.5\n\
                l 0 40 0 60 0 1e-3 0.02\n";
    let loaded = parse_netlist(text).expect("parse");
    let els = loaded.circuit.elements();
    assert_eq!(
        els[0]
            .as_any()
            .downcast_ref::<circuit_sim::VoltageSource>()
            .unwrap()
            .waveform(),
        Waveform::Ac
    );
    let cap = els[2]
        .as_any()
        .downcast_ref::<circuit_sim::Capacitor>()
        .unwrap();
    assert!((cap.capacitance() - 1e-6).abs() < 1e-18);
    assert!((cap.initial_voltage() - 2.5).abs() < 1e-12);
    let ind = els[3]
        .as_any()
        .downcast_ref::<circuit_sim::Inductor>()
        .unwrap();
    assert!((ind.inductance() - 1e-3).abs() < 1e-15);
    assert!((ind.initial_current() - 0.02).abs() < 1e-15);
}

#[test]
fn unknown_elements_warn_but_do_not_fail() {
    let text = "v 0 0 0 40 0 0 0 5 0\nO 10 10 10 40 1\nr 0 40 0 80 0 1000\n";
    let loaded = parse_netlist(text).expect("parse");
    assert_eq!(loaded.warnings.len(), 1, "{:?}", loaded.warnings);
    assert_eq!(loaded.circuit.elements().len(), 2);
}

#[test]
fn malformed_lines_are_errors() {
    // Truncated resistor: no resistance value.
    let err = parse_netlist("r 0 0 0 40 0\n").unwrap_err();
    assert!(
        matches!(err, NetlistError::MissingToken { line: 1, .. }),
        "{err:?}"
    );

    // Non-numeric coordinate.
    let err = parse_netlist("r 0 0 zero 40 0 1000\n").unwrap_err();
    assert!(
        matches!(err, NetlistError::BadNumber { line: 1, .. }),
        "{err:?}"
    );

    // Waveform code outside the WF_* range.
    let err = parse_netlist("v 0 0 0 40 0 99 60 5 0\n").unwrap_err();
    assert!(
        matches!(
            err,
            NetlistError::UnknownWaveform {
                line: 1,
                code: 99,
                ..
            }
        ),
        "{err:?}"
    );
}

/// A `d` line with no extra tokens is the default diode; one naming a model uses that model
/// verbatim, which is how a 1N4148 gets its series resistance.
#[test]
fn diode_lines_pick_their_model() {
    // FLAG_MODEL (2) is what makes the token a model name rather than a forward drop.
    let text = "d 0 0 0 40 0\n\
                d 0 40 0 80 2 1N4148\n\
                d 0 80 0 120 1 0.7\n";
    let loaded = parse_netlist(text).expect("parse");
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    let els = loaded.circuit.elements();

    let plain = els[0]
        .as_any()
        .downcast_ref::<circuit_sim::Diode>()
        .unwrap();
    assert_eq!(plain.kind(), circuit_sim::DiodeKind::Plain);
    assert_eq!(plain.model().series_resistance, 0.0);

    // A named model is used as-is, series resistance included.
    let named = els[1]
        .as_any()
        .downcast_ref::<circuit_sim::Diode>()
        .unwrap();
    assert!((named.model().series_resistance - 0.6458).abs() < 1e-12);
    assert!((named.model().breakdown_voltage - 75.0).abs() < 1e-12);

    // FLAG_FWDROP back-fits the curve to the requested drop.
    let fitted = els[2]
        .as_any()
        .downcast_ref::<circuit_sim::Diode>()
        .unwrap();
    assert!((fitted.model().forward_voltage_drop() - 0.7).abs() < 1e-8);
}

/// A `z` line reads its breakdown voltage from the token after the flags, and an `162` line
/// gets the modern LED model rather than the numerically unusable one the Java constructor
/// would have derived.
#[test]
fn zener_and_led_lines_parse() {
    let text = "z 0 0 0 40 0 5.6\n\
                z 0 40 0 80 1 0.7 3.3\n\
                162 0 80 0 120 0 1 0 0\n";
    let loaded = parse_netlist(text).expect("parse");
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    let els = loaded.circuit.elements();

    let z = els[0]
        .as_any()
        .downcast_ref::<circuit_sim::Diode>()
        .unwrap();
    assert_eq!(z.kind(), circuit_sim::DiodeKind::Zener);
    assert!((z.model().breakdown_voltage - 5.6).abs() < 1e-12);

    // An explicit forward drop shifts the curve, the Zener voltage does not.
    let z2 = els[1]
        .as_any()
        .downcast_ref::<circuit_sim::Diode>()
        .unwrap();
    assert!((z2.model().forward_voltage_drop() - 0.7).abs() < 1e-8);
    assert!((z2.model().breakdown_voltage - 3.3).abs() < 1e-12);

    let led = els[2]
        .as_any()
        .downcast_ref::<circuit_sim::Diode>()
        .unwrap();
    assert_eq!(led.kind(), circuit_sim::DiodeKind::Led);
    // `default-led` carries 42 mΩ of series resistance, so the element needs a third node.
    assert!((led.model().series_resistance - 0.042).abs() < 1e-12);
    assert_eq!(circuit_sim::Element::internal_node_count(led), 1);
}

/// An `S` line reads its link and optional throw count after the position and momentary
/// flags, and the throw posts have to land where the engine expects them.
#[test]
fn spdt_line_parses() {
    let text = "S 0 0 100 50 0 0 false 0\n\
                S 0 100 100 150 0 1 true 3\n";
    let loaded = parse_netlist(text).expect("parse");
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    let els = loaded.circuit.elements();

    let spdt = els[0]
        .as_any()
        .downcast_ref::<circuit_sim::SpdtSwitch>()
        .unwrap();
    assert_eq!(spdt.position(), 0);
    assert!(!spdt.is_momentary());
    assert_eq!(spdt.throw_count(), 2);
    assert_eq!(circuit_sim::Element::post_count(spdt), 3);

    let spdt2 = els[1]
        .as_any()
        .downcast_ref::<circuit_sim::SpdtSwitch>()
        .unwrap();
    assert_eq!(spdt2.position(), 1);
    assert!(spdt2.is_momentary());
    assert_eq!(spdt2.link(), 3);
    assert_eq!(spdt2.throw_count(), 2);
}

/// An `a` line reads its output range and gain. The gain token is optional; without it the
/// value comes from `FLAG_LOWGAIN`, matching `OpAmpElm.setGain`.
#[test]
fn opamp_line_parses() {
    let text = "a 0 0 100 50 0 15 -15 1e6 0 0 100000\n\
                a 0 100 100 150 0\n\
                a 0 200 100 250 4\n";
    let loaded = parse_netlist(text).expect("parse");
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    let els = loaded.circuit.elements();

    let a = els[0]
        .as_any()
        .downcast_ref::<circuit_sim::OpAmp>()
        .unwrap();
    assert_eq!(a.max_out(), 15.0);
    assert_eq!(a.min_out(), -15.0);
    assert_eq!(a.gain(), 100_000.0);

    // No gain token, no FLAG_LOWGAIN: the default 100k.
    let a2 = els[1]
        .as_any()
        .downcast_ref::<circuit_sim::OpAmp>()
        .unwrap();
    assert_eq!(a2.gain(), 100_000.0);

    // FLAG_LOWGAIN (4) with no gain token: 1000.
    let a3 = els[2]
        .as_any()
        .downcast_ref::<circuit_sim::OpAmp>()
        .unwrap();
    assert_eq!(a3.gain(), 1000.0);
}

/// A `401` line is the comparator, which takes no trailing fields.
#[test]
fn comparator_line_parses() {
    let text = "401 0 0 100 50 0\n";
    let loaded = parse_netlist(text).expect("parse");
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    let els = loaded.circuit.elements();
    assert!(els[0]
        .as_any()
        .downcast_ref::<circuit_sim::Comparator>()
        .is_some());
    assert_eq!(circuit_sim::Element::post_count(els[0].as_ref()), 3);
}

/// An op-amp loaded from a netlist must actually amplify, which is the end-to-end check that
/// the parser wired the three posts up to real nodes.
#[test]
fn netlist_opamp_amplifies() {
    // The op-amp's inputs are offset from its body, so the source has to land on the
    // non-inverting post at (16,100) rather than the body's end at (0,100).
    let text = "v 0 200 16 100 0 0 0 1 0\n\
                a 0 100 0 0 0 15 -15 1e6 0 0 100000\n\
                r 0 0 0 200 0 1000\n";
    let mut loaded = parse_netlist(text).expect("parse");
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    loaded.circuit.analyze().expect("analyze");
    loaded.circuit.step().expect("step");

    let c = &loaded.circuit;
    let opa = c.elements()[1]
        .as_any()
        .downcast_ref::<circuit_sim::OpAmp>()
        .unwrap();
    let out = circuit_sim::Element::post(opa, 2);
    let v = c.node_voltage(c.node_at_point(out).unwrap());
    // 1 V on the non-inverting input, open loop, so the output slams into the +15 V rail.
    assert!((v - 15.0).abs() < 0.05, "output = {v}");
}

/// An `s` line reads its position and momentary flag, where `"true"` means open for a plain
/// switch (the opposite of `LogicInputElm`, which this engine does not model).
#[test]
fn switch_line_parses() {
    let text = "s 0 0 0 40 0 false false\n\
                s 0 40 0 80 0 true true\n\
                s 0 80 0 120 0 1 false\n";
    let loaded = parse_netlist(text).expect("parse");
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    let els = loaded.circuit.elements();

    let closed = els[0]
        .as_any()
        .downcast_ref::<circuit_sim::Switch>()
        .unwrap();
    assert_eq!(closed.position(), 0);
    assert!(!closed.is_momentary());

    let open = els[1]
        .as_any()
        .downcast_ref::<circuit_sim::Switch>()
        .unwrap();
    assert_eq!(open.position(), 1);
    assert!(open.is_momentary());

    // A numeric position is accepted too.
    let numeric = els[2]
        .as_any()
        .downcast_ref::<circuit_sim::Switch>()
        .unwrap();
    assert_eq!(numeric.position(), 1);
}

/// A `174` line reads its maximum resistance and wiper position, and a `159` line reads its
/// on/off resistances with the CircuitJS defaults when they are omitted.
#[test]
fn pot_and_analog_switch_lines_parse() {
    let text = "174 0 0 0 100 0 1000 0.5 Volume\n\
                159 0 100 0 200 0\n\
                159 0 200 0 300 0 50 1e8\n";
    let loaded = parse_netlist(text).expect("parse");
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    let els = loaded.circuit.elements();

    let pot = els[0].as_any().downcast_ref::<circuit_sim::Pot>().unwrap();
    assert_eq!(pot.max_resistance(), 1000.0);
    assert_eq!(pot.position(), 0.5);
    assert_eq!(circuit_sim::Element::post_count(pot), 3);

    // Omitted resistances fall back to the CircuitJS defaults.
    let sw = els[1]
        .as_any()
        .downcast_ref::<circuit_sim::AnalogSwitch>()
        .unwrap();
    assert_eq!(sw.r_on(), 20.0);
    assert_eq!(sw.r_off(), 1e10);

    let sw2 = els[2]
        .as_any()
        .downcast_ref::<circuit_sim::AnalogSwitch>()
        .unwrap();
    assert_eq!(sw2.r_on(), 50.0);
    assert_eq!(sw2.r_off(), 1e8);
}

/// A potentiometer loaded from a netlist must actually divide, which is the end-to-end check
/// that the parser wired the wiper up to a real node.
#[test]
fn netlist_potentiometer_divides() {
    let text = "v 0 200 0 0 0 0 0 10 0\n\
                174 0 0 0 200 0 1000 0.25\n";
    let mut loaded = parse_netlist(text).expect("parse");
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    loaded.circuit.analyze().expect("analyze");
    loaded.circuit.step().expect("step");

    let c = &loaded.circuit;
    let pot = c.elements()[1]
        .as_any()
        .downcast_ref::<circuit_sim::Pot>()
        .unwrap();
    let wiper = circuit_sim::Element::post(pot, 2);
    let w = c.node_at_point(wiper).unwrap();
    let v = c.node_voltage(w);
    assert!((v - 7.5).abs() < 1e-6, "wiper = {v}");
}

/// An SPDT switch loaded from a netlist must actually route, which is the end-to-end check
/// that the parser wired the throws up to real nodes.
#[test]
fn netlist_zener_clamps() {
    let text = "v 0 200 0 40 0 0 0 12 0\n\
                r 0 40 0 160 0 1000\n\
                z 0 200 0 160 0 5.6\n";
    let mut loaded = parse_netlist(text).expect("parse");
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    loaded.circuit.analyze().expect("analyze");
    for _ in 0..20 {
        loaded.circuit.step().expect("step");
    }

    let c = &loaded.circuit;
    let cathode = c.node_at_point(Point::new(0, 160)).unwrap();
    let v = c.node_voltage(cathode);
    assert!((v - 5.6).abs() < 0.05, "clamped node = {v}");
}
