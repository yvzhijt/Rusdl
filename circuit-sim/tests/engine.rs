//! End-to-end engine tests: build a circuit, run it, check the physics.
//!
//! These are the tests that matter for the port, because they exercise the whole pipeline
//! (`analyze` -> node allocation -> validation -> stamping -> simplification -> factorisation
//! -> Newton loop -> wire-current reconstruction) rather than any one layer in isolation.
//! The unit tests in `lu.rs` and `stamper.rs` cover the algebra; nothing below covers the
//! plumbing that joins it to the elements.
//!
//! Two conventions the rest of these tests rely on, both inherited from circuitjs1 rather
//! than invented here:
//!
//! * `dc_between(p1, p2, v)` drives `p2` positive. The stamp is `V(p2) - V(p1) = v`, so the
//!   **first** argument is the terminal the source current returns to. A source that feeds a
//!   load therefore has to be built `load_side -> ground_side`, not the other way round.
//! * Node numbers are handed out in the order posts are first seen, so they are not worth
//!   hard-coding. These tests read node voltages through `node_at` instead.

use circuit_sim::*;

/// Runs `n` timesteps, failing with context rather than a bare unwrap.
fn run(c: &mut Circuit, n: usize) {
    for i in 0..n {
        if let Err(e) = c.step() {
            panic!("step {i} failed: {e:?}");
        }
    }
}

/// The node attached to `p`, or a panic naming the point.
///
/// Hard-coded node numbers make these tests lie the moment an element is inserted or
/// reordered, and a wrong guess looks exactly like a solver bug.
fn node_at(c: &Circuit, p: Point) -> usize {
    c.node_at_point(p)
        .unwrap_or_else(|| panic!("no element claims a node at ({}, {})", p.x, p.y))
}

/// A resistive divider is solved exactly in one step, since it is linear and needs no
/// Newton iterations.
#[test]
fn resistive_divider() {
    let (tap, top, bottom) = (Point::new(0, 100), Point::new(0, 0), Point::new(0, 200));
    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(bottom, top, 10.0)));
    c.add(Box::new(Resistor::between(top, tap, 1_000.0)));
    c.add(Box::new(Resistor::between(tap, bottom, 3_000.0)));
    c.add(Box::new(Ground::at_point(bottom)));

    c.analyze().expect("analyze");
    run(&mut c, 1);

    // 10V across 1k+3k. The tap is 3k from ground, so it sits at 10 * 3/4 = 7.5V.
    assert!(
        (c.node_voltage(node_at(&c, tap)) - 7.5).abs() < 1e-9,
        "tap = {}",
        c.node_voltage(node_at(&c, tap))
    );
    // Same current through both, and 2.5mA overall.
    let v_top = c.node_voltage(node_at(&c, top));
    let v_tap = c.node_voltage(node_at(&c, tap));
    let v_bot = c.node_voltage(node_at(&c, bottom));
    let i_top = (v_top - v_tap) / 1_000.0;
    let i_bot = (v_tap - v_bot) / 3_000.0;
    assert!((i_top - i_bot).abs() < 1e-12, "series current must match");
    assert!((i_top - 2.5e-3).abs() < 1e-12, "current = {i_top}");
}

/// Wires must merge coincident endpoints rather than consume matrix rows, and their currents
/// must then be reconstructed from the neighbouring elements. A wire is *not* a short: a
/// chain of wires in series with a resistor is an ordinary circuit, not a voltage loop.
#[test]
fn wires_join_nodes_and_carry_current() {
    let (hi, gnd) = (Point::new(0, 40), Point::new(0, 120));
    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(gnd, hi, 5.0)));
    c.add(Box::new(Wire::from_base(Base::new(0, 40, 0, 60, 0, 2))));
    c.add(Box::new(Wire::from_base(Base::new(0, 60, 0, 80, 0, 2))));
    c.add(Box::new(Resistor::from_base(
        Base::new(0, 80, 0, 120, 0, 2),
        1_000.0,
    )));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze()
        .expect("wires in series with a resistor are not a short");
    run(&mut c, 1);

    // 5V through 1k is 5mA; the whole wire chain must report the same current.
    let mut wires = 0;
    for (i, e) in c.elements().iter().enumerate() {
        if e.is_wire() {
            wires += 1;
            assert!(
                (e.current().abs() - 5e-3).abs() < 1e-9,
                "wire {i} current = {}",
                e.current()
            );
        }
    }
    assert_eq!(wires, 2, "both wires should carry current");
    assert!(
        (c.node_voltage(node_at(&c, hi)) - 5.0).abs() < 1e-9,
        "the top of the chain is the supply"
    );
}

/// A wire in a loop with a voltage source and nothing resistive *is* an error, and the
/// voltage-loop check is what has to catch it.
#[test]
fn wire_shorting_a_source_is_a_voltage_loop() {
    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(
        Point::new(0, 120),
        Point::new(0, 40),
        5.0,
    )));
    c.add(Box::new(Wire::from_base(Base::new(0, 40, 0, 120, 0, 2))));

    assert_eq!(c.analyze(), Err(SimError::VoltageLoop));
}

/// An RC step response should charge exponentially toward the supply. The capacitor's
/// companion model only produces this if `start_iteration`/`do_step` are wired correctly, so
/// this is the test for the dynamic path.
#[test]
fn rc_step_response_charges_exponentially() {
    let r = 1_000.0;
    let c_f = 1e-6;
    let tau = r * c_f; // 1ms
    let v = 10.0;
    let (gnd, cap_top) = (Point::new(0, 200), Point::new(0, 120));

    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(
        gnd,
        Point::new(0, 40),
        v,
    )));
    c.add(Box::new(Resistor::between(Point::new(0, 40), cap_top, r)));
    c.add(Box::new(Capacitor::from_base(
        Base::new(0, 120, 0, 200, 0, 2),
        c_f,
    )));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    c.adjust_time_step = false; // keep the timestep fixed so the comparison is exact
                                // Trapezoidal integration is only second-order-accurate while h << RC. At h = RC it
                                // rings, so use h = RC/200 and check against the continuous response.
    c.max_time_step = tau / 200.0;
    c.time_step = tau / 200.0;

    let cap_node = node_at(&c, cap_top);
    let mut samples = Vec::new();
    // 1000 steps of tau/200 is 5 time constants, where the response is within 1% of the
    // supply (1 - 1/e^5 = 0.9933).
    for _ in 0..1000 {
        run(&mut c, 1);
        samples.push((c.time, c.node_voltage(cap_node)));
    }

    // Starts near 0 and rises monotonically toward the supply.
    assert!(samples[0].1 < 0.1, "initial = {}", samples[0].1);
    assert!(
        samples.windows(2).all(|w| w[1].1 >= w[0].1 - 1e-12),
        "voltage must rise monotonically"
    );

    // At t = tau the response is v(1 - 1/e) = 6.32V, and by 2 tau it is within 1% of the
    // supply.
    let near_tau = samples
        .iter()
        .min_by(|a, b| (a.0 - tau).abs().partial_cmp(&(b.0 - tau).abs()).unwrap())
        .unwrap();
    let expected = v * (1.0 - (-near_tau.0 / tau).exp());
    assert!(
        (near_tau.1 - expected).abs() < 0.05 * v,
        "at t={} got {} want ~{}",
        near_tau.0,
        near_tau.1,
        expected
    );
    let final_v = samples.last().unwrap().1;
    assert!(
        (final_v - v).abs() < 0.01 * v,
        "final = {final_v}, want ~{v}"
    );
}

/// A current source needs a closed loop or its current has nowhere to go; in parallel with a
/// resistor to ground it sets the node by Ohm's law. This also checks the current-source
/// stamp's sign, which is easy to get backwards: current flows into post 1.
#[test]
fn current_source_drives_resistor() {
    let (gnd, tap) = (Point::new(0, 80), Point::new(0, 40));
    let mut c = Circuit::new();
    // 1mA pushed into `tap` and returned through ground.
    c.add(Box::new(CurrentSource::from_base(
        Base::new(0, 80, 0, 40, 0, 2),
        1e-3,
    )));
    c.add(Box::new(Resistor::between(tap, gnd, 2_000.0)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    run(&mut c, 1);

    // 1mA * 2k = 2V.
    assert!(
        (c.node_voltage(node_at(&c, tap)) - 2.0).abs() < 1e-9,
        "node = {}",
        c.node_voltage(node_at(&c, tap))
    );
}

/// A diode is the one element here that is genuinely nonlinear, so this is the test that the
/// Newton loop, the convergence flag, and the `gmin` ramp all work. Forward-biasing it
/// through a resistor must land on a realistic ~0.6V drop at a realistic current.
#[test]
fn forward_biased_diode_converges() {
    let (gnd, anode, cathode) = (Point::new(0, 200), Point::new(0, 120), Point::new(0, 160));
    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(
        gnd,
        Point::new(0, 40),
        5.0,
    )));
    c.add(Box::new(Resistor::between(
        Point::new(0, 40),
        anode,
        1_000.0,
    )));
    c.add(Box::new(Diode::from_base(
        Base::new(0, 120, 0, 160, 0, 2),
        &DiodeModel::default(),
        DiodeKind::Plain,
    )));
    c.add(Box::new(Wire::from_base(Base::new(0, 160, 0, 200, 0, 2))));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    assert!(c.circuit_non_linear, "diode must switch on the Newton loop");
    run(&mut c, 20);

    let vd = c.node_voltage(node_at(&c, anode)) - c.node_voltage(node_at(&c, cathode));
    let i = (5.0 - c.node_voltage(node_at(&c, anode))) / 1_000.0;
    // 5V through 1k into a diode: the resistor takes ~4.3V and the diode ~0.6V.
    assert!(vd > 0.4 && vd < 0.8, "forward drop = {vd}");
    assert!(i > 1e-3 && i < 5e-3, "diode current = {i}");
}

/// Reverse-biasing a Zener past its breakdown voltage must clamp the node at `Vz`, which is
/// the whole point of the element. The current is set by the series resistor, not by the
/// diode, so the clamp is what makes the node voltage independent of the supply.
#[test]
fn zener_clamps_at_breakdown_voltage() {
    let (gnd, top, cathode) = (Point::new(0, 200), Point::new(0, 40), Point::new(0, 160));
    let vz = 5.6;

    let mut c = Circuit::new();
    // 12V is well past Vz, so the Zener is firmly in breakdown.
    c.add(Box::new(VoltageSource::dc_between(gnd, top, 12.0)));
    c.add(Box::new(Resistor::between(top, cathode, 1_000.0)));
    // Post 0 is the anode, so anode to ground and cathode to the resistor is reverse bias.
    c.add(Box::new(Diode::zener(gnd, cathode, 0, vz)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    run(&mut c, 20);

    let v = c.node_voltage(node_at(&c, cathode));
    assert!((v - vz).abs() < 0.05, "clamped node = {v}, expected {vz}");
    // (12 - 5.6) / 1k = 6.4 mA, all of it through the Zener.
    let i = (12.0 - v) / 1_000.0;
    assert!(i > 5e-3 && i < 8e-3, "zener current = {i}");
}

/// Below its breakdown voltage a Zener is just a leaky reverse-biased diode, so the node
/// must *not* be pinned to Vz. Without this the clamp test above would pass even for an
/// element that always forced Vz.
#[test]
fn zener_does_not_conduct_below_breakdown() {
    let (gnd, top, cathode) = (Point::new(0, 200), Point::new(0, 40), Point::new(0, 160));

    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(gnd, top, 2.0)));
    c.add(Box::new(Resistor::between(top, cathode, 1_000.0)));
    c.add(Box::new(Diode::zener(gnd, cathode, 0, 5.6)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    run(&mut c, 20);

    // 2V reverse across a 5.6V Zener: only leakage flows, so the node sits at the supply.
    let v = c.node_voltage(node_at(&c, cathode));
    assert!(v > 1.9, "node = {v}, Zener conducted below Vz");
}

/// An LED's series resistance is what makes it a 3-node element, and it has to show up in the
/// solution: the junction drop plus the resistor drop must add up to the supply.
#[test]
fn led_drops_its_forward_voltage() {
    let (gnd, top, cathode) = (Point::new(0, 200), Point::new(0, 40), Point::new(0, 160));

    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(gnd, top, 5.0)));
    c.add(Box::new(Resistor::between(top, cathode, 100.0)));
    c.add(Box::new(Diode::led(cathode, gnd, 0)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    run(&mut c, 20);

    let v = c.node_voltage(node_at(&c, cathode));
    // default-led is a ~2.2 V part; 5V through 100 ohm leaves plenty of headroom.
    assert!(v > 1.8 && v < 2.6, "LED drop = {v}");
    let i = (5.0 - v) / 100.0;
    assert!(i > 0.01 && i < 0.04, "LED current = {i}");
}

/// A non-inverting amplifier must produce `gain * (V+ - V-)`, which for a 100k gain and a
/// 1 mV difference is 100 V — far past the 15 V rail. So this is really a test that the
/// clamp engages: the output must sit at the rail, not run away.
#[test]
fn opamp_saturates_at_its_output_rail() {
    let (gnd, out) = (Point::new(0, 200), Point::new(0, 0));
    let opa = OpAmp::new(Point::new(0, 100), out, 0, 15.0, -15.0, 100_000.0);
    // The input posts are offset from the body, so read them off the element.
    let noninv = Element::post(&opa, 1);

    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(gnd, noninv, 0.001)));
    c.add(Box::new(opa));
    // A load so the output has somewhere to drive current.
    c.add(Box::new(Resistor::between(out, gnd, 1_000.0)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    assert!(c.circuit_non_linear, "op-amp must be nonlinear");
    run(&mut c, 20);

    let v = c.node_voltage(node_at(&c, out));
    assert!(
        (v - 15.0).abs() < 0.05,
        "output = {v}, expected the +15 V rail"
    );
}

/// With negative feedback the op-amp instead holds `V-` at `V+`, so a 1 V reference on the
/// non-inverting input must appear on the inverting input and across the feedback network.
#[test]
fn opamp_with_feedback_holds_the_inputs_equal() {
    let (gnd, out) = (Point::new(0, 200), Point::new(0, 80));
    let opa = OpAmp::new(Point::new(0, 100), out, 0, 15.0, -15.0, 100_000.0);
    let inv_post = Element::post(&opa, 0);
    let noninv_post = Element::post(&opa, 1);

    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(gnd, noninv_post, 1.0)));
    c.add(Box::new(opa));
    // Feedback from the output to the inverting input, and the inverting input to ground.
    c.add(Box::new(Resistor::between(out, inv_post, 10_000.0)));
    c.add(Box::new(Resistor::between(inv_post, gnd, 10_000.0)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    run(&mut c, 20);

    // The op-amp drives the output until V- == V+, so the inverting node sits at 1 V and the
    // output at 2 V (a non-inverting amplifier with gain 2).
    let v_inv = c.node_voltage(node_at(&c, inv_post));
    let v_out = c.node_voltage(node_at(&c, out));
    assert!((v_inv - 1.0).abs() < 1e-3, "V- = {v_inv}");
    assert!((v_out - 2.0).abs() < 1e-3, "Vout = {v_out}");
}

/// A potentiometer is a voltage divider whose tap position is set by `position`, so moving
/// the wiper must move the output monotonically between the two rails.
#[test]
fn potentiometer_divides_between_the_rails() {
    let (gnd, top) = (Point::new(0, 200), Point::new(0, 0));

    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(gnd, top, 10.0)));
    // The pot spans the supply, so the wiper is a real divider output rather than a node
    // left floating by an unconnected bottom rail.
    c.add(Box::new(Pot::new(top, gnd, 0, 1_000.0, 0.25)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    run(&mut c, 5);

    // The wiper post is derived, so read it off the element rather than hard-coding it.
    let w = Element::post(c.elements()[1].as_ref(), 2);
    let v = c.node_voltage(node_at(&c, w));
    // position 0.25 puts 25% of the resistance above the wiper and 75% below it, so the
    // wiper sits 75% of the way up from ground: 10 * 0.75 = 7.5 V.
    assert!((v - 7.5).abs() < 1e-6, "wiper = {v}");

    // Moving the wiper toward the top rail must lower the voltage.
    let pot = c.elements_mut()[1]
        .as_any_mut()
        .downcast_mut::<Pot>()
        .expect("pot");
    pot.set_position(0.75);
    c.reanalyze().expect("reanalyze");
    run(&mut c, 5);
    let v2 = c.node_voltage(node_at(&c, w));
    assert!((v2 - 2.5).abs() < 1e-6, "wiper at 0.75 = {v2}");
}

/// An analog switch is a resistor whose value follows the control voltage, so driving the
/// control terminal must change the load current by orders of magnitude.
#[test]
fn analog_switch_follows_its_control_voltage() {
    let (gnd, top, load) = (Point::new(0, 200), Point::new(0, 40), Point::new(0, 120));
    let sw = AnalogSwitch::new(top, load, 0, 20.0, 1e10);
    let control = Element::post(&sw, 2);

    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(gnd, top, 5.0)));
    c.add(Box::new(sw));
    c.add(Box::new(Resistor::between(load, gnd, 100.0)));
    // A second source sets the control terminal, which is a sense node and carries no current.
    c.add(Box::new(VoltageSource::dc_between(gnd, control, 0.0)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    assert!(c.circuit_non_linear, "analog switch must be nonlinear");
    run(&mut c, 5);
    // Control at 0 V is below the 2.5 V threshold, so the switch is open: almost no current.
    let i_open = c.node_voltage(node_at(&c, load)) / 100.0;
    assert!(i_open < 1e-6, "open current = {i_open}");

    // Raising the control above the threshold closes the switch through 20 ohms, so the
    // current is set by 20 + 100 ohms across 5 V.
    c.elements_mut()[3]
        .as_any_mut()
        .downcast_mut::<VoltageSource>()
        .expect("control source")
        .set_max_voltage(5.0);
    c.reanalyze().expect("reanalyze");
    run(&mut c, 5);
    let i_closed = (5.0 - c.node_voltage(node_at(&c, load))) / 20.0;
    assert!(i_closed > 0.03, "closed current = {i_closed}");
    assert!(i_closed > 100.0 * i_open, "switch did not turn on");
}

/// An SPDT switch routes the pole to one throw or the other, so moving it must move the
/// output between two loads while the unselected load is left floating.
#[test]
fn spdt_switch_routes_between_throws() {
    let (gnd, pole) = (Point::new(0, 200), Point::new(0, 40));
    // The throw posts are derived from the geometry, so read them off the element rather
    // than guessing coordinates that happen to be near.
    let spdt = SpdtSwitch::between(pole, Point::new(100, 50));
    let throw0 = Element::post(&spdt, 1);
    let throw1 = Element::post(&spdt, 2);

    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(gnd, pole, 5.0)));
    c.add(Box::new(spdt));
    c.add(Box::new(Resistor::between(throw0, gnd, 1_000.0)));
    c.add(Box::new(Resistor::between(throw1, gnd, 1_000.0)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    run(&mut c, 5);
    // Position 0 connects the pole to throw 0.
    assert!(
        (c.node_voltage(node_at(&c, throw0)) - 5.0).abs() < 1e-9,
        "throw0 = {}",
        c.node_voltage(node_at(&c, throw0))
    );
    assert!(
        c.node_voltage(node_at(&c, throw1)).abs() < 1e-9,
        "throw1 = {}",
        c.node_voltage(node_at(&c, throw1))
    );

    c.elements_mut()[1]
        .as_any_mut()
        .downcast_mut::<SpdtSwitch>()
        .expect("spdt")
        .toggle();
    c.reanalyze().expect("reanalyze");
    run(&mut c, 5);
    assert!(c.node_voltage(node_at(&c, throw0)).abs() < 1e-9);
    assert!((c.node_voltage(node_at(&c, throw1)) - 5.0).abs() < 1e-9);
}

/// A closed switch is a 0 V source, so it must carry the load current with no voltage drop.
/// Opening it has to leave the far side of the switch as an island with no path to ground,
/// which is what the unconnected-node search exists to catch.
#[test]
fn switch_closed_conducts_and_open_disconnects() {
    let (gnd, top, mid, far) = (
        Point::new(0, 200),
        Point::new(0, 40),
        Point::new(0, 120),
        Point::new(0, 160),
    );

    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(gnd, top, 5.0)));
    c.add(Box::new(Switch::closed(top, mid)));
    // `far` is reachable only through `mid`, so it floats the moment the switch opens.
    c.add(Box::new(Resistor::between(mid, far, 1_000.0)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    run(&mut c, 5);
    // The switch is a 0 V source, so `mid` sits at the supply and `far` follows it.
    assert!(
        (c.node_voltage(node_at(&c, mid)) - 5.0).abs() < 1e-9,
        "closed switch dropped voltage: {}",
        c.node_voltage(node_at(&c, mid))
    );
    assert!((c.node_voltage(node_at(&c, far)) - 5.0).abs() < 1e-9);

    // Opening the switch removes the only feed to the mid/far island, which the engine then
    // ties to ground through 1e8 ohms. Both nodes collapse to 0 V.
    c.elements_mut()[1]
        .as_any_mut()
        .downcast_mut::<Switch>()
        .expect("switch")
        .toggle();
    c.reanalyze().expect("reanalyze");
    run(&mut c, 5);
    let mid_node = node_at(&c, mid);
    assert!(
        c.node_voltage(mid_node).abs() < 1e-3,
        "island voltage = {}",
        c.node_voltage(mid_node)
    );
    assert!(c.node_voltage(node_at(&c, far)).abs() < 1e-3);
}

/// A parallel LC tank with a pre-charged capacitor should oscillate rather than settle,
/// proving the inductor's companion model and the trapezoidal rule are both live.
#[test]
fn lc_tank_oscillates() {
    let (gnd, cap_top) = (Point::new(0, 200), Point::new(0, 40));
    let (l, c_f) = (1e-3_f64, 1e-6_f64);
    // Light damping: Q = R*sqrt(C/L), so R = 100k gives Q = 100 and the tank rings for
    // roughly a hundred cycles. Too small an R and it is critically damped and never
    // oscillates at all, which would make this test pass for the wrong reason.
    let r = 1e5;

    let mut c = Circuit::new();
    // C, L and R all in parallel between the tank node and ground. A dangling plate on
    // either reactive element would leave its far node unconnected, and the engine would
    // quietly tie that island to ground through a 1e8 resistor — which is enough to spoil
    // both the initial condition and the period.
    c.add(Box::new(Capacitor::from_base(
        Base::new(0, 40, 0, 200, 0, 2),
        c_f,
    )));
    c.add(Box::new(Inductor::from_base(
        Base::new(0, 40, 0, 200, 0, 2),
        l,
        0.0,
    )));
    c.add(Box::new(Resistor::between(cap_top, gnd, r)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    c.adjust_time_step = false;
    let period = 2.0 * std::f64::consts::PI * (l * c_f).sqrt();
    c.max_time_step = period / 400.0;
    c.time_step = period / 400.0;

    let node = node_at(&c, cap_top);
    let samples: Vec<f64> = (0..400)
        .map(|_| {
            run(&mut c, 1);
            c.node_voltage(node)
        })
        .collect();

    // The capacitor starts at `initial_voltage` (1mV) and the tank swings symmetrically
    // about ground.
    let max = samples.iter().copied().fold(f64::MIN, f64::max);
    let min = samples.iter().copied().fold(f64::MAX, f64::min);
    assert!(max > 5e-4, "tank should swing positive, peak = {max}");
    assert!(min < -5e-4, "tank should swing negative, trough = {min}");

    // One period later it should be back near the start, which only holds if the period is
    // the one the L and C predict.
    let again = (c.time / period).round() as usize;
    let expected = samples[0];
    let actual = samples[again.min(samples.len() - 1)];
    assert!(
        (actual - expected).abs() < 0.2 * 1e-3,
        "after {:.0} periods: {actual} vs {expected}",
        c.time / period
    );
}

/// A voltage source loop with no resistance anywhere has no defined solution, and the engine
/// must reject it rather than produce garbage.
#[test]
fn voltage_source_loop_is_rejected() {
    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(
        Point::new(0, 0),
        Point::new(0, 40),
        5.0,
    )));
    // A second source closing a loop with the first, with nothing resistive in between.
    c.add(Box::new(VoltageSource::dc_between(
        Point::new(0, 40),
        Point::new(0, 0),
        5.0,
    )));

    assert_eq!(c.analyze(), Err(SimError::VoltageLoop));
}

/// A shorted capacitor must be reported so the element can drop its stored voltage, which is
/// what stops it ringing forever.
#[test]
fn shorted_capacitor_is_flagged() {
    let mut c = Circuit::new();
    c.add(Box::new(Capacitor::from_base(
        Base::new(0, 0, 0, 40, 0, 2),
        1e-6,
    )));
    c.add(Box::new(Wire::from_base(Base::new(0, 40, 0, 0, 0, 2))));
    c.add(Box::new(Ground::at(0, 0)));

    // Analysis must complete; the short path through a wire is what triggers the flag.
    c.analyze()
        .expect("a shorted cap is repaired, not rejected");
}

/// Time must advance by exactly the timestep, and a linear circuit should never need a
/// Newton sub-iteration.
#[test]
fn timestep_bookkeeping() {
    let (gnd, tap) = (Point::new(0, 80), Point::new(0, 40));
    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::dc_between(
        gnd,
        Point::new(0, 0),
        1.0,
    )));
    c.add(Box::new(Resistor::between(Point::new(0, 0), tap, 1_000.0)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    c.adjust_time_step = false;
    let dt = c.time_step;
    run(&mut c, 5);
    assert!((c.time - 5.0 * dt).abs() < 1e-15, "time = {}", c.time);
    assert_eq!(c.steps, 5);
    assert_eq!(
        c.sub_iterations, 0,
        "a linear circuit should need no sub-iteration"
    );
}

/// A time-varying source has to actually vary with time.
///
/// The engine only refreshes `SimContext` when it re-stamps, which happens on a topology or
/// timestep change rather than on every step. A source that reads the time from there would
/// emit the value it had at the last re-stamp forever, and for a sine starting at zero phase
/// that is silence — so this test pins the waveform, not just the first step.
#[test]
fn a_time_varying_source_actually_varies() {
    let (gnd, top) = (Point::new(0, 0), Point::new(0, 100));
    let mut c = Circuit::new();
    c.add(Box::new(VoltageSource::from_base(
        Base::from_points(gnd, top, 0, 2),
        Waveform::Ac,
        1_000.0,
        10.0,
        0.0,
        0.0,
        0.5,
        false,
    )));
    c.add(Box::new(Resistor::between(top, gnd, 1_000.0)));
    c.add(Box::new(Ground::at_point(gnd)));

    c.analyze().expect("analyze");
    c.adjust_time_step = false;
    c.max_time_step = 1e-6;
    c.time_step = 1e-6;
    c.stamp_circuit().expect("restamp at the new timestep");

    let node = node_at(&c, top);
    // A whole period at 1 kHz is 1 ms, so 2000 steps of 1 us covers it several times over.
    let mut peak = 0.0f64;
    let mut sum = 0.0;
    let samples = 2_000;
    for _ in 0..samples {
        c.step().expect("step");
        let v = c.node_voltage(node);
        peak = peak.max(v.abs());
        sum += v;
    }
    assert!(peak > 9.9, "peak = {peak}, expected the 10 V amplitude");
    // Half a period is symmetric about zero, so the mean of a whole number of periods is 0.
    assert!(
        (sum / samples as f64).abs() < 0.2,
        "mean = {}, expected a symmetric swing",
        sum / samples as f64
    );
}
