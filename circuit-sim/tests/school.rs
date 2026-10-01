use circuit_sim::{Ammeter, Circuit, Ground, Point, PowerSupply, Resistor, Voltmeter};

#[test]
fn school_level_supply_and_meters() {
    let (positive, after_meter, ground) = (Point::new(0, 0), Point::new(0, 40), Point::new(0, 80));
    let mut circuit = Circuit::new();
    circuit.add(Box::new(PowerSupply::dc_between(ground, positive, 6.0)));
    circuit.add(Box::new(Ammeter::between(positive, after_meter)));
    circuit.add(Box::new(Resistor::between(after_meter, ground, 2_000.0)));
    circuit.add(Box::new(Ground::at_point(ground)));

    circuit.analyze().expect("analyze");
    circuit.step().expect("step");

    let meter = circuit.elements()[1]
        .as_any()
        .downcast_ref::<Ammeter>()
        .expect("ammeter");
    assert!(
        (meter.amperes() - 3e-3).abs() < 1e-12,
        "meter current = {}",
        meter.amperes()
    );

    // 6 V across 2 k is 3 mA, so the meter and the voltmeter have to agree.
    let voltmeter = Voltmeter::between(after_meter, ground);
    assert!((voltmeter.read(&circuit).expect("voltage") - 6.0).abs() < 1e-12);
}

#[test]
fn instruments_report_unconnected_points() {
    let circuit = Circuit::new();
    let meter = Voltmeter::between(Point::new(0, 0), Point::new(0, 40));
    assert!(meter.read(&circuit).is_err());
}
