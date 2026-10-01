use circuit_sim::{Point, Waveform};

pub(crate) const DEFAULT_SPAN: i32 = 3;

/// A span with a whole number of cells in the middle of it, for the parts that put a terminal
/// there. See [`PartKind::placement_span`].
pub(crate) const EVEN_SPAN: i32 = 4;

/// A part the user can place on the canvas.
///
/// `value` and `value2` are the two user-visible numbers a part can carry — an amplitude
/// and a frequency, a resistance and a rating, two turn counts. Which of the two a part
/// actually uses, and what it means, is decided by `label` and `validate`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PartKind {
    Supply,
    AcSupply,
    SignalSource,
    CurrentSource,
    Resistor,
    Capacitor,
    Inductor,
    Diode,
    Transformer,
    Transistor,
    OpAmp,
    Comparator,
    Switch,
    SpdtSwitch,
    AnalogSwitch,
    Pot,
    Wire,
    Ground,
    Voltmeter,
    Ammeter,
}

/// Which shelf of the palette a part sits on.
///
/// Grouped by what the part *is* rather than alphabetically, because a palette of nineteen
/// items in one list is a list nobody scans. The split follows the usual textbook one, with
/// the parts that are neither plainly passive nor plainly active — the ones you operate by
/// hand — given a shelf of their own.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Group {
    /// 无源器件: store and shape energy.
    Passive,
    /// 有源器件: supply it, or amplify or switch it.
    Active,
    /// 开关与调节: parts the user operates, or whose behaviour a number sets.
    Adjustable,
    /// 仪表: observation rather than circuit.
    Instrument,
}

impl Group {
    pub(crate) const ALL: [Self; 4] = [
        Self::Passive,
        Self::Active,
        Self::Adjustable,
        Self::Instrument,
    ];

    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::Passive => "无源器件",
            Self::Active => "有源器件",
            Self::Adjustable => "开关与调节",
            Self::Instrument => "仪表",
        }
    }
}

impl PartKind {
    /// Which shelf of the palette this part belongs on.
    pub(crate) fn group(self) -> Group {
        match self {
            Self::Resistor
            | Self::Capacitor
            | Self::Inductor
            | Self::Transformer
            | Self::Diode
            | Self::Wire
            | Self::Ground => Group::Passive,
            Self::Supply
            | Self::AcSupply
            | Self::SignalSource
            | Self::CurrentSource
            | Self::Transistor
            | Self::OpAmp
            | Self::Comparator => Group::Active,
            Self::Switch | Self::SpdtSwitch | Self::AnalogSwitch | Self::Pot => Group::Adjustable,
            Self::Voltmeter | Self::Ammeter => Group::Instrument,
        }
    }

    /// Every placeable part, in palette order.
    ///
    /// Grouped, because this is what the palette walks; the group decides the heading, so the
    /// order here only has to be sensible within a shelf.
    pub(crate) const ALL: [Self; 20] = [
        Self::Resistor,
        Self::Capacitor,
        Self::Inductor,
        Self::Transformer,
        Self::Diode,
        Self::Wire,
        Self::Ground,
        Self::Supply,
        Self::AcSupply,
        Self::SignalSource,
        Self::CurrentSource,
        Self::Transistor,
        Self::OpAmp,
        Self::Comparator,
        Self::Switch,
        Self::SpdtSwitch,
        Self::AnalogSwitch,
        Self::Pot,
        Self::Voltmeter,
        Self::Ammeter,
    ];

    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::Supply => "直流电源",
            Self::AcSupply => "交流电源",
            Self::SignalSource => "函数信号源",
            Self::CurrentSource => "恒流源",
            Self::Resistor => "电阻",
            Self::Capacitor => "电容",
            Self::Inductor => "电感",
            Self::Diode => "二极管",
            Self::Transformer => "变压器",
            Self::Transistor => "三极管",
            Self::OpAmp => "运放",
            Self::Comparator => "比较器",
            Self::Switch => "开关",
            Self::SpdtSwitch => "双掷开关",
            Self::AnalogSwitch => "压控开关",
            Self::Pot => "滑动变阻器",
            Self::Wire => "导线",
            Self::Ground => "接地",
            Self::Voltmeter => "电压探针",
            Self::Ammeter => "电流表",
        }
    }

    /// Also what the palette search matches, so an English query still finds a part.
    pub(crate) fn keywords(self) -> &'static str {
        match self {
            Self::Supply => "battery dc supply",
            Self::AcSupply => "ac sine acsource waveform",
            Self::SignalSource => "signal function expression generator",
            Self::CurrentSource => "current isource",
            Self::Resistor => "resistance",
            Self::Capacitor => "capacitance",
            Self::Inductor => "inductance coil",
            Self::Diode => "rectifier",
            Self::Transformer => "transformer coil turns",
            Self::Transistor => "bjt npn transistor",
            Self::OpAmp => "opamp amplifier op-amp",
            Self::Comparator => "comparator cmp",
            Self::Switch => "switch spst toggle",
            Self::SpdtSwitch => "spdt switch2 throw selector",
            Self::AnalogSwitch => "analog switch voltage controlled",
            Self::Pot => "rheostat variable resistor slider",
            Self::Wire => "wire link",
            Self::Ground => "ground earth",
            Self::Voltmeter => "voltmeter probe",
            Self::Ammeter => "ammeter",
        }
    }

    pub(crate) fn code(self) -> &'static str {
        match self {
            Self::Supply => "supply",
            Self::AcSupply => "acsupply",
            Self::SignalSource => "signalsource",
            Self::CurrentSource => "isource",
            Self::Resistor => "resistor",
            Self::Capacitor => "capacitor",
            Self::Inductor => "inductor",
            Self::Diode => "diode",
            Self::Transformer => "transformer",
            Self::Transistor => "transistor",
            Self::OpAmp => "opamp",
            Self::Comparator => "comparator",
            Self::Switch => "switch",
            Self::SpdtSwitch => "switch2",
            Self::AnalogSwitch => "aswitch",
            Self::Pot => "pot",
            Self::Wire => "wire",
            Self::Ground => "ground",
            Self::Voltmeter => "voltmeter",
            Self::Ammeter => "ammeter",
        }
    }

    pub(crate) fn from_code(code: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|kind| kind.code() == code)
    }

    /// The part's first user value, or zero for a part that has none.
    ///
    /// For the two switches this is not a quantity but a state: zero is open or thrown to the
    /// first contact, one is closed or thrown to the second. The property dialog offers a
    /// button for those rather than a spinner, so the number never reaches the screen.
    pub(crate) fn default_value(self) -> f64 {
        match self {
            Self::Supply | Self::AcSupply => 5.0,
            Self::CurrentSource => 0.01,
            Self::Resistor => 1_000.0,
            Self::Capacitor => 1e-6,
            Self::Inductor => 0.1,
            Self::Diode => 0.7,
            Self::Transformer => 10.0,
            Self::Transistor => 100.0,
            Self::OpAmp => 1_000.0,
            // Closed by default, so a switch the user has just dropped into a circuit is
            // conducting and the thing they built actually does something.
            Self::Switch => 1.0,
            Self::SpdtSwitch => 0.0,
            Self::AnalogSwitch => 20.0,
            Self::Pot => 10_000.0,
            _ => 0.0,
        }
    }

    /// The part's second user value, or zero for a part that has none: a source's frequency,
    /// a transformer's secondary turns, a rheostat's slider position, or a
    /// voltage-controlled switch's off-resistance.
    pub(crate) fn default_value2(self) -> f64 {
        match self {
            Self::AcSupply => 50.0,
            Self::Transformer => 1.0,
            // The slider starts halfway along the rheostat track.
            Self::Pot => 0.5,
            // Off until its control is driven, so the off-resistance is the large number and
            // the on-resistance above is the small one.
            Self::AnalogSwitch => 1e9,
            _ => 0.0,
        }
    }

    /// The shape a source starts out producing. Only the a.c. source is ever anything but a
    /// sine; everything else ignores this.
    pub(crate) fn default_waveform(self) -> Waveform {
        match self {
            Self::Supply => Waveform::Dc,
            _ => Waveform::Ac,
        }
    }

    /// Whether the waveform and expression apply to this part at all.
    pub(crate) fn has_waveform(self) -> bool {
        self == Self::AcSupply
    }

    /// Whether the property dialog should offer a spinner for `value`.
    ///
    /// Explicit rather than derived from the default being non-zero, because the two switches
    /// have a meaningful first value that is a state rather than a quantity and must not be
    /// given a spinner.
    pub(crate) fn has_value(self) -> bool {
        match self {
            Self::Switch | Self::SpdtSwitch | Self::Comparator => false,
            _ => self.default_value() != 0.0,
        }
    }

    /// Whether the property dialog should offer a spinner for `value2`.
    pub(crate) fn has_value2(self) -> bool {
        matches!(
            self,
            Self::AcSupply | Self::Transformer | Self::Pot | Self::AnalogSwitch
        )
    }

    /// The two positions a switch can be in, as the labels for each.
    ///
    /// `None` for a part with no switch, which is how the property dialog decides to leave the
    /// spinner out: a switch's position is a state, not a number, so it gets a button. The two
    /// positions are always zero and one for both kinds of switch, which is what lets a single
    /// `1.0 - value` flip either of them.
    pub(crate) fn switch_labels(self) -> Option<(&'static str, &'static str)> {
        match self {
            // Closed reads as one, open as zero.
            Self::Switch => Some(("闭合", "断开")),
            // A double-throw switch has no open position, only a choice of two contacts.
            Self::SpdtSwitch => Some(("掷向 2", "掷向 1")),
            _ => None,
        }
    }

    /// Whether the part occupies a span at all. A ground and a voltage probe have one terminal.
    pub(crate) fn has_two_posts(self) -> bool {
        !matches!(self, Self::Ground | Self::Voltmeter)
    }

    /// How the part is laid out when first placed.
    ///
    /// A transformer's span runs along its windings. A transistor's runs along its *base lead*,
    /// because the collector and emitter are placed either side of the far end of it — the
    /// orientation that puts the base on the left and the collector on top, which is how the
    /// symbol is drawn. Both are taller than they are wide on screen, but for opposite reasons.
    ///
    /// The rheostat and voltage-controlled switch get an even span so the symbol's central
    /// slider/control point lands on a grid cell.
    pub(crate) fn placement_span(self) -> Point {
        match self {
            Self::Transformer => Point::new(0, DEFAULT_SPAN),
            Self::Pot | Self::AnalogSwitch => Point::new(EVEN_SPAN, 0),
            _ => Point::new(DEFAULT_SPAN, 0),
        }
    }

    /// Rejects values the engine cannot use, so a bad edit is caught before the circuit is
    /// handed over. An amplitude of zero or a negative resistance has no physical meaning
    /// and would either stall the solver or divide by zero.
    pub(crate) fn validate(self, value: f64, value2: f64) -> Result<(), String> {
        let positive = match self {
            Self::Supply
            | Self::Resistor
            | Self::Capacitor
            | Self::Inductor
            | Self::Diode
            | Self::Transformer
            | Self::Transistor
            | Self::OpAmp => value > 0.0,
            Self::AcSupply => value > 0.0 && value2 > 0.0,
            // A current source has one number and no second value, so asking for a positive
            // frequency it does not have rejected every current source the user could place.
            Self::CurrentSource => value > 0.0,
            Self::AnalogSwitch => value > 0.0 && value2 > 0.0,
            // The rheostat slider is a fraction of its travel, so it is the one value that is
            // allowed to reach zero — that is the end of the track, not a mistake.
            Self::Pot => value > 0.0 && (0.0..=1.0).contains(&value2),
            _ => true,
        };
        if value.is_finite() && value2.is_finite() && positive {
            Ok(())
        } else {
            Err(format!("{} 的数值必须是有效的正数", self.name()))
        }
    }

    /// Unit of `value`, for the property dialog.
    pub(crate) fn unit(self) -> &'static str {
        match self {
            Self::Supply | Self::AcSupply | Self::Diode => "V",
            Self::CurrentSource => "A",
            Self::Resistor => "Ω",
            Self::Capacitor => "F",
            Self::Inductor => "H",
            Self::OpAmp => "",
            Self::AnalogSwitch | Self::Pot => "Ω",
            _ => "",
        }
    }

    /// Unit of `value2`, for the property dialog.
    pub(crate) fn unit2(self) -> &'static str {
        match self {
            Self::AcSupply => "Hz",
            Self::Transformer => "匝",
            Self::Pot => "0–1",
            Self::AnalogSwitch => "Ω",
            _ => "",
        }
    }

    /// What the second value is called, since a frequency and a slider position are not both
    /// numbers of something.
    pub(crate) fn value2_label(self) -> &'static str {
        match self {
            Self::AcSupply => "频率",
            Self::Transformer => "副边匝数",
            Self::Pot => "滑块位置",
            Self::AnalogSwitch => "关断电阻",
            _ => "数值2",
        }
    }

    /// How fast the property dialog should drag, so a 50 Hz frequency and a 1 nF
    /// capacitance are both comfortable to adjust.
    pub(crate) fn value_speed(self) -> f64 {
        match self {
            Self::Supply | Self::AcSupply | Self::Diode => 0.1,
            Self::Transformer | Self::Transistor => 1.0,
            // A gain and a current source both want fine control near their useful values.
            Self::OpAmp | Self::CurrentSource => 0.001,
            // A wiper is a fraction, so it wants a slow drag too.
            Self::Pot => 0.01,
            _ => 10.0,
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct Part {
    pub(crate) kind: PartKind,
    /// First terminal. For a two-terminal part this is the one end of its axis.
    pub(crate) a: Point,
    /// Second terminal, for a two-terminal part.
    pub(crate) b: Point,
    pub(crate) value: f64,
    pub(crate) value2: f64,
    /// The shape of a source's output. Only a source uses it; every other part leaves it at
    /// the default.
    pub(crate) waveform: Waveform,
    /// A custom waveform written as an expression in the simulation time `t`, used only when
    /// `waveform` is `Custom`. Held as text rather than as a parsed tree so the property
    /// dialog can show back exactly what the user typed, spaces and all.
    pub(crate) expression: String,
}

/// The waveforms a built-in source can be set to, in the order the picker offers them.
///
/// `Dc` is absent because the d.c. source already *is* that, and `Noise` and `Var` because a
/// waveform nobody can reproduce makes a circuit that cannot be reasoned about — which is the
/// opposite of what a teaching tool is for.
pub(crate) const WAVEFORMS: [Waveform; 5] = [
    Waveform::Ac,
    Waveform::Square,
    Waveform::Triangle,
    Waveform::Sawtooth,
    Waveform::Pulse,
];

/// A waveform's name, as the picker shows it.
pub(crate) fn waveform_name(waveform: Waveform) -> &'static str {
    match waveform {
        Waveform::Dc => "直流",
        Waveform::Ac => "正弦波",
        Waveform::Square => "方波",
        Waveform::Triangle => "三角波",
        Waveform::Sawtooth => "锯齿波",
        Waveform::Pulse => "脉冲",
        Waveform::Noise => "噪声",
        Waveform::Var => "可变",
        Waveform::Custom => "自定义",
    }
}

impl Part {
    pub(crate) fn at(kind: PartKind, a: Point) -> Self {
        let span = kind.placement_span();
        let b = if kind.has_two_posts() {
            Point::new(a.x + span.x, a.y + span.y)
        } else {
            a
        };
        Self {
            kind,
            a,
            b,
            value: kind.default_value(),
            value2: kind.default_value2(),
            waveform: kind.default_waveform(),
            expression: String::new(),
        }
    }

    /// A part of this kind spanning two points, with both values at their defaults.
    pub(crate) fn between(kind: PartKind, a: Point, b: Point) -> Self {
        Self {
            kind,
            a,
            b,
            value: kind.default_value(),
            value2: kind.default_value2(),
            waveform: kind.default_waveform(),
            expression: String::new(),
        }
    }

    pub(crate) fn translated(&self, dx: i32, dy: i32) -> Self {
        Self {
            a: Point::new(self.a.x + dx, self.a.y + dy),
            b: Point::new(self.b.x + dx, self.b.y + dy),
            ..self.clone()
        }
    }

    /// Every connection terminal, in the order the engine's element expects them.
    ///
    /// The engine identifies a node purely by the coordinates of a post that lands on it, so
    /// this list is the whole of a part's connectivity: anything missing here is a terminal the
    /// user cannot see, click or wire to.
    ///
    /// The four- and five-terminal parts do **not** derive their extra posts here. They ask the
    /// engine element for them, through the same `setPoints` ports the element itself uses.
    /// That is deliberate: re-deriving a wiper or a pair of op-amp inputs here would be a
    /// second implementation of the same geometry, and the two would drift apart the moment one
    /// of them was fixed.
    pub(crate) fn posts(&self) -> Vec<Point> {
        use circuit_sim::{AnalogSwitch, Comparator, OpAmp, SpdtSwitch};
        // The four derived layouts below work in engine units, so the span goes in that way
        // and the answers come back out. Getting this wrong puts an op-amp's inputs sixteen
        // grid cells off its body rather than one, which looks like a part with terminals
        // scattered across the canvas.
        let (a, b) = (to_engine_point(self.a), to_engine_point(self.b));
        match self.kind {
            // A ground is a single reference point, and the instruments measure between
            // two of their own so both ends are terminals.
            PartKind::Ground | PartKind::Voltmeter => vec![self.a],
            PartKind::Transformer => {
                let (s1, s2) = circuit_sim::Transformer::secondary_posts(self.a, self.b);
                vec![self.a, self.b, s1, s2]
            }
            PartKind::Transistor => {
                // A transistor's three terminals are clustered at one end of its span rather
                // than spread along it, which is the arrangement its symbol is drawn around.
                // `a` is the base; the collector and emitter sit at `b`, either side of the
                // axis. Port of `TransistorElm.setPoints`, which interpolates the collector
                // and emitter at the far end with a perpendicular offset and puts the base on
                // the axis behind them.
                let (ux, uy) = unit_axis(self.a, self.b);
                let (px, py) = (-uy * TRANSISTOR_ARM, ux * TRANSISTOR_ARM);
                vec![
                    self.a,
                    Point::new(self.b.x - px, self.b.y - py),
                    Point::new(self.b.x + px, self.b.y + py),
                ]
            }
            PartKind::Pot => vec![self.a, self.b],
            // The pole and the two throws, and *not* `b`. A double-throw switch's second point is only
            // the reference the throws are derived from — `Switch2Elm.getPostCount` returns
            // `1 + throwCount` and `point2` is not one of them — so listing it here would
            // offer a terminal that is not connected to anything.
            PartKind::SpdtSwitch => {
                let mut posts = vec![self.a];
                posts.extend(
                    SpdtSwitch::throw_post_layout(a, b, 2)
                        .into_iter()
                        .map(from_engine_point),
                );
                posts
            }
            // The control terminal is the midpoint pushed off the body, per
            // `AnalogSwitchElm.point3`.
            PartKind::AnalogSwitch => {
                vec![
                    self.a,
                    self.b,
                    from_engine_point(AnalogSwitch::control_post(a, b)),
                ]
            }
            // Both amplifiers put their two inputs either side of the body at the `point1`
            // end, which is the same layout in each.
            PartKind::OpAmp => {
                let (inverting, non_inverting) = OpAmp::input_posts(a, b);
                vec![
                    from_engine_point(inverting),
                    from_engine_point(non_inverting),
                    self.b,
                ]
            }
            PartKind::Comparator => {
                let (inverting, non_inverting) = Comparator::input_posts(a, b);
                vec![
                    from_engine_point(inverting),
                    from_engine_point(non_inverting),
                    self.b,
                ]
            }
            _ => vec![self.a, self.b],
        }
    }

    /// The terminal pairs a current-carrying body is drawn between. A three- or
    /// four-terminal part has no single span, so its body is drawn as a chain instead.
    pub(crate) fn body_segments(&self) -> Vec<(Point, Point)> {
        let posts = self.posts();
        match self.kind {
            PartKind::Transistor => {
                vec![(posts[0], self.b), (self.b, posts[1]), (self.b, posts[2])]
            }
            _ => vec![(self.a, self.b)],
        }
    }
}

/// How far either side of its axis a transistor's collector and emitter sit, in grid cells.
///
/// circuitjs1 spreads them by 16 units on a part 100 units long, so they end up about a third
/// of the span apart. Two cells against a three-cell span is the nearest the integer grid
/// allows that still leaves the base bar somewhere to sit.
const TRANSISTOR_ARM: i32 = 2;

/// The unit vector along `from`-`to`, which is only ever axis-aligned on the grid.
fn unit_axis(from: Point, to: Point) -> (i32, i32) {
    if from.y == to.y {
        (1, 0)
    } else {
        (0, 1)
    }
}

pub(crate) fn to_engine_point(point: Point) -> Point {
    Point::new(point.x * 16, point.y * 16)
}

/// The inverse of [`to_engine_point`].
///
/// Needed because the engine derives its extra terminals in *its* units — an op-amp's inputs
/// sit 16 engine units off its axis, which is one grid cell — so a part that wants those
/// terminals has to go into engine units to ask for them and come back out again. Rounding is
/// deliberate: a derived post can land off the grid, and the canvas can only draw on the grid.
pub(crate) fn from_engine_point(point: Point) -> Point {
    Point::new(
        (point.x as f64 / 16.0).round() as i32,
        (point.y as f64 / 16.0).round() as i32,
    )
}
