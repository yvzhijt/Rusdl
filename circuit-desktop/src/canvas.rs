use super::model::{Part, PartKind};
use circuit_sim::{Circuit, Element, Point};
use eframe::egui::{Align2, Color32, FontId, Painter, Pos2, Rect, Shape, Stroke, Vec2};

pub(crate) const GRID: f32 = 24.0;
pub(crate) const MIN_ZOOM: f32 = 0.35;
pub(crate) const MAX_ZOOM: f32 = 3.0;

fn distance_to_segment(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let length_sq = ab.length_sq();
    if length_sq <= f32::EPSILON {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / length_sq).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// Unit vector along `v`, or zero if `v` has no direction.
///
/// A part whose two terminals coincide has no axis to draw a symbol along, and
/// `Vec2::normalized` panics on that, so every symbol helper goes through here.
fn unit(v: Vec2) -> Vec2 {
    if v.length_sq() <= f32::EPSILON {
        Vec2::ZERO
    } else {
        v / v.length()
    }
}

/// How far back from each terminal a part's body wants to start, in screen pixels.
///
/// The leads run from the terminals to here, which is what keeps a line from being drawn
/// straight through the symbol that identifies the part. Shared by the drawing and the current
/// animation so the two can never disagree about where a body starts.
fn body_inset(kind: PartKind) -> f32 {
    match kind {
        PartKind::Capacitor => 5.0,
        PartKind::Diode => 10.0,
        PartKind::Supply | PartKind::AcSupply | PartKind::SignalSource => 14.0,
        PartKind::Resistor => 16.0,
        PartKind::Inductor | PartKind::Transformer => 22.0,
        PartKind::Ammeter => 14.0,
        // The two amplifiers draw a triangle whose base is their input end, so the body starts
        // where the inputs attach rather than half way along.
        PartKind::OpAmp | PartKind::Comparator => 10.0,
        PartKind::Switch | PartKind::SpdtSwitch | PartKind::AnalogSwitch => 12.0,
        PartKind::Pot => 16.0,
        PartKind::CurrentSource => 14.0,
        // A wire has no body, a ground is one bar, and a transistor's three terminals each
        // get their own lead drawn towards the bar.
        PartKind::Wire | PartKind::Ground | PartKind::Transistor | PartKind::Voltmeter => 0.0,
    }
}

/// Splits `a`-`b` into the stretch the body occupies and the stretch of lead either side.
fn body_and_leads(a: Pos2, b: Pos2, inset: f32) -> ((Pos2, Pos2), [(Pos2, Pos2); 2]) {
    let length = (b - a).length();
    if length <= f32::EPSILON {
        return ((a, b), [(a, a), (b, b)]);
    }
    // Clamped so a short part still shows some lead rather than a symbol spilling over its
    // own terminals.
    let inset = inset.min(length * 0.35);
    let direction = (b - a) / length;
    let body = (a + direction * inset, b - direction * inset);
    (body, [(a, body.0), (body.1, b)])
}

/// How many separately-phased runs of current a part carries, and how large each one is.
///
/// The runs are the desktop's counterpart of circuitjs1's one `curcount` accumulator per run
/// of current. Most parts need exactly one, but a transformer needs one per winding and a
/// transistor one per terminal, because those are genuinely different currents: drawing a
/// transformer's secondary off its primary number, or a transistor's emitter off its
/// collector current, is precisely what makes a flow diagram look like it ignores Kirchhoff.
///
/// Deliberately geometry-free. The phase accumulator has to advance on every frame, but the
/// segments can only be built at paint time, once the canvas rectangle and zoom are known.
/// Both walk the parts in the same order, so the two line up; a test pins the counts against
/// [`part_leg_segments`] to keep them honest.
pub(crate) fn leg_currents(part: &Part, element: Option<&dyn Element>) -> Vec<f64> {
    let Some(element) = element else {
        return vec![0.0; leg_count(part)];
    };
    match part.kind {
        // Nothing flows through a ground, so there is nothing to march.
        PartKind::Ground | PartKind::Voltmeter => Vec::new(),
        // One run per winding, each oriented from its first post to its second, so its current
        // is the one entering the far post. `TransformerElm.draw` keeps `curcount[0]` and
        // `curcount[1]` apart for exactly this split, and a loaded secondary carries a
        // different current from the primary, so sharing one number would misstate the flux
        // balance the part exists to show.
        PartKind::Transformer => {
            let posts = element.post_currents();
            [1, 3]
                .iter()
                .map(|&n| posts.get(n).copied().unwrap_or(0.0))
                .collect()
        }
        // One run per terminal, each from its post towards the bar, so the current is the one
        // entering that post. The BJT reports all three, and they satisfy `Ie = -(Ib + Ic)`.
        PartKind::Transistor => element.post_currents(),
        // Everything else is a single run along its own axis, so one phase covers both
        // leads and the stretch of body between them.
        _ => vec![element.current()],
    }
}

/// How many legs [`leg_currents`] will report for `part`, with no engine element to consult.
pub(crate) fn leg_count(part: &Part) -> usize {
    match part.kind {
        PartKind::Ground | PartKind::Voltmeter => 0,
        PartKind::Transformer => 2,
        PartKind::Transistor => 3,
        _ => 1,
    }
}

/// A part's legs as screen-space segments, one entry per [`leg_currents`] result.
///
/// The ordering has to match `leg_currents` exactly, since the caller walks the two together
/// to attach a phase to a stretch of conductor.
pub(crate) fn part_leg_segments(
    part: &Part,
    rect: Rect,
    transform: CanvasTransform,
) -> Vec<Vec<(Pos2, Pos2)>> {
    let at = |point: Point| transform.grid_to_screen(point, rect);
    // A two-terminal part: both leads, oriented post 0 to post 1 so a positive current runs
    // from the first terminal through the body to the second. The stretch of body in the
    // middle is deliberately absent — the markers step over the symbol rather than smearing
    // across the glyph that identifies the part.
    let plain = |from: Point, to: Point| {
        let (_, leads) = body_and_leads(at(from), at(to), body_inset(part.kind));
        leads.to_vec()
    };
    match part.kind {
        PartKind::Ground | PartKind::Voltmeter => Vec::new(),
        // A wire's whole span is lead, so it must not go through `body_and_leads`: that would
        // inset both ends against each other and leave it with nothing to draw or to carry
        // markers on.
        PartKind::Wire => vec![vec![(at(part.a), at(part.b))]],
        PartKind::Transformer => {
            // Three stretches per winding, as in `TransformerElm.draw`: the lead in, the gap
            // the core bridges, and the lead out. The core gap is included so the stream
            // crosses the whole symbol instead of stopping dead at each coil.
            let posts = part.posts();
            let winding = |from: Point, coil: Point, across: Point| {
                vec![(at(from), at(coil)), (at(coil), at(across))]
            };
            vec![
                winding(part.a, part.b, posts[2]),
                winding(posts[2], posts[3], part.b),
            ]
        }
        PartKind::Transistor => {
            // One run per terminal, each running from its post towards the base bar, as in
            // `TransistorElm.draw`. The sign comes from the terminal's own current, so an
            // active NPN's emitter markers march *out* of the emitter while its collector
            // markers march in: the three add up to zero, and the picture shows it.
            transistor_leads(part, rect, transform)
                .iter()
                .map(|&lead| vec![lead])
                .collect()
        }
        _ => vec![plain(part.a, part.b)],
    }
}

/// A transistor's three leads in post order, each running from its post to the point on the
/// base bar where the symbol actually attaches it.
///
/// Shared by the symbol drawing and the current animation, so markers cannot end up marching
/// to a point the symbol never reaches. Port of `TransistorElm.draw`, where the base lead
/// meets the middle of the bar and the other two leave it a third of the way out.
fn transistor_leads(part: &Part, rect: Rect, transform: CanvasTransform) -> [(Pos2, Pos2); 3] {
    let posts: Vec<Pos2> = part
        .posts()
        .iter()
        .map(|point| transform.grid_to_screen(*point, rect))
        .collect();
    let (base, collector, emitter) = (posts[0], posts[1], posts[2]);
    let far = transform.grid_to_screen(part.b, rect);
    let bar = base.lerp(far, 0.65);
    let across = unit(collector - emitter);
    let half_width = (collector - emitter).length() * 0.5;
    [
        (base, bar),
        (collector, bar + across * (half_width * 0.4)),
        (emitter, bar - across * (half_width * 0.4)),
    ]
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct CanvasTransform {
    pub(crate) zoom: f32,
    pub(crate) pan: Vec2,
}

impl Default for CanvasTransform {
    fn default() -> Self {
        Self {
            zoom: 1.0,
            pan: Vec2::ZERO,
        }
    }
}

impl CanvasTransform {
    pub(crate) fn zoom_at(&mut self, rect: Rect, pointer: Pos2, scroll: f32) {
        if scroll == 0.0 {
            return;
        }
        let new_zoom = (self.zoom * (scroll * 0.002).exp()).clamp(MIN_ZOOM, MAX_ZOOM);
        if (new_zoom - self.zoom).abs() < f32::EPSILON {
            return;
        }
        let cursor_from_center = pointer - rect.center();
        self.pan = cursor_from_center - (cursor_from_center - self.pan) * (new_zoom / self.zoom);
        self.zoom = new_zoom;
    }

    pub(crate) fn grid_to_screen(self, point: Point, rect: Rect) -> Pos2 {
        rect.center() + self.pan + Vec2::new(point.x as f32, point.y as f32) * GRID * self.zoom
    }

    pub(crate) fn screen_to_grid(self, pos: Pos2, rect: Rect) -> Point {
        let relative = (pos - rect.center() - self.pan) / (GRID * self.zoom);
        Point::new(relative.x.round() as i32, relative.y.round() as i32)
    }
}

pub(crate) fn draw_board(
    painter: &Painter,
    rect: Rect,
    parts: &[Part],
    selected: Option<usize>,
    wire_start: Option<Point>,
    wire_bends: &[Point],
    transform: CanvasTransform,
    leg_values: &[Vec<f64>],
    current_phases: &[f64],
    readings: &[Option<String>],
    pointer: Option<Pos2>,
    placement_preview: Option<(PartKind, Pos2)>,
) {
    painter.rect_filled(rect, 4.0, Color32::from_rgb(250, 251, 253));
    painter.rect_stroke(rect, 4.0, Stroke::new(1.0_f32, Color32::from_gray(190)));
    let origin = rect.center() + transform.pan;
    let grid_size = GRID * transform.zoom;
    let mut x = rect.left() + (origin.x - rect.left()).rem_euclid(grid_size);
    while x < rect.right() {
        painter.line_segment(
            [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
            Stroke::new(1.0_f32, Color32::from_rgb(231, 235, 241)),
        );
        x += grid_size;
    }
    let mut y = rect.top() + (origin.y - rect.top()).rem_euclid(grid_size);
    while y < rect.bottom() {
        painter.line_segment(
            [Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)],
            Stroke::new(1.0_f32, Color32::from_rgb(231, 235, 241)),
        );
        y += grid_size;
    }
    draw_current_tracks(painter, parts, leg_values, rect, transform);
    for (index, part) in parts.iter().enumerate() {
        draw_part(
            painter,
            rect,
            part,
            selected == Some(index),
            transform,
            readings.get(index).and_then(|reading| reading.as_deref()),
        );
    }
    draw_current_dots(painter, parts, leg_values, current_phases, rect, transform);
    draw_junctions(painter, rect, parts, transform);
    if let Some((kind, pointer)) = placement_preview.filter(|(_, pointer)| rect.contains(*pointer))
    {
        let preview = Part::at(kind, transform.screen_to_grid(pointer, rect));
        draw_part(painter, rect, &preview, true, transform, None);
    }
    if let Some(start) = wire_start {
        for bend in wire_bends {
            painter.circle_filled(
                transform.grid_to_screen(*bend, rect),
                4.0,
                Color32::from_rgb(235, 135, 30),
            );
        }
        if let Some(pointer) = pointer.filter(|pointer| rect.contains(*pointer)) {
            let end = transform.screen_to_grid(pointer, rect);
            for part in wire_route(start, wire_bends, end) {
                painter.line_segment(
                    [
                        transform.grid_to_screen(part.a, rect),
                        transform.grid_to_screen(part.b, rect),
                    ],
                    Stroke::new(1.8_f32, Color32::from_rgb(235, 135, 30)),
                );
            }
        }
    }
    painter.text(
        rect.left_top() + Vec2::new(12.0, 12.0),
        Align2::LEFT_TOP,
        "电路画布 · 滚轮缩放 · 左键拖动空白处平移",
        FontId::proportional(13.0),
        Color32::from_gray(110),
    );
}

fn draw_junctions(painter: &Painter, rect: Rect, parts: &[Part], transform: CanvasTransform) {
    let mut posts = std::collections::HashMap::<Point, usize>::new();
    for part in parts {
        for post in part.posts() {
            *posts.entry(post).or_default() += 1;
        }
    }
    for (point, count) in posts {
        if count >= 3 {
            painter.circle_filled(
                transform.grid_to_screen(point, rect),
                5.0,
                Color32::from_rgb(40, 48, 58),
            );
        }
    }
}

/// Every part's runs of current, flattened into one list in the same order as the phases.///
/// The phase accumulator walks this on each frame and the marker drawing walks it again at
/// paint time; both take the order from here, so a phase always lands on the stretch of
/// conductor it was advanced for.
pub(crate) fn all_leg_currents(
    parts: &[Part],
    circuit: Option<&Circuit>,
    engine_indices: &[Option<usize>],
) -> Vec<Vec<f64>> {
    let stepped = circuit.filter(|circuit| circuit.steps > 0);
    parts
        .iter()
        .enumerate()
        .map(|(part_index, part)| {
            let element = stepped.and_then(|circuit| {
                engine_indices
                    .get(part_index)
                    .copied()
                    .flatten()
                    .and_then(|engine_index| {
                        circuit.elements().get(engine_index).map(|e| e.as_ref())
                    })
            });
            leg_currents(part, element)
        })
        .collect()
}

/// How many runs of current the whole board carries, which is how many phases it keeps.
pub(crate) fn leg_count_for(leg_values: &[Vec<f64>]) -> usize {
    leg_values.iter().map(|runs| runs.len()).sum()
}

fn draw_current_tracks(
    painter: &Painter,
    parts: &[Part],
    leg_values: &[Vec<f64>],
    rect: Rect,
    transform: CanvasTransform,
) {
    for (part_index, part) in parts.iter().enumerate() {
        let runs = part_leg_segments(part, rect, transform);
        for (run_index, segments) in runs.iter().enumerate() {
            let Some(&current) = leg_values.get(part_index).and_then(|v| v.get(run_index)) else {
                continue;
            };
            if current.abs() <= 1e-12 {
                continue;
            }
            // Only the stretches that carry markers are tinted, so the highlight reads as
            // current entering and leaving the part rather than as a smear over the symbol
            // that identifies it.
            for (from, to) in segments {
                painter.line_segment(
                    [*from, *to],
                    Stroke::new(5.0_f32, Color32::from_rgba_unmultiplied(36, 190, 145, 55)),
                );
            }
        }
    }
}

/// Screen pixels between current markers, as a circuitjs1 `drawDots` spacing of 16.
const CURRENT_DOT_SPACING: f32 = 16.0;

/// Half the side of a current marker, matching `drawDots`' 4x4 `fillRect`.
const CURRENT_DOT_HALF: f32 = 2.0;

/// Marches small markers along each run of conductor, in the direction the current flows.
///
/// This follows circuitjs1's `drawDots`, which is handed a pair of points, a distance along
/// them and a phase, and squares off at 16 px intervals. The phase is signed and accumulates
/// by the current itself every frame, so reversing the current reverses the march without
/// anything having to decide which end to start from. Two details from the original matter
/// most for whether the flow reads as obeying Kirchhoff:
///
/// * a part with several runs of current — a transformer's two windings, a transistor's
///   three terminals — gets a *separate* phase for each, so each carries its own current's
///   speed and direction rather than a stand-in for the whole part;
/// * a run made of several stretches continues its phase from one into the next, so a single
///   unbroken stream crosses the part instead of restarting at every lead.
fn draw_current_dots(
    painter: &Painter,
    parts: &[Part],
    leg_values: &[Vec<f64>],
    phases: &[f64],
    rect: Rect,
    transform: CanvasTransform,
) {
    let dot = Color32::from_rgb(45, 195, 255);
    let mut phase_index = 0;
    for (part_index, part) in parts.iter().enumerate() {
        let runs = part_leg_segments(part, rect, transform);
        for (run_index, segments) in runs.iter().enumerate() {
            let Some(&current) = leg_values.get(part_index).and_then(|v| v.get(run_index)) else {
                continue;
            };
            if current.abs() <= 1e-12 {
                continue;
            }
            let phase = phases.get(phase_index).copied().unwrap_or(0.0);
            phase_index += 1;
            let forward = phase >= 0.0;
            let mut offset = (phase % CURRENT_DOT_SPACING as f64) as f32;
            if offset < 0.0 {
                offset += CURRENT_DOT_SPACING;
            }
            // Carried across the stretches of one run, which is what makes the pattern
            // continuous rather than restarting at each lead.
            let mut travelled = offset;
            for (from, to) in segments {
                let (from, to) = if forward { (*from, *to) } else { (*to, *from) };
                let length = (to - from).length();
                if length <= f32::EPSILON {
                    continue;
                }
                let direction = unit(to - from);
                let mut distance = travelled % length.max(CURRENT_DOT_SPACING);
                while distance < length {
                    let at = from + direction * distance;
                    painter.rect_filled(
                        Rect::from_center_size(at, Vec2::splat(CURRENT_DOT_HALF * 2.0)),
                        0.0,
                        dot,
                    );
                    distance += CURRENT_DOT_SPACING;
                }
                travelled += length;
            }
        }
    }
}

/// How far one amp of current pushes the markers along a lead, in screen pixels per second.
///
/// A school circuit carries milliamps, so a raw pixels-per-amp figure would make the markers
/// crawl; this is circuitjs1's `currentMult` chosen for the currents this editor actually
/// simulates. Bigger current still moves faster, and the *sign* is the direction.
pub(crate) const CURRENT_PIXELS_PER_AMP: f64 = 24_000.0;

/// Advances one part's marker phase by the current flowing through it.
///
/// This is the whole of circuitjs1's `updateDotCount`: an accumulator per element, advanced by
/// that element's own current. Nothing is normalised by a global clock, which is why series
/// elements stay in step and a branch carrying twice the current visibly runs twice as fast.
pub(crate) fn advance_current_phase(phase: f64, current: f64, elapsed_seconds: f64) -> f64 {
    phase + current * CURRENT_PIXELS_PER_AMP * elapsed_seconds
}

fn draw_part(
    painter: &Painter,
    rect: Rect,
    part: &Part,
    selected: bool,
    transform: CanvasTransform,
    reading: Option<&str>,
) {
    let a = transform.grid_to_screen(part.a, rect);
    let b = transform.grid_to_screen(part.b, rect);
    let color = if selected {
        Color32::from_rgb(25, 105, 210)
    } else {
        Color32::from_rgb(40, 48, 58)
    };
    let stroke = Stroke::new(if selected { 2.8_f32 } else { 2.2_f32 }, color);

    match part.kind {
        // A wire has no body, so its whole span *is* its lead. Drawing it as an ordinary
        // lead-plus-body would collapse the body to nothing and leave the wire invisible.
        PartKind::Wire => {
            painter.line_segment([a, b], stroke);
        }
        PartKind::Ground => {
            painter.line_segment([a, a + Vec2::new(0.0, 10.0)], stroke);
            painter.line_segment(
                [a + Vec2::new(-12.0, 10.0), a + Vec2::new(12.0, 10.0)],
                stroke,
            );
            painter.line_segment(
                [a + Vec2::new(-8.0, 15.0), a + Vec2::new(8.0, 15.0)],
                stroke,
            );
            painter.line_segment(
                [a + Vec2::new(-4.0, 20.0), a + Vec2::new(4.0, 20.0)],
                stroke,
            );
        }
        PartKind::Resistor => {
            draw_leaded(
                painter,
                a,
                b,
                body_inset(PartKind::Resistor),
                stroke,
                |from, to| {
                    draw_resistor_body(painter, from, to, stroke);
                },
            );
        }
        PartKind::Capacitor => {
            draw_leaded(
                painter,
                a,
                b,
                body_inset(PartKind::Capacitor),
                stroke,
                |from, to| {
                    draw_capacitor_body(painter, from, to, stroke);
                },
            );
        }
        PartKind::Inductor => {
            draw_leaded(
                painter,
                a,
                b,
                body_inset(PartKind::Inductor),
                stroke,
                |from, to| {
                    draw_coil_body(painter, from, to, stroke);
                },
            );
        }
        PartKind::Diode => {
            draw_leaded(
                painter,
                a,
                b,
                body_inset(PartKind::Diode),
                stroke,
                |from, to| {
                    draw_diode_body(painter, from, to, stroke, color);
                },
            );
        }
        PartKind::Supply | PartKind::AcSupply => {
            draw_leaded(painter, a, b, body_inset(part.kind), stroke, |from, to| {
                draw_source_body(painter, from, to, stroke, color, part.kind);
            });
        }
        PartKind::Ammeter => {
            draw_leaded(painter, a, b, body_inset(part.kind), stroke, |from, to| {
                draw_instrument_body(painter, from, to, stroke, color, part.kind);
            });
            draw_ammeter_polarity(painter, a, b, stroke.color);
        }
        PartKind::Voltmeter => {
            draw_voltage_probe(painter, a, stroke, color);
        }
        PartKind::CurrentSource => {
            // A circle with an arrow through it, which is the international symbol for a
            // current source and the one thing that distinguishes it at a glance from the
            // voltage source it otherwise resembles.
            draw_leaded(painter, a, b, body_inset(part.kind), stroke, |from, to| {
                draw_current_source_body(painter, from, to, stroke)
            });
        }
        PartKind::Switch => {
            draw_leaded(painter, a, b, body_inset(part.kind), stroke, |from, to| {
                draw_switch_body(painter, from, to, part.value >= 0.5, stroke);
            });
        }
        PartKind::SpdtSwitch => {
            draw_spdt_switch(painter, part, rect, transform, stroke, color);
        }
        PartKind::AnalogSwitch => {
            draw_analog_switch(painter, part, rect, transform, stroke);
        }
        PartKind::Pot => {
            draw_rheostat(painter, a, b, part.value2, stroke, color);
        }
        PartKind::SignalSource => {
            draw_leaded(
                painter,
                a,
                b,
                body_inset(PartKind::Supply),
                stroke,
                |from, to| {
                    draw_signal_source_body(painter, from, to, stroke);
                },
            );
        }
        PartKind::OpAmp | PartKind::Comparator => {
            draw_amplifier(painter, part, rect, transform, stroke);
        }
        PartKind::Transformer => {
            // Two windings, each with its own leads.
            let (secondary_a, secondary_b) = transformer_posts(part, rect, transform);
            draw_leaded(
                painter,
                a,
                b,
                body_inset(PartKind::Transformer),
                stroke,
                |from, to| {
                    draw_coil_body(painter, from, to, stroke);
                },
            );
            draw_leaded(
                painter,
                secondary_a,
                secondary_b,
                body_inset(PartKind::Transformer),
                stroke,
                |from, to| {
                    draw_coil_body(painter, from, to, stroke);
                },
            );
            // The core is two short lines *across* the gap between the windings, offset from
            // each other along the axis. Port of `TransformerElm.draw`, which joins each
            // winding's midpoint to the other winding's; running them alongside the windings
            // instead gives two long verticals, which is not what a core looks like.
            let along = unit(b - a);
            let primary_mid = a.lerp(b, 0.5);
            let secondary_mid = secondary_a.lerp(secondary_b, 0.5);
            let separation = (b - a).length() * 0.07;
            for offset in [-1.0, 1.0] {
                let shift = along * (offset * separation);
                painter.line_segment([primary_mid + shift, secondary_mid + shift], stroke);
            }
        }
        PartKind::Transistor => {
            // Built from the three posts rather than from the span, because they are what
            // fixes the symbol's shape: base on the axis, collector and emitter either side of
            // the far end. Port of `TransistorElm.setPoints`, where the bar is a short slab
            // spanning the full collector-emitter width, sitting most of the way along the
            // axis so the base lead is long and the collector and emitter leads are short.
            let leads = transistor_leads(part, rect, transform);
            let (base, collector, emitter) = (leads[0].0, leads[1].0, leads[2].0);
            let (bar, collector_foot, emitter_foot) = (leads[0].1, leads[1].1, leads[2].1);
            let across = unit(collector - emitter);
            let half_width = (collector - emitter).length() * 0.5;

            // The bar spans the full collector-emitter width. `TransistorElm.setPoints`
            // builds it as a rectangle between two faces 3 units apart, but at this editor's
            // scale that lands as a solid slab, so it is drawn as a plain bar instead — which
            // is what a schematic shows anyway.
            painter.line_segment(
                [bar - across * half_width, bar + across * half_width],
                stroke,
            );
            // The base lead comes in along the axis to the middle of the bar, and the other
            // two leave it a third of the way out and angle away to their own terminals.
            painter.line_segment([base, bar], stroke);
            painter.line_segment([collector, collector_foot], stroke);
            painter.line_segment([emitter, emitter_foot], stroke);
            // The arrow on the emitter, pointing away from the bar, is what marks the device
            // as an NPN. It sits just off the bar so it stays attached to its own terminal.
            let down = unit(emitter - emitter_foot);
            let sideways = Vec2::new(-down.y, down.x);
            let tip = emitter_foot + down * (half_width * 0.34);
            painter.add(Shape::convex_polygon(
                vec![
                    tip,
                    tip - down * 6.0 + sideways * 3.5,
                    tip - down * 6.0 - sideways * 3.5,
                ],
                color,
                Stroke::new(1.0_f32, color),
            ));
        }
    }

    for point in part.posts() {
        let screen = transform.grid_to_screen(point, rect);
        painter.circle_filled(screen, 4.0, Color32::WHITE);
        painter.circle_stroke(screen, 4.0, stroke);
    }

    // An instrument's own reading is more use under it than its name is, since the letter
    // in the dial already says what it is. A wire carries no value and gets no label, which
    // is also what circuitjs1 does — it is not a component, just a connection.
    let label = match reading {
        Some(reading) if is_instrument(part.kind) => reading.to_owned(),
        _ => part_label(part),
    };
    if !label.is_empty() {
        painter.text(
            label_position(part, a, b, &extra_terminals(part, rect, transform))
                + Vec2::new(0.0, 4.0),
            Align2::CENTER_TOP,
            label,
            FontId::proportional(12.0),
            color,
        );
    }
}

/// A part's terminals other than `a` and `b`, in screen space, so a symbol's label can be
/// placed against the whole of it rather than against the first two posts.
fn extra_terminals(part: &Part, rect: Rect, transform: CanvasTransform) -> Vec<Pos2> {
    part.posts()
        .into_iter()
        .skip(2)
        .map(|point| transform.grid_to_screen(point, rect))
        .collect()
}

/// Where a part's value is written: under everything the part draws, not under the midpoint
/// of its first two posts.
///
/// A part's posts only mark where its *leads* end, and the symbol between them is what has
/// to stay clear. Anchoring on the midpoint instead puts the text straight through the body
/// of any part placed vertically, and through the middle of a transformer whose two windings
/// sit on either side of it.
fn label_position(part: &Part, a: Pos2, b: Pos2, extra: &[Pos2]) -> Pos2 {
    // The margin is how far the drawn symbol reaches beyond its posts, per side: a coil's
    // humps, a plate's length, a dial's radius, or a ground symbol's bars.
    let margin = match part.kind {
        PartKind::Ground => 24.0,
        PartKind::Transformer | PartKind::Transistor => 16.0,
        PartKind::Wire => 0.0,
        _ => 18.0,
    };
    let mut min_x = a.x.min(b.x);
    let mut max_x = a.x.max(b.x);
    let mut min_y = a.y.min(b.y);
    let mut max_y = a.y.max(b.y);
    for point in extra {
        min_x = min_x.min(point.x);
        max_x = max_x.max(point.x);
        min_y = min_y.min(point.y);
        max_y = max_y.max(point.y);
    }
    Pos2::new((min_x + max_x) / 2.0, max_y + margin)
}

fn is_instrument(kind: PartKind) -> bool {
    matches!(kind, PartKind::Voltmeter | PartKind::Ammeter)
}

/// Draws a part as two leads with a symbol between them, and hands the symbol only the
/// stretch that is left over.
///
/// A line straight through the body is what a schematic never looks like: the body is what
/// identifies the part, and the leads only have to reach it.
fn draw_leaded(
    painter: &Painter,
    a: Pos2,
    b: Pos2,
    inset: f32,
    stroke: Stroke,
    body: impl FnOnce(Pos2, Pos2),
) {
    let ((from, to), leads) = body_and_leads(a, b, inset);
    for (lead_from, lead_to) in leads {
        painter.line_segment([lead_from, lead_to], stroke);
    }
    body(from, to);
}

/// The zig-zag body of a resistor, sized to whatever stretch it is given.
fn draw_resistor_body(painter: &Painter, from: Pos2, to: Pos2, stroke: Stroke) {
    let length = (to - from).length();
    if length <= f32::EPSILON {
        return;
    }
    let direction = (to - from) / length;
    let normal = Vec2::new(-direction.y, direction.x);
    let humps = 6;
    let pitch = length / humps as f32;
    let amplitude = (pitch * 0.9).clamp(2.0, 7.0);
    let mut points = vec![from];
    for i in 0..humps {
        let base = from + direction * (i as f32 * pitch);
        points.push(base + normal * if i % 2 == 0 { amplitude } else { -amplitude });
        points.push(
            base + direction * (pitch * 0.5)
                + normal * if i % 2 == 0 { -amplitude } else { amplitude },
        );
    }
    points.push(to);
    painter.add(Shape::line(points, stroke));
}

/// The two parallel plates of a capacitor, straddling the middle of the body.
fn draw_capacitor_body(painter: &Painter, from: Pos2, to: Pos2, stroke: Stroke) {
    let direction = unit(to - from);
    let normal = Vec2::new(-direction.y, direction.x);
    let gap = ((to - from).length() * 0.18).min(4.0);
    let middle = from.lerp(to, 0.5);
    for offset in [-gap, gap] {
        let plate = middle + direction * offset;
        painter.line_segment([plate + normal * 12.0, plate - normal * 12.0], stroke);
    }
}

/// A run of humps: the standard symbol for a winding, used for the inductor and for each of
/// the transformer's two windings. The humps all bulge the same way, which is what separates
/// a coil from a resistor's alternating zig-zag.
fn draw_coil_body(painter: &Painter, from: Pos2, to: Pos2, stroke: Stroke) {
    let length = (to - from).length();
    if length <= f32::EPSILON {
        return;
    }
    let direction = (to - from) / length;
    let normal = Vec2::new(-direction.y, direction.x);
    let humps = 4;
    let pitch = length / humps as f32;
    let amplitude = (pitch * 0.85).clamp(2.0, 7.0);
    let mut points = vec![from];
    for i in 0..humps {
        let base = from + direction * (i as f32 * pitch);
        points.push(base + normal * amplitude);
        points.push(base + direction * (pitch * 0.5) + normal * amplitude);
        points.push(base + direction * pitch);
    }
    points.push(to);
    painter.add(Shape::line(points, stroke));
}

/// A triangle-and-bar diode, pointing from `from` towards `to` so the anode is at `from`.
fn draw_diode_body(painter: &Painter, from: Pos2, to: Pos2, stroke: Stroke, color: Color32) {
    let direction = unit(to - from);
    let normal = Vec2::new(-direction.y, direction.x);
    let half = ((to - from).length() * 0.5).clamp(4.0, 9.0);
    let middle = from.lerp(to, 0.5);
    let tip = middle + direction * half;
    let back = middle - direction * half;
    painter.add(Shape::convex_polygon(
        vec![tip, back + normal * 9.0, back - normal * 9.0],
        color,
        stroke,
    ));
    let bar = middle - direction * (half * 0.25);
    painter.line_segment([bar + normal * 10.0, bar - normal * 10.0], stroke);
}

/// A source is drawn as a circle with its polarity marked inside it.
///
/// `to` is the *positive* terminal: a d.c. source is stamped as `V(to) - V(from) = volts`, so
/// the higher potential always appears there, and a user who has to work that out from the
/// value alone will wire a d.c. circuit backwards. The marks sit inside the circle because a
/// part's value is drawn just below it, and a mark outside the circle lands exactly there for
/// a vertically placed part. An a.c. source has no fixed polarity, so it carries a tilde
/// instead.
fn draw_source_body(
    painter: &Painter,
    from: Pos2,
    to: Pos2,
    stroke: Stroke,
    color: Color32,
    kind: PartKind,
) {
    let middle = from.lerp(to, 0.5);
    painter.circle_stroke(middle, 14.0, stroke);
    if kind == PartKind::AcSupply {
        painter.text(
            middle,
            Align2::CENTER_CENTER,
            "~",
            FontId::proportional(18.0),
            color,
        );
        return;
    }
    let direction = unit(to - from);
    draw_polarity_mark(painter, middle + direction * 6.0, true, color);
    draw_polarity_mark(painter, middle - direction * 6.0, false, color);
}

fn draw_signal_source_body(painter: &Painter, from: Pos2, to: Pos2, stroke: Stroke) {
    let middle = from.lerp(to, 0.5);
    painter.circle_stroke(middle, 14.0, stroke);
    let points: Vec<Pos2> = (0..=16)
        .map(|index| {
            let phase = index as f32 / 16.0 * std::f32::consts::TAU;
            middle
                + Vec2::new(
                    (phase / std::f32::consts::TAU - 0.5) * 17.0,
                    phase.sin() * 5.0,
                )
        })
        .collect();
    painter.add(Shape::line(points, stroke));
}

fn draw_voltage_probe(painter: &Painter, tip: Pos2, stroke: Stroke, color: Color32) {
    let center = tip + Vec2::new(0.0, -20.0);
    painter.line_segment([tip, center + Vec2::new(0.0, 12.0)], stroke);
    painter.circle_stroke(center, 10.0, stroke);
    painter.text(
        center,
        Align2::CENTER_CENTER,
        "V",
        FontId::proportional(12.0),
        color,
    );
    painter.circle_filled(tip, 3.0, color);
}

fn draw_ammeter_polarity(painter: &Painter, from: Pos2, to: Pos2, color: Color32) {
    let direction = unit(to - from);
    let normal = Vec2::new(-direction.y, direction.x);
    let center = from.lerp(to, 0.5);
    draw_polarity_mark(
        painter,
        center - direction * 19.0 + normal * 8.0,
        true,
        color,
    );
    draw_polarity_mark(
        painter,
        center + direction * 19.0 + normal * 8.0,
        false,
        color,
    );
}

fn draw_polarity_mark(painter: &Painter, center: Pos2, positive: bool, color: Color32) {
    let half = 3.5_f32;
    painter.line_segment(
        [
            center + Vec2::new(-half, 0.0),
            center + Vec2::new(half, 0.0),
        ],
        Stroke::new(1.5_f32, color),
    );
    if positive {
        painter.line_segment(
            [
                center + Vec2::new(0.0, -half),
                center + Vec2::new(0.0, half),
            ],
            Stroke::new(1.5_f32, color),
        );
    }
}

/// An instrument is a dial with its letter inside; the live reading is drawn beneath it by
/// the caller.
fn draw_instrument_body(
    painter: &Painter,
    from: Pos2,
    to: Pos2,
    stroke: Stroke,
    color: Color32,
    kind: PartKind,
) {
    let middle = from.lerp(to, 0.5);
    painter.circle_stroke(middle, 13.0, stroke);
    let letter = match kind {
        PartKind::Voltmeter => "V",
        PartKind::Ammeter => "A",
        _ => "?",
    };
    painter.text(
        middle,
        Align2::CENTER_CENTER,
        letter,
        FontId::proportional(15.0),
        color,
    );
}

/// A current source: a circle with an arrow down its axis, pointing the way positive current
/// runs — from `from` to `to`, matching the engine's post order.
fn draw_current_source_body(painter: &Painter, from: Pos2, to: Pos2, stroke: Stroke) {
    let middle = from.lerp(to, 0.5);
    let radius = 13.0_f32;
    painter.circle_stroke(middle, radius, stroke);
    let along = unit(to - from);
    let across = Vec2::new(-along.y, along.x);
    let tip = middle + along * (radius - 3.0);
    let tail = middle - along * (radius - 3.0);
    painter.line_segment([tail, tip], stroke);
    // Two barbs, so the arrow reads as a direction rather than as a line.
    for side in [-1.0, 1.0] {
        painter.line_segment([tip, tip - along * 7.0 + across * (3.5 * side)], stroke);
    }
}

/// A single-pole switch: two terminals, a fixed contact, and a lever that either reaches it
/// or stands clear of it. Port of `SwitchElm.draw`, which is a lever hinged at `point1`.
fn draw_switch_body(painter: &Painter, from: Pos2, to: Pos2, closed: bool, stroke: Stroke) {
    let along = unit(to - from);
    let across = Vec2::new(-along.y, along.x);
    // Hinge and fixed contact a little in from the ends, so the lever has room to swing.
    let hinge = from + along * 7.0;
    let contact = to - along * 7.0;
    painter.circle_filled(hinge, 2.6, stroke.color);
    painter.circle_filled(contact, 2.6, stroke.color);
    // Open stands up out of the axis; closed lies along it. Forty degrees reads as "open"
    // at a glance in a way that a small kink does not.
    let angle: f32 = if closed { 0.0 } else { -0.7 };
    let (sin, cos) = angle.sin_cos();
    let rotated = along * cos + across * sin;
    painter.line_segment(
        [hinge, hinge + rotated * (contact - hinge).length()],
        stroke,
    );
}

/// A single-pole double-throw switch: a pole that can reach either of two throws.
///
/// The terminals are the pole at `posts[0]` and the two throws at `posts[1]` and `posts[2]`;
/// `part.b` is only the reference the throws are derived from and is not a terminal. The
/// throws are read from the part rather than worked out here, so the lever is drawn onto
/// whatever the engine will actually connect.
fn draw_spdt_switch(
    painter: &Painter,
    part: &Part,
    rect: Rect,
    transform: CanvasTransform,
    stroke: Stroke,
    color: Color32,
) {
    let posts: Vec<Pos2> = part
        .posts()
        .iter()
        .map(|point| transform.grid_to_screen(*point, rect))
        .collect();
    let pole_terminal = posts[0];
    let (first, second) = (posts[1], posts[2]);

    // The two fixed contacts, pulled in from their terminals so the symbol has room to be a
    // symbol, and the moving pole between them, one step back along the axis.
    let across = unit(second - first);
    let back = unit(pole_terminal - first.lerp(second, 0.5));
    let contact_a = first - across * 5.0;
    let contact_b = second + across * 5.0;
    painter.circle_filled(contact_a, 2.6, color);
    painter.circle_filled(contact_b, 2.6, color);

    let pivot = first.lerp(second, 0.5) - back * 4.0;
    painter.circle_filled(pivot, 2.8, color);
    painter.line_segment([pole_terminal, pivot], stroke);

    let throw = if part.value >= 0.5 {
        contact_b
    } else {
        contact_a
    };
    painter.line_segment([pivot, throw], stroke);
}

/// A voltage-controlled switch: the two switched terminals, plus a control terminal arriving
/// from off the body.
fn draw_analog_switch(
    painter: &Painter,
    part: &Part,
    rect: Rect,
    transform: CanvasTransform,
    stroke: Stroke,
) {
    let posts: Vec<Pos2> = part
        .posts()
        .iter()
        .map(|point| transform.grid_to_screen(*point, rect))
        .collect();
    let (first, second, control) = (posts[0], posts[1], posts[2]);
    // The switch proper is always drawn closed: the resistance it actually takes comes from
    // its control voltage, and a symbol that flickered with the solution would be unreadable.
    draw_switch_body(painter, first, second, true, stroke);
    // The control arrives from the far side of the body and ends in a bar, the same
    // convention a Schmitt trigger or a relay uses for a sensing input.
    let axis = first.lerp(second, 0.5);
    let away = unit(axis - control);
    let bar = axis + away * 10.0;
    painter.line_segment([control, bar], stroke);
    let across = Vec2::new(-away.y, away.x);
    painter.line_segment([bar - across * 6.0, bar + across * 6.0], stroke);
}

/// A two-terminal rheostat, with a visible slider that can be dragged along the resistor.
fn draw_rheostat(
    painter: &Painter,
    from: Pos2,
    to: Pos2,
    position: f64,
    stroke: Stroke,
    color: Color32,
) {
    let ((body_from, body_to), _) = body_and_leads(from, to, body_inset(PartKind::Pot));
    draw_leaded(
        painter,
        from,
        to,
        body_inset(PartKind::Pot),
        stroke,
        |start, end| {
            draw_resistor_body(painter, start, end, stroke);
        },
    );
    let axis = unit(body_to - body_from);
    let across = Vec2::new(-axis.y, axis.x);
    let slider = body_from.lerp(body_to, position.clamp(0.0, 1.0) as f32);
    let tip = slider + across * 13.0;
    painter.line_segment([tip, slider + across * 4.0], stroke);
    painter.add(Shape::convex_polygon(
        vec![
            slider + across * 2.0,
            slider - axis * 4.0 + across * 8.0,
            slider + axis * 4.0 + across * 8.0,
        ],
        color,
        Stroke::new(1.0_f32, color),
    ));
}

pub(crate) fn rheostat_position_from_pointer(
    part: &Part,
    pos: Pos2,
    rect: Rect,
    transform: CanvasTransform,
) -> Option<f64> {
    let from = transform.grid_to_screen(part.a, rect);
    let to = transform.grid_to_screen(part.b, rect);
    let axis = to - from;
    let length_sq = axis.length_sq();
    if length_sq <= f32::EPSILON || distance_to_segment(pos, from, to) > 18.0 {
        return None;
    }
    Some(((pos - from).dot(axis) / length_sq).clamp(0.0, 1.0) as f64)
}

/// An ideal op-amp or a comparator: a triangle with the two inputs on its base and the output
/// at its point.
///
/// The two are the same triangle, and that is not a shortcut: a schematic draws an op-amp and
/// a comparator identically and distinguishes them by caption. Port of `OpAmpElm.draw`, which
/// puts the input leads in from the left and the output out to the right.
fn draw_amplifier(
    painter: &Painter,
    part: &Part,
    rect: Rect,
    transform: CanvasTransform,
    stroke: Stroke,
) {
    let posts: Vec<Pos2> = part
        .posts()
        .iter()
        .map(|point| transform.grid_to_screen(*point, rect))
        .collect();
    let (inverting, non_inverting, output) = (posts[0], posts[1], posts[2]);

    // The triangle spans the whole part, so its apex is the output terminal and its base is
    // the line the two inputs arrive on.
    let apex = output;
    let base_mid = inverting.lerp(non_inverting, 0.5);
    // Back the apex off a little so the output lead is a lead and not an overlap.
    let axis = unit(apex - base_mid);
    let apex_inner = apex - axis * 12.0;
    let across = Vec2::new(-axis.y, axis.x);
    let half = (inverting - non_inverting).length() * 0.5 + 12.0;
    let base_a = base_mid - across * half;
    let base_b = base_mid + across * half;
    // Filled with the board's own colour rather than left open: `convex_polygon` always fills,
    // and a solid triangle is not what an amplifier looks like on a schematic. The part's name
    // is captioned underneath, so there is no need to letter the inside either.
    painter.add(Shape::convex_polygon(
        vec![base_a, base_b, apex_inner],
        Color32::from_rgb(250, 251, 253),
        Stroke::new(stroke.width, stroke.color),
    ));

    // Input marks sit inside the triangle, clear of the leads, and are drawn as strokes so
    // they remain visible even when the selected UI font has no plus/minus glyphs.
    let inverting_mark = base_mid - across * (half * 0.5) + axis * 7.0;
    let non_inverting_mark = base_mid + across * (half * 0.5) + axis * 7.0;
    painter.line_segment([inverting, inverting_mark - axis * 5.0], stroke);
    painter.line_segment([non_inverting, non_inverting_mark - axis * 5.0], stroke);
    painter.line_segment([apex_inner, apex], stroke);

    draw_polarity_mark(painter, inverting_mark, false, stroke.color);
    draw_polarity_mark(painter, non_inverting_mark, true, stroke.color);
}

/// The second pair of terminals of a multi-terminal part, in screen space. A transformer
/// has a secondary winding here and a transistor its base, which is why both parts need it.
fn transformer_posts(part: &Part, rect: Rect, transform: CanvasTransform) -> (Pos2, Pos2) {
    let posts = part.posts();
    let second = posts.get(2).copied().unwrap_or(part.b);
    let third = posts.get(3).copied().unwrap_or(part.b);
    (
        transform.grid_to_screen(second, rect),
        transform.grid_to_screen(third, rect),
    )
}

/// The value shown under a part, formatted for the quantity it holds rather than in raw
/// SI units, which is how a component is labelled on a schematic.
pub(crate) fn part_label(part: &Part) -> String {
    match part.kind {
        PartKind::Supply | PartKind::AcSupply => format!("{:.2} V", part.value),
        PartKind::SignalSource => "f(t)".to_owned(),
        PartKind::Resistor => format!("{:.3} kΩ", part.value / 1_000.0),
        PartKind::Capacitor => format!("{:.3} µF", part.value * 1e6),
        PartKind::Inductor => format!("{:.3} H", part.value),
        PartKind::Diode => format!("{:.2} V", part.value),
        PartKind::Transformer => format!("{:.0}:{:.0}", part.value, part.value2),
        PartKind::Transistor => format!("β={:.0}", part.value),
        // A wire is a connection rather than a component, and circuitjs1 does not caption
        // one; a ground is worth naming in a teaching tool, so it keeps its label.
        PartKind::Wire => String::new(),
        _ => part.kind.name().to_owned(),
    }
}

/// One point on an instrument's trace.
#[derive(Clone, Copy, Debug)]
pub(crate) struct TracePoint {
    /// Simulation time in seconds, which is what the horizontal axis is scaled in.
    pub(crate) time: f64,
    /// Volts for a voltmeter, amps for an ammeter.
    pub(crate) value: f64,
}

/// A readable "nice" number at or above `rough`: 1, 2 or 5 times a power of ten.
///
/// Axis ticks have to land on values a person can say out loud, so a range of 0..3.7 V
/// divides into 1 V steps and 0..0.0043 A into 2 mA steps rather than into thirds.
pub(crate) fn nice_step(rough: f64) -> f64 {
    if !rough.is_finite() || rough <= 0.0 {
        return 1.0;
    }
    let magnitude = 10f64.powf(rough.log10().floor());
    let normalised = rough / magnitude;
    let multiple = if normalised <= 1.0 {
        1.0
    } else if normalised <= 2.0 {
        2.0
    } else if normalised <= 5.0 {
        5.0
    } else {
        10.0
    };
    multiple * magnitude
}

/// A measurement in the unit a person would use for it, so a current reads in milliamps
/// rather than in six decimal places of amps, and an axis tick is readable at a glance.
pub(crate) fn format_measurement(value: f64, unit: &str) -> String {
    let magnitude = value.abs();
    // Zero is the one value that belongs in the base unit whatever the rest of the axis is
    // scaled in, so a ±5 V axis does not label its middle line in microvolts. A non-finite
    // reading is shown the same way rather than as a wall of digits.
    if magnitude == 0.0 || !magnitude.is_finite() {
        return format!("0.000 {unit}");
    }
    if unit == "A" {
        if magnitude >= 1.0 {
            format!("{value:.3} A")
        } else if magnitude >= 1e-3 {
            format!("{:.3} mA", value * 1e3)
        } else if magnitude >= 1e-6 {
            format!("{:.3} µA", value * 1e6)
        } else {
            format!("{:.3} nA", value * 1e9)
        }
    } else if magnitude >= 1.0 {
        format!("{value:.3} V")
    } else if magnitude >= 1e-3 {
        format!("{:.3} mV", value * 1e3)
    } else {
        format!("{:.3} µV", value * 1e6)
    }
}

fn waveform_value_bounds(v_min: f64, v_max: f64, unit: &str) -> (f64, f64, f64) {
    let flat = v_max - v_min < 1e-12;
    let midpoint = if flat { (v_min + v_max) * 0.5 } else { 0.0 };
    let flat_scale = midpoint.abs().max(if unit == "A" { 1e-6 } else { 1e-3 });
    let step = nice_step(if flat {
        flat_scale / 5.0
    } else {
        (v_max - v_min) / 5.0
    });
    if flat {
        let center = (midpoint / step).round() * step;
        (center - 3.0 * step, center + 3.0 * step, step)
    } else {
        (
            ((v_min - midpoint) / step).floor() * step + midpoint,
            ((v_max - midpoint) / step).ceil() * step + midpoint,
            step,
        )
    }
}

/// An instrument's waveform: a gridded, labelled plot of its recent readings.
///
/// The axes scale themselves to the trace, because the whole point of looking at a waveform
/// is seeing its shape, and a fixed range would either clip a signal that grows or waste most
/// of the window on a signal that does not. The horizontal axis is labelled in simulation
/// time so the trace can be read against the source's frequency, and the vertical one in the
/// unit the instrument actually measures.
pub(crate) fn draw_waveform(
    painter: &Painter,
    rect: Rect,
    trace: &[TracePoint],
    unit: &str,
    cursor: Option<Pos2>,
) -> Option<TracePoint> {
    let plot = Rect::from_min_max(
        rect.left_top() + Vec2::new(76.0, 10.0),
        rect.right_bottom() - Vec2::new(10.0, 26.0),
    );
    if plot.width() <= 0.0 || plot.height() <= 0.0 {
        return None;
    }
    painter.rect_filled(rect, 4.0, Color32::from_rgb(25, 31, 40));
    painter.rect_stroke(rect, 4.0, Stroke::new(1.0_f32, Color32::from_gray(90)));
    let grid = Stroke::new(1.0_f32, Color32::from_gray(52));
    // FontId is not Copy in this egui version, so each use gets its own.
    let label = || FontId::proportional(11.0);
    let dim = Color32::from_gray(150);

    let trace: Vec<TracePoint> = trace
        .iter()
        .copied()
        .filter(|point| point.time.is_finite() && point.value.is_finite())
        .collect();
    if trace.is_empty() {
        painter.text(
            plot.center(),
            Align2::CENTER_CENTER,
            "等待采样…",
            FontId::proportional(13.0),
            dim,
        );
        return None;
    }

    let (t_min, t_max) = trace
        .iter()
        .map(|point| point.time)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), t| {
            (lo.min(t), hi.max(t))
        });
    let (v_min, v_max) = trace
        .iter()
        .map(|point| point.value)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
            (lo.min(v), hi.max(v))
        });
    // A flat trace has no range to divide, and a single sample has no time span, so both get
    // a nominal window that the trace sits in the middle of.
    // Dividing by five rather than four: `nice_step` always rounds *up* to the next 1, 2 or
    // 5, so asking for four divisions of a ±5 V range lands on a 5 V step and leaves only
    // three gridlines. Five asks for a step small enough to survive the rounding.
    let (v_lo, v_hi, v_step) = waveform_value_bounds(v_min, v_max, unit);
    let t_span = (t_max - t_min).max(1e-12);
    let t_step = nice_step(t_span / 6.0);
    let mut t_lo = (t_min / t_step).floor() * t_step;
    let mut t_hi = (t_max / t_step).ceil() * t_step;
    if t_hi <= t_lo {
        t_lo -= t_step;
        t_hi += t_step;
    }

    let y_of = |value: f64| {
        let fraction = (value - v_lo) / (v_hi - v_lo);
        plot.bottom() - fraction as f32 * plot.height()
    };
    let x_of = |time: f64| {
        let fraction = (time - t_lo) / (t_hi - t_lo);
        plot.left() + fraction as f32 * plot.width()
    };

    // Horizontal grid, labelled on the left with the value in the instrument's own unit.
    let mut value = v_lo;
    while value <= v_hi + v_step * 0.5 {
        let y = y_of(value);
        if y >= plot.top() - 1.0 && y <= plot.bottom() + 1.0 {
            painter.line_segment(
                [Pos2::new(plot.left(), y), Pos2::new(plot.right(), y)],
                grid,
            );
            painter.text(
                Pos2::new(plot.left() - 6.0, y),
                Align2::RIGHT_CENTER,
                format_measurement(value, unit),
                label(),
                dim,
            );
        }
        value += v_step;
    }
    // Vertical grid, labelled along the bottom in milliseconds.
    let mut time = t_lo;
    while time <= t_hi + t_step * 0.5 {
        let x = x_of(time);
        if x >= plot.left() - 1.0 && x <= plot.right() + 1.0 {
            painter.line_segment(
                [Pos2::new(x, plot.top()), Pos2::new(x, plot.bottom())],
                grid,
            );
            painter.text(
                Pos2::new(x, plot.bottom() + 6.0),
                Align2::CENTER_TOP,
                format!("{:.1} ms", time * 1e3),
                label(),
                dim,
            );
        }
        time += t_step;
    }
    // Zero gets its own line, because it is the one value a reader looks for first.
    if v_lo < 0.0 && v_hi > 0.0 {
        painter.line_segment(
            [
                Pos2::new(plot.left(), y_of(0.0)),
                Pos2::new(plot.right(), y_of(0.0)),
            ],
            Stroke::new(1.0_f32, Color32::from_gray(95)),
        );
    }
    painter.rect_stroke(plot, 0.0, Stroke::new(1.0_f32, Color32::from_gray(90)));
    painter.with_clip_rect(plot).add(Shape::line(
        trace
            .iter()
            .map(|point| Pos2::new(x_of(point.time), y_of(point.value)))
            .collect(),
        Stroke::new(1.8_f32, Color32::from_rgb(80, 220, 150)),
    ));
    let selected = cursor.filter(|pos| plot.contains(*pos)).and_then(|pos| {
        let time = t_lo + (pos.x - plot.left()) as f64 / plot.width() as f64 * (t_hi - t_lo);
        trace
            .iter()
            .min_by(|left, right| {
                (left.time - time)
                    .abs()
                    .total_cmp(&(right.time - time).abs())
            })
            .copied()
    });
    if let Some(point) = selected {
        let at = Pos2::new(x_of(point.time), y_of(point.value));
        let cursor_stroke = Stroke::new(1.0_f32, Color32::from_rgb(235, 190, 80));
        painter.with_clip_rect(plot).line_segment(
            [Pos2::new(at.x, plot.top()), Pos2::new(at.x, plot.bottom())],
            cursor_stroke,
        );
        painter.with_clip_rect(plot).line_segment(
            [Pos2::new(plot.left(), at.y), Pos2::new(plot.right(), at.y)],
            cursor_stroke,
        );
        painter.circle_filled(at, 3.5, Color32::from_rgb(255, 210, 90));
    }
    selected
}

pub(crate) fn hit_part(
    parts: &[Part],
    pos: Pos2,
    rect: Rect,
    transform: CanvasTransform,
) -> Option<usize> {
    parts
        .iter()
        .enumerate()
        .rev()
        .find(|(_, part)| {
            part.body_segments().iter().any(|(from, to)| {
                let from = transform.grid_to_screen(*from, rect);
                let to = transform.grid_to_screen(*to, rect);
                distance_to_segment(pos, from, to) < 14.0
            }) || (part.kind == PartKind::Ground
                && pos.distance(transform.grid_to_screen(part.a, rect) + Vec2::new(0.0, 8.0))
                    < 16.0)
                || (part.kind == PartKind::Voltmeter
                    && pos.distance(transform.grid_to_screen(part.a, rect) + Vec2::new(0.0, -20.0))
                        < 14.0)
        })
        .map(|(index, _)| index)
}

pub(crate) fn fit_to_parts(parts: &[Part], rect: Rect) -> CanvasTransform {
    if parts.is_empty() {
        return CanvasTransform::default();
    }

    let min_x = parts
        .iter()
        .flat_map(|part| [part.a.x, part.b.x])
        .min()
        .unwrap_or(0);
    let max_x = parts
        .iter()
        .flat_map(|part| [part.a.x, part.b.x])
        .max()
        .unwrap_or(0);
    let min_y = parts
        .iter()
        .flat_map(|part| [part.a.y, part.b.y])
        .min()
        .unwrap_or(0);
    let max_y = parts
        .iter()
        .flat_map(|part| [part.a.y, part.b.y])
        .max()
        .unwrap_or(0);
    let width = ((max_x - min_x).max(1) as f32 * GRID) + 100.0;
    let height = ((max_y - min_y).max(1) as f32 * GRID) + 100.0;
    let zoom = (rect.width() / width)
        .min(rect.height() / height)
        .clamp(MIN_ZOOM, MAX_ZOOM);
    let center = Vec2::new((min_x + max_x) as f32 / 2.0, (min_y + max_y) as f32 / 2.0);
    CanvasTransform {
        zoom,
        pan: -center * GRID * zoom,
    }
}

pub(crate) fn find_post_at(
    parts: &[Part],
    pos: Pos2,
    rect: Rect,
    transform: CanvasTransform,
) -> Option<Point> {
    parts
        .iter()
        .flat_map(|part| part.posts())
        .filter(|point| transform.grid_to_screen(*point, rect).distance(pos) <= 12.0)
        .min_by(|a, b| {
            transform
                .grid_to_screen(*a, rect)
                .distance(pos)
                .total_cmp(&transform.grid_to_screen(*b, rect).distance(pos))
        })
}

pub(crate) fn wire_between(start: Point, end: Point) -> Option<Part> {
    (start != end).then(|| Part::between(PartKind::Wire, start, end))
}

pub(crate) fn wire_route(start: Point, bends: &[Point], end: Point) -> Vec<Part> {
    let mut anchors = Vec::with_capacity(bends.len() + 2);
    anchors.push(start);
    anchors.extend(bends.iter().copied());
    anchors.push(end);

    let mut route = Vec::new();
    for segment in anchors.windows(2) {
        let from = segment[0];
        let to = segment[1];
        if from == to {
            continue;
        }
        if from.x == to.x || from.y == to.y {
            route.extend(wire_between(from, to));
        } else {
            let corner = Point::new(to.x, from.y);
            route.extend(wire_between(from, corner));
            route.extend(wire_between(corner, to));
        }
    }
    route
}

#[cfg(test)]
mod tests {
    use super::waveform_value_bounds;

    #[test]
    fn a_constant_waveform_gets_a_nonzero_display_range() {
        for (value, unit) in [(0.0, "V"), (4.5, "V"), (-2e-6, "A")] {
            let (low, high, step) = waveform_value_bounds(value, value, unit);
            assert!(low.is_finite() && high.is_finite() && step.is_finite());
            assert!(low < high && step > 0.0);
            assert!(low <= value && value <= high);
        }
    }
}
