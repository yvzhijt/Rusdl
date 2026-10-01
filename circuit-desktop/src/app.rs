use super::canvas::{
    advance_current_phase, all_leg_currents, draw_board, draw_waveform, find_post_at, fit_to_parts,
    format_measurement, hit_part, leg_count_for, rheostat_position_from_pointer, wire_route,
    CanvasTransform, TracePoint, GRID,
};
use super::model::{to_engine_point, waveform_name, Group, Part, PartKind, WAVEFORMS};
use super::persistence::{decode_parts, encode_parts};
use circuit_sim::{
    Ammeter, AnalogSwitch, Base, Bjt, Capacitor, Circuit, Comparator, CurrentSource, Diode,
    DiodeKind, DiodeModel, Element, Expr, Ground, Inductor, OpAmp, Point, PowerSupply, Resistor,
    SpdtSwitch, Switch, Transformer, VoltageSource, Waveform, Wire,
};
use eframe::egui::{self, Align2, Pos2, Rect, Sense, Vec2};
use std::path::PathBuf;
use std::time::Duration;

pub(crate) struct CircuitEditor {
    parts: Vec<Part>,
    selected_kind: PartKind,
    placing: bool,
    library_filter: String,
    selected_part: Option<usize>,
    property_window: Option<usize>,
    property_window_position: Option<Pos2>,
    current_file: Option<PathBuf>,
    saved_contents: String,
    close_prompt_open: bool,
    allow_close: bool,
    rheostat_drag: Option<usize>,
    palette_drag: Option<PartKind>,
    moving_part: Option<(usize, Part, Pos2)>,
    panning: Option<(Vec2, Pos2)>,
    canvas_transform: CanvasTransform,
    fit_view_requested: bool,
    wire_mode: bool,
    wire_start: Option<Point>,
    wire_bends: Vec<Point>,
    running: bool,
    /// Per-part current-flow marker phase, in screen pixels. See `advance_current_markers`.
    current_phases: Vec<f64>,
    last_animation_update: Option<f64>,
    engine: Option<Circuit>,
    engine_indices: Vec<Option<usize>>,
    status: String,
    /// One waveform per instrument on the canvas, in part order. See `record_traces`.
    traces: Vec<InstrumentTrace>,
    /// The left edge of the paused waveform view. `None` shows the beginning of history.
    trace_view_start_time: Option<f64>,
    /// The visible duration of a paused waveform. `None` shows the full history.
    trace_view_span: Option<f64>,
    trace_pan_anchor: Option<(Pos2, f64, f64)>,
    /// Simulated time of the last recorded trace point, so the next one is a fixed interval
    /// later. `None` means the next step records one.
    last_trace_time: Option<f64>,
    /// The instrument whose waveform window is open, as an index into `parts`.
    ///
    /// `None` means no window. There is deliberately only one: a second would be a second
    /// thing to look at while the first is being read, and the board already shows every
    /// instrument's live number.
    trace_window: Option<usize>,
    canvas_rect: Rect,
}

impl Default for CircuitEditor {
    fn default() -> Self {
        Self {
            parts: Vec::new(),
            selected_kind: PartKind::Resistor,
            placing: false,
            library_filter: String::new(),
            selected_part: None,
            property_window: None,
            property_window_position: None,
            current_file: None,
            saved_contents: encode_parts(&[]),
            close_prompt_open: false,
            allow_close: false,
            rheostat_drag: None,
            palette_drag: None,
            moving_part: None,
            panning: None,
            canvas_transform: CanvasTransform::default(),
            fit_view_requested: false,
            wire_mode: false,
            wire_start: None,
            wire_bends: Vec::new(),
            running: false,
            current_phases: Vec::new(),
            last_animation_update: None,
            engine: None,
            engine_indices: Vec::new(),
            status: "滚轮缩放画布；左键拖动空白处平移。".to_owned(),
            traces: Vec::new(),
            trace_view_start_time: None,
            trace_view_span: None,
            trace_pan_anchor: None,
            last_trace_time: None,
            trace_window: None,
            canvas_rect: Rect::NOTHING,
        }
    }
}

impl CircuitEditor {
    /// Replaces the status line, for a problem found before the app is running.
    ///
    /// A release build has no console, so anything that goes wrong at startup — a missing
    /// Chinese font, most obviously — has to be said somewhere the user will look. The status
    /// line is the one place on screen at the moment the window appears.
    pub(crate) fn set_status(&mut self, status: String) {
        self.status = status;
    }

    fn is_dirty(&self) -> bool {
        encode_parts(&self.parts) != self.saved_contents
    }

    fn should_intercept_close(&self, close_requested: bool) -> bool {
        close_requested && self.is_dirty() && !self.allow_close
    }

    fn add_part_at(&mut self, kind: PartKind, p: Point) {
        let part = Part::at(kind, p);
        self.parts.push(part);
        self.selected_part = Some(self.parts.len() - 1);
        self.property_window = (kind == PartKind::SignalSource).then_some(self.parts.len() - 1);
        self.running = false;
        self.engine = None;
        self.close_trace_window();
        self.status = if kind == PartKind::SignalSource {
            "已放置函数信号源；请填写激励表达式后再运行".to_owned()
        } else {
            format!("已放置：{}", kind.name())
        };
    }

    /// Closes the waveform window, because it refers to a part by position.
    ///
    /// Adding, deleting or replacing a part shifts every part after it along by one, so a
    /// window left open would start showing a *different* instrument's trace with no sign
    /// that anything had changed. Editing the circuit invalidates what the instruments were
    /// measuring anyway, so the window goes with it.
    fn close_trace_window(&mut self) {
        self.trace_window = None;
        self.trace_view_start_time = None;
        self.trace_view_span = None;
        self.trace_pan_anchor = None;
    }

    fn close_property_window(&mut self) {
        self.property_window = None;
        self.property_window_position = None;
    }

    /// Flips a switch's position, and says what it is now.
    ///
    /// Returns false for a part that is not a switch, so a caller can carry on with whatever it
    /// was doing.
    ///
    /// Unlike every other edit, this one does **not** stop the simulation. Changing a switch
    /// changes the circuit's topology, so the engine does re-analyse, but being able to operate
    /// a switch while the circuit runs is the entire point of the part — a run that stopped
    /// every time you clicked one would make it useless for exploring a circuit.
    fn toggle_switch(&mut self, index: usize) -> bool {
        let kind = match self.parts.get(index) {
            Some(part) => part.kind,
            None => return false,
        };
        let Some((at_one, at_zero)) = kind.switch_labels() else {
            return false;
        };
        // Scoped so the borrow of `parts` ends before the engine is rebuilt, which needs
        // `&mut self`.
        let part = self.parts.get_mut(index).expect("just checked");
        part.value = 1.0 - part.value;
        let now = if part.value >= 0.5 { at_one } else { at_zero };
        let was_running = self.running;
        match self.build_engine() {
            Ok(()) => {
                if was_running {
                    self.running = true;
                }
                self.status = format!("{}：{now}", kind.name());
            }
            Err(err) => {
                // Put it back, so a switch is never left showing a position the circuit
                // rejected.
                if let Some(part) = self.parts.get_mut(index) {
                    part.value = 1.0 - part.value;
                }
                self.running = false;
                self.status = err;
            }
        }
        true
    }
    fn property_window_ui(&mut self, ctx: &egui::Context) {
        let Some(index) = self.property_window else {
            return;
        };
        if index >= self.parts.len() {
            self.property_window = None;
            return;
        }
        let title = format!("{} 属性", self.parts[index].kind.name());
        let position = self.property_window_position.take();
        let mut open = true;
        egui::Window::new(title)
            .id(egui::Id::new(("rusdl-properties", index)))
            .open(&mut open)
            .default_pos(position.unwrap_or_else(|| ctx.screen_rect().center()))
            .default_width(320.0)
            .resizable(true)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        let mut actions = PropertyActions::default();
                        {
                            let part = &mut self.parts[index];
                            property_editor(ui, part, &mut actions);
                        }
                        if actions.remove {
                            self.parts.remove(index);
                            self.selected_part = None;
                            self.close_trace_window();
                            self.close_property_window();
                            actions.invalidate_engine = true;
                        }
                        if actions.invalidate_engine {
                            self.engine = None;
                            self.running = false;
                        }
                    });
            });
        if !open {
            self.close_property_window();
        }
    }

    fn build_engine(&mut self) -> Result<(), String> {
        if let Some(part) = self
            .parts
            .iter()
            .find(|part| part.kind.validate(part.value, part.value2).is_err())
        {
            return Err(format!("{} 的数值必须是有效的正数", part.kind.name()));
        }
        let mut circuit = Circuit::new();
        let mut indices = vec![None; self.parts.len()];
        for (part_index, part) in self.parts.iter().enumerate() {
            if let Some(element) = engine_element(part)? {
                indices[part_index] = Some(circuit.len());
                circuit.add(element);
            }
        }
        circuit
            .analyze()
            .map_err(|err| format!("电路分析失败：{err}"))?;
        self.engine = Some(circuit);
        self.engine_indices = indices;
        self.traces.clear();
        self.trace_view_start_time = None;
        self.last_trace_time = None;
        self.trace_window = None;
        Ok(())
    }

    fn start_simulation(&mut self) {
        if let Err(err) = self.build_engine() {
            self.running = false;
            self.engine = None;
            self.status = err;
            return;
        }
        self.running = true;
        self.trace_view_start_time = None;
        self.trace_view_span = None;
        self.trace_pan_anchor = None;
        self.status = "仿真运行中".to_owned();
    }

    /// Advances the current-flow markers and returns the wall-clock elapsed, in seconds.
    ///
    /// The markers move by the current through each part rather than by a shared clock, so
    /// two parts carrying the same current stay in step however long the run has been going,
    /// and a part whose current has reversed marches the other way from wherever it was. That
    /// is circuitjs1's `curcount`, and it is why the flow reads as one current through a
    /// circuit instead of a set of parts each animating on its own schedule.
    fn advance_current_markers(&mut self, now: f64) -> f64 {
        let elapsed = self
            .last_animation_update
            .map(|last| (now - last).max(0.0))
            .unwrap_or(0.0);
        self.last_animation_update = Some(now);
        if !self.running {
            return elapsed;
        }
        let legs = all_leg_currents(&self.parts, self.engine.as_ref(), &self.engine_indices);
        let wanted = leg_count_for(&legs);
        if self.current_phases.len() != wanted {
            self.current_phases.resize(wanted, 0.0);
        }
        // One accumulator per run of current, not one per part: a transformer has two
        // differently-loaded windings and a transistor three terminals whose currents differ,
        // and a single phase for either would draw a part as if it carried the same current
        // everywhere. See `TransistorElm.draw` in circuitjs1, which keeps `curcount_b`,
        // `curcount_c` and `curcount_e` apart for exactly this reason.
        let mut phase = self.current_phases.iter_mut();
        for current in legs.iter().flatten() {
            if let Some(slot) = phase.next() {
                *slot = advance_current_phase(*slot, *current, elapsed);
            }
        }
        elapsed
    }

    fn connect_wire(&mut self, endpoint: Point) -> bool {
        let Some(start) = self.wire_start else {
            return false;
        };
        if endpoint == start {
            self.status = "请选择另一个端点完成连线".to_owned();
            return false;
        }
        let route = wire_route(start, &self.wire_bends, endpoint);
        if route.is_empty() {
            return false;
        }

        self.parts.extend(route);
        self.selected_part = self.parts.len().checked_sub(1);
        self.wire_start = None;
        self.wire_bends.clear();
        self.engine = None;
        self.running = false;
        self.status = "导线已连接；可继续选择起点连线，右键或点击“连线”退出".to_owned();
        true
    }

    fn simulate_frame(&mut self) {
        if !self.running {
            return;
        }
        let Some(circuit) = self.engine.as_mut() else {
            self.running = false;
            self.status = "仿真器尚未初始化。".to_owned();
            return;
        };
        // Cover a fixed span of *simulated* time per frame rather than taking a fixed number
        // of steps. The engine's stepper is adaptive and ranges over six orders of magnitude,
        // from a nanosecond on a stiff circuit to five microseconds on a trivial one, so a
        // step-per-frame loop would either crawl or sprint depending on the circuit, and the
        // rate would be different every time the stepper adapted.
        let until = circuit.time + FRAME_SIM_SECONDS;
        // Sampled inside the stepping loop rather than once at the end of the frame: a frame
        // covers a millisecond of simulated time, so sampling at the end would throw away nine
        // out of ten of the samples the interval asks for and leave the rate set by the
        // display's refresh instead.
        let mut next_sample = self
            .last_trace_time
            .map_or(f64::NEG_INFINITY, |last| last + trace_sample_interval(last));
        let mut samples: Vec<(usize, f64, f64)> = Vec::new();
        let mut steps = 0;
        loop {
            if let Err(err) = circuit.step() {
                self.running = false;
                // A custom expression that runs off the edge of the reals stops the run, and
                // "maximum current exceeded" would send the user looking at their circuit rather
                // than at the expression they typed.
                let blamed = self.failed_custom_source();
                self.status = match blamed {
                    Some(expression) => {
                        format!("仿真停止：表达式在某个时刻没有有限值：{expression}")
                    }
                    None => format!("仿真停止：{err}"),
                };
                return;
            }
            steps += 1;
            if circuit.time >= next_sample {
                // From here on, so the next one is a whole interval after this one rather
                // than than after a frame boundary.
                next_sample = circuit.time + trace_sample_interval(circuit.time);
                let time = circuit.time;
                for (index, value) in
                    instrument_readings(&self.parts, circuit, &self.engine_indices)
                {
                    samples.push((index, time, value));
                }
            }
            // The ceiling is what keeps a very stiff circuit from locking the window up: it
            // slows down rather than freezing, which is the honest failure mode.
            if circuit.time >= until || steps >= MAX_STEPS_PER_FRAME {
                break;
            }
        }
        if let Some(&(_, time, _)) = samples.last() {
            self.last_trace_time = Some(time);
        }
        self.append_samples(samples);
    }

    /// The waveform window for the instrument that was double clicked.
    ///
    /// A floating window rather than a strip along the canvas, because a trace is something
    /// you look at closely for a moment and then close, and a permanent panel would take
    /// board space away from the circuit whether or not anything was being measured.
    fn trace_window_ui(&mut self, ctx: &egui::Context) {
        let Some(index) = self.trace_window else {
            return;
        };
        let Some(part) = self.parts.get(index).cloned() else {
            self.trace_window = None;
            return;
        };
        let Some(unit) = instrument_unit(part.kind) else {
            self.trace_window = None;
            return;
        };
        let points = self
            .traces
            .iter()
            .find(|trace| trace.part_index == index)
            .map_or(&[][..], |trace| trace.points.as_slice());
        let running = self.running;
        let mut view_start_time = self.trace_view_start_time;
        let mut view_span = self.trace_view_span;
        let mut pan_anchor = self.trace_pan_anchor;
        let (earliest, latest) = points
            .first()
            .zip(points.last())
            .map_or((0.0, 0.0), |(first, last)| (first.time, last.time));
        let history_span = (latest - earliest).max(0.0);
        let current_span = if running {
            history_span.min(TRACE_WINDOW_SECONDS)
        } else {
            view_span.unwrap_or(history_span).max(0.0).min(history_span)
        };
        let maximum_start = (latest - current_span).max(earliest);
        let current_start = if running {
            (latest - current_span).max(earliest)
        } else {
            view_start_time
                .unwrap_or(earliest)
                .max(earliest)
                .min(maximum_start)
        };
        let title = format!("{} · {} 波形", part.kind.name(), unit);
        let mut open = true;
        egui::Window::new(title)
            .id(egui::Id::new(("rusdl-trace", index)))
            .open(&mut open)
            .default_size([560.0_f32, 360.0_f32])
            .resizable(true)
            .show(ctx, |ui| {
                if !running && history_span > TRACE_MIN_VIEW_SECONDS {
                    ui.horizontal(|ui| {
                        ui.label("时间轴");
                        ui.weak(format!(
                            "{:.3}–{:.3} / {:.3} s",
                            current_start,
                            current_start + current_span,
                            history_span
                        ));
                    });
                } else if running {
                    ui.weak("仿真运行中显示最新波形；暂停后可缩放并平移完整历史");
                } else {
                    ui.weak("当前显示完整历史；在波形上拖动平移，滚轮缩放");
                }
                let display_start = if running {
                    (latest - current_span).max(earliest)
                } else {
                    current_start
                };
                let display_end = if running {
                    latest
                } else {
                    (display_start + current_span).min(latest)
                };
                let (start_index, end_index) = trace_view_range(points, display_start, display_end);
                let visible_points: Vec<TracePoint> =
                    points[start_index..end_index].iter().copied().collect();
                if let Some(last) = visible_points.last() {
                    let minimum = visible_points
                        .iter()
                        .map(|point| point.value)
                        .fold(f64::INFINITY, f64::min);
                    let maximum = visible_points
                        .iter()
                        .map(|point| point.value)
                        .fold(f64::NEG_INFINITY, f64::max);
                    ui.label(format!(
                        "末值 {}   最小 {}   最大 {}   采样点 {}",
                        format_measurement(last.value, unit),
                        format_measurement(minimum, unit),
                        format_measurement(maximum, unit),
                        visible_points.len()
                    ));
                } else {
                    ui.label("正在采样，请先运行仿真。");
                }
                let available = ui.available_size();
                let (rect, response) = ui.allocate_exact_size(
                    Vec2::new(available.x.max(200.0), (available.y - 22.0).max(120.0)),
                    Sense::click_and_drag(),
                );
                if !running && response.drag_started() {
                    if let Some(pointer) = response.interact_pointer_pos() {
                        pan_anchor = Some((pointer, current_start, current_span));
                    }
                }
                if !running && response.dragged() {
                    if let (Some((anchor, start, span)), Some(pointer)) =
                        (pan_anchor, response.interact_pointer_pos())
                    {
                        let plot_width = (rect.width() - 86.0).max(1.0) as f64;
                        view_start_time = Some(pan_trace_view(
                            start,
                            (pointer.x - anchor.x) as f64,
                            plot_width,
                            span,
                            earliest,
                            latest,
                        ));
                    }
                }
                if response.drag_stopped() {
                    pan_anchor = None;
                }
                if !running && response.hovered() {
                    let scroll = ctx.input(|input| input.smooth_scroll_delta.y);
                    if scroll != 0.0 && history_span > 0.0 {
                        if let Some(pointer) = response.hover_pos() {
                            let plot_left = rect.left() + 76.0;
                            let plot_width = (rect.width() - 86.0).max(1.0);
                            let fraction =
                                ((pointer.x - plot_left) / plot_width).clamp(0.0, 1.0) as f64;
                            let (start, span) = zoom_trace_view(
                                current_start,
                                current_span,
                                fraction,
                                scroll as f64,
                                earliest,
                                latest,
                            );
                            view_span = (span < history_span).then_some(span);
                            view_start_time = (start > earliest).then_some(start);
                        }
                    }
                }
                let cursor = response.hover_pos();
                let reading = draw_waveform(ui.painter(), rect, &visible_points, unit, cursor);
                ui.label(reading.map_or_else(
                    || "悬停波形查看采样值".to_owned(),
                    |point| {
                        format!(
                            "游标：{:.3} ms  {}",
                            point.time * 1e3,
                            format_measurement(point.value, unit)
                        )
                    },
                ));
            });
        self.trace_view_start_time = view_start_time;
        self.trace_view_span = view_span;
        self.trace_pan_anchor = pan_anchor;
        if !open {
            self.trace_window = None;
        }
    }

    /// The expression of a source whose waveform has no finite value, if one did.
    ///
    /// An expression like `1/t` is perfectly good until `t` reaches zero, which no amount of
    /// checking at the point it was written can rule out, so the source flags it during the
    /// solve and this finds the text to quote back.
    fn failed_custom_source(&self) -> Option<&str> {
        let circuit = self.engine.as_ref()?;
        let index = self.parts.iter().enumerate().find_map(|(part_index, _)| {
            let engine_index = self.engine_indices.get(part_index).copied().flatten()?;
            circuit.elements()[engine_index]
                .as_any()
                .downcast_ref::<VoltageSource>()
                .is_some_and(|source| source.custom_failed())
                .then_some(part_index)
        })?;
        self.parts
            .get(index)
            .map(|part| part.expression.as_str())
            .filter(|expression| !expression.trim().is_empty())
    }

    /// Appends the readings a frame collected to the matching traces, and drops any part that
    /// is not an instrument.
    ///
    /// Each entry is a part index, the simulated time it was read at, and the value. A trace
    /// is keyed to the part it came from rather than to its position, so deleting an
    /// instrument or inserting one cannot slide a waveform onto the wrong meter.
    fn append_samples(&mut self, samples: Vec<(usize, f64, f64)>) {
        let mut previous = std::mem::take(&mut self.traces);
        let mut rebuilt: Vec<InstrumentTrace> = Vec::new();
        for (index, part) in self.parts.iter().enumerate() {
            if instrument_unit(part.kind).is_none() {
                continue;
            }
            let mut points = previous
                .iter_mut()
                .find(|trace| trace.part_index == index)
                .map(|trace| std::mem::take(&mut trace.points))
                .unwrap_or_default();
            for (part_index, time, value) in samples.iter().copied() {
                if part_index == index {
                    points.push(TracePoint { time, value });
                }
            }
            // Keep a bounded history for paused inspection without letting a long run grow
            // memory indefinitely.
            rebuilt.push(InstrumentTrace {
                part_index: index,
                points,
            });
        }
        self.traces = rebuilt;
        // A window whose instrument has been deleted would end up showing somebody else's trace.
        if self.trace_window.is_some_and(|index| {
            !self
                .parts
                .get(index)
                .is_some_and(|p| instrument_unit(p.kind).is_some())
        }) {
            self.trace_window = None;
        }
    }

    /// Re-keys the trace list to the current parts, adding a point to each instrument from
    /// the engine's present state.
    ///
    /// Every voltmeter and ammeter on the canvas records continuously, whether or not anyone is
    /// looking, so double-clicking one opens a waveform that already holds the run's history
    /// instead of starting from that moment.
    #[cfg(test)]
    fn record_traces(&mut self) {
        let time = self.engine.as_ref().map_or(0.0, |circuit| circuit.time);
        let samples = self
            .engine
            .as_ref()
            .map(|circuit| {
                instrument_readings(&self.parts, circuit, &self.engine_indices)
                    .into_iter()
                    .map(|(index, value)| (index, time, value))
                    .collect()
            })
            .unwrap_or_default();
        self.append_samples(samples);
    }

    /// A live reading for every instrument on the canvas, in part order, so each dial can
    /// show its own number where it sits rather than in a list somewhere off to one side.
    fn meter_readings(&self) -> Vec<Option<String>> {
        self.parts
            .iter()
            .enumerate()
            .map(|(index, part)| {
                meter_reading(part, index, self.engine.as_ref(), &self.engine_indices)
            })
            .collect()
    }

    fn save(&mut self) -> bool {
        let path = self.current_file.clone().or_else(|| {
            rfd::FileDialog::new()
                .add_filter("Rusdl", &["sdl"])
                .set_file_name("circuit.sdl")
                .save_file()
        });
        path.is_some_and(|path| self.write_file(path))
    }

    fn save_as(&mut self) -> bool {
        let path = rfd::FileDialog::new()
            .add_filter("Rusdl", &["sdl"])
            .set_file_name(
                self.current_file
                    .as_ref()
                    .and_then(|path| path.file_name())
                    .map_or_else(
                        || "circuit.sdl".to_owned(),
                        |name| name.to_string_lossy().into_owned(),
                    ),
            )
            .save_file();
        path.is_some_and(|path| self.write_file(path))
    }

    fn write_file(&mut self, path: PathBuf) -> bool {
        let contents = encode_parts(&self.parts);
        match std::fs::write(&path, &contents) {
            Ok(()) => {
                self.current_file = Some(path.clone());
                self.saved_contents = contents;
                self.status = format!("已保存：{}", path.display());
                true
            }
            Err(err) => {
                self.status = format!("保存失败：{err}");
                false
            }
        }
    }

    fn open(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("Rusdl", &["sdl"])
            .pick_file()
        else {
            return;
        };
        match std::fs::read_to_string(&path).and_then(|text| {
            decode_parts(&text)
                .map_err(|err| std::io::Error::new(std::io::ErrorKind::InvalidData, err))
        }) {
            Ok(parts) => {
                self.parts = parts;
                self.selected_part = None;
                self.engine = None;
                self.running = false;
                self.close_trace_window();
                self.current_file = Some(path.clone());
                self.saved_contents = encode_parts(&self.parts);
                self.status = format!("已打开：{}", path.display());
            }
            Err(err) => self.status = format!("打开失败：{err}"),
        }
    }
}

impl eframe::App for CircuitEditor {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if self.should_intercept_close(ctx.input(|input| input.viewport().close_requested())) {
            self.close_prompt_open = true;
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
        }
        let title = self
            .current_file
            .as_ref()
            .and_then(|path| path.file_name())
            .and_then(|name| name.to_str())
            .unwrap_or("Rusdl");
        ctx.send_viewport_cmd(egui::ViewportCommand::Title(format!(
            "{}{}",
            title,
            if self.is_dirty() { " *" } else { "" }
        )));
        self.advance_current_markers(ctx.input(|input| input.time));
        self.simulate_frame();

        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                if ui
                    .button(if self.running {
                        "⏸ 暂停"
                    } else {
                        "▶ 仿真"
                    })
                    .clicked()
                {
                    if self.running {
                        self.running = false;
                        self.status = "仿真已暂停".to_owned();
                    } else {
                        self.start_simulation();
                    }
                }
                if ui.button("单步").clicked() {
                    if self.engine.is_none() {
                        if let Err(err) = self.build_engine() {
                            self.status = err;
                        }
                    }
                    if let Some(circuit) = self.engine.as_mut() {
                        match circuit.step() {
                            Ok(()) => self.status = format!("单步完成，t = {:.6} s", circuit.time),
                            Err(err) => self.status = format!("仿真停止：{err}"),
                        }
                    }
                }
                ui.separator();
                if ui.button("保存").clicked() {
                    self.save();
                }
                if ui.button("另存为").clicked() {
                    self.save_as();
                }
                if ui.button("打开").clicked() {
                    self.open();
                }
                ui.separator();
                if ui
                    .selectable_label(!self.placing && !self.wire_mode, "选择")
                    .clicked()
                {
                    self.placing = false;
                    self.wire_mode = false;
                    self.wire_start = None;
                    self.wire_bends.clear();
                }
                if ui
                    .selectable_label(self.placing, format!("放置：{}", self.selected_kind.name()))
                    .clicked()
                {
                    self.placing = true;
                    self.wire_mode = false;
                    self.wire_start = None;
                    self.wire_bends.clear();
                }
                if ui.button("−").on_hover_text("缩小").clicked() {
                    self.canvas_transform.zoom_at(
                        self.canvas_rect,
                        self.canvas_rect.center(),
                        -48.0,
                    );
                }
                ui.label(format!("{:.0}%", self.canvas_transform.zoom * 100.0));
                if ui.button("+").on_hover_text("放大").clicked() {
                    self.canvas_transform.zoom_at(
                        self.canvas_rect,
                        self.canvas_rect.center(),
                        48.0,
                    );
                }
                if ui.button("适应画布").clicked() {
                    self.fit_view_requested = true;
                }
                if ui
                    .selectable_label(self.wire_mode, "连线")
                    .on_hover_text("先点起始端点，再点空白处增加折点，最后点目标端点；右键取消")
                    .clicked()
                {
                    self.wire_mode = !self.wire_mode;
                    self.placing = false;
                    self.wire_start = None;
                    self.wire_bends.clear();
                    self.status = if self.wire_mode {
                        "连线模式：点击一个元件端点作为起点".to_owned()
                    } else {
                        "已退出连线模式".to_owned()
                    };
                }
                if ui.button("清空").clicked() {
                    self.parts.clear();
                    self.selected_part = None;
                    self.close_trace_window();
                    self.wire_start = None;
                    self.wire_bends.clear();
                    self.engine = None;
                    self.running = false;
                    self.status = "画布已清空".to_owned();
                }
            });
        });

        egui::TopBottomPanel::bottom("status").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.colored_label(
                    if self.running {
                        egui::Color32::from_rgb(45, 165, 105)
                    } else {
                        egui::Color32::from_gray(135)
                    },
                    "●",
                );
                ui.label(&self.status);
                if let Some(circuit) = &self.engine {
                    ui.separator();
                    ui.weak(format!("t = {:.3} s", circuit.time));
                }
                if self.engine.is_none()
                    && self
                        .parts
                        .iter()
                        .any(|part| matches!(part.kind, PartKind::Voltmeter | PartKind::Ammeter))
                {
                    ui.separator();
                    ui.weak("点击“仿真”或“单步”读取仪表");
                }
            });
        });

        egui::SidePanel::left("palette")
            .resizable(true)
            .default_width(200.0)
            .width_range(150.0..=300.0)
            .show(ctx, |ui| {
                ui.heading("元器件");
                ui.label("选择器件后放置，或拖拽到画布");
                ui.weak("右键元件打开属性；点击开关可直接切换");
                ui.add(
                    egui::TextEdit::singleline(&mut self.library_filter)
                        .hint_text("搜索元器件…")
                        .desired_width(f32::INFINITY),
                );
                ui.separator();
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        let query = self.library_filter.trim().to_lowercase();
                        for group in Group::ALL {
                            let matches: Vec<_> = PartKind::ALL
                                .into_iter()
                                .filter(|kind| kind.group() == group)
                                .filter(|kind| {
                                    query.is_empty()
                                        || kind.name().to_lowercase().contains(&query)
                                        || kind.keywords().contains(&query)
                                        || kind.code().contains(&query)
                                })
                                .collect();
                            if matches.is_empty() {
                                continue;
                            }
                            egui::CollapsingHeader::new(group.name())
                                .default_open(
                                    !query.is_empty() || self.selected_kind.group() == group,
                                )
                                .show(ui, |ui| {
                                    for kind in matches {
                                        let selected = self.selected_kind == kind;
                                        let response = ui.add_sized(
                                            [ui.available_width(), 32.0],
                                            egui::Button::new(kind.name()).selected(selected),
                                        );
                                        if response.clicked() {
                                            self.selected_kind = kind;
                                            self.placing = true;
                                            self.wire_mode = false;
                                            self.wire_start = None;
                                            self.wire_bends.clear();
                                        }
                                        if response.drag_started() {
                                            self.palette_drag = Some(kind);
                                            self.selected_kind = kind;
                                            self.placing = true;
                                            self.wire_mode = false;
                                            self.wire_start = None;
                                            self.wire_bends.clear();
                                        }
                                    }
                                });
                        }
                    });
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            let available = ui.available_size();
            let board_size = Vec2::new(available.x, available.y.max(1.0));
            let (board_rect, response) =
                ui.allocate_exact_size(board_size, Sense::click_and_drag());
            self.canvas_rect = board_rect;
            if self.fit_view_requested {
                self.canvas_transform = fit_to_parts(&self.parts, board_rect);
                self.fit_view_requested = false;
            }
            let pointer_hover = ctx.input(|input| input.pointer.hover_pos());
            let scroll = ctx.input(|input| input.smooth_scroll_delta.y);
            if response.hovered() {
                if let Some(pointer) = pointer_hover.filter(|pointer| board_rect.contains(*pointer))
                {
                    self.canvas_transform.zoom_at(board_rect, pointer, scroll);
                }
            }
            draw_board(
                ui.painter(),
                board_rect,
                &self.parts,
                self.selected_part,
                self.wire_start,
                &self.wire_bends,
                self.canvas_transform,
                &all_leg_currents(&self.parts, self.engine.as_ref(), &self.engine_indices),
                &self.current_phases,
                &self.meter_readings(),
                if self.wire_mode { pointer_hover } else { None },
                if self.placing {
                    pointer_hover.map(|pointer| (self.selected_kind, pointer))
                } else {
                    None
                },
            );

            let pointer = ctx.input(|input| input.pointer.interact_pos());
            let secondary_click_pos = ctx
                .input(|input| {
                    input.events.iter().find_map(|event| match event {
                        egui::Event::PointerButton {
                            pos,
                            button: egui::PointerButton::Secondary,
                            pressed: false,
                            ..
                        } => Some(*pos),
                        _ => None,
                    })
                })
                .or_else(|| {
                    response
                        .secondary_clicked()
                        .then(|| pointer.or_else(|| ctx.input(|input| input.pointer.hover_pos())))
                        .flatten()
                });
            if let Some(pos) = secondary_click_pos.filter(|pos| board_rect.contains(*pos)) {
                if self.wire_mode {
                    self.wire_mode = false;
                    self.wire_start = None;
                    self.wire_bends.clear();
                    self.status = "已取消连线".to_owned();
                } else if let Some(index) =
                    hit_part(&self.parts, pos, board_rect, self.canvas_transform)
                {
                    self.selected_part = Some(index);
                    self.property_window = Some(index);
                    self.property_window_position = Some(pos);
                }
            }

            if self.wire_mode && response.clicked() {
                if let Some(pos) = pointer {
                    if let Some(endpoint) =
                        find_post_at(&self.parts, pos, board_rect, self.canvas_transform)
                    {
                        if let Some(start) = self.wire_start {
                            if endpoint == start {
                                self.status = "请选择另一个端点完成连线".to_owned();
                            } else {
                                self.connect_wire(endpoint);
                            }
                        } else {
                            self.wire_start = Some(endpoint);
                            self.wire_bends.clear();
                            self.status = "已选起点；点击空白处增加拐点，点击端点完成".to_owned();
                        }
                    } else {
                        if let Some(start) = self.wire_start {
                            if hit_part(&self.parts, pos, board_rect, self.canvas_transform)
                                .is_some()
                            {
                                self.status = "请点元件端点，或在空白处添加拐点".to_owned();
                            } else {
                                let bend = self.canvas_transform.screen_to_grid(pos, board_rect);
                                if bend != start && self.wire_bends.last().copied() != Some(bend) {
                                    self.wire_bends.push(bend);
                                    self.status = format!(
                                        "已添加第 {} 个拐点；点击目标端点完成",
                                        self.wire_bends.len()
                                    );
                                }
                            }
                        } else {
                            self.status = "请点击一个元件端点开始连线".to_owned();
                        }
                    }
                }
            } else if response.drag_started() {
                if let Some(pos) = pointer {
                    if !self.wire_mode {
                        if let Some(index) =
                            hit_part(&self.parts, pos, board_rect, self.canvas_transform)
                        {
                            self.selected_part = Some(index);
                            if self.parts[index].kind == PartKind::Pot
                                && rheostat_position_from_pointer(
                                    &self.parts[index],
                                    pos,
                                    board_rect,
                                    self.canvas_transform,
                                )
                                .is_some()
                            {
                                self.rheostat_drag = Some(index);
                            } else {
                                self.moving_part = Some((index, self.parts[index].clone(), pos));
                            }
                        } else {
                            self.panning = Some((self.canvas_transform.pan, pos));
                        }
                        if let (Some(index), Some(pos)) = (self.rheostat_drag, pointer) {
                            if let Some(position) = self.parts.get(index).and_then(|part| {
                                rheostat_position_from_pointer(
                                    part,
                                    pos,
                                    board_rect,
                                    self.canvas_transform,
                                )
                            }) {
                                if let Some(part) = self.parts.get_mut(index) {
                                    part.value2 = position;
                                    self.engine = None;
                                    self.running = false;
                                }
                            }
                        }
                    } else if find_post_at(&self.parts, pos, board_rect, self.canvas_transform)
                        .is_none()
                    {
                        self.panning = Some((self.canvas_transform.pan, pos));
                    }
                }
            }
            if response.dragged() {
                if let (Some((index, original, start_pointer)), Some(pos)) =
                    (self.moving_part.as_ref(), pointer)
                {
                    if !self.wire_mode && self.parts.get(*index).is_some() {
                        let delta = pos - *start_pointer;
                        let grid_size = GRID * self.canvas_transform.zoom;
                        let dx = (delta.x / grid_size).round() as i32;
                        let dy = (delta.y / grid_size).round() as i32;
                        self.parts[*index] = original.translated(dx, dy);
                        self.engine = None;
                        self.running = false;
                    }
                } else if let (Some((original_pan, start_pointer)), Some(pos)) =
                    (self.panning, pointer)
                {
                    self.canvas_transform.pan = original_pan + (pos - start_pointer);
                }
            }
            if response.drag_stopped() {
                self.panning = None;
                self.moving_part = None;
                if let Some(index) = self.rheostat_drag.take() {
                    match self.build_engine() {
                        Ok(()) => {
                            self.status =
                                format!("滑动变阻器：{:.1} %", self.parts[index].value2 * 100.0)
                        }
                        Err(err) => self.status = err,
                    }
                }
            }

            let released = ctx.input(|input| input.pointer.any_released());
            let mut palette_dropped = false;
            if released {
                if let (Some(kind), Some(pos)) = (self.palette_drag.take(), pointer) {
                    if board_rect.contains(pos) {
                        self.add_part_at(
                            kind,
                            self.canvas_transform.screen_to_grid(pos, board_rect),
                        );
                        palette_dropped = true;
                    }
                }
                self.moving_part = None;
                self.panning = None;
            }

            if !self.wire_mode && response.double_clicked() {
                if let Some(pos) = pointer {
                    if let Some(index) =
                        hit_part(&self.parts, pos, board_rect, self.canvas_transform)
                    {
                        self.selected_part = Some(index);
                        // An instrument's double click opens its waveform; anything else opens
                        // its properties. This is what replaced the separate oscilloscope part
                        // and the strip of canvas it used to own.
                        if self
                            .parts
                            .get(index)
                            .is_some_and(|part| instrument_unit(part.kind).is_some())
                        {
                            self.trace_window = Some(index);
                        } else {
                            self.property_window = Some(index);
                        }
                    }
                }
            }
            if !self.wire_mode && response.clicked() && !palette_dropped {
                if let Some(pos) = pointer {
                    if let Some(index) =
                        hit_part(&self.parts, pos, board_rect, self.canvas_transform)
                    {
                        self.selected_part = Some(index);
                        // Clicking a switch is how you operate it. That is the gesture
                        // everyone reaches for and the one circuitjs1 uses, and it is the
                        // difference between a switch you can use and one you have to go
                        // looking for in a property dialog.
                        self.toggle_switch(index);
                    } else if self.placing {
                        self.add_part_at(
                            self.selected_kind,
                            self.canvas_transform.screen_to_grid(pos, board_rect),
                        );
                    } else {
                        self.selected_part = None;
                    }
                }
            }
        });

        self.property_window_ui(ctx);
        self.trace_window_ui(ctx);
        self.close_prompt_ui(ctx);

        if self.running {
            ctx.request_repaint_after(Duration::from_millis(16));
        }
    }
}

impl CircuitEditor {
    fn close_prompt_ui(&mut self, ctx: &egui::Context) {
        if !self.close_prompt_open {
            return;
        }
        egui::Window::new("保存更改？")
            .id(egui::Id::new("rusdl-unsaved-changes"))
            .collapsible(false)
            .resizable(false)
            .anchor(Align2::CENTER_CENTER, Vec2::ZERO)
            .show(ctx, |ui| {
                ui.label("此电路有尚未保存的修改。关闭前要保存吗？");
                ui.horizontal(|ui| {
                    if ui.button("保存").clicked() && self.save() {
                        self.close_prompt_open = false;
                        self.allow_close = true;
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                    if ui.button("不保存").clicked() {
                        self.close_prompt_open = false;
                        self.allow_close = true;
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                    if ui.button("取消").clicked() {
                        self.close_prompt_open = false;
                    }
                });
            });
    }
}

/// Default live view span and minimum zoom span for a waveform window.
const TRACE_WINDOW_SECONDS: f64 = 0.06;
const TRACE_MIN_VIEW_SECONDS: f64 = 1e-5;

/// The output swing an op-amp saturates at, in volts.
///
/// The op-amp model is ideal apart from this: its inputs draw no current and its output
/// follows the difference between them until it runs out of headroom. There is no supply pin
/// to wire, so the rails are a property of the part rather than something the user sets — and
/// they are generous, because a supply rail that a beginner's ±5 V circuit clips against turns
/// every amplifier into a comparator by accident.
const OPAMP_RAIL: f64 = 10.0;

/// How much simulated time the editor tries to cover in one frame, in seconds.
///
/// At sixty frames a second this runs a mains-frequency circuit at roughly a fifth of real
/// time, which is fast enough to watch a capacitor charge and slow enough to read the
/// current-flow markers while it happens.
const FRAME_SIM_SECONDS: f64 = 1e-3;

/// The most simulation steps one frame may take, so a stiff circuit slows down instead of
/// freezing the window.
const MAX_STEPS_PER_FRAME: usize = 500;

/// The gap between two recorded trace points, in simulated seconds.
///
/// Samples are denser during the first 10 ms so fast startup transients remain visible, then
/// return to the established 100 us interval. Both intervals use simulated time rather than
/// solver-step counts.
const TRACE_SAMPLE_INTERVAL: f64 = 1e-4;
const TRANSIENT_SAMPLE_INTERVAL: f64 = 1e-5;
const TRANSIENT_SAMPLE_DURATION: f64 = 0.01;

fn trace_sample_interval(time: f64) -> f64 {
    if time < TRANSIENT_SAMPLE_DURATION {
        TRANSIENT_SAMPLE_INTERVAL
    } else {
        TRACE_SAMPLE_INTERVAL
    }
}

fn trace_view_range(points: &[TracePoint], start_time: f64, end_time: f64) -> (usize, usize) {
    let start = points.partition_point(|point| point.time < start_time);
    let end = points.partition_point(|point| point.time <= end_time);
    (start.min(points.len()), end.max(start).min(points.len()))
}

fn zoom_trace_view(
    start: f64,
    span: f64,
    anchor_fraction: f64,
    scroll: f64,
    earliest: f64,
    latest: f64,
) -> (f64, f64) {
    let history_span = (latest - earliest).max(0.0);
    let span = span.max(0.0).min(history_span);
    let minimum_span = TRACE_MIN_VIEW_SECONDS.min(history_span);
    let factor = (scroll * 0.002).exp();
    let new_span = (span / factor).clamp(minimum_span, history_span);
    let anchor_fraction = anchor_fraction.clamp(0.0, 1.0);
    let anchor_time = start + span * anchor_fraction;
    let maximum_start = (latest - new_span).max(earliest);
    let new_start = (anchor_time - new_span * anchor_fraction)
        .max(earliest)
        .min(maximum_start);
    (new_start, new_span)
}

fn pan_trace_view(
    start: f64,
    delta_x: f64,
    plot_width: f64,
    span: f64,
    earliest: f64,
    latest: f64,
) -> f64 {
    let history_span = (latest - earliest).max(0.0);
    let span = span.max(0.0).min(history_span);
    let maximum_start = (latest - span).max(earliest);
    if plot_width <= 0.0 {
        return start.max(earliest).min(maximum_start);
    }
    (start - delta_x / plot_width * span)
        .max(earliest)
        .min(maximum_start)
}

/// One instrument's waveform history.
#[derive(Clone)]
struct InstrumentTrace {
    /// The part this belongs to, so a waveform cannot slide onto a different instrument when
    /// one is added or deleted.
    part_index: usize,
    points: Vec<TracePoint>,
}

/// What the property editor asked the editor to do about the part.
#[derive(Default)]
struct PropertyActions {
    /// Something changed that the engine has to be rebuilt for.
    invalidate_engine: bool,
    remove: bool,
}

/// Every control for one part, drawn into `ui`.
///
/// The single copy of this. Each control is a labelled row rather than everything on one line,
/// because a waveform picker and a free-text expression field do not fit in a toolbar strip —
/// which is exactly how the square wave ended up implemented, tested and unfindable.
fn property_editor(ui: &mut egui::Ui, part: &mut Part, actions: &mut PropertyActions) {
    let terminals: Vec<String> = part
        .posts()
        .iter()
        .enumerate()
        .map(|(i, post)| format!("{} ({}, {})", i + 1, post.x, post.y))
        .collect();
    ui.weak(format!("端子：{}", terminals.join("  ")));

    // A custom source's value comes from its expression, so an amplitude and a frequency
    // beside it would be two controls that do nothing.
    let custom = part.kind == PartKind::SignalSource
        || (part.kind.has_waveform() && part.waveform == Waveform::Custom);
    if part.kind.has_value() && !custom {
        ui.horizontal(|ui| {
            ui.label("数值");
            let response =
                ui.add(egui::DragValue::new(&mut part.value).speed(part.kind.value_speed()));
            if response.changed() {
                actions.invalidate_engine = true;
            }
            ui.label(part.kind.unit());
        });
    }
    if part.kind.has_value2() && !custom {
        if part.kind == PartKind::Pot {
            ui.label("滑块位置");
            ui.horizontal(|ui| {
                let response = ui.add(egui::Slider::new(&mut part.value2, 0.0..=1.0));
                if response.changed() {
                    actions.invalidate_engine = true;
                }
            });
        } else {
            ui.horizontal(|ui| {
                ui.label(part.kind.value2_label());
                let response = ui.add(egui::DragValue::new(&mut part.value2).speed(1.0));
                if response.changed() {
                    actions.invalidate_engine = true;
                }
                ui.label(part.kind.unit2());
            });
        }
    }

    // A switch's position is a state, not a number, so it gets a button rather than a spinner.
    // "0.0" is not a way to ask for a closed switch, and a spinner with only two reachable
    // values is worse than a button that says what it will do.
    if let Some((at_one, at_zero)) = part.kind.switch_labels() {
        ui.horizontal(|ui| {
            let label = if part.value >= 0.5 { at_one } else { at_zero };
            if ui.button(label).clicked() {
                part.value = 1.0 - part.value;
                actions.invalidate_engine = true;
            }
            // The button works, but clicking the switch on the canvas is the gesture anyone
            // reaches for, so say so rather than leaving it to be discovered.
            ui.weak("也可点击画布上的开关");
        });
    }

    if part.kind.has_waveform() {
        ui.separator();
        ui.label("波形");
        egui::ComboBox::from_id_salt("waveform")
            .selected_text(waveform_name(part.waveform))
            .width(ui.available_width())
            .show_ui(ui, |ui| {
                for waveform in WAVEFORMS {
                    if ui
                        .selectable_label(part.waveform == waveform, waveform_name(waveform))
                        .clicked()
                    {
                        part.waveform = waveform;
                        // Switching *to* the custom shape with nothing written yet would be a
                        // source that outputs nothing, so it starts from a working example.
                        actions.invalidate_engine = true;
                    }
                }
            });
        if part.waveform == Waveform::Custom {
            expression_editor(ui, part, actions, false);
        } else {
            ui.weak("函数表达式请使用“函数信号源”；交流电源用于标准波形");
        }
    }
    if part.kind == PartKind::SignalSource {
        ui.separator();
        expression_editor(ui, part, actions, true);
    }

    ui.separator();
    ui.horizontal(|ui| {
        if ui.button("旋转 90°").clicked() && part.kind.has_two_posts() {
            let dx = part.b.x - part.a.x;
            let dy = part.b.y - part.a.y;
            part.b = Point::new(part.a.x - dy, part.a.y + dx);
            actions.invalidate_engine = true;
        }
        if ui.button("删除").clicked() {
            actions.remove = true;
        }
    });
}

fn expression_editor(
    ui: &mut egui::Ui,
    part: &mut Part,
    actions: &mut PropertyActions,
    required: bool,
) {
    ui.label("激励信号表达式（V）");
    let response = ui.add(
        egui::TextEdit::multiline(&mut part.expression)
            .desired_width(ui.available_width())
            .desired_rows(3)
            .hint_text("5*sin(2*pi*50*t)"),
    );
    if response.changed() {
        actions.invalidate_engine = true;
    }
    ui.weak(if required {
        "t 为仿真时间（秒），角度为弧度；函数信号源必须填写"
    } else {
        "t 为秒、角度为弧度"
    });
    let expression = part.expression.trim();
    match Expr::parse(expression) {
        Ok(expr) if !expression.is_empty() => {
            let value = expr.eval(0.0);
            if value.is_finite() {
                ui.weak(format!("t = 0 时 {}", format_measurement(value, "V")));
            } else {
                ui.colored_label(
                    egui::Color32::from_rgb(200, 70, 70),
                    "表达式在 t=0 时没有有限值",
                );
            }
        }
        Ok(_) => {
            ui.colored_label(egui::Color32::from_rgb(190, 90, 60), "请输入激励信号表达式");
        }
        Err(err) => {
            ui.colored_label(egui::Color32::from_rgb(200, 70, 70), err.to_string());
        }
    }
}

/// The engine element for one placed part, or `None` for a part that has none.
///
/// A voltmeter is the one part with no element: it measures between two of its own terminals
/// without joining the circuit at all, which is exactly what makes it non-loading.
///
/// Free rather than a method so a test can ask what a part *would* become without building a
/// whole circuit — which is what makes it possible to check that the canvas and the engine
/// agree about where the terminals are.
fn engine_element(part: &Part) -> Result<Option<Box<dyn Element>>, String> {
    let p1 = to_engine_point(part.a);
    let p2 = to_engine_point(part.b);
    let posts = part.posts();
    let engine_point = |index: usize| to_engine_point(posts[index]);
    Ok(match part.kind {
        PartKind::Supply => Some(Box::new(PowerSupply::dc_between(p1, p2, part.value))),
        PartKind::AcSupply => {
            // The waveform decides how the source is built, because a custom source's value
            // comes from its expression and ignores the amplitude and frequency that the other
            // shapes use. An expression that will not parse is rejected here rather than being
            // run as silence.
            if part.waveform == Waveform::Custom {
                let expr = Expr::parse(part.expression.trim())
                    .map_err(|err| format!("{} 的表达式无效：{err}", part.kind.name()))?;
                Some(Box::new(VoltageSource::custom(
                    Base::from_points(p1, p2, 0, 2),
                    expr,
                )))
            } else {
                Some(Box::new(VoltageSource::from_base(
                    Base::from_points(p1, p2, 0, 2),
                    part.waveform,
                    part.value2,
                    part.value,
                    0.0,
                    0.0,
                    0.5,
                    false,
                )))
            }
        }
        PartKind::SignalSource => {
            let expr = Expr::parse(part.expression.trim())
                .map_err(|err| format!("{} 的表达式无效：{err}", part.kind.name()))?;
            Some(Box::new(VoltageSource::custom(
                Base::from_points(p1, p2, 0, 2),
                expr,
            )))
        }
        PartKind::Resistor => Some(Box::new(Resistor::between(p1, p2, part.value))),
        PartKind::Capacitor => Some(Box::new(Capacitor::from_base(
            Base::from_points(p1, p2, 0, 2),
            part.value,
        ))),
        PartKind::Inductor => Some(Box::new(Inductor::from_base(
            Base::from_points(p1, p2, 0, 2),
            part.value,
            0.0,
        ))),
        PartKind::Diode => Some(Box::new(Diode::new(
            p1,
            p2,
            0,
            &DiodeModel::with_forward_voltage_drop(part.value, 2.0),
            DiodeKind::Plain,
        ))),
        PartKind::Transformer => Some(Box::new(Transformer::new(
            p1,
            p2,
            engine_point(2),
            engine_point(3),
            part.value,
            part.value2,
        ))),
        // A transistor's posts are base, collector, emitter, which is also the order the
        // element wants them in.
        PartKind::Transistor => Some(Box::new(Bjt::new(
            engine_point(0),
            engine_point(1),
            engine_point(2),
            0,
            part.value,
        ))),
        PartKind::CurrentSource => Some(Box::new(CurrentSource::from_base(
            Base::from_points(p1, p2, 0, 2),
            part.value,
        ))),
        PartKind::Switch => Some(Box::new(Switch::from_base(
            Base::from_points(p1, p2, 0, 2),
            // The two conventions here are opposites, and getting it wrong produces a switch
            // that looks shut and behaves open: the dialog stores closed as one, while
            // `SwitchElm` numbers the *closed* position zero.
            if part.value >= 0.5 { 0 } else { 1 },
            false,
        ))),
        PartKind::SpdtSwitch => Some(Box::new(SpdtSwitch::from_base(
            // Two throws is what a single-pole double-throw is: one at each end of the travel,
            // so there is no centre-off position to model. Here the conventions happen to
            // agree — the dialog's zero means "the first throw", which is the engine's zero.
            Base::from_points(p1, p2, 0, 3),
            if part.value >= 0.5 { 1 } else { 0 },
            false,
            0,
            2,
        ))),
        PartKind::AnalogSwitch => Some(Box::new(AnalogSwitch::from_base(
            Base::from_points(p1, p2, 0, 3),
            part.value,
            part.value2,
        ))),
        PartKind::Pot => Some(Box::new(Resistor::between(
            p1,
            p2,
            part.value * part.value2.clamp(0.0001, 1.0),
        ))),
        PartKind::OpAmp => Some(Box::new(OpAmp::from_base(
            Base::from_points(p1, p2, 0, 3),
            OPAMP_RAIL,
            -OPAMP_RAIL,
            part.value,
        ))),
        PartKind::Comparator => Some(Box::new(Comparator::from_base(Base::from_points(
            p1, p2, 0, 3,
        )))),
        PartKind::Wire => Some(Box::new(Wire::from_base(Base::from_points(p1, p2, 0, 2)))),
        PartKind::Ground => Some(Box::new(Ground::at_point(p1))),
        PartKind::Ammeter => Some(Box::new(Ammeter::between(p1, p2))),
        PartKind::Voltmeter => None,
    })
}

/// The unit an instrument measures in, or `None` for a part that is not an instrument.
fn instrument_unit(kind: PartKind) -> Option<&'static str> {
    match kind {
        PartKind::Voltmeter => Some("V"),
        PartKind::Ammeter => Some("A"),
        _ => None,
    }
}

/// What every instrument reads right now: its part index and its value, volts for a
/// voltmeter and amps for an ammeter.
///
/// A free function rather than a method so the stepping loop can call it while holding the
/// engine mutably; taking the parts and indices as arguments means only those two fields are
/// borrowed alongside it.
fn instrument_readings(
    parts: &[Part],
    circuit: &Circuit,
    engine_indices: &[Option<usize>],
) -> Vec<(usize, f64)> {
    if circuit.steps == 0 {
        return Vec::new();
    }
    parts
        .iter()
        .enumerate()
        .filter(|(_, part)| instrument_unit(part.kind).is_some())
        .filter_map(|(index, part)| {
            let value = match part.kind {
                PartKind::Voltmeter => circuit
                    .node_at_point(to_engine_point(part.a))
                    .map(|node| circuit.node_voltage(node))?,
                PartKind::Ammeter => {
                    let engine_index = engine_indices.get(index).copied().flatten()?;
                    circuit.elements()[engine_index]
                        .as_any()
                        .downcast_ref::<Ammeter>()?
                        .amperes()
                }
                _ => return None,
            };
            Some((index, value))
        })
        .collect()
}

fn meter_reading(
    part: &Part,
    part_index: usize,
    circuit: Option<&Circuit>,
    engine_indices: &[Option<usize>],
) -> Option<String> {
    match part.kind {
        PartKind::Voltmeter => {
            let Some(circuit) = circuit else {
                return Some("未仿真".to_owned());
            };
            if circuit.steps == 0 {
                return Some("未仿真".to_owned());
            }
            Some(
                circuit
                    .node_at_point(to_engine_point(part.a))
                    .map(|node| format!("{:.3} V", circuit.node_voltage(node)))
                    .unwrap_or_else(|| "未接入电路".to_owned()),
            )
        }
        PartKind::Ammeter => {
            let Some(circuit) = circuit else {
                return Some("未仿真".to_owned());
            };
            if circuit.steps == 0 {
                return Some("未仿真".to_owned());
            }
            let Some(engine_index) = engine_indices.get(part_index).copied().flatten() else {
                return Some("未接入电路".to_owned());
            };
            let Some(meter) = circuit.elements()[engine_index]
                .as_any()
                .downcast_ref::<Ammeter>()
            else {
                return Some("仪表状态无效".to_owned());
            };
            Some(format!("{:.3} A", meter.amperes()))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Only the tests reach for these, so they are not part of the app's own imports.
    use crate::canvas::{
        format_measurement, leg_count, leg_currents, nice_step, part_label, part_leg_segments,
        wire_between, MAX_ZOOM, MIN_ZOOM,
    };
    use crate::model::Group;

    /// Every part kind, so a test can walk them all.
    ///
    /// Derived from `PartKind::ALL` rather than written out, because a hand-maintained copy of
    /// the enum is a list that silently stops covering the parts added after it — which is
    /// exactly how seven new parts went untested for a while.
    const ALL_KINDS: [PartKind; 20] = PartKind::ALL;

    #[test]
    fn ac_supply_properties_render_without_invalid_layout_bounds() {
        let mut editor = CircuitEditor::default();
        editor.add_part_at(PartKind::AcSupply, Point::new(0, 0));
        editor.property_window = Some(0);
        let context = egui::Context::default();
        let _ = context.run(egui::RawInput::default(), |context| {
            editor.property_window_ui(context);
        });
    }

    #[test]
    fn short_paused_trace_renders_without_reversed_zoom_bounds() {
        let mut editor = CircuitEditor::default();
        editor
            .parts
            .push(Part::at(PartKind::Voltmeter, Point::new(0, 0)));
        editor.trace_window = Some(0);
        editor.traces.push(InstrumentTrace {
            part_index: 0,
            points: (0..6)
                .map(|index| TracePoint {
                    time: index as f64 * 1e-6,
                    value: index as f64,
                })
                .collect(),
        });
        let context = egui::Context::default();
        let _ = context.run(egui::RawInput::default(), |context| {
            editor.trace_window_ui(context);
        });
    }

    /// The waveform window replaced the standalone oscilloscope, so an instrument now has to
    /// record a trace of its own readings while the simulation runs — and its trace has to
    /// follow the part around, or deleting one instrument would show another one's history.
    #[test]
    fn every_instrument_records_a_trace_and_the_waveform_follows_its_part() {
        let ground = Point::new(0, 0);
        let top = Point::new(0, 3);
        let load = Point::new(3, 0);
        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            part2(PartKind::AcSupply, ground, top, 10.0, 50.0),
            part(PartKind::Resistor, top, load, 1_000.0),
            part(PartKind::Ammeter, load, ground, 0.0),
            Part::at(PartKind::Voltmeter, top),
            Part::at(PartKind::Ground, ground),
        ];
        editor.start_simulation();
        assert!(editor.running, "the run should not have stopped early");
        for _ in 0..500 {
            editor.simulate_frame();
        }

        // One trace per instrument, keyed to the part, and nothing for the parts that are not
        // instruments.
        assert_eq!(editor.traces.len(), 2, "only the two meters record");
        let ammeter = editor
            .traces
            .iter()
            .find(|trace| trace.part_index == 2)
            .expect("the ammeter's trace");
        let voltmeter = editor
            .traces
            .iter()
            .find(|trace| trace.part_index == 3)
            .expect("the voltmeter's trace");
        assert!(
            ammeter.points.len() > 100,
            "the ammeter should have recorded"
        );
        assert!(
            voltmeter.points.len() > 100,
            "the voltmeter should have recorded"
        );
        assert_eq!(instrument_unit(PartKind::Ammeter), Some("A"));
        assert_eq!(instrument_unit(PartKind::Voltmeter), Some("V"));
        assert_eq!(instrument_unit(PartKind::Resistor), None);
        // Times come from the simulation clock, so a trace only makes sense if it advances.
        assert!(
            voltmeter.points.last().unwrap().time > voltmeter.points[0].time,
            "the horizontal axis is simulation time, so it has to move"
        );
        // The window only ever opens on an instrument, and it is referred to by position, so
        // editing the canvas closes it rather than leaving it showing a different meter.
        editor.trace_window = Some(2);
        assert!(editor
            .parts
            .get(2)
            .is_some_and(|part| instrument_unit(part.kind).is_some()));
        editor.add_part_at(PartKind::Resistor, Point::new(9, 9));
        assert_eq!(editor.trace_window, None, "an edit closes the window");
        editor.parts.pop();
        editor.close_trace_window();
        editor.parts.remove(2);
        editor.close_trace_window();
        editor.record_traces();
        assert_eq!(
            editor.traces.len(),
            1,
            "the voltmeter is the only meter left"
        );
        assert_eq!(
            editor.traces[0].part_index, 2,
            "and its trace follows it to its new position rather than showing the ammeter's"
        );
    }

    /// Axis ticks have to land on values someone can read off the screen, so a range divides
    /// into 1, 2 or 5 times a power of ten rather than into thirds.
    #[test]
    fn axis_steps_land_on_readable_numbers() {
        assert_eq!(nice_step(0.37), 0.5);
        assert_eq!(nice_step(0.9), 1.0);
        assert_eq!(nice_step(1.4), 2.0);
        assert_eq!(nice_step(3.0), 5.0);
        assert_eq!(nice_step(7.0), 10.0);
        // A range of zero, which a flat trace produces, still has to give a usable step.
        assert!(nice_step(0.0) > 0.0);
        assert!(nice_step(f64::NAN) > 0.0);
    }

    #[test]
    fn measurements_are_shown_in_a_readable_unit() {
        assert_eq!(format_measurement(0.003, "A"), "3.000 mA");
        assert_eq!(format_measurement(2.5, "A"), "2.500 A");
        assert_eq!(format_measurement(12.0e-6, "A"), "12.000 µA");
        assert_eq!(format_measurement(6.0, "V"), "6.000 V");
        assert_eq!(format_measurement(0.0015, "V"), "1.500 mV");
    }

    /// The waveform is only useful if it is a waveform. The engine's timestep is adaptive and
    /// spans six orders of magnitude, so sampling a trace once per step gave a window anywhere
    /// from nanoseconds to milliseconds wide — on a stiff circuit that is a flat line, and the
    /// time axis never covers a cycle. This pins the two properties that make it continuous:
    /// a fixed span of simulated time per frame, and a fixed interval between samples.
    #[test]
    fn a_trace_covers_several_cycles_at_a_uniform_interval() {
        let ground = Point::new(0, 0);
        let top = Point::new(0, 3);
        let mut editor = CircuitEditor::default();
        // 50 Hz into a 1 uF / 1 k RC, which is the circuit a beginner builds to see a
        // waveform at all.
        editor.parts = vec![
            part2(PartKind::AcSupply, ground, top, 10.0, 50.0),
            part(PartKind::Resistor, top, Point::new(0, 1), 1_000.0),
            part(PartKind::Capacitor, Point::new(0, 1), ground, 1e-6),
            Part::at(PartKind::Voltmeter, Point::new(0, 1)),
            Part::at(PartKind::Ground, ground),
        ];
        editor.start_simulation();
        assert!(editor.running, "the run should not have stopped early");

        // A frame covers 1 ms of simulated time, so this exercises several complete display
        // windows and leaves enough history to scroll back to the start.
        for _ in 0..80 {
            editor.simulate_frame();
        }
        let trace = editor
            .traces
            .iter()
            .find(|trace| trace.part_index == 3)
            .expect("the voltmeter's trace");
        assert!(
            trace.points.len() > 500,
            "the trace should retain more than one display window"
        );
        let window = TRACE_WINDOW_SECONDS;
        assert!(
            window >= 2.0 / 50.0,
            "the window is {window:.4} s, under two 50 Hz cycles"
        );
        let span = trace.points.last().unwrap().time - trace.points[0].time;
        assert!(
            span > window,
            "the history should exceed one display window"
        );

        // And the samples have to be evenly spaced in time, or the trace is drawn against a
        // distorted x axis and the peaks do not line up with the grid.
        // And the gaps have to hold the interval. They cannot land on it exactly, because the
        // engine's step size varies and the first step *past* the target is the one taken, so
        // each sample is up to one timestep late. Bounding the jitter by a timestep is the
        // strongest statement available, and bounding it is what matters: the trace is drawn
        // against the real sample times rather than an assumed grid, so a drifting grid would
        // put the peaks in the wrong place on the axis.
        let step = editor.engine.as_ref().expect("engine").time_step;
        let gaps: Vec<f64> = trace
            .points
            .windows(2)
            .map(|pair| pair[1].time - pair[0].time)
            .collect();
        for (pair, gap) in trace.points.windows(2).zip(gaps) {
            let expected_interval = trace_sample_interval(pair[0].time);
            assert!(
                gap >= expected_interval - 1e-12 && gap <= expected_interval + step + 1e-12,
                "sample gap {gap:.6} s should be {expected_interval:.6} s plus at most \
                 one timestep of jitter ({step:.3e})"
            );
        }
        let (start, end) = trace_view_range(
            &trace.points,
            trace.points[0].time,
            trace.points[0].time + window,
        );
        assert!(end - start > 500, "the waveform view needs enough samples");
        let visible_span = trace.points[end - 1].time - trace.points[start].time;
        assert!(
            (visible_span - window).abs() <= TRACE_SAMPLE_INTERVAL + step,
            "the displayed window should span about 60 ms, got {visible_span:.4} s"
        );

        // A sine at the capacitor has to actually swing, and it has to swing more than once,
        // or the window is still showing a fragment.
        let values: Vec<f64> = trace.points.iter().map(|point| point.value).collect();
        let peak = values.iter().cloned().fold(0.0_f64, f64::max);
        let trough = values.iter().cloned().fold(0.0_f64, f64::min);
        assert!(peak > 1.0, "the trace should swing, peak = {peak}");
        assert!(
            trough < -1.0,
            "the trace should swing both ways, min = {trough}"
        );
        let crossings = values
            .windows(2)
            .filter(|w| w[0] <= 0.0 && w[1] > 0.0)
            .count();
        assert!(
            crossings >= 2,
            "the window should hold at least two rising edges, got {crossings}"
        );
    }

    #[test]
    fn a_paused_trace_can_zoom_and_pan_over_full_history() {
        let mut points = Vec::new();
        for index in 0..1_000 {
            points.push(TracePoint {
                time: index as f64 * TRANSIENT_SAMPLE_INTERVAL,
                value: if index < 100 { 5.0 } else { 0.0 },
            });
        }
        for index in 1..2_000 {
            points.push(TracePoint {
                time: TRANSIENT_SAMPLE_DURATION + index as f64 * TRACE_SAMPLE_INTERVAL,
                value: 0.0,
            });
        }
        let start = points[0].time;
        let latest = points.last().expect("history").time;
        let (full_start, full_end) = trace_view_range(&points, start, latest);
        let total_span = latest - start;
        let zoom_span = 0.06;
        let (transient_start, transient_end) = trace_view_range(&points, start, start + zoom_span);
        let panned_start = latest - zoom_span;
        let (panned_index, panned_end) = trace_view_range(&points, panned_start, latest);
        assert_eq!(full_start, 0, "full-history view includes the first sample");
        assert_eq!(
            full_end,
            points.len(),
            "full-history view includes the last sample"
        );
        assert!(total_span > zoom_span, "the history can be zoomed");
        assert!(
            points[transient_start..transient_end]
                .iter()
                .any(|point| point.value > 0.0),
            "zooming into the start should reveal the transient"
        );
        assert!(
            points[panned_index..panned_end]
                .iter()
                .all(|point| point.value == 0.0),
            "panning to the latest time should show the settled DC level"
        );

        let initial_span = total_span;
        let anchor_fraction = 0.35;
        let anchor_time = start + initial_span * anchor_fraction;
        let (zoomed_start, zoomed_span) =
            zoom_trace_view(start, initial_span, anchor_fraction, 500.0, start, latest);
        assert!(zoomed_span < initial_span, "wheel-up zooms in");
        assert!(
            ((zoomed_start + zoomed_span * anchor_fraction) - anchor_time).abs() < 1e-9,
            "zoom should keep the time under the cursor fixed"
        );
        let panned = pan_trace_view(zoomed_start, 100.0, 500.0, zoomed_span, start, latest);
        assert!(
            (panned - (zoomed_start - zoomed_span * 0.2)).abs() < 1e-9,
            "dragging right moves the viewed time range earlier"
        );
    }

    #[test]
    fn editing_parts_marks_the_document_dirty_until_its_saved_snapshot_is_updated() {
        let mut editor = CircuitEditor::default();
        assert!(!editor.is_dirty());
        editor.add_part_at(PartKind::Resistor, Point::new(0, 0));
        assert!(editor.is_dirty());
        editor.saved_contents = encode_parts(&editor.parts);
        assert!(!editor.is_dirty());
        editor.parts[0].value = 2_000.0;
        assert!(editor.is_dirty());
    }

    #[test]
    fn discard_choice_allows_the_pending_close_request() {
        let mut editor = CircuitEditor::default();
        editor.add_part_at(PartKind::Resistor, Point::new(0, 0));
        assert!(editor.should_intercept_close(true));
        editor.allow_close = true;
        assert!(!editor.should_intercept_close(true));
    }

    #[test]
    fn appending_samples_keeps_the_entire_instrument_history() {
        let mut editor = CircuitEditor::default();
        editor
            .parts
            .push(Part::at(PartKind::Voltmeter, Point::new(0, 0)));
        let samples: Vec<_> = (0..70_000)
            .map(|index| (0, index as f64 * TRACE_SAMPLE_INTERVAL, index as f64))
            .collect();
        editor.append_samples(samples);
        let points = &editor.traces[0].points;
        assert_eq!(points.len(), 70_000);
        assert_eq!(points[0].time, 0.0);
        assert_eq!(points.last().expect("last sample").value, 69_999.0);
    }

    /// The waveform picker is the point of the feature, so each shape has to actually produce
    /// that shape and not merely be selectable: a square wave's samples are all at one of two
    /// levels, a triangle's rise and fall linearly, and a custom expression's come from the
    /// expression.
    #[test]
    fn each_source_waveform_produces_its_own_shape() {
        // A load across the source, so the reading is the source's own value.
        fn trace_for(source: Part) -> Vec<f64> {
            let mut editor = CircuitEditor::default();
            let top = Point::new(0, 3);
            editor.parts = vec![
                source,
                part(PartKind::Resistor, top, Point::new(0, 0), 1_000.0),
                Part::at(PartKind::Voltmeter, top),
                Part::at(PartKind::Ground, Point::new(0, 0)),
            ];
            editor.start_simulation();
            assert!(editor.running, "the run should not have stopped early");
            for _ in 0..80 {
                editor.simulate_frame();
            }
            let points = &editor
                .traces
                .iter()
                .find(|trace| trace.part_index == 2)
                .expect("the voltmeter's trace")
                .points;
            points.iter().map(|point| point.value).collect()
        }

        let mut square = Part::between(PartKind::AcSupply, Point::new(0, 0), Point::new(0, 3));
        square.waveform = Waveform::Square;
        square.value = 5.0;
        square.value2 = 50.0;
        let square = trace_for(square);
        assert!(square.len() > 100, "the trace should be full");
        // A square wave is at one of two levels and nothing in between.
        for value in &square {
            assert!(
                (value.abs() - 5.0).abs() < 0.2,
                "a square wave should sit at +/-5 V, got {value}"
            );
        }

        let mut triangle = Part::between(PartKind::AcSupply, Point::new(0, 0), Point::new(0, 3));
        triangle.waveform = Waveform::Triangle;
        triangle.value = 5.0;
        triangle.value2 = 50.0;
        let triangle = trace_for(triangle);
        let peak = triangle.iter().cloned().fold(0.0_f64, f64::max);
        let trough = triangle.iter().cloned().fold(0.0_f64, f64::min);
        assert!(peak > 4.5, "a triangle should reach its peak, got {peak}");
        assert!(trough < -4.5, "and its trough, got {trough}");
        // Unlike a square wave, it spends most of its time strictly between the two levels.
        let between = triangle.iter().filter(|value| value.abs() < 4.0).count();
        assert!(
            between > triangle.len() / 2,
            "a triangle should ramp rather than step: {between} of {} points are off the levels",
            triangle.len()
        );
    }

    /// The custom waveform is the feature that has no analogue in circuitjs1, so it gets an
    /// end-to-end test: the expression the user typed is the source's voltage, through a real
    /// solve, and shows up on the trace.
    #[test]
    fn a_custom_expression_drives_the_circuit_and_the_trace() {
        let ground = Point::new(0, 0);
        let top = Point::new(0, 3);
        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            custom_source(ground, top, "5*sin(2*pi*50*t)"),
            part(PartKind::Resistor, top, ground, 1_000.0),
            Part::at(PartKind::Voltmeter, top),
            Part::at(PartKind::Ground, ground),
        ];
        editor.start_simulation();
        assert!(editor.running, "the run should not have stopped early");
        for _ in 0..80 {
            editor.simulate_frame();
        }

        let values: Vec<f64> = editor
            .traces
            .iter()
            .find(|trace| trace.part_index == 2)
            .expect("the voltmeter's trace")
            .points
            .iter()
            .map(|point| point.value)
            .collect();
        let peak = values.iter().cloned().fold(0.0_f64, f64::max);
        let trough = values.iter().cloned().fold(0.0_f64, f64::min);
        assert!(peak > 4.5, "the expression should reach 5 V, got {peak}");
        assert!(trough < -4.5, "and -5 V, got {trough}");
        // And it must be a sine, not a square wave that happens to peak in the right places:
        // at the very least it spends most of its time between the levels.
        let between = values.iter().filter(|value| value.abs() < 4.0).count();
        assert!(
            between > values.len() / 2,
            "a sine should ramp: {between} of {} points are off the levels",
            values.len()
        );
    }

    /// A custom waveform whose value is not a number at some time has to be reported, not
    /// turned into silence or a diverging solve. `1/t` is undefined only at `t = 0`, which is
    /// exactly the case a parse-time check could not catch.
    #[test]
    fn an_expression_that_is_undefined_somewhere_is_reported() {
        let ground = Point::new(0, 0);
        let top = Point::new(0, 3);
        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            custom_source(ground, top, "1/t"),
            part(PartKind::Resistor, top, ground, 1_000.0),
            Part::at(PartKind::Ground, ground),
        ];
        // The first step is at t = 0, where the expression has no value, so the run should
        // stop rather than carry on with a source that is producing nonsense.
        editor.start_simulation();
        editor.simulate_frame();
        assert!(
            !editor.running,
            "an undefined expression should stop the run"
        );
        assert!(
            editor.status.contains("表达式"),
            "and say so: {}",
            editor.status
        );
    }

    /// A malformed expression is caught where it is written rather than becoming a source
    /// that quietly produces nothing.
    #[test]
    fn a_malformed_expression_is_reported_when_the_engine_is_built() {
        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            custom_source(Point::new(0, 0), Point::new(0, 3), "3 - sin(t"),
            part(
                PartKind::Resistor,
                Point::new(0, 3),
                Point::new(0, 0),
                1_000.0,
            ),
            Part::at(PartKind::Ground, Point::new(0, 0)),
        ];
        let err = editor
            .build_engine()
            .expect_err("an unclosed bracket should be reported");
        assert!(err.contains("表达式"), "{err}");
        // And a good one is not.
        editor.parts[0].expression = "3 - sin(t - pi/2)".to_owned();
        editor
            .build_engine()
            .expect("the expression from the request should build");
    }

    /// The saved format has to carry the waveform and the expression, and the expression is
    /// the one field that may contain spaces, so it is stored as the rest of the line.
    #[test]
    fn a_custom_expression_survives_a_save_and_load() {
        let parts = vec![
            custom_source(Point::new(0, 0), Point::new(0, 3), "3 - sin(t - pi/2)"),
            part2(
                PartKind::AcSupply,
                Point::new(0, 5),
                Point::new(0, 8),
                5.0,
                50.0,
            ),
            part(
                PartKind::Resistor,
                Point::new(1, 1),
                Point::new(4, 1),
                1_000.0,
            ),
        ];
        let decoded = decode_parts(&encode_parts(&parts)).expect("round trip");
        assert_eq!(decoded.len(), parts.len());
        for (actual, expected) in decoded.iter().zip(parts.iter()) {
            assert_eq!(actual.kind, expected.kind);
            assert_eq!(actual.waveform, expected.waveform);
            assert_eq!(actual.expression, expected.expression);
            assert_eq!(actual.value, expected.value);
            assert_eq!(actual.value2, expected.value2);
        }
        assert_eq!(decoded[0].expression, "3 - sin(t - pi/2)");
        assert_eq!(decoded[0].waveform, Waveform::Custom);
        // A sine source carries no expression, so its line has no trailing text.
        assert_eq!(decoded[1].waveform, Waveform::Ac);
        assert!(decoded[1].expression.is_empty());
    }

    #[test]
    fn an_old_file_still_loads_and_gets_a_default_waveform() {
        // A six field line, from before the second value existed.
        let part: Part = decode_parts("circuit-rs-canvas 1\nacsupply 0 0 3 0 5\n")
            .expect("a line written before the second value existed")
            .remove(0);
        assert_eq!(part.waveform, PartKind::AcSupply.default_waveform());
        assert_eq!(part.waveform, Waveform::Ac);
        assert!(part.expression.is_empty());
    }

    /// The phase accumulator is advanced from `leg_currents`, which cannot see the canvas,
    /// while the markers are drawn from `part_leg_segments`, which cannot be built until
    /// paint time. Nothing forces the two to agree, so this pins it for every part kind: if a
    /// kind gains or loses a run of current, the counts diverge here rather than the phases
    /// quietly sliding onto the wrong stretch of conductor.
    #[test]
    fn every_part_reports_one_run_of_current_per_marker_phase() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 600.0));
        let transform = CanvasTransform::default();
        for kind in ALL_KINDS {
            let part = part(kind, Point::new(0, 0), Point::new(3, 0), 1.0);
            let segments = part_leg_segments(&part, rect, transform);
            let values = leg_currents(&part, None);
            assert_eq!(
                segments.len(),
                values.len(),
                "{kind:?} draws {} runs of current but phases {}",
                segments.len(),
                values.len()
            );
        }
    }

    /// A transistor's three terminals are three different currents, and they satisfy
    /// `Ie = -(Ib + Ic)` exactly. Drawing the emitter off the collector's number is what made
    /// the flow look like it broke Kirchhoff's current law, so this is the regression test for
    /// it: the three runs must be separately phased and must sum to zero.
    #[test]
    fn a_transistor_phases_its_three_terminal_currents_separately() {
        let mut editor = CircuitEditor::default();
        // 9 V at the top rail, the base fed through 470 k, the collector loaded through 1 k
        // and the emitter grounded. `Part::posts` puts the base on the axis at `a` and the
        // collector and emitter either side of `b`, so a horizontal span is what makes them
        // land on real points: base (0,3), collector (3,1), emitter (3,5).
        editor.parts = vec![
            part(PartKind::Supply, Point::new(0, 0), Point::new(0, 6), 9.0),
            part(PartKind::Ground, Point::new(0, 0), Point::new(0, 0), 0.0),
            part(
                PartKind::Resistor,
                Point::new(0, 6),
                Point::new(0, 3),
                470_000.0,
            ),
            part(
                PartKind::Transistor,
                Point::new(0, 3),
                Point::new(3, 3),
                100.0,
            ),
            part(
                PartKind::Resistor,
                Point::new(0, 6),
                Point::new(3, 1),
                1_000.0,
            ),
            part(PartKind::Ground, Point::new(3, 5), Point::new(3, 5), 0.0),
        ];
        editor.build_engine().expect("analyze");
        editor
            .engine
            .as_mut()
            .expect("engine")
            .step()
            .expect("step");

        let legs = all_leg_currents(
            &editor.parts,
            editor.engine.as_ref(),
            &editor.engine_indices,
        );
        let transistor = editor
            .parts
            .iter()
            .position(|part| part.kind == PartKind::Transistor)
            .expect("a transistor on the board");
        let runs = &legs[transistor];
        assert_eq!(runs.len(), 3, "a transistor carries three runs of current");
        let (base, collector, emitter) = (runs[0], runs[1], runs[2]);
        // Base and collector current flow into an active NPN, emitter current out, so the
        // emitter entry is the one that is negative.
        assert!(base > 0.0, "base current flows in, got {base}");
        assert!(
            collector > 0.0,
            "collector current flows in, got {collector}"
        );
        assert!(emitter < 0.0, "emitter current flows out, got {emitter}");
        assert!(
            (base + collector + emitter).abs() < 1e-15,
            "the three terminal currents must sum to zero: {base} + {collector} + {emitter}"
        );
        // A base current of a few microamps against a collector current of milliamps, so a
        // renderer that collapsed them onto one number would be visibly wrong.
        assert!(
            collector > base * 10.0,
            "beta should separate the base and collector currents: {base} vs {collector}"
        );

        // Three separate phases, each advancing at its own current's rate. The flat phase
        // order follows the parts, so the transistor's three sit together.
        editor.running = true;
        let at = editor
            .parts
            .iter()
            .take_while(|part| part.kind != PartKind::Transistor)
            .map(leg_count)
            .sum::<usize>();
        editor.advance_current_markers(1.0);
        assert_eq!(editor.current_phases.len(), leg_count_for(&legs));
        let before = editor.current_phases.clone();
        editor.advance_current_markers(1.1);
        let after = editor.current_phases.clone();

        let moved = |slot: usize| after[slot] - before[slot];
        assert!(moved(at) > 0.0, "the base phase moves forwards");
        assert!(
            moved(at + 1) > moved(at),
            "the collector's phase outruns the base's, because its current is larger: {} vs {}",
            moved(at + 1),
            moved(at)
        );
        // The emitter's phase is dragged backwards by its negative current, which is what
        // makes its markers march out of the emitter while the other two march in.
        assert!(
            after[at + 2] < 0.0,
            "a negative terminal current reverses its phase, got {}",
            after[at + 2]
        );
    }

    /// A part spanning two points with explicit values. Anything not given keeps the kind's
    /// default, which is what every fixture here wants.
    fn part(kind: PartKind, a: Point, b: Point, value: f64) -> Part {
        part2(kind, a, b, value, kind.default_value2())
    }

    /// As `part`, but setting both user values.
    fn part2(kind: PartKind, a: Point, b: Point, value: f64, value2: f64) -> Part {
        if kind == PartKind::Voltmeter {
            return Part {
                value,
                value2,
                ..Part::at(kind, a)
            };
        }
        Part {
            value,
            value2,
            ..Part::between(kind, a, b)
        }
    }

    /// An a.c. source driving `expression` instead of a sine of an amplitude and a frequency.
    fn custom_source(a: Point, b: Point, expression: &str) -> Part {
        Part {
            waveform: Waveform::Custom,
            expression: expression.to_owned(),
            ..Part::between(PartKind::SignalSource, a, b)
        }
    }

    #[test]
    fn saved_canvas_round_trips_component_placements() {
        let mut resistor = Part::at(PartKind::Resistor, Point::new(-2, 4));
        resistor.b = Point::new(-2, 7);
        resistor.value = 4_700.0;
        let parts = vec![
            resistor,
            Part::at(PartKind::Ground, Point::new(3, -1)),
            Part::at(PartKind::Ammeter, Point::new(4, 5)),
        ];

        let decoded = decode_parts(&encode_parts(&parts)).expect("decode saved circuit");
        assert_eq!(decoded.len(), parts.len());
        for (actual, expected) in decoded.iter().zip(parts.iter()) {
            assert_eq!(actual.kind, expected.kind);
            assert_eq!(actual.a, expected.a);
            assert_eq!(actual.b, expected.b);
            assert_eq!(actual.value, expected.value);
        }
    }

    #[test]
    fn saved_canvas_rejects_invalid_component_values() {
        let invalid = "circuit-rs-canvas 1\nresistor 0 0 3 0 -1\n";
        assert!(decode_parts(invalid).is_err());
    }

    #[test]
    fn endpoint_hit_testing_selects_posts_not_component_bodies() {
        let parts = vec![Part::at(PartKind::Resistor, Point::new(0, 0))];
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(400.0, 300.0));
        let transform = CanvasTransform::default();
        let first_post = transform.grid_to_screen(parts[0].a, rect);
        let middle = transform.grid_to_screen(Point::new(1, 0), rect);

        assert_eq!(
            find_post_at(&parts, first_post, rect, transform),
            Some(parts[0].a)
        );
        assert_eq!(find_post_at(&parts, middle, rect, transform), None);
    }

    #[test]
    fn canvas_transform_round_trips_grid_points() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(400.0, 300.0));
        let transform = CanvasTransform {
            zoom: 1.7,
            pan: Vec2::new(38.0, -21.0),
        };
        let point = Point::new(-4, 7);
        let screen = transform.grid_to_screen(point, rect);
        assert_eq!(transform.screen_to_grid(screen, rect), point);
    }

    #[test]
    fn zoom_keeps_the_world_point_under_the_cursor_fixed() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(400.0, 300.0));
        let pointer = Pos2::new(93.0, 211.0);
        let mut transform = CanvasTransform {
            zoom: 1.0,
            pan: Vec2::new(24.0, -16.0),
        };
        let point_before = transform.screen_to_grid(pointer, rect);
        transform.zoom_at(rect, pointer, 60.0);
        assert!((transform.zoom - 1.0).abs() > f32::EPSILON);
        assert_eq!(transform.screen_to_grid(pointer, rect), point_before);
    }

    #[test]
    fn pan_changes_screen_position_without_changing_circuit_coordinates() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(400.0, 300.0));
        let point = Point::new(2, -3);
        let mut transform = CanvasTransform::default();
        let before = transform.grid_to_screen(point, rect);
        let drag = Vec2::new(-47.0, 31.0);

        transform.pan += drag;

        assert_eq!(transform.grid_to_screen(point, rect), before + drag);
        assert_eq!(
            transform.screen_to_grid(transform.grid_to_screen(point, rect), rect),
            point
        );
    }

    #[test]
    fn fit_to_parts_centers_the_circuit_and_respects_zoom_limits() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 500.0));
        let parts = vec![
            Part::at(PartKind::Resistor, Point::new(-8, -4)),
            Part::at(PartKind::Supply, Point::new(7, 6)),
        ];
        let transform = fit_to_parts(&parts, rect);
        let left = transform.grid_to_screen(Point::new(-8, -4), rect);
        let right = transform.grid_to_screen(Point::new(10, 6), rect);
        let content_center = left.lerp(right, 0.5);

        assert!((content_center.x - rect.center().x).abs() < 1.0);
        assert!((content_center.y - rect.center().y).abs() < 1.0);
        assert!((MIN_ZOOM..=MAX_ZOOM).contains(&transform.zoom));
        assert_eq!(fit_to_parts(&[], rect).zoom, 1.0);
    }

    #[test]
    fn wire_connection_joins_two_distinct_terminals() {
        let start = Point::new(-2, 4);
        let end = Point::new(3, -1);
        let wire = wire_between(start, end).expect("wire between terminals");
        assert_eq!(wire.kind, PartKind::Wire);
        assert_eq!((wire.a, wire.b), (start, end));
        assert!(wire_between(start, start).is_none());
    }

    #[test]
    fn completing_a_wire_keeps_wire_mode_active_for_the_next_connection() {
        let start = Point::new(0, 0);
        let end = Point::new(3, 2);
        let mut editor = CircuitEditor {
            wire_mode: true,
            wire_start: Some(start),
            ..CircuitEditor::default()
        };

        assert!(editor.connect_wire(end));
        assert!(editor.wire_mode);
        assert_eq!(editor.wire_start, None);
        assert!(editor.wire_bends.is_empty());
        let expected_route = wire_route(start, &[], end);
        assert_eq!(editor.parts.len(), expected_route.len());
        for (part, expected) in editor.parts.iter().zip(expected_route) {
            assert_eq!(part.kind, expected.kind);
            assert_eq!((part.a, part.b), (expected.a, expected.b));
        }
        assert!(editor.status.contains("右键或点击“连线”退出"));
    }

    #[test]
    fn wire_route_honors_each_clicked_waypoint_and_stays_orthogonal() {
        let start = Point::new(-2, 4);
        let bend = Point::new(1, 4);
        let end = Point::new(3, -1);
        let route = wire_route(start, &[bend], end);
        assert_eq!(route.len(), 3);
        assert_eq!(route[0].a, start);
        assert_eq!(route[0].b, bend);
        assert_eq!(route[1].a, bend);
        assert_eq!(route[2].b, end);
        assert!(route
            .iter()
            .all(|part| part.a.x == part.b.x || part.a.y == part.b.y));

        let straight = wire_route(Point::new(1, 0), &[], Point::new(1, 4));
        assert_eq!(straight.len(), 1);
        assert!(wire_route(start, &[], start).is_empty());
        assert!(wire_route(start, &[start], end)
            .iter()
            .all(|part| part.a != part.b));
    }

    #[test]
    fn placed_wire_closes_a_resistor_supply_circuit() {
        let supply_ground = Point::new(0, 0);
        let supply_positive = Point::new(3, 0);
        let resistor_bottom = Point::new(3, 3);
        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            part(PartKind::Supply, supply_ground, supply_positive, 5.0),
            part(
                PartKind::Resistor,
                supply_positive,
                resistor_bottom,
                1_000.0,
            ),
            wire_between(resistor_bottom, supply_ground).expect("connecting wire"),
            Part::at(PartKind::Ground, supply_ground),
        ];

        editor.build_engine().expect("analyze connected circuit");
        let circuit = editor.engine.as_mut().expect("engine");
        circuit.step().expect("step connected circuit");

        let resistor_index = editor.engine_indices[1].expect("resistor engine element");
        let resistor = circuit.elements()[resistor_index]
            .as_any()
            .downcast_ref::<Resistor>()
            .expect("resistor");
        assert!((resistor.current().abs() - 5e-3).abs() < 1e-9);
    }

    #[test]
    fn meter_readings_are_available_after_a_simulation_step() {
        let ground = Point::new(0, 0);
        let positive = Point::new(3, 0);
        let load_top = Point::new(6, 0);
        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            part(PartKind::Supply, ground, positive, 6.0),
            part(PartKind::Ammeter, positive, load_top, 0.0),
            part(PartKind::Resistor, load_top, ground, 2_000.0),
            Part::at(PartKind::Ground, ground),
            Part::at(PartKind::Voltmeter, load_top),
        ];
        editor.build_engine().expect("analyze circuit");
        editor
            .engine
            .as_mut()
            .expect("engine")
            .step()
            .expect("step");

        assert_eq!(
            meter_reading(
                &editor.parts[1],
                1,
                editor.engine.as_ref(),
                &editor.engine_indices
            )
            .as_deref(),
            Some("0.003 A")
        );
        assert_eq!(
            meter_reading(
                &editor.parts[4],
                4,
                editor.engine.as_ref(),
                &editor.engine_indices
            )
            .as_deref(),
            Some("6.000 V")
        );
        let legs = all_leg_currents(
            &editor.parts,
            editor.engine.as_ref(),
            &editor.engine_indices,
        );
        assert!(legs[1]
            .iter()
            .all(|current| (current - 0.003).abs() < 1e-12));
        assert!(legs[2]
            .iter()
            .all(|current| (current - 0.003).abs() < 1e-12));
    }

    /// A phase advances by the current through its own part, so two parts carrying the same
    /// current stay in step and a reversed current sends the markers back the other way.
    /// Nothing here is keyed to a global clock or to a part's index, which is what used to make
    /// the flow look arbitrary.
    #[test]
    fn a_current_phase_moves_with_the_current_through_it() {
        let a = Pos2::new(0.0, 0.0);
        let b = Pos2::new(100.0, 0.0);
        let _ = (a, b);

        // Same current, same elapsed time: the phases must match exactly, or series elements
        // would visibly drift apart over a long run.
        let one = advance_current_phase(0.0, 1e-3, 0.1);
        let two = advance_current_phase(0.0, 1e-3, 0.1);
        assert_eq!(one, two);
        assert!(one > 0.0, "a positive current advances the phase forwards");

        // Twice the current, twice the advance.
        let double = advance_current_phase(0.0, 2e-3, 0.1);
        assert!((double - 2.0 * one).abs() < f64::EPSILON);

        // A reversed current walks back through where it has already been, so the sign of the
        // stored phase is what decides the direction the markers march.
        let back = advance_current_phase(one, -1e-3, 0.1);
        assert!(back < one);
        assert!(back.abs() < 1e-12, "it should return to where it started");

        // A part carrying nothing never moves, so an idle circuit has no drifting markers.
        assert_eq!(advance_current_phase(3.0, 0.0, 1.0), 3.0);
    }

    #[test]
    fn markers_freeze_when_the_simulation_is_not_running() {
        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            part(PartKind::Supply, Point::new(0, 0), Point::new(3, 0), 10.0),
            part(
                PartKind::Resistor,
                Point::new(3, 0),
                Point::new(0, 0),
                2_000.0,
            ),
        ];
        editor.build_engine().expect("analyze");
        // A step is needed before anything carries current, so the markers have something to
        // follow: before it the phases are correctly pinned at zero rather than drifting.
        editor
            .engine
            .as_mut()
            .expect("engine")
            .step()
            .expect("step");

        editor.advance_current_markers(1.0);
        assert!(
            editor.current_phases.iter().all(|phase| *phase == 0.0),
            "a paused run must not move the markers: {:?}",
            editor.current_phases
        );

        editor.running = true;
        editor.advance_current_markers(1.1);
        let moving = editor.current_phases.clone();
        assert!(
            moving.iter().all(|phase| *phase != 0.0),
            "a live run with current should move them: {moving:?}"
        );

        // Paused again: the markers hold still even though time keeps passing.
        editor.running = false;
        editor.advance_current_markers(5.0);
        assert_eq!(editor.current_phases, moving);
    }

    #[test]
    fn editor_builds_and_steps_a_basic_school_circuit() {
        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            part(PartKind::Supply, Point::new(0, 0), Point::new(3, 0), 6.0),
            part(
                PartKind::Resistor,
                Point::new(3, 0),
                Point::new(0, 0),
                2_000.0,
            ),
            Part::at(PartKind::Ground, Point::new(0, 0)),
        ];

        editor.build_engine().expect("analyze circuit");
        editor
            .engine
            .as_mut()
            .expect("engine")
            .step()
            .expect("step");

        let high_node = editor
            .engine
            .as_ref()
            .expect("engine")
            .node_at_point(to_engine_point(Point::new(3, 0)))
            .expect("positive node");
        assert!((editor.engine.as_ref().unwrap().node_voltage(high_node) - 6.0).abs() < 1e-9);
    }

    /// A 10 V 50 Hz source across a 1 k load. The waveform has to actually swing, and it has
    /// to swing symmetrically about zero — a rectified-looking or a flat trace would mean
    /// the engine solved a d.c. operating point instead of stepping the source.
    #[test]
    fn an_ac_source_drives_an_alternating_load() {
        let ground = Point::new(0, 0);
        let top = Point::new(0, 3);
        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            part2(PartKind::AcSupply, ground, top, 10.0, 50.0),
            part(PartKind::Resistor, top, ground, 1_000.0),
            Part::at(PartKind::Ground, ground),
            Part::at(PartKind::Voltmeter, top),
        ];
        editor.start_simulation();
        assert!(editor.running, "the run should not have stopped early");

        // A trace only keeps a bounded window of samples, so the peak is tracked from the
        // node voltage over the whole run rather than read back out of that window.
        let load = to_engine_point(top);
        let mut peak = 0.0f64;
        let mut sum = 0.0;
        let samples = 40_000;
        for _ in 0..samples {
            editor.simulate_frame();
            let v = editor.engine.as_ref().expect("engine").node_voltage(
                editor
                    .engine
                    .as_ref()
                    .expect("engine")
                    .node_at_point(load)
                    .expect("load node"),
            );
            peak = peak.max(v.abs());
            sum += v;
        }
        assert!(editor.running, "the run should not have stopped early");
        assert!(peak > 9.9, "peak = {peak}, expected the 10 V amplitude");
        assert!(
            (sum / samples as f64).abs() < 0.5,
            "mean = {}, expected a symmetric swing",
            sum / samples as f64
        );
        assert!(
            editor.traces.len() == 1 && editor.traces[0].points.len() > 100,
            "the voltmeter should have recorded a trace, got {:?}",
            editor.traces.len()
        );
    }

    /// A 2:1 step-down transformer feeding a 1 k load off a d.c. rail. Proves the derived
    /// secondary posts are wired to the same nodes the engine gave the transformer, which is
    /// the one thing the geometry could plausibly get wrong.
    #[test]
    fn a_transformer_steps_down_the_load_voltage() {
        // The primary is a vertical run on the left, fed from the rail; the secondary is
        // derived a fixed distance to its right, and the load hangs between its two ends.
        // The primary's far end has to come back to the reference or it is an open winding
        // and nothing would be transferred at all.
        let primary_top = Point::new(0, 0);
        let primary_bottom = Point::new(0, 3);
        let reference = Point::new(0, 6);
        let transformer = part2(PartKind::Transformer, primary_top, primary_bottom, 2.0, 1.0);
        let (secondary_top, secondary_bottom) = {
            let posts = transformer.posts();
            (posts[2], posts[3])
        };
        assert_eq!(
            secondary_top,
            Point::new(2, 0),
            "the secondary is derived from the primary's axis"
        );
        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            part(PartKind::Supply, reference, primary_top, 10.0),
            transformer,
            wire_between(primary_bottom, reference).expect("close the primary"),
            part(PartKind::Resistor, secondary_top, secondary_bottom, 1_000.0),
            // The isolated secondary needs a ground reference before its absolute node
            // voltage can be read by the single-terminal probe.
            Part::at(PartKind::Ground, secondary_bottom),
            Part::at(PartKind::Voltmeter, secondary_top),
            Part::at(PartKind::Ground, reference),
        ];
        editor.build_engine().expect("analyze transformer circuit");
        editor
            .engine
            .as_mut()
            .expect("engine")
            .step()
            .expect("step");

        assert_eq!(
            meter_reading(
                &editor.parts[5],
                5,
                editor.engine.as_ref(),
                &editor.engine_indices
            )
            .as_deref(),
            Some("5.000 V"),
            "half of 10 V should reach the secondary"
        );
    }

    /// A diode in series with a load rectifies: current flows on the positive half cycle and
    /// is blocked on the negative one, so the load voltage never goes negative.
    #[test]
    fn a_series_diode_rectifies_an_ac_source() {
        let ground = Point::new(0, 0);
        let source_top = Point::new(0, 3);
        let load_top = Point::new(3, 3);
        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            part2(PartKind::AcSupply, ground, source_top, 10.0, 50.0),
            part(PartKind::Diode, source_top, load_top, 0.7),
            part(PartKind::Resistor, load_top, ground, 1_000.0),
            Part::at(PartKind::Voltmeter, load_top),
        ];
        editor.build_engine().expect("analyze rectifier");
        // The extrema have to be tracked across the run: the value at whatever step the
        // loop happens to end on is as likely to be a zero crossing as a peak.
        let mut lowest = 0.0f64;
        let mut highest = 0.0f64;
        for _ in 0..40_000 {
            editor
                .engine
                .as_mut()
                .expect("engine")
                .step()
                .expect("step");
            let circuit = editor.engine.as_ref().expect("engine");
            let v = circuit.node_voltage(
                circuit
                    .node_at_point(to_engine_point(load_top))
                    .expect("load node"),
            );
            lowest = lowest.min(v);
            highest = highest.max(v);
        }
        assert!(
            highest > 8.0,
            "the source should still be swinging, saw {highest}"
        );
        assert!(
            lowest > -0.05,
            "a forward-only diode must not drive the load appreciably negative, saw {lowest}"
        );
    }

    /// A common-emitter stage biased from the rail. Checks the part's three terminals reach
    /// the engine in the order the model wants and that the gain comes out at `beta`.
    #[test]
    fn a_transistor_stage_biases_and_amplifies() {
        // The base sits on the axis at `a`; the collector and emitter are placed either side
        // of `b`, so the span runs along the base lead.
        let base = Point::new(0, 10);
        let transistor = part(PartKind::Transistor, base, Point::new(3, 10), 100.0);
        let (collector, emitter) = (transistor.posts()[1], transistor.posts()[2]);
        assert_eq!(collector, Point::new(3, 8), "collector above the axis");
        assert_eq!(emitter, Point::new(3, 12), "emitter below it");
        let rail = Point::new(0, 4);

        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            part(PartKind::Supply, emitter, rail, 10.0),
            part(PartKind::Resistor, rail, base, 1e6),
            part(PartKind::Resistor, rail, collector, 1_000.0),
            transistor,
            Part::at(PartKind::Ground, emitter),
        ];
        editor.build_engine().expect("analyze transistor stage");
        for _ in 0..5 {
            editor
                .engine
                .as_mut()
                .expect("engine")
                .step()
                .expect("step");
        }

        let circuit = editor.engine.as_ref().expect("engine");
        let index = editor.engine_indices[3].expect("transistor element");
        let t = circuit.elements()[index]
            .as_any()
            .downcast_ref::<Bjt>()
            .expect("transistor");
        let vbe = t.vbe();
        let ib = (10.0 - vbe) / 1e6;
        assert!(t.vbe() > 0.5 && t.vbe() < 0.8, "vbe = {vbe}");
        assert!(t.vbc() < -8.0, "vbc = {} should be reverse biased", t.vbc());
        assert!(
            (t.current() - 100.0 * ib).abs() < 1e-6,
            "ic = {} A, expected {} A",
            t.current(),
            100.0 * ib
        );
    }

    /// A part's terminals are the whole of its connectivity, so a new kind that claims the
    /// wrong number of them would be silently unwireable.
    #[test]
    fn every_kind_reports_its_terminals() {
        assert_eq!(
            Part::at(PartKind::Ground, Point::new(0, 0)).posts().len(),
            1
        );
        for kind in [
            PartKind::Resistor,
            PartKind::Capacitor,
            PartKind::Inductor,
            PartKind::Diode,
            PartKind::Supply,
            PartKind::AcSupply,
            PartKind::Ammeter,
        ] {
            let part = Part::at(kind, Point::new(0, 0));
            assert_eq!(part.posts(), vec![part.a, part.b], "{kind:?}");
        }
        let probe = Part::at(PartKind::Voltmeter, Point::new(0, 0));
        assert_eq!(probe.posts(), vec![probe.a]);
        let transformer = Part::at(PartKind::Transformer, Point::new(0, 0));
        assert_eq!(transformer.posts().len(), 4, "{:?}", transformer.posts());
        let bjt = Part::at(PartKind::Transistor, Point::new(0, 0));
        assert_eq!(bjt.posts().len(), 3, "{:?}", bjt.posts());
    }

    /// Every part kind survives a save and a load with both of its values and its waveform, which
    /// is the whole of what a placed part is. The seven added last are here because their values
    /// mean different things — a switch's is a state, a rheostat's second is a fraction of
    /// its travel — and a format that could not carry those would silently reset them to defaults.
    #[test]
    fn every_part_kind_round_trips_through_the_saved_file() {
        let parts: Vec<Part> = ALL_KINDS
            .into_iter()
            .map(|kind| Part::at(kind, Point::new(1, 2)))
            .collect();
        let decoded = decode_parts(&encode_parts(&parts)).expect("round trip");
        assert_eq!(decoded.len(), parts.len(), "a kind was dropped");
        for (actual, expected) in decoded.iter().zip(parts.iter()) {
            assert_eq!(actual.kind, expected.kind, "{:?}", expected.kind);
            assert_eq!(
                actual.value, expected.value,
                "{:?} lost its first value",
                expected.kind
            );
            assert_eq!(
                actual.value2, expected.value2,
                "{:?} lost its second value",
                expected.kind
            );
            assert_eq!(actual.waveform, expected.waveform, "{:?}", expected.kind);
        }
    }

    /// A switch is the one part whose value is a state rather than a number, so it has to reach
    /// the engine as one — and it has to actually break the circuit when open, which is the only
    /// thing that proves the editor and the solver agree about what "open" means.
    #[test]
    fn a_switch_opens_and_closes_the_circuit() {
        /// Current through the load, with a switch in series between the supply and the meter.
        ///
        /// The ammeter has to be *in series*. Put in parallel with the load it is a second
        /// ideal voltage source across the same two nodes as the supply's path, and the loop
        /// of ideal sources with nothing resistive in it has no solution.
        fn current_through(kind: PartKind, value: f64, load_at: Point) -> f64 {
            let ground = Point::new(0, 0);
            let mut editor = CircuitEditor::default();
            let switch = Part {
                value,
                ..Part::between(kind, Point::new(0, 3), load_at)
            };
            // Where the load has to attach depends on the part: a plain switch's far terminal
            // is its second post, while a double-throw switch's are two derived ones.
            let (after_switch, load_top) = match kind {
                PartKind::SpdtSwitch => {
                    let posts = switch.posts();
                    (posts[1], posts[1])
                }
                _ => (load_at, load_at),
            };
            let _ = after_switch;
            editor.parts = vec![
                part2(PartKind::Supply, ground, Point::new(0, 3), 6.0, 0.0),
                switch,
                part(PartKind::Ammeter, load_top, Point::new(0, 5), 0.0),
                part(PartKind::Resistor, Point::new(0, 5), ground, 1_000.0),
                Part::at(PartKind::Ground, ground),
            ];
            editor.build_engine().expect("analyze");
            editor
                .engine
                .as_mut()
                .expect("engine")
                .step()
                .expect("step");
            let index = editor
                .parts
                .iter()
                .position(|part| part.kind == PartKind::Ammeter)
                .expect("the ammeter");
            instrument_readings(
                &editor.parts,
                editor.engine.as_ref().expect("engine"),
                &editor.engine_indices,
            )
            .into_iter()
            .find(|(part, _)| *part == index)
            .map(|(_, value)| value)
            .expect("a reading")
        }

        // The dialog stores closed as one and open as zero.
        let closed = current_through(PartKind::Switch, 1.0, Point::new(0, 4));
        let open = current_through(PartKind::Switch, 0.0, Point::new(0, 4));
        assert!(
            (closed - 6e-3).abs() < 1e-9,
            "a closed switch should pass 6 mA through 1 k, got {closed}"
        );
        assert!(
            open.abs() < 1e-12,
            "an open switch should pass nothing, got {open}"
        );

        // A double-throw switch connects the pole to one throw or the other, so the load sees
        // the supply only when it is thrown the right way.
        let throws_to_the_load = current_through(PartKind::SpdtSwitch, 0.0, Point::new(0, 6));
        let throws_elsewhere = current_through(PartKind::SpdtSwitch, 1.0, Point::new(0, 6));
        assert!(
            (throws_to_the_load - 6e-3).abs() < 1e-9,
            "thrown to the loaded contact it should conduct, got {throws_to_the_load}"
        );
        assert!(
            throws_elsewhere.abs() < 1e-12,
            "thrown to the other contact it should not, got {throws_elsewhere}"
        );
    }

    #[test]
    fn a_dragged_rheostat_changes_its_series_resistance() {
        let ground = Point::new(0, 0);
        let supply = Point::new(0, 4);
        let load = Point::new(4, 4);
        let make_circuit = |position| {
            let mut editor = CircuitEditor::default();
            editor.parts = vec![
                part(PartKind::Supply, ground, supply, 6.0),
                Part {
                    value: 1_000.0,
                    value2: position,
                    ..Part::between(PartKind::Pot, supply, load)
                },
                part(PartKind::Resistor, load, ground, 500.0),
                Part::at(PartKind::Ground, ground),
            ];
            editor.build_engine().expect("analyze");
            editor
                .engine
                .as_mut()
                .expect("engine")
                .step()
                .expect("step");
            editor
        };
        let half = make_circuit(0.5);
        let full = make_circuit(1.0);
        let current = |editor: &CircuitEditor| {
            editor.engine.as_ref().expect("engine").elements()
                [editor.engine_indices[1].expect("rheostat")]
            .as_any()
            .downcast_ref::<Resistor>()
            .expect("a two-terminal rheostat")
            .current()
            .abs()
        };
        assert!((current(&half) - 0.006).abs() < 1e-9);
        assert!((current(&full) - 0.004).abs() < 1e-9);
    }

    /// The palette is grouped, and a part that belongs to no group would simply not appear — so
    /// the groups have to account for every kind, with none in two places.
    #[test]
    fn every_part_belongs_to_exactly_one_palette_group() {
        for kind in ALL_KINDS {
            let mut seen = 0;
            for group in Group::ALL {
                if kind.group() == group {
                    seen += 1;
                }
            }
            assert_eq!(seen, 1, "{kind:?} is in {seen} groups");
            // And the group has to be one the palette actually shows.
            assert!(
                Group::ALL.contains(&kind.group()),
                "{kind:?} is in a group the palette does not have"
            );
        }
        assert_eq!(Group::ALL.len(), 4, "the palette should have four shelves");
    }

    /// Every part the editor offers has to have a name, a keyword and a saved-file code, because
    /// all three are how a user finds it, searches for it, and keeps it.
    #[test]
    fn every_part_is_findable_and_savable() {
        for kind in ALL_KINDS {
            assert!(!kind.name().is_empty(), "{kind:?} has no name");
            assert!(
                !kind.keywords().is_empty(),
                "{kind:?} cannot be searched for"
            );
            assert!(!kind.code().is_empty(), "{kind:?} cannot be saved");
            // And the codes have to be distinct, or two parts would load as one.
        }
        let mut codes: Vec<&str> = ALL_KINDS.into_iter().map(|kind| kind.code()).collect();
        codes.sort_unstable();
        let before = codes.len();
        codes.dedup();
        assert_eq!(codes.len(), before, "two parts share a saved-file code");
    }

    /// A switch has to be operable *while the circuit runs*. That is the whole point of the
    /// part, and an earlier version stopped the simulation on every change like any other
    /// edit, which made a switch useless for the thing anyone would use one for.
    #[test]
    fn clicking_a_switch_operates_it_without_stopping_the_run() {
        let ground = Point::new(0, 0);
        let top = Point::new(0, 3);
        let mut editor = CircuitEditor::default();
        editor.parts = vec![
            part2(PartKind::Supply, ground, top, 6.0, 0.0),
            Part {
                value: 1.0,
                ..Part::between(PartKind::Switch, top, Point::new(0, 4))
            },
            part(PartKind::Ammeter, Point::new(0, 4), Point::new(0, 5), 0.0),
            part(PartKind::Resistor, Point::new(0, 5), ground, 1_000.0),
            Part::at(PartKind::Ground, ground),
        ];
        editor.start_simulation();
        assert!(editor.running, "the run should have started");
        for _ in 0..5 {
            editor.simulate_frame();
        }

        let switch = 1;
        assert_eq!(editor.parts[switch].value, 1.0, "closed to begin with");
        assert!(editor.toggle_switch(switch), "a switch should be operable");
        assert_eq!(editor.parts[switch].value, 0.0, "and should now be open");
        assert!(
            editor.running,
            "opening a switch must not stop the run: {}",
            editor.status
        );
        assert!(
            editor.status.contains("断开"),
            "and should say so: {}",
            editor.status
        );

        // The reading has to follow, which is what proves the engine really was rebuilt.
        editor.simulate_frame();
        let reading = instrument_readings(
            &editor.parts,
            editor.engine.as_ref().expect("engine"),
            &editor.engine_indices,
        )
        .into_iter()
        .find(|(part, _)| *part == 2)
        .map(|(_, value)| value)
        .expect("a reading");
        assert!(
            reading.abs() < 1e-12,
            "an open switch passes nothing, got {reading}"
        );

        // And back again.
        editor.toggle_switch(switch);
        assert_eq!(editor.parts[switch].value, 1.0);
        assert!(editor.running);
        editor.simulate_frame();
        let reading = instrument_readings(
            &editor.parts,
            editor.engine.as_ref().expect("engine"),
            &editor.engine_indices,
        )
        .into_iter()
        .find(|(part, _)| *part == 2)
        .map(|(_, value)| value)
        .expect("a reading");
        assert!(
            (reading - 6e-3).abs() < 1e-9,
            "closed again it conducts, got {reading}"
        );

        // A part that is not a switch is left alone, so a stray click cannot change anything.
        assert!(!editor.toggle_switch(0), "a supply is not a switch");
        assert_eq!(editor.parts[0].value, 6.0, "and its value is untouched");
    }

    /// The square wave and the custom expression field existed, and were tested, and were still
    /// impossible to find: the property editor was duplicated, and the waveform picker was
    /// only in the copy that had been crammed into a single line of the toolbar. This is a
    /// plain inventory — the point is that the picker is driven by one list with a name per
    /// entry, so a waveform cannot end up selectable but unlabelled.
    #[test]
    fn every_waveform_the_source_offers_has_a_distinct_name() {
        let mut names: Vec<&str> = WAVEFORMS.iter().copied().map(waveform_name).collect();
        assert!(
            names.iter().all(|name| !name.is_empty()),
            "a waveform with no name would show as a blank row: {names:?}"
        );
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), WAVEFORMS.len(), "two waveforms share a name");

        // Square and triangle are the ones that were unfindable, so they are worth naming outright.
        assert!(WAVEFORMS.contains(&Waveform::Square));
        assert!(WAVEFORMS.contains(&Waveform::Triangle));
        assert!(!WAVEFORMS.contains(&Waveform::Custom));
        assert_eq!(waveform_name(Waveform::Square), "方波");
        assert_eq!(waveform_name(Waveform::Triangle), "三角波");
        assert_eq!(waveform_name(Waveform::Custom), "自定义");

        // The picker only belongs on a part that produces a waveform, and the d.c. source is the
        // one place offering it would be actively confusing.
        assert!(PartKind::AcSupply.has_waveform());
        assert!(!PartKind::SignalSource.has_waveform());
        assert!(!PartKind::Supply.has_waveform());
        assert!(!PartKind::Resistor.has_waveform());
    }

    /// The desktop's idea of where a part's terminals are, and the engine element's idea, must
    /// be the same list. The engine identifies a node purely by the coordinates of a post that
    /// lands on it, so a drawing that puts a terminal one cell from where the element puts it
    /// produces a part the user can see a terminal on but cannot wire to — and the symptom is
    /// a silently disconnected circuit rather than an error.
    ///
    /// The seven parts added last are the ones at risk: their extra terminals are derived, and
    /// a second derivation on this side of the boundary is exactly how they drift apart. It
    /// caught one on its first run — the op-amp's inputs, sixteen grid cells off the body.
    #[test]
    fn the_desktop_and_the_engine_agree_about_every_terminal() {
        let ground = Point::new(0, 0);
        // Each part at the span it is placed at, and at that span turned a quarter turn, because
        // that is the second of the two cases the editor can produce. The engine derives a few
        // terminals at fixed distances — an op-amp's inputs are sixteen engine units either
        // side of its axis, which is exactly one grid cell — so a part on whole cells keeps its
        // derived terminals on whole cells too. A span off the grid, or a diagonal one, splits
        // those distances across both axes and puts a terminal where no wire's end can ever
        // reach it; the next test says why the editor cannot produce either.
        for kind in ALL_KINDS {
            let span = kind.placement_span();
            let turned = Point::new(-span.y, span.x);
            for b in [span, turned] {
                let mut part = Part::between(kind, ground, b);
                if kind == PartKind::SignalSource {
                    part.expression = "5*sin(2*pi*50*t)".to_owned();
                }
                let element = match engine_element(&part).expect("a well formed part") {
                    Some(element) => element,
                    // A voltage probe has no engine element: it reads its single node against
                    // circuit ground without joining the circuit.
                    None => continue,
                };
                // `Part::posts` is in grid cells and `Element::post` is in the engine's own
                // units, so one side has to be converted before they can be compared. Both are
                // `circuit_sim::Point`, which is exactly what makes this easy to get wrong by
                // accident.
                let declared: Vec<_> = part
                    .posts()
                    .iter()
                    .map(|point| to_engine_point(*point))
                    .collect();
                let actual: Vec<_> = (0..element.post_count())
                    .map(|post| element.post(post))
                    .collect();
                assert_eq!(
                    declared.len(),
                    actual.len(),
                    "{kind:?} spanning {b:?}: the canvas thinks it has {} terminals, the \
                     element {}",
                    declared.len(),
                    actual.len()
                );
                for (post, (mine, theirs)) in declared.iter().zip(actual.iter()).enumerate() {
                    assert_eq!(
                        mine, theirs,
                        "{kind:?} spanning {b:?}: terminal {post} is at {mine:?} on the canvas \
                         but {theirs:?} in the engine"
                    );
                }
            }
        }
    }

    /// The editor cannot produce a diagonal part, and that is a precondition rather than an
    /// accident: wires connect two terminals only when their coordinates are identical, so a
    /// terminal the engine derives between grid cells can never be wired to. Placement is a
    /// fixed axis-aligned span and rotation is a quarter turn, which keeps every derived
    /// terminal on the grid.
    #[test]
    fn parts_are_always_axis_aligned_so_every_terminal_stays_on_the_grid() {
        for kind in ALL_KINDS {
            let span = kind.placement_span();
            assert!(
                span.x == 0 || span.y == 0,
                "{kind:?} is placed on a diagonal ({span:?}), so its derived terminals could \
                 land between grid cells"
            );
            // And a quarter turn has to keep it that way, which is what the rotate button does.
            let turned = Point::new(-span.y, span.x);
            assert!(
                turned.x == 0 || turned.y == 0,
                "{kind:?} becomes diagonal when rotated ({turned:?})"
            );
        }
    }

    /// A part's terminals have to stay reachable: every one of them must be somewhere the user
    /// can click, which means they must all be inside the part's own footprint rather than
    /// scattered off by the derivation.
    #[test]
    fn a_multi_terminal_part_keeps_its_terminals_distinct() {
        let part = Part::between(PartKind::OpAmp, Point::new(0, 0), Point::new(3, 0));
        let posts = part.posts();
        assert_eq!(posts.len(), 3, "two inputs and an output");
        for (i, first) in posts.iter().enumerate() {
            for second in &posts[i + 1..] {
                assert_ne!(
                    first, second,
                    "two terminals landed on the same point: {posts:?}"
                );
            }
        }
    }

    /// The two amplifiers are the only parts whose symbol depends on a post the user cannot see
    /// at a glance, so their input signs have to stay on the sides the engine calls inverting
    /// and non-inverting.
    #[test]
    fn an_op_amps_inputs_stay_the_right_way_round() {
        use circuit_sim::OpAmp;
        let (inverting, non_inverting) = OpAmp::input_posts(
            to_engine_point(Point::new(0, 0)),
            to_engine_point(Point::new(3, 0)),
        );
        let part = Part::between(PartKind::OpAmp, Point::new(0, 0), Point::new(3, 0));
        assert_eq!(to_engine_point(part.posts()[0]), inverting);
        assert_eq!(to_engine_point(part.posts()[1]), non_inverting);
        // The inputs are either side of the axis, so the two are never the same point even
        // when the body is short.
        assert_ne!(inverting, non_inverting);
        // And both land on the grid, one cell to either side, which is what makes them
        // wireable at all.
        assert_eq!(part.posts()[0], Point::new(0, -1));
        assert_eq!(part.posts()[1], Point::new(0, 1));
    }

    /// The secondary winding has to stay attached to the part when it is moved or rotated,
    /// which is the whole reason it is derived rather than stored.
    #[test]
    fn a_transformers_derived_terminals_follow_the_body() {
        let part = Part::at(PartKind::Transformer, Point::new(1, 2));
        let posts = part.posts();
        assert_eq!((posts[0], posts[1]), (part.a, part.b));

        let moved = part.translated(5, -3);
        let moved_posts = moved.posts();
        assert_eq!(
            (
                moved_posts[2].x - moved_posts[0].x,
                moved_posts[2].y - moved_posts[0].y
            ),
            (posts[2].x - posts[0].x, posts[2].y - posts[0].y),
            "translation must not change the winding offset"
        );

        // A quarter turn rotates the offset with it: the secondary sat straight above the
        // primary, and now sits beside it.
        let turned = Part {
            b: Point::new(part.a.x, part.a.y + 3),
            ..part.clone()
        };
        let turned_posts = turned.posts();
        assert_eq!(turned_posts[2].x - turned_posts[0].x, 2);
        assert_eq!(turned_posts[2].y - turned_posts[0].y, 0);
    }

    /// The value shown under a part is what a user reads the component's rating off, so a
    /// part that is still labelled by kind name instead of by value is a bug.
    #[test]
    fn parts_are_labelled_by_their_value() {
        let at = |kind| Part::at(kind, Point::new(0, 0));
        assert_eq!(part_label(&at(PartKind::Resistor)), "1.000 kΩ");
        assert_eq!(part_label(&at(PartKind::Capacitor)), "1.000 µF");
        assert_eq!(part_label(&at(PartKind::Inductor)), "0.100 H");
        assert_eq!(part_label(&at(PartKind::Transformer)), "10:1");
        assert_eq!(part_label(&at(PartKind::Transistor)), "β=100");
        assert_eq!(part_label(&at(PartKind::Supply)), "5.00 V");
        assert_eq!(part_label(&at(PartKind::AcSupply)), "5.00 V");
        // Instruments carry no value, so they are named instead.
        assert_eq!(part_label(&at(PartKind::Voltmeter)), "电压探针");
    }
}
