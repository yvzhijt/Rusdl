//! A software renderer for the canvas, so the part symbols can be looked at as a picture
//! rather than reasoned about.
//!
//! egui draws through a GPU backend, so there is no headless way to get at the pixels. The
//! tessellated output is a plain list of triangles, though, and rasterising those is little
//! more work than the GPU does: sample the font atlas for glyphs, blend a flat colour for
//! everything else. It is only as good as it needs to be to be worth looking at.
//!
//! Run with `cargo test -p circuit-desktop render_symbol_sheet -- --nocapture`; the image
//! lands in `circuit-desktop/target/symbols.png`.

use circuit_sim::{Expr, Point, Waveform};
use eframe::egui::{pos2, Align2, Color32, Context, FontId, Pos2, Rect, Sense, TextureId, Vec2};
use eframe::epaint;
use png::Encoder;

use crate::canvas::{draw_board, draw_waveform, leg_count, CanvasTransform, TracePoint, GRID};
use crate::model::{waveform_name, Part, PartKind};

/// An RGBA canvas with straight (non-premultiplied) alpha, which is what a PNG wants.
struct Target {
    width: usize,
    height: usize,
    pixels: Vec<[u8; 4]>,
}

impl Target {
    fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            pixels: vec![[255, 255, 255, 255]; width * height],
        }
    }

    /// Source-over composite of a premultiplied source colour, as epaint defines them.
    fn blend(&mut self, x: usize, y: usize, source: Color32) {
        if x >= self.width || y >= self.height || source.a() == 0 {
            return;
        }
        let alpha = source.a() as u32;
        let inverse = 255 - alpha;
        let slot = &mut self.pixels[y * self.width + x];
        for (channel, value) in [source.r(), source.g(), source.b()].into_iter().enumerate() {
            // The incoming colour has to be un-premultiplied before a non-premultiplied
            // blend, or a translucent pixel comes out too dark.
            let straight = (value as u32 * 255) / alpha.max(1);
            slot[channel] = ((straight * alpha + slot[channel] as u32 * inverse) / 255) as u8;
        }
        slot[3] = (alpha + slot[3] as u32 * inverse / 255) as u8;
    }
}

/// Fills one triangle, interpolating texture coordinates and colour across it.
///
/// Winding is ignored on purpose: egui does not use a consistent order, and a symbol is not
/// a solid object, so a back-facing triangle is still meant to be seen.
fn fill_triangle(
    target: &mut Target,
    atlas: &egui::epaint::FontImage,
    v: [Pos2; 3],
    uv: [[f32; 2]; 3],
    colors: [Color32; 3],
    textured: bool,
    clip: Rect,
) {
    let min_x = v
        .iter()
        .map(|p| p.x)
        .fold(f32::MAX, f32::min)
        .floor()
        .max(0.0) as usize;
    let max_x = v
        .iter()
        .map(|p| p.x)
        .fold(f32::MIN, f32::max)
        .ceil()
        .max(0.0) as usize;
    let min_y = v
        .iter()
        .map(|p| p.y)
        .fold(f32::MAX, f32::min)
        .floor()
        .max(0.0) as usize;
    let max_y = v
        .iter()
        .map(|p| p.y)
        .fold(f32::MIN, f32::max)
        .ceil()
        .max(0.0) as usize;
    let edge = |a: Pos2, b: Pos2, p: Pos2| (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    let area = edge(v[0], v[1], v[2]);
    if area.abs() < f32::EPSILON {
        return;
    }
    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let p = pos2(x as f32 + 0.5, y as f32 + 0.5);
            if !clip.contains(p) {
                continue;
            }
            let w0 = edge(v[1], v[2], p) / area;
            let w1 = edge(v[2], v[0], p) / area;
            let w2 = edge(v[0], v[1], p) / area;
            if !(w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0) && !(w0 <= 0.0 && w1 <= 0.0 && w2 <= 0.0) {
                continue;
            }
            let mut color = Color32::BLACK;
            for channel in 0..4 {
                let value = w0 * colors[0][channel] as f32
                    + w1 * colors[1][channel] as f32
                    + w2 * colors[2][channel] as f32;
                color[channel] = value.round().clamp(0.0, 255.0) as u8;
            }
            if textured {
                let u = (w0 * uv[0][0] + w1 * uv[1][0] + w2 * uv[2][0])
                    .clamp(0.0, atlas.size[0] as f32 - 1.0) as usize;
                let t = (w0 * uv[0][1] + w1 * uv[1][1] + w2 * uv[2][1])
                    .clamp(0.0, atlas.size[1] as f32 - 1.0) as usize;
                let coverage = atlas.pixels[t * atlas.size[0] + u];
                // The atlas is white with coverage in the alpha channel, so scaling the
                // vertex colour by it is the whole of texturing.
                let coverage = coverage.clamp(0.0, 1.0);
                for channel in 0..4 {
                    color[channel] = (color[channel] as f32 * coverage).round() as u8;
                }
            }
            target.blend(x, y, color);
        }
    }
}

fn render(ctx: &Context, size: Vec2, build: impl Fn(&egui::Painter, Rect)) -> Target {
    let input = egui::RawInput {
        screen_rect: Some(Rect::from_min_size(pos2(0.0, 0.0), size)),
        ..Default::default()
    };
    // Two passes. Glyphs are added to the font atlas *while* the first frame is being laid
    // out, and adding one can repack the atlas, which moves every glyph that frame already
    // recorded a UV for. Text drawn on the second pass therefore samples an atlas that has
    // stopped moving — which is also what the real app effectively gets after its first frame.
    let build = build;
    for pass in 0..2 {
        let full = ctx.run(input.clone(), |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                let (rect, _) = ui.allocate_exact_size(size, Sense::hover());
                build(ui.painter(), rect);
            });
        });
        if pass == 0 {
            continue;
        }
        return rasterize(ctx, &full);
    }
    unreachable!("the loop returns on its second pass")
}

fn rasterize(ctx: &Context, full: &egui::FullOutput) -> Target {
    let atlas = ctx.fonts(|fonts| fonts.image());
    let pixels_per_point = ctx.pixels_per_point();
    let screen = ctx.input(|i| i.screen_rect.size());
    let width = (screen.x * pixels_per_point).round() as usize;
    let height = (screen.y * pixels_per_point).round() as usize;
    let mut target = Target::new(width.max(1), height.max(1));

    for clipped in ctx.tessellate(full.shapes.clone(), pixels_per_point) {
        let epaint::Primitive::Mesh(mesh) = &clipped.primitive else {
            continue;
        };
        let textured = mesh.texture_id == TextureId::default();
        for triangle in mesh.indices.chunks_exact(3) {
            let corners: [Pos2; 3] =
                std::array::from_fn(|i| mesh.vertices[triangle[i] as usize].pos * pixels_per_point);
            let uvs: [[f32; 2]; 3] = std::array::from_fn(|i| {
                let uv = mesh.vertices[triangle[i] as usize].uv;
                if textured {
                    // Glyph UVs are normalised against the atlas; everything else is
                    // already in screen pixels.
                    [uv.x * atlas.size[0] as f32, uv.y * atlas.size[1] as f32]
                } else {
                    [uv.x * width as f32, uv.y * height as f32]
                }
            });
            let colors: [Color32; 3] =
                std::array::from_fn(|i| mesh.vertices[triangle[i] as usize].color);
            let clip = Rect::from_min_max(
                clipped.clip_rect.min * pixels_per_point,
                clipped.clip_rect.max * pixels_per_point,
            );
            fill_triangle(&mut target, &atlas, corners, uvs, colors, textured, clip);
        }
    }
    target
}

fn write_png(target: &Target, path: &std::path::Path) {
    let mut bytes = Vec::with_capacity(target.width * target.height * 3);
    for pixel in &target.pixels {
        bytes.extend_from_slice(&pixel[..3]);
    }
    let mut encoder = Encoder::new(
        std::fs::File::create(path).expect("create png"),
        target.width as u32,
        target.height as u32,
    );
    encoder.set_color(png::ColorType::Rgb);
    encoder
        .write_header()
        .expect("png header")
        .write_image_data(&bytes)
        .expect("png data");
}

/// Every symbol the palette can place, on a horizontal and a vertical span, with a meter
/// reading under the instruments so the label path is exercised too.
#[test]
fn render_symbol_sheet() {
    // Every kind the editor offers, so a new symbol cannot be added without showing up here.
    let kinds = PartKind::ALL;
    // Laid out on a coarse grid so nothing overlaps, then scaled up and panned so the whole
    // thing lands inside the sheet without any part being clipped. The row pitch has to clear
    // the transformer's secondary winding, which reaches four cells to the side of its primary.
    let columns = 5;
    let cell = 8;
    let row_pitch = 14;
    // Every kind at its own span, both laid out along the grid and turned a quarter turn. The
    // span comes from the part rather than a number here, because the parts no longer agree on
    // one: a rheostat needs an even span for its slider to land on the grid, and a
    // transformer's runs along its windings instead of across them.
    let parts: Vec<Part> = kinds
        .iter()
        .enumerate()
        .flat_map(|(i, kind)| {
            [false, true].map(move |vertical| {
                let origin = Point::new(
                    ((i % columns) * cell) as i32,
                    ((i / columns) * row_pitch + usize::from(vertical) * 6) as i32,
                );
                let mut part = Part::at(*kind, origin);
                let span = kind.placement_span();
                part.b = if vertical {
                    Point::new(origin.x, origin.y + span.y.abs().max(span.x.abs()))
                } else {
                    Point::new(origin.x + span.x, origin.y + span.y)
                };
                part
            })
        })
        .collect();
    // The sheet is sized from the parts' own extent rather than from the layout constants, so
    // adding a part with a wider symbol cannot quietly push the top row off the image.
    let extent = parts.iter().flat_map(|part| part.posts()).fold(
        (i32::MAX, i32::MIN, i32::MAX, i32::MIN),
        |(min_x, max_x, min_y, max_y), point| {
            (
                min_x.min(point.x),
                max_x.max(point.x),
                min_y.min(point.y),
                max_y.max(point.y),
            )
        },
    );
    let (grid_w, grid_h) = (
        (extent.1 - extent.0 + 4) as f32,
        (extent.3 - extent.2 + 8) as f32,
    );
    let pan = Point::new(extent.0 - 2, extent.2 - 4);
    let readings: Vec<Option<String>> = parts
        .iter()
        .map(|part| {
            matches!(part.kind, PartKind::Voltmeter | PartKind::Ammeter)
                .then(|| "3.14 A".to_owned())
        })
        .collect();

    // A working common-emitter stage below the sheet, so the marker flow can be looked at as
    // well as the symbols. The currents are the ones that stage really runs at — 5 mA down
    // the wire, and at the transistor 10 uA in at the base, 5 mA in at the collector and
    // 5.01 mA *out* at the emitter — because the whole point of the three-phase rendering is
    // that the emitter marches the other way from the other two while the three still sum to
    // zero. A single phase per part cannot draw that, which is what used to look like a
    // broken Kirchhoff.
    let loop_origin = Point::new(0, 34);
    let at = |x: i32, y: i32| Point::new(loop_origin.x + x, loop_origin.y + y);
    let loop_parts = vec![
        Part {
            value: 10.0,
            ..Part::between(PartKind::Supply, loop_origin, at(4, 0))
        },
        Part::between(PartKind::Ammeter, at(4, 0), at(6, 0)),
        Part {
            value: 2_000.0,
            ..Part::between(PartKind::Resistor, at(6, 0), at(9, 0))
        },
        Part {
            value: 500.0,
            ..Part::between(PartKind::Transistor, at(3, 1), at(4, 1))
        },
        Part {
            value: 100_000.0,
            ..Part::between(PartKind::Resistor, at(3, 1), at(1, 1))
        },
        Part::between(PartKind::Wire, at(9, 0), at(9, 5)),
        Part::between(PartKind::Wire, at(9, 5), loop_origin),
        Part::at(PartKind::Ground, at(9, 5)),
        Part::at(PartKind::Ground, loop_origin),
    ];
    let all_parts: Vec<Part> = parts.iter().chain(loop_parts.iter()).cloned().collect();
    let loop_readings = vec![
        None,
        Some("5.000 mA".to_owned()),
        Some("2.000 k".to_owned()),
        Some("500".to_owned()),
        Some("100.0 k".to_owned()),
        None,
        None,
        None,
        None,
    ];
    let readings: Vec<Option<String>> = readings.into_iter().chain(loop_readings).collect();
    // The sheet has no current; the stage below it runs at the currents listed above. Each part
    // reports one entry per run of current, so the transistor contributes three.
    let currents: Vec<Vec<f64>> = all_parts
        .iter()
        .enumerate()
        .map(|(i, part)| {
            let runs = leg_count(part);
            let mut values = vec![0.0; runs];
            if i >= parts.len() {
                for value in values.iter_mut() {
                    *value = 5e-3;
                }
                if part.kind == PartKind::Transistor {
                    // Base 10 uA in, collector 5 mA in, emitter 5.01 mA out.
                    values = vec![1e-5, 5e-3, -(5e-3 + 1e-5)];
                }
            }
            values
        })
        .collect();
    // The phases a real run would have reached by now: the wire and the series parts carry the
    // same 5 mA so their markers line up, and the emitter's is negative because its current is.
    let phases: Vec<f64> = all_parts
        .iter()
        .enumerate()
        .flat_map(|(i, part)| {
            let on = i >= parts.len();
            (0..leg_count(part)).map(move |_| if on { 40.0 } else { 0.0 })
        })
        .collect();

    let sheet = Vec2::new(1600.0, 1200.0);
    let zoom = (sheet.x / (grid_w * GRID)).min(sheet.y / (grid_h * GRID));
    let transform = CanvasTransform {
        zoom,
        // The parts start at `pan` in grid coordinates, so the view is shifted by that rather
        // than assumed to start at the origin.
        pan: Vec2::new(
            pan.x as f32 * GRID * zoom - grid_w * GRID * zoom / 2.0,
            pan.y as f32 * GRID * zoom - grid_h * GRID * zoom / 2.0,
        ),
    };
    let ctx = Context::default();

    let target = render(&ctx, sheet, |painter, rect| {
        draw_board(
            painter,
            rect,
            &all_parts,
            None,
            None,
            &[],
            transform,
            &currents,
            &phases,
            &readings,
            None,
            None,
        );
    });

    // The waveform the double click opens, drawn straight to the sheet so the axes and the
    // trace can be checked without launching the app.
    let waveform = render(&ctx, Vec2::new(560.0, 300.0), |painter, rect| {
        let trace: Vec<TracePoint> = (0..300)
            .map(|i| {
                let t = i as f64 * 2e-4;
                TracePoint {
                    time: t,
                    // 5 V at 50 Hz, which is a mains-frequency signal with a 60 ms window: a
                    // few cycles, so the axis ticks and the trace are both readable. A range
                    // that is not symmetric about zero is worth checking too, so a little
                    // DC offset is added.
                    value: 0.6 + 5.0 * (2.0 * std::f64::consts::PI * 50.0 * t).sin(),
                }
            })
            .collect();
        draw_waveform(painter, rect, &trace, "V", None);
    });

    // The waveform the double click opens, drawn straight to the sheet so the axes and the
    // trace can be checked without launching the app. One panel per source waveform, so the
    // shapes can be compared against each other rather than taken on trust.
    //
    // The custom panel uses `3 - sin(2*pi*50*t - pi/2)` rather than the bare `3 - sin(t -
    // pi/2)`: the expression is in *seconds*, so without the `2*pi*50*` frequency term it
    // completes a cycle every 6.28 s and would sit almost still across a 60 ms window. The
    // offset is the point of the example, and this keeps it visible.
    let waveforms = [
        (Waveform::Ac, "5*sin(2*pi*50*t)"),
        (Waveform::Square, ""),
        (Waveform::Triangle, ""),
        (Waveform::Sawtooth, ""),
        (Waveform::Pulse, ""),
        (Waveform::Custom, "3 - sin(2*pi*50*t - pi/2)"),
    ];
    let column = 420.0_f32;
    let row = 200.0_f32;
    let trace_sheet = Vec2::new(column * waveforms.len() as f32 + 20.0, row + 20.0);
    let traces = render(&ctx, trace_sheet, |painter, _rect| {
        for (index, (waveform, expression)) in waveforms.iter().enumerate() {
            // A 60 ms window of a 50 Hz signal, which is three cycles, at 100 us per sample.
            let points: Vec<TracePoint> = (0..600)
                .map(|i| {
                    let time = i as f64 * 1e-4;
                    let amplitude = 5.0;
                    let value = match waveform {
                        Waveform::Ac => {
                            amplitude * (2.0 * std::f64::consts::PI * 50.0 * time).sin()
                        }
                        Waveform::Square => {
                            let phase = (2.0 * std::f64::consts::PI * 50.0 * time)
                                .rem_euclid(2.0 * std::f64::consts::PI);
                            if phase > std::f64::consts::PI {
                                -amplitude
                            } else {
                                amplitude
                            }
                        }
                        Waveform::Triangle => {
                            let phase = (2.0 * std::f64::consts::PI * 50.0 * time)
                                .rem_euclid(2.0 * std::f64::consts::PI);
                            amplitude
                                * (if phase < std::f64::consts::PI {
                                    phase * (2.0 / std::f64::consts::PI) - 1.0
                                } else {
                                    1.0 - (phase - std::f64::consts::PI)
                                        * (2.0 / std::f64::consts::PI)
                                })
                        }
                        Waveform::Sawtooth => {
                            let phase = (2.0 * std::f64::consts::PI * 50.0 * time)
                                .rem_euclid(2.0 * std::f64::consts::PI);
                            phase * (amplitude / std::f64::consts::PI) - amplitude
                        }
                        Waveform::Pulse => {
                            let phase = (2.0 * std::f64::consts::PI * 50.0 * time)
                                .rem_euclid(2.0 * std::f64::consts::PI);
                            if phase < std::f64::consts::PI * 0.5 {
                                amplitude
                            } else {
                                0.0
                            }
                        }
                        Waveform::Custom => Expr::parse(expression)
                            .expect("the example expression should parse")
                            .eval(time),
                        _ => 0.0,
                    };
                    TracePoint { time, value }
                })
                .collect();
            let cell = Rect::from_min_size(
                Pos2::new(10.0 + index as f32 * column, 10.0),
                Vec2::new(column - 10.0, row - 20.0),
            );
            painter.text(
                cell.left_top(),
                Align2::LEFT_TOP,
                waveform_name(*waveform),
                FontId::proportional(12.0),
                Color32::LIGHT_GRAY,
            );
            draw_waveform(
                painter,
                Rect::from_min_size(cell.left_top() + Vec2::new(0.0, 16.0), cell.size()),
                &points,
                "V",
                None,
            );
        }
    });

    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/symbols.png");
    std::fs::create_dir_all(path.parent().expect("target")).expect("create target");
    write_png(&target, &path);
    let waveform_path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/waveform.png");
    write_png(&waveform, &waveform_path);
    let traces_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/waveforms.png");
    write_png(&traces, &traces_path);
    println!(
        "wrote {}, {} and {} (zoom {zoom:.2})",
        path.display(),
        waveform_path.display(),
        traces_path.display()
    );
}
