//! A Windows GUI-subsystem binary in release, so starting it does not flash up a console
//! window behind the app.
//!
//! Only for release builds. A debug build keeps its console, because that is where a panic
//! message and the font warning below go, and taking it away would make a debug run fail
//! silently. The two places that used to print to stderr now report into the window itself,
//! and a panic in release opens a message box, because a process with no console has nowhere
//! else to say what went wrong.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod app;
mod canvas;
mod model;
mod persistence;
#[cfg(test)]
mod render;

use app::CircuitEditor;
use eframe::egui::{self, FontFamily};
use std::path::PathBuf;

/// Shows a message in a dialog, for the failures a console-less build cannot print.
///
/// A no-op off Windows, and never returns an error of its own: it is called from a panic hook,
/// where a second panic would be worse than a lost message.
#[cfg(target_os = "windows")]
fn show_message(title: &str, body: &str) {
    rfd::MessageDialog::new()
        .set_title(title)
        .set_description(body.to_owned())
        .set_buttons(rfd::MessageButtons::Ok)
        .show();
}

#[cfg(not(target_os = "windows"))]
fn show_message(_title: &str, _body: &str) {}

fn main() -> eframe::Result<()> {
    // Installed before anything can panic, so that a failure in release still says why instead
    // of closing without a word. `std` prints the panic itself to the console, which a
    // GUI-subsystem build does not have, hence the dialog.
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        default_hook(info);
        show_message("Rusdl 崩溃", &info.to_string());
    }));

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1120.0, 760.0])
            .with_min_inner_size([820.0, 560.0])
            .with_icon(load_icon()),
        ..Default::default()
    };
    let result = eframe::run_native(
        "Rusdl - 电路仿真",
        options,
        Box::new(|creation_context| {
            // A missing Chinese font used to be an `eprintln!`, which a console-less build
            // cannot show. It goes into the editor's status line instead, where it is seen at
            // the moment the window appears rather than never.
            let warning = install_chinese_font(&creation_context.egui_ctx);
            let mut editor = CircuitEditor::default();
            if let Some(warning) = warning {
                editor.set_status(warning);
            }
            Ok(Box::new(editor))
        }),
    );
    if let Err(err) = &result {
        // Same reasoning: without a console this error would otherwise go nowhere, and the
        // symptom would be a double click that appears to do nothing.
        show_message("Rusdl 无法启动", &err.to_string());
    }
    result
}

/// Installs the Chinese UI font, returning a message to show if it could not be found.
///
/// Returns rather than prints, because there is nowhere to print to in a release build.
fn install_chinese_font(ctx: &egui::Context) -> Option<String> {
    let bytes = match load_chinese_font() {
        Ok(bytes) => bytes,
        Err(reason) => {
            return Some(format!(
                "未找到可用的中文字体，界面将使用默认字体显示中文，可能出现方框。{reason}"
            ))
        }
    };

    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "circuit_chinese".to_owned(),
        egui::FontData::from_owned(bytes).into(),
    );
    for family in [FontFamily::Proportional, FontFamily::Monospace] {
        fonts
            .families
            .entry(family)
            .or_default()
            .insert(0, "circuit_chinese".to_owned());
    }
    ctx.set_fonts(fonts);
    None
}

fn load_chinese_font() -> Result<Vec<u8>, String> {
    #[cfg(target_os = "windows")]
    let candidates = {
        let fonts = std::env::var_os("SystemRoot")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from(r"C:\Windows"))
            .join("Fonts");
        ["Deng.ttf", "simhei.ttf", "simsunb.ttf"]
            .into_iter()
            .map(|name| fonts.join(name))
            .collect::<Vec<_>>()
    };

    #[cfg(target_os = "linux")]
    let candidates = [
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttf",
        "/usr/share/fonts/truetype/arphic/ukai.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    ]
    .into_iter()
    .map(PathBuf::from)
    .collect::<Vec<_>>();

    #[cfg(target_os = "macos")]
    let candidates = [
        "/System/Library/Fonts/Supplemental/Songti.ttc",
        "/System/Library/Fonts/STHeiti Light.ttc",
    ]
    .into_iter()
    .map(PathBuf::from)
    .collect::<Vec<_>>();

    let mut failures = Vec::new();
    for path in candidates {
        match std::fs::read(&path) {
            Ok(bytes) => return Ok(bytes),
            // A file that was there and could not be read is worth reporting; one that is not
            // there at all is just the next candidate on the list.
            Err(err) if path.exists() => failures.push(format!("{}: {err}", path.display())),
            Err(_) => {}
        }
    }
    Err(if failures.is_empty() {
        "系统中没有找到任何候选中文字体。".to_owned()
    } else {
        format!("读取中文字体失败：{}", failures.join("; "))
    })
}

fn load_icon() -> egui::IconData {
    let icon = image::load_from_memory(include_bytes!("../m.ico"))
        .expect("circuit-desktop/m.ico must be a valid ICO file")
        .into_rgba8();
    let (width, height) = icon.dimensions();
    egui::IconData {
        rgba: icon.into_raw(),
        width,
        height,
    }
}
