use super::model::{Part, PartKind};
use circuit_sim::{Point, Waveform};

/// The file header. Version 1 lines carry six fields; a seventh, the part's second value, is
/// optional so files written before the AC source and the transformer existed still load. An
/// eighth, the waveform code, and a ninth, a custom expression, are optional for the same
/// reason — a part that needs them falls back to its default rather than being rejected.
const HEADER: &str = "circuit-rs-canvas 1";
/// Fields present in the earliest version of the format, and the count below which a line is
/// too short to be a part at all.
const LEGACY_FIELDS: usize = 6;
/// The most fields a line can have: eight values plus the expression, which absorbs the rest
/// of the line. `splitn` caps here so the last field keeps any spaces inside it.
const FIELDS: usize = 9;

pub(crate) fn encode_parts(parts: &[Part]) -> String {
    let mut text = String::from(HEADER);
    text.push('\n');
    for part in parts {
        text.push_str(&format!(
            "{} {} {} {} {} {} {:.17} {}",
            part.kind.code(),
            part.a.x,
            part.a.y,
            part.b.x,
            part.b.y,
            part.value,
            part.value2,
            part.waveform.code()
        ));
        // The expression is the rest of the line rather than a ninth whitespace-separated
        // field, because it is the one field that can legitimately contain spaces — the
        // example in the property dialog is `3 - sin(t - pi/2)`.
        let expression = part.expression.trim();
        if !expression.is_empty() {
            text.push(' ');
            text.push_str(expression);
        }
        text.push('\n');
    }
    text
}

pub(crate) fn decode_parts(text: &str) -> Result<Vec<Part>, String> {
    let mut lines = text.lines();
    if lines.next() != Some(HEADER) {
        return Err("文件格式不支持或版本不正确".to_owned());
    }
    let mut parts = Vec::new();
    for (line_index, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        // Split into at most eight tokens, so the eighth is the whole remainder of the line
        // and any spaces in it stay with the expression.
        let fields: Vec<&str> = line.splitn(FIELDS, char::is_whitespace).collect();
        if fields.len() < LEGACY_FIELDS {
            return Err(format!("第 {} 行字段数量错误", line_index + 2));
        }
        let Some(mut kind) = PartKind::from_code(fields[0].trim()) else {
            // A part that has since been folded into another is dropped rather than
            // failing the whole file: an old canvas should still open, minus the one part
            // that no longer exists. Anything genuinely unrecognised is still an error,
            // because silently dropping it would look like the file was mostly fine.
            if RETIRED_CODES.contains(&fields[0].trim()) {
                continue;
            }
            return Err(format!("第 {} 行元件类型未知", line_index + 2));
        };
        let parse_i32 = |index: usize| {
            fields[index]
                .trim()
                .parse::<i32>()
                .map_err(|_| format!("第 {} 行坐标无效", line_index + 2))
        };
        let parse_value = |index: usize| {
            fields[index]
                .trim()
                .parse::<f64>()
                .map_err(|_| format!("第 {} 行元件数值无效", line_index + 2))
        };
        let value = parse_value(5)?;
        let value2 = match fields.get(6) {
            Some(field) if !field.trim().is_empty() => parse_value(6)?,
            // Absent means the file predates the second value, so the part's default is
            // the only sensible reading — a zero frequency or a zero secondary turn
            // count would analyse but never behave like the part it is meant to be.
            _ => kind.default_value2(),
        };
        let waveform = match fields.get(7) {
            Some(field) if !field.trim().is_empty() => {
                let code = field
                    .trim()
                    .parse::<i32>()
                    .map_err(|_| format!("第 {} 行波形代码无效", line_index + 2))?;
                Waveform::from_code(code)
                    .ok_or_else(|| format!("第 {} 行波形代码未知", line_index + 2))?
            }
            _ => kind.default_waveform(),
        };
        let expression = fields.get(8).map_or("", |rest| rest.trim()).to_owned();
        if kind == PartKind::AcSupply && waveform == Waveform::Custom {
            kind = PartKind::SignalSource;
        }
        kind.validate(value, value2)
            .map_err(|err| format!("第 {} 行：{err}", line_index + 2))?;
        parts.push(Part {
            kind,
            a: Point::new(parse_i32(1)?, parse_i32(2)?),
            b: Point::new(parse_i32(3)?, parse_i32(4)?),
            value,
            value2,
            waveform,
            expression,
        });
    }
    Ok(parts)
}

/// Codes that used to be written but no longer name a part. The standalone oscilloscope is
/// now the waveform window a voltmeter or ammeter opens on double click, so a canvas that
/// still holds one is simply loaded without it.
const RETIRED_CODES: [&str; 1] = ["scope"];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_old_six_field_line_still_loads() {
        let part: Part = decode_parts("circuit-rs-canvas 1\nacsupply 0 0 3 0 5\n")
            .expect("a line written before the second value existed")
            .remove(0);
        assert_eq!(part.kind, PartKind::AcSupply);
        assert_eq!(part.value, 5.0);
        // The missing second value is the part's default, not a silent zero: a 0 Hz source
        // would analyse but never produce a waveform.
        assert_eq!(part.value2, PartKind::AcSupply.default_value2());
    }

    #[test]
    fn an_old_custom_ac_source_becomes_a_function_signal_source() {
        let part = decode_parts("circuit-rs-canvas 1\nacsupply 0 0 3 0 5 50 8 5*sin(2*pi*50*t)\n")
            .expect("a saved custom source")
            .remove(0);
        assert_eq!(part.kind, PartKind::SignalSource);
        assert_eq!(part.waveform, Waveform::Custom);
        assert_eq!(part.expression, "5*sin(2*pi*50*t)");
        let loaded = decode_parts(&encode_parts(&[part.clone()]))
            .expect("the migrated source still round-trips")
            .remove(0);
        assert_eq!(loaded.kind, PartKind::SignalSource);
        assert_eq!(loaded.expression, part.expression);
    }

    #[test]
    fn a_rejected_value_names_the_offending_line() {
        let err = decode_parts("circuit-rs-canvas 1\nresistor 0 0 3 0 -1 0\n")
            .expect_err("a negative resistance is not a valid part");
        assert!(err.contains("第 2 行"), "{err}");
    }

    #[test]
    fn a_canvas_still_holding_a_retired_scope_still_opens() {
        let parts =
            decode_parts("circuit-rs-canvas 1\nscope 0 0 3 0 0 0\nresistor 0 0 3 0 1000 0\n")
                .expect("a canvas from before the scope was folded into the meters");
        assert_eq!(parts.len(), 1, "the retired part is dropped, not fatal");
        assert_eq!(parts[0].kind, PartKind::Resistor);
    }

    #[test]
    fn a_genuinely_unknown_part_is_still_an_error() {
        let err = decode_parts("circuit-rs-canvas 1\nfluxcap 0 0 3 0 0 0\n")
            .expect_err("dropping a typo would look like the file was fine");
        assert!(err.contains("第 2 行"), "{err}");
    }
}
