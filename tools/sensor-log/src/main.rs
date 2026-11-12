//! Decodes sensor carrier SD logs into one Parquet file per table.
use std::fmt::Display;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anstyle::{AnsiColor, Color, Style};
use arrow_array::{
    ArrayRef, BooleanArray, Float32Array, Float64Array, Int8Array, Int16Array, Int32Array,
    Int64Array, RecordBatch, StringArray, UInt8Array, UInt16Array, UInt32Array, UInt64Array,
};
use arrow_schema::{DataType, Field, Schema};
use clap::Parser;
use indicatif::{ProgressBar, ProgressStyle};
use parquet::arrow::ArrowWriter;
use parquet::basic::Compression;
use parquet::file::metadata::KeyValue;
use parquet::file::properties::WriterProperties;
use postcard_schema::schema::owned::{
    OwnedDataModelType as Ty, OwnedDataModelVariant, OwnedNamedType,
};

const GREEN: Style = Style::new().fg_color(Some(Color::Ansi(AnsiColor::Green)));
const YELLOW: Style = Style::new().fg_color(Some(Color::Ansi(AnsiColor::Yellow)));
const RED: Style = Style::new().fg_color(Some(Color::Ansi(AnsiColor::Red)));
const STRING_WIDTH: usize = 64;
const ROWS_PER_BATCH: usize = 1_000_000;

struct Table {
    name: String,
    columns: Vec<(String, Ty)>,
    rows: Vec<u8>,
    count: usize,
}

/// Decodes sensor carrier SD logs into one Parquet file per table.
///
/// INPUT is a LOGnnnn session directory, or a directory of sessions such as the SD card
/// root, in which case each session is decoded into its own subdirectory of OUTPUT.
/// Exits with 1 if a session could not be decoded.
#[derive(Parser)]
#[command(version)]
struct Args {
    /// Session directory, or a directory of sessions
    input: PathBuf,
    /// Directory the Parquet files are written to
    output: PathBuf,
}

fn main() {
    let Args { input, output } = Args::parse();
    let mut ok = true;
    if input.join("SCHEMA.TXT").exists() {
        ok = run(&input, &output);
    } else {
        // Otherwise a directory of sessions, such as the SD card root: one subdirectory each.
        let sessions = fs::read_dir(&input).unwrap_or_else(|e| die(&input, e));
        let mut sessions: Vec<_> = sessions.map(|e| e.unwrap().path()).collect();
        sessions.retain(|p| p.file_name().unwrap().to_string_lossy().starts_with("LOG"));
        sessions.sort();
        if sessions.is_empty() {
            die(&input, "no SCHEMA.TXT and no LOGnnnn session directories");
        }
        for session in sessions {
            let name = session.file_name().unwrap().to_os_string();
            if session.join("SCHEMA.TXT").exists() {
                ok &= run(&session, &output.join(name));
            } else {
                report(YELLOW, "skipped", &name.to_string_lossy(), "no SCHEMA.TXT");
            }
        }
    }
    if !ok {
        std::process::exit(1);
    }
}

fn die(what: &Path, error: impl Display) -> ! {
    anstream::eprintln!("{RED}error{RED:#}: {}: {error}", what.display());
    std::process::exit(1);
}

/// One status line per session.
fn report(style: Style, status: &str, session: &str, detail: impl Display) {
    anstream::println!("{session}  {style}{status:<7}{style:#} {detail}");
}

/// Decodes one session, reporting a failure instead of stopping.
fn run(session: &Path, out: &Path) -> bool {
    let name = session
        .file_name()
        .unwrap_or(session.as_os_str())
        .to_string_lossy();
    match convert(session, out) {
        Ok(summary) => report(GREEN, "ok", &name, summary),
        Err(error) => {
            anstream::eprintln!("{name}  {RED}failed{RED:#}  {error}");
            return false;
        }
    }
    true
}

/// Decodes the session into `out` and returns a one line summary.
fn convert(session: &Path, out: &Path) -> Result<String, String> {
    // The variants of the firmware's Record enum are the tables, their fields the columns.
    let schema_path = session.join("SCHEMA.TXT");
    let at = |e: &dyn std::fmt::Display| format!("{}: {e}", schema_path.display());
    let schema_json = fs::read_to_string(&schema_path).map_err(|e| at(&e))?;
    let schema: OwnedNamedType = serde_json::from_str(&schema_json).map_err(|e| at(&e))?;
    let Ty::Enum(variants) = schema.ty else {
        return Err(at(&"schema is not an enum"));
    };
    let mut tables = vec![];
    for variant in variants {
        let OwnedDataModelVariant::StructVariant(fields) = variant.ty else {
            return Err(at(&"schema variant without fields"));
        };
        tables.push(Table {
            name: variant.name.to_lowercase(),
            columns: fields.into_iter().map(|f| (f.name, f.ty.ty)).collect(),
            rows: vec![],
            count: 0,
        });
    }

    let mut segments: Vec<_> = fs::read_dir(session)
        .map_err(|e| e.to_string())?
        .map(|e| e.unwrap().path())
        .collect();
    segments.retain(|p| p.extension().is_some_and(|e| e == "BIN"));
    segments.sort();
    let total: u64 = segments
        .iter()
        .map(|s| fs::metadata(s).map_or(0, |m| m.len()))
        .sum();
    let progress = ProgressBar::new(total)
        .with_prefix(
            session
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned(),
        )
        .with_style(
            ProgressStyle::with_template("{prefix}  {bar:30} {bytes}/{total_bytes}  eta {eta}")
                .unwrap(),
        );
    let mut skipped = 0;
    let mut done = 0;
    for segment in segments {
        let data = fs::read(segment).unwrap();
        done += data.len() as u64;
        // A zero byte ends each frame; empty ones are padding.
        for frame in data.split(|&b| b == 0).filter(|f| !f.is_empty()) {
            if decode(frame, &mut tables).is_none() {
                skipped += 1; // cut off by a lost block or a power cut
            }
            progress.inc(frame.len() as u64 + 1);
        }
        progress.set_position(done);
    }
    progress.finish_and_clear();
    fs::create_dir_all(out).map_err(|e| format!("{}: {e}", out.display()))?;
    for table in &tables {
        write_parquet(out, table, &schema_json);
    }
    let records: usize = tables.iter().map(|t| t.count).sum();
    // Damaged frames are expected after a power cut, but worth noticing.
    let damaged = if skipped > 0 {
        format!("{YELLOW}{skipped} damaged{YELLOW:#}")
    } else {
        "0 damaged".to_string()
    };
    Ok(format!(
        "{records} records, {damaged}, written to {}",
        out.display()
    ))
}

fn decode(frame: &[u8], tables: &mut [Table]) -> Option<()> {
    let data = cobs::decode_vec(frame).ok()?;
    let mut pos = 0;
    let table = tables.get_mut(varint(&data, &mut pos)? as usize)?;
    let mut row = vec![];
    for (_, ty) in &table.columns {
        value(ty, &data, &mut pos, &mut row)?;
    }
    if pos != data.len() {
        return None;
    }
    table.rows.extend(row);
    table.count += 1;
    Some(())
}

/// Appends the value as fixed-width little endian bytes, `width` of them.
fn value(ty: &Ty, data: &[u8], pos: &mut usize, row: &mut Vec<u8>) -> Option<()> {
    match ty {
        Ty::Bool | Ty::U8 | Ty::I8 => row.extend(take(data, pos, 1)?),
        Ty::F32 => row.extend(take(data, pos, 4)?),
        Ty::F64 => row.extend(take(data, pos, 8)?),
        Ty::U16 => row.extend((varint(data, pos)? as u16).to_le_bytes()),
        Ty::U32 => row.extend((varint(data, pos)? as u32).to_le_bytes()),
        Ty::U64 => row.extend(varint(data, pos)?.to_le_bytes()),
        Ty::I16 => row.extend((zigzag(data, pos)? as i16).to_le_bytes()),
        Ty::I32 => row.extend((zigzag(data, pos)? as i32).to_le_bytes()),
        Ty::I64 => row.extend(zigzag(data, pos)?.to_le_bytes()),
        Ty::String => {
            let len = varint(data, pos)? as usize;
            let mut text = take(data, pos, len)?.to_vec();
            text.resize(STRING_WIDTH, 0);
            row.extend(text);
        }
        _ => panic!("unsupported type {ty:?}"),
    }
    Some(())
}

fn width(ty: &Ty) -> usize {
    match ty {
        Ty::Bool | Ty::U8 | Ty::I8 => 1,
        Ty::U16 | Ty::I16 => 2,
        Ty::U32 | Ty::I32 | Ty::F32 => 4,
        Ty::U64 | Ty::I64 | Ty::F64 => 8,
        Ty::String => STRING_WIDTH,
        _ => panic!("unsupported type {ty:?}"),
    }
}

fn data_type(ty: &Ty) -> DataType {
    match ty {
        Ty::Bool => DataType::Boolean,
        Ty::U8 => DataType::UInt8,
        Ty::I8 => DataType::Int8,
        Ty::U16 => DataType::UInt16,
        Ty::U32 => DataType::UInt32,
        Ty::U64 => DataType::UInt64,
        Ty::I16 => DataType::Int16,
        Ty::I32 => DataType::Int32,
        Ty::I64 => DataType::Int64,
        Ty::F32 => DataType::Float32,
        Ty::F64 => DataType::Float64,
        Ty::String => DataType::Utf8,
        _ => panic!("unsupported type {ty:?}"),
    }
}

fn take<'a>(data: &'a [u8], pos: &mut usize, len: usize) -> Option<&'a [u8]> {
    let bytes = data.get(*pos..*pos + len)?;
    *pos += len;
    Some(bytes)
}

fn varint(data: &[u8], pos: &mut usize) -> Option<u64> {
    let mut value = 0;
    for shift in (0..64).step_by(7) {
        let byte = *take(data, pos, 1)?.first()?;
        value |= u64::from(byte & 0x7F) << shift;
        if byte < 0x80 {
            return Some(value);
        }
    }
    None
}

fn zigzag(data: &[u8], pos: &mut usize) -> Option<i64> {
    let value = varint(data, pos)?;
    Some((value >> 1) as i64 ^ -((value & 1) as i64))
}

macro_rules! numbers {
    ($array:ty, $number:ty, $cells:expr) => {
        Arc::new(<$array>::from_iter_values(
            $cells.map(|c| <$number>::from_le_bytes(c.try_into().unwrap())),
        ))
    };
}

/// One column's values, from its fixed-width cells.
fn column<'a>(ty: &Ty, cells: impl Iterator<Item = &'a [u8]>) -> ArrayRef {
    match ty {
        Ty::Bool => Arc::new(BooleanArray::from(
            cells.map(|c| c[0] != 0).collect::<Vec<_>>(),
        )),
        Ty::U8 => numbers!(UInt8Array, u8, cells),
        Ty::I8 => numbers!(Int8Array, i8, cells),
        Ty::U16 => numbers!(UInt16Array, u16, cells),
        Ty::U32 => numbers!(UInt32Array, u32, cells),
        Ty::U64 => numbers!(UInt64Array, u64, cells),
        Ty::I16 => numbers!(Int16Array, i16, cells),
        Ty::I32 => numbers!(Int32Array, i32, cells),
        Ty::I64 => numbers!(Int64Array, i64, cells),
        Ty::F32 => numbers!(Float32Array, f32, cells),
        Ty::F64 => numbers!(Float64Array, f64, cells),
        Ty::String => Arc::new(StringArray::from_iter_values(cells.map(|c| {
            String::from_utf8_lossy(c.split(|&b| b == 0).next().unwrap()).into_owned()
        }))),
        _ => panic!("unsupported type {ty:?}"),
    }
}

fn write_parquet(out: &Path, table: &Table, schema_json: &str) {
    let fields: Vec<Field> = table
        .columns
        .iter()
        .map(|(name, ty)| Field::new(name, data_type(ty), false))
        .collect();
    let schema = Arc::new(Schema::new(fields));
    // Each file carries the session's full schema, so it still describes itself on its own.
    let properties = WriterProperties::builder()
        .set_compression(Compression::SNAPPY)
        .set_key_value_metadata(Some(vec![KeyValue::new(
            "schema".into(),
            schema_json.to_string(),
        )]))
        .build();
    let file = fs::File::create(out.join(format!("{}.parquet", table.name))).unwrap();
    let mut writer = ArrowWriter::try_new(file, schema.clone(), Some(properties)).unwrap();

    let row_width: usize = table.columns.iter().map(|(_, ty)| width(ty)).sum();
    for rows in table.rows.chunks(row_width * ROWS_PER_BATCH) {
        let mut start = 0;
        let arrays = table
            .columns
            .iter()
            .map(|(_, ty)| {
                let cell = start..start + width(ty);
                start = cell.end;
                column(
                    ty,
                    rows.chunks_exact(row_width).map(|row| &row[cell.clone()]),
                )
            })
            .collect();
        writer
            .write(&RecordBatch::try_new(schema.clone(), arrays).unwrap())
            .unwrap();
    }
    writer.close().unwrap();
}
