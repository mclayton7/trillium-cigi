// build.rs — Orion Protocol Code Generator
// Parses OrionPublicProtocol.xml and emits orion_generated.rs into $OUT_DIR.

use std::collections::HashMap;
use std::env;
use std::fmt::Write as FmtWrite;
use std::fs;
use std::path::Path;

fn main() {
    // Declaring any rerun-if-changed line opts us out of Cargo's default
    // "rerun on any source change" rule, so list every input the codegen
    // depends on — both the schema and the generator itself.
    println!("cargo:rerun-if-changed=OrionPublicProtocol.xml");
    println!("cargo:rerun-if-changed=build.rs");
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let xml_path = Path::new(&manifest_dir).join("OrionPublicProtocol.xml");
    let xml = fs::read_to_string(&xml_path).expect("Cannot read OrionPublicProtocol.xml");
    let doc = roxmltree::Document::parse(&xml).expect("Invalid XML in OrionPublicProtocol.xml");
    let generated = generate_code(&doc);
    let out_dir = env::var("OUT_DIR").unwrap();
    fs::write(Path::new(&out_dir).join("orion_generated.rs"), generated)
        .expect("Cannot write orion_generated.rs");
}

// ─────────────────────────────────────────────────────── helpers ──

/// Evaluate a simple arithmetic expression (handles +, -, *, /, and `pi`).
fn eval_expr(s: &str) -> f64 {
    let s = s.trim().replace("pi", &std::f64::consts::PI.to_string());
    eval_add_sub(s.trim())
}

fn eval_add_sub(s: &str) -> f64 {
    // split on last + or - that is not inside parentheses
    let mut depth: i32 = 0;
    let bytes = s.as_bytes();
    let mut i = bytes.len();
    while i > 0 {
        i -= 1;
        match bytes[i] {
            b')' => depth += 1,
            b'(' => depth -= 1,
            b'+' | b'-' if depth == 0 && i > 0 => {
                let left = eval_add_sub(s[..i].trim());
                let right = eval_mul_div(s[i + 1..].trim());
                return if bytes[i] == b'+' { left + right } else { left - right };
            }
            _ => {}
        }
    }
    eval_mul_div(s)
}

fn eval_mul_div(s: &str) -> f64 {
    let mut depth: i32 = 0;
    let bytes = s.as_bytes();
    let mut i = bytes.len();
    while i > 0 {
        i -= 1;
        match bytes[i] {
            b')' => depth += 1,
            b'(' => depth -= 1,
            b'*' | b'/' if depth == 0 => {
                let left = eval_mul_div(s[..i].trim());
                let right = eval_atom(s[i + 1..].trim());
                return if bytes[i] == b'*' { left * right } else { left / right };
            }
            _ => {}
        }
    }
    eval_atom(s)
}

fn eval_atom(s: &str) -> f64 {
    let s = s.trim();
    if s.starts_with('(') && s.ends_with(')') {
        return eval_add_sub(&s[1..s.len() - 1]);
    }
    if s.starts_with('-') {
        return -eval_atom(&s[1..]);
    }
    s.parse::<f64>().unwrap_or(0.0)
}

/// CamelCase / mixed → snake_case
fn to_snake(s: &str) -> String {
    let mut out = String::new();
    let chars: Vec<char> = s.chars().collect();
    for i in 0..chars.len() {
        let c = chars[i];
        if c.is_uppercase() {
            let prev_lower = i > 0 && chars[i - 1].is_lowercase();
            let next_lower = i + 1 < chars.len() && chars[i + 1].is_lowercase();
            let prev_upper = i > 0 && chars[i - 1].is_uppercase();
            if i > 0 && (prev_lower || (next_lower && prev_upper)) {
                out.push('_');
            }
            out.push(c.to_ascii_lowercase());
        } else {
            out.push(c);
        }
    }
    // escape Rust keywords
    match out.as_str() {
        "type" | "mod" | "use" | "ref" | "move" | "box" | "loop" | "match" | "where"
        | "impl" | "pub" | "fn" | "let" | "mut" | "in" | "for" | "if" | "else"
        | "return" | "true" | "false" | "self" | "super" | "crate" | "static" | "const" => {
            out.push('_');
            out
        }
        _ => out,
    }
}

/// SCREAMING_SNAKE_CASE or camelCase → PascalCase.
fn screaming_to_pascal(s: &str) -> String {
    if s.contains('_') {
        // SCREAMING_SNAKE_CASE
        s.split('_')
            .filter(|p| !p.is_empty())
            .map(|p| {
                let mut c = p.chars();
                match c.next() {
                    None => String::new(),
                    Some(f) => f.to_uppercase().collect::<String>() + &c.as_str().to_lowercase(),
                }
            })
            .collect()
    } else {
        // Already camelCase or PascalCase — just ensure first letter is uppercase
        let mut c = s.chars();
        match c.next() {
            None => String::new(),
            Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        }
    }
}

/// Strip trailing `_t` from XML enum names → Rust enum name.
/// Preserves PascalCase names (e.g. "OrionMode_t" → "OrionMode").
fn rust_enum_name(xml: &str) -> String {
    let base = xml.trim_end_matches("_t");
    // If it starts with uppercase and contains at least one lowercase it's already PascalCase.
    let first_upper = base.chars().next().map(|c| c.is_uppercase()).unwrap_or(false);
    let has_lower = base.chars().any(|c| c.is_lowercase());
    if first_upper && has_lower {
        base.to_owned()
    } else if base.contains('_') {
        screaming_to_pascal(base)
    } else {
        // All-caps single word like "DATE" → "Date"
        screaming_to_pascal(base)
    }
}

/// Convert XML struct/packet name to Rust PascalCase name.
fn rust_type_name(xml: &str) -> String {
    let first_upper = xml.chars().next().map(|c| c.is_uppercase()).unwrap_or(false);
    let has_lower = xml.chars().any(|c| c.is_lowercase());
    if first_upper && has_lower {
        // Already PascalCase — leave it alone
        xml.to_owned()
    } else {
        screaming_to_pascal(xml)
    }
}

// ─────────────────────────────────────────── data model ──

#[derive(Debug, Clone)]
struct EnumDef {
    xml_name: String,
    rust_name: String,
    /// (rust_variant_name, discriminant_value, is_alias)
    values: Vec<(String, i64, bool)>,
}

#[derive(Debug, Clone)]
enum ScaleKind {
    None,
    Scaler(f64),          // explicit scaler attribute
    MaxOnly(f64),         // max attribute only
    MinMax(f64, f64),     // min + max
}

#[derive(Debug, Clone)]
struct FieldDef {
    /// snake_case field name for Rust
    rust_name: String,
    /// None → no struct field (padding / null in-memory)
    in_mem_rust_type: Option<String>,
    /// For enum fields, the Rust enum type name
    enum_type: Option<String>,
    /// For struct fields, the Rust struct type name
    struct_type: Option<String>,
    /// None → same as in_mem_rust_type (no conversion)
    encoded_rust_type: Option<String>,
    scale: ScaleKind,
    /// Fixed array size, or None
    array_size: Option<usize>,
    /// The name of the length field (variable array)
    variable_array_len_field: Option<String>,
    /// Number of bits if bitfield
    bit_width: Option<u8>,
    /// inMemoryType="null"
    is_null_mem: bool,
    /// constant value to encode (skip field in struct)
    constant: Option<i64>,
    /// dependsOn — make Option<T> in struct
    depends_on: Option<String>,
    /// Skip encoding/decoding entirely
    skip_enc: bool,
}

#[derive(Debug, Clone)]
struct StructDef {
    xml_name: String,
    rust_name: String,
    fields: Vec<FieldDef>,
    /// inline array size (for nested struct arrays inside packets)
    #[allow(dead_code)]
    array_size: Option<usize>,
    #[allow(dead_code)]
    variable_array_len_field: Option<String>,
}

#[derive(Debug, Clone)]
struct PacketDef {
    xml_name: String,
    rust_name: String,
    id_value: i64,
    fields: Vec<FieldDef>,
    inline_enums: Vec<EnumDef>,
    inline_structs: Vec<StructDef>,
}

// ─────────────────────────────────────────────────── parsing ──

fn parse_in_mem(s: &str) -> (Option<String>, Option<u8>, bool) {
    // Returns (rust_type, bit_width_if_bitfield, is_null)
    match s {
        "unsigned8" => (Some("u8".into()), None, false),
        "unsigned16" => (Some("u16".into()), None, false),
        "unsigned24" | "unsigned32" => (Some("u32".into()), None, false),
        "unsigned64" => (Some("u64".into()), None, false),
        "signed8" => (Some("i8".into()), None, false),
        "signed16" => (Some("i16".into()), None, false),
        "signed24" | "signed32" => (Some("i32".into()), None, false),
        "float" | "float32" => (Some("f32".into()), None, false),
        "float64" => (Some("f64".into()), None, false),
        "bitfield1" => (Some("bool".into()), Some(1), false),
        "null" => (None, None, true),
        s if s.starts_with("bitfield") => {
            let n: u8 = s[8..]
                .parse()
                .unwrap_or_else(|_| panic!("invalid bitfield width in inMemoryType: {s:?}"));
            (Some("u8".into()), Some(n), false)
        }
        // Treat string types as null — not needed by the simulator.
        "string" | "fixedstring" => (None, None, true),
        // Surface schema drift loudly: an unknown inMemoryType used to silently
        // become a u8, which produced a packet that compiled but was wrong on
        // the wire. Prefer a build-time error so new types must be added here
        // explicitly.
        other => panic!(
            "unknown inMemoryType {other:?} — add a branch to parse_in_mem in build.rs"
        ),
    }
}

fn parse_enc(s: &str) -> (Option<String>, Option<u8>) {
    // Returns (rust_type, bit_width_if_bitfield)
    match s {
        "unsigned8" => (Some("u8".into()), None),
        "unsigned16" => (Some("u16".into()), None),
        "unsigned24" | "unsigned32" => (Some("u32".into()), None),
        "unsigned64" => (Some("u64".into()), None),
        "signed8" => (Some("i8".into()), None),
        "signed16" => (Some("i16".into()), None),
        "signed24" | "signed32" => (Some("i32".into()), None),
        "float" | "float32" => (Some("f32".into()), None),
        "null" => (None, None),
        // IEEE half-precision is referenced by some diagnostic packets
        // (InsQuality / *ChiSquare) but the generator has no half-precision
        // encode/decode path. Treat as `null` (skip on the wire) and emit a
        // cargo warning so the gap stays visible until proper support is
        // added — silently mapping to u8 (the previous behaviour) corrupted
        // the byte stream for any consumer that did parse these packets.
        "float16" => {
            println!(
                "cargo:warning=encodedType \"float16\" is not implemented; \
                 affected fields are skipped in the generated codec"
            );
            (None, None)
        }
        s if s.starts_with("bitfield") => {
            let n: u8 = s[8..]
                .parse()
                .unwrap_or_else(|_| panic!("invalid bitfield width in encodedType: {s:?}"));
            (None, Some(n))
        }
        other => panic!(
            "unknown encodedType {other:?} — add a branch to parse_enc in build.rs"
        ),
    }
}

fn max_int_for(enc_type: &str) -> f64 {
    match enc_type {
        "i8" => 127.0,
        "i16" => 32767.0,
        "i32" => 2_147_483_647.0,
        "u8" => 255.0,
        "u16" => 65535.0,
        "u32" => 4_294_967_295.0,
        _ => 1.0,
    }
}

fn is_signed_enc(enc_type: &str) -> bool {
    matches!(enc_type, "i8" | "i16" | "i32" | "i64")
}

/// Parse a `<Data>` element into a FieldDef.
fn parse_field(
    node: roxmltree::Node,
    enum_consts: &HashMap<String, i64>,
    enum_names: &HashMap<String, String>, // xml_enum_name → rust_enum_name
) -> FieldDef {
    let xml_name = node.attribute("name").unwrap_or("unknown").to_owned();
    let rust_name = to_snake(&xml_name);

    let in_mem_attr = node.attribute("inMemoryType").unwrap_or("unsigned8");
    let (mem_rust, bit_w_mem, is_null_mem) = parse_in_mem(in_mem_attr);

    let enc_attr = node.attribute("encodedType");
    let (enc_rust, bit_w_enc) = enc_attr.map(parse_enc).unwrap_or((None, None));

    // Bitfield width: use encoded type's bits if in-mem is null, else use in-mem bitfield
    let bit_width = bit_w_mem.or(bit_w_enc);

    // Array size
    let array_attr = node.attribute("array");
    let array_size: Option<usize> = array_attr.map(|a| {
        a.parse::<usize>()
            .unwrap_or_else(|_| *enum_consts.get(a).unwrap_or(&1) as usize)
    });

    // Variable array
    let variable_array_len_field = node.attribute("variableArray").map(|s| s.to_owned());

    // Scale
    let scaler_attr = node.attribute("scaler");
    let max_attr = node.attribute("max");
    let min_attr = node.attribute("min");
    let scale = if let Some(s) = scaler_attr {
        ScaleKind::Scaler(eval_expr(s))
    } else if let (Some(min_s), Some(max_s)) = (min_attr, max_attr) {
        ScaleKind::MinMax(eval_expr(min_s), eval_expr(max_s))
    } else if let Some(max_s) = max_attr {
        ScaleKind::MaxOnly(eval_expr(max_s))
    } else {
        ScaleKind::None
    };

    // Constant
    let constant: Option<i64> = node.attribute("constant").and_then(|v| v.parse().ok());

    // dependsOn
    let depends_on = node.attribute("dependsOn").map(|s| to_snake(s));

    // Enum or struct reference
    let enum_ref = node.attribute("enum");
    let struct_ref = node.attribute("struct");

    let enum_type = enum_ref.and_then(|e| enum_names.get(e)).cloned();
    let struct_type = struct_ref.map(|s| rust_type_name(s));

    // In-memory type (may be overridden by struct/enum type)
    let in_mem_rust_type = if struct_type.is_some() || enum_type.is_some() {
        None // handled separately
    } else {
        mem_rust
    };

    // encoded type for code gen
    let encoded_rust_type = enc_rust.or_else(|| {
        // default encoded type = same as in-memory for non-bitfield fields
        if bit_width.is_none() && !is_null_mem {
            in_mem_rust_type.clone()
        } else {
            None
        }
    });

    // skip encoding if encodedType="null"
    let skip_enc = enc_attr == Some("null");

    FieldDef {
        rust_name,
        in_mem_rust_type,
        enum_type,
        struct_type,
        encoded_rust_type,
        scale,
        array_size,
        variable_array_len_field,
        bit_width,
        is_null_mem,
        constant,
        depends_on,
        skip_enc,
    }
}

/// Parse all <Enum> children of a node, add values to enum_consts, return EnumDefs.
fn parse_enums(
    parent: roxmltree::Node,
    enum_consts: &mut HashMap<String, i64>,
) -> Vec<EnumDef> {
    let mut defs = Vec::new();
    for child in parent.children() {
        if child.tag_name().name() != "Enum" {
            continue;
        }
        let xml_name = child.attribute("name").unwrap_or("Unknown").to_owned();
        let rust_name = rust_enum_name(&xml_name);
        let mut counter: i64 = 0;
        let mut values: Vec<(String, i64, bool)> = Vec::new();
        let mut seen_values: HashMap<i64, bool> = HashMap::new();
        for val in child.children() {
            if val.tag_name().name() != "Value" {
                continue;
            }
            let vname = val.attribute("name").unwrap_or("Unknown").to_owned();
            let discriminant = if let Some(v) = val.attribute("value") {
                // might be a reference to another constant or a hex/decimal number
                if let Some(existing) = enum_consts.get(v) {
                    *existing
                } else if v.starts_with("0x") || v.starts_with("0X") {
                    i64::from_str_radix(&v[2..], 16).unwrap_or(counter)
                } else {
                    v.parse::<i64>().unwrap_or_else(|_| eval_expr(v) as i64)
                }
            } else {
                counter
            };
            counter = discriminant + 1;
            // Register in global constant map
            enum_consts.insert(vname.clone(), discriminant);
            let rust_variant = screaming_to_pascal(&vname);
            let is_alias = seen_values.contains_key(&discriminant);
            seen_values.insert(discriminant, true);
            values.push((rust_variant, discriminant, is_alias));
        }
        defs.push(EnumDef { xml_name, rust_name, values });
    }
    defs
}

/// Parse all <Structure> direct children of a node.
fn parse_structs(
    parent: roxmltree::Node,
    enum_consts: &HashMap<String, i64>,
    enum_names: &HashMap<String, String>,
) -> Vec<StructDef> {
    let mut defs = Vec::new();
    for child in parent.children() {
        if child.tag_name().name() != "Structure" {
            continue;
        }
        let xml_name = child.attribute("name").unwrap_or("Unknown").to_owned();
        let rust_name = rust_type_name(&xml_name);
        let array_size = child.attribute("array").map(|a| {
            a.parse::<usize>()
                .unwrap_or_else(|_| *enum_consts.get(a).unwrap_or(&1) as usize)
        });
        let variable_array_len_field =
            child.attribute("variableArray").map(|s| to_snake(s));
        let mut fields = Vec::new();
        for data in child.children() {
            if data.tag_name().name() == "Data" {
                fields.push(parse_field(data, enum_consts, enum_names));
            }
        }
        defs.push(StructDef {
            xml_name,
            rust_name,
            fields,
            array_size,
            variable_array_len_field,
        });
    }
    defs
}

/// Parse all <Packet> elements.
fn parse_packets(
    root: roxmltree::Node,
    enum_consts: &mut HashMap<String, i64>,
    enum_names: &mut HashMap<String, String>,
) -> Vec<PacketDef> {
    let mut packets = Vec::new();
    for child in root.children() {
        if child.tag_name().name() != "Packet" {
            continue;
        }
        let xml_name = child.attribute("name").unwrap_or("Unknown").to_owned();
        let rust_name = rust_type_name(&xml_name);
        let id_attr = child.attribute("ID").unwrap_or("0");
        let id_value = enum_consts
            .get(id_attr)
            .copied()
            .unwrap_or_else(|| {
                if id_attr.starts_with("0x") || id_attr.starts_with("0X") {
                    i64::from_str_radix(&id_attr[2..], 16).unwrap_or(0)
                } else {
                    id_attr.parse().unwrap_or(0)
                }
            });

        // Inline enums
        let inline_enums = parse_enums(child, enum_consts);
        for e in &inline_enums {
            enum_names.insert(e.xml_name.clone(), e.rust_name.clone());
        }

        // Inline structs
        let inline_structs = parse_structs(child, enum_consts, enum_names);

        // Direct <Data> fields
        let mut fields = Vec::new();
        for node in child.children() {
            if node.tag_name().name() == "Data" {
                fields.push(parse_field(node, enum_consts, enum_names));
            }
        }

        packets.push(PacketDef {
            xml_name,
            rust_name: format!("{}Packet", rust_name),
            id_value,
            fields,
            inline_enums,
            inline_structs,
        });
    }
    packets
}

// ──────────────────────────────────────── code generation ──

fn field_rust_type(f: &FieldDef) -> String {
    let base = if let Some(ref st) = f.struct_type {
        st.clone()
    } else if let Some(ref et) = f.enum_type {
        et.clone()
    } else if let Some(ref mt) = f.in_mem_rust_type {
        mt.clone()
    } else {
        return String::new(); // null — no field
    };

    // Variable arrays and struct arrays use Vec<T>; plain scalars use [T; N]
    let base = if f.variable_array_len_field.is_some() || f.struct_type.is_some() && f.array_size.is_some() {
        format!("Vec<{}>", base)
    } else if let Some(n) = f.array_size {
        format!("[{}; {}]", base, n)
    } else {
        base
    };

    if f.depends_on.is_some() {
        format!("Option<{}>", base)
    } else {
        base
    }
}

fn has_struct_field(f: &FieldDef) -> bool {
    // Fields with a constant value are never user-visible struct fields
    if f.constant.is_some() {
        return false;
    }
    if f.is_null_mem {
        return false;
    }
    if f.skip_enc && f.in_mem_rust_type.is_none() && f.struct_type.is_none() && f.enum_type.is_none() {
        return false;
    }
    let ft = field_rust_type(f);
    !ft.is_empty()
}

/// Generate encode expression for a single element (not array).
fn encode_elem(
    f: &FieldDef,
    value_expr: &str,
    out: &mut String,
) {
    if f.skip_enc {
        return;
    }
    if let Some(bw) = f.bit_width {
        // Bitfield — the caller handles accumulation
        writeln!(
            out,
            "        // bitfield {} bits — handled by caller",
            bw
        )
        .ok();
        return;
    }

    let enc = match &f.encoded_rust_type {
        Some(t) => t.clone(),
        None => {
            if let Some(ref _et) = f.enum_type {
                "u8".into() // enums default to u8 wire type
            } else {
                return;
            }
        }
    };

    // Determine encode expression
    let cast_expr: String = match &f.scale {
        ScaleKind::Scaler(s) => {
            format!(
                "({} as f64 * {:.10}_f64).round().clamp({}_f64, {}_f64) as {}",
                value_expr,
                s,
                -max_int_for(&enc),
                max_int_for(&enc),
                enc
            )
        }
        ScaleKind::MaxOnly(max) => {
            let mi = max_int_for(&enc);
            format!(
                "({} as f64 / {:.10}_f64 * {}_f64).round().clamp({}_f64, {}_f64) as {}",
                value_expr,
                max,
                mi,
                if is_signed_enc(&enc) { -mi } else { 0.0 },
                mi,
                enc
            )
        }
        ScaleKind::MinMax(min, max) => {
            let mi = max_int_for(&enc);
            format!(
                "(({} as f64 - {:.10}_f64) / {:.10}_f64 * {}_f64).round().clamp(0.0_f64, {}_f64) as {}",
                value_expr,
                min,
                max - min,
                mi,
                mi,
                enc
            )
        }
        ScaleKind::None => {
            if f.enum_type.is_some() {
                format!("{} as u8", value_expr)
            } else if enc == "f32" || enc == "f64" {
                format!("{} as {}", value_expr, enc)
            } else {
                format!("{} as {}", value_expr, enc)
            }
        }
    };

    // Write bytes
    let byte_count = match enc.as_str() {
        "u8" | "i8" => 1,
        "u16" | "i16" => 2,
        "u32" | "i32" | "f32" => 4,
        "u64" | "i64" | "f64" => 8,
        _ => 1,
    };

    if byte_count == 1 {
        writeln!(out, "        out.push(({}) as u8);", cast_expr).ok();
    } else if enc == "f32" || enc == "f64" {
        // Float-to-float encoding writes the raw value — no integer-style
        // scaler/clamp can be applied here meaningfully. The integer scale
        // formulas in `cast_expr` clamp to i32/u32 ranges, which would
        // truncate a float silently, so reject scaled float-to-float fields
        // at codegen time rather than silently dropping the scale.
        if !matches!(f.scale, ScaleKind::None) {
            panic!(
                "field {:?}: scale on float-to-{} encoding is not supported \
                 (the generator would silently drop the scaler)",
                f.rust_name, enc
            );
        }
        writeln!(
            out,
            "        out.extend_from_slice(&({} as {}).to_be_bytes());",
            value_expr, enc
        )
        .ok();
    } else {
        writeln!(
            out,
            "        out.extend_from_slice(&({}).to_be_bytes());",
            cast_expr
        )
        .ok();
    }
}

/// Generate decode expression for a single element (not array).
/// Returns the expression that yields the in-memory value.
fn decode_elem(f: &FieldDef, out: &mut String) -> String {
    if f.skip_enc {
        return "Default::default()".into();
    }

    let enc = match &f.encoded_rust_type {
        Some(t) => t.clone(),
        None => {
            if f.enum_type.is_some() {
                "u8".into()
            } else {
                return "Default::default()".into();
            }
        }
    };

    let byte_count = match enc.as_str() {
        "u8" | "i8" => 1usize,
        "u16" | "i16" => 2,
        "u32" | "i32" | "f32" => 4,
        "u64" | "i64" | "f64" => 8,
        _ => 1,
    };

    let mem_type = if let Some(ref et) = f.enum_type {
        et.clone()
    } else if let Some(ref mt) = f.in_mem_rust_type {
        mt.clone()
    } else {
        "u8".into()
    };

    // Read raw bytes
    let raw_var = format!("raw_{}", f.rust_name.replace('-', "_"));
    if byte_count == 1 {
        writeln!(
            out,
            "        let {} = {{ if buf.len() < 1 {{ return None; }} let v = buf[0]; *buf = &buf[1..]; v as {} }};",
            raw_var, enc
        ).ok();
    } else {
        writeln!(
            out,
            "        let {} = {{ if buf.len() < {sz} {{ return None; }} let arr: [{enc}; 1] = [{{let mut a = [0u8; {sz}]; a.copy_from_slice(&buf[..{sz}]); *buf = &buf[{sz}..]; {enc}::from_be_bytes(a)}}]; arr[0] }};",
            raw_var, sz = byte_count, enc = enc
        ).ok();
    }

    // Convert raw → in-memory
    match &f.scale {
        ScaleKind::Scaler(s) => {
            format!("({} as f64 / {:.10}_f64) as {}", raw_var, s, mem_type)
        }
        ScaleKind::MaxOnly(max) => {
            let mi = max_int_for(&enc);
            format!(
                "({} as f64 / {}_f64 * {:.10}_f64) as {}",
                raw_var, mi, max, mem_type
            )
        }
        ScaleKind::MinMax(min, max) => {
            let mi = max_int_for(&enc);
            format!(
                "({} as f64 / {}_f64 * {:.10}_f64 + {:.10}_f64) as {}",
                raw_var,
                mi,
                max - min,
                min,
                mem_type
            )
        }
        ScaleKind::None => {
            if f.enum_type.is_some() {
                format!(
                    "{}::try_from({} as u8).unwrap_or_default()",
                    mem_type, raw_var
                )
            } else if enc == mem_type {
                raw_var.clone()
            } else {
                format!("{} as {}", raw_var, mem_type)
            }
        }
    }
}

/// Generate encode/decode for a list of fields (shared between struct & packet).
/// `out_is_owned`: true when the encode method owns `out` (packet context: `let mut out = Vec::new()`),
///  false when `out: &mut Vec<u8>` is a parameter (struct context).
fn gen_field_encode(fields: &[FieldDef], out: &mut String, prefix: &str, out_is_owned: bool) {
    writeln!(out, "        let mut bit_acc: u8 = 0;").ok();
    writeln!(out, "        let mut bit_pos: u8 = 0;").ok();
    let mut need_flush = false;

    for f in fields {
        if f.skip_enc {
            continue;
        }

        // Null/padding field with bitfield encoding (constant attribute or null memory)
        if (f.is_null_mem || f.constant.is_some()) && f.bit_width.is_some() {
            let bw = f.bit_width.unwrap();
            let val = f.constant.unwrap_or(0) as u16;
            let mask = if bw >= 8 { 0xFFu16 } else { (1u16 << bw) - 1 };
            writeln!(out, "        // padding {} bits", bw).ok();
            writeln!(out, "        {{ let v = ({}u16 & {}) as u8; bit_acc |= v << bit_pos; bit_pos += {}; if bit_pos >= 8 {{ out.push(bit_acc); bit_acc = 0; bit_pos = 0; }} }}", val, mask, bw).ok();
            need_flush = true;
            continue;
        }

        // Bitfield field
        if let Some(bw) = f.bit_width {
            if !f.is_null_mem {
                let access = format!("{}{}", prefix, f.rust_name);
                let mask = if bw >= 8 { 0xFFu16 } else { (1u16 << bw) - 1 };
                let cast = if f.in_mem_rust_type.as_deref() == Some("bool") {
                    format!("if {} {{ 1u8 }} else {{ 0u8 }}", access)
                } else {
                    // Enum and integer in-memory types both encode as `as u8`.
                    format!("{} as u8", access)
                };
                writeln!(out, "        {{ let v = ({}) & {}u8; bit_acc |= v << bit_pos; bit_pos += {}; if bit_pos >= 8 {{ out.push(bit_acc); bit_acc = 0; bit_pos = 0; }} }}", cast, mask as u8, bw).ok();
                need_flush = true;
            }
            continue;
        }

        // Non-bitfield field — flush any pending bits first
        if need_flush {
            writeln!(out, "        if bit_pos > 0 {{ out.push(bit_acc); bit_acc = 0; bit_pos = 0; }}").ok();
            need_flush = false;
        }

        // Null/padding with byte-aligned encoding
        if f.is_null_mem {
            if let Some(ref enc) = f.encoded_rust_type {
                let bytes = match enc.as_str() {
                    "u8" | "i8" => 1usize,
                    "u16" | "i16" => 2,
                    "u32" | "i32" | "f32" => 4,
                    _ => 1,
                };
                writeln!(out, "        out.extend_from_slice(&[0u8; {}]); // padding", bytes).ok();
            }
            continue;
        }

        // Struct field
        if let Some(ref _st) = f.struct_type {
            let access = format!("{}{}", prefix, f.rust_name);
            // In packet context `out` is Vec<u8> (owned), struct.encode needs &mut Vec<u8>
            let out_arg = if out_is_owned { "&mut out" } else { "out" };
            if f.depends_on.is_some() {
                writeln!(out, "        if let Some(ref v) = {} {{ v.encode({}); }}", access, out_arg).ok();
            } else if let Some(n) = f.array_size {
                writeln!(out, "        for i in 0..{} {{ {}[i].encode({}); }}", n, access, out_arg).ok();
            } else {
                writeln!(out, "        {}.encode({});", access, out_arg).ok();
            }
            continue;
        }

        // Regular field
        let access = if f.depends_on.is_some() {
            let inner = format!("{}{}_.as_ref().copied().unwrap_or_default()", prefix, f.rust_name);
            inner
        } else if let Some(_n) = f.array_size {
            // handled below
            String::new()
        } else {
            format!("{}{}", prefix, f.rust_name)
        };

        if f.depends_on.is_some() {
            // encode conditional
            let acc = format!("{}{}", prefix, f.rust_name);
            if let Some(n) = f.array_size {
                writeln!(out, "        if let Some(ref arr) = {} {{", acc).ok();
                writeln!(out, "            for i in 0..{} {{", n).ok();
                encode_elem(f, "arr[i]", out);
                writeln!(out, "            }}").ok();
                writeln!(out, "        }}").ok();
            } else {
                writeln!(out, "        if let Some(v_opt) = {} {{", acc).ok();
                encode_elem(f, "v_opt", out);
                writeln!(out, "        }}").ok();
            }
        } else if let Some(n) = f.array_size {
            let acc = format!("{}{}", prefix, f.rust_name);
            writeln!(out, "        for i in 0..{} {{", n).ok();
            encode_elem(f, &format!("{}[i]", acc), out);
            writeln!(out, "        }}").ok();
        } else {
            encode_elem(f, &access, out);
        }
    }

    if need_flush {
        writeln!(out, "        if bit_pos > 0 {{ out.push(bit_acc); bit_acc = 0; bit_pos = 0; }}").ok();
    }
}

fn gen_field_decode(fields: &[FieldDef], out: &mut String) {
    writeln!(out, "        let mut _bit_byte: u8 = 0;").ok();
    writeln!(out, "        let mut _bit_pos: u8 = 8; // force read on first bitfield").ok();

    for f in fields {
        if f.skip_enc {
            // In-memory only field, skip with default
            if has_struct_field(f) {
                writeln!(out, "        let {} = Default::default(); // encodedType=null", f.rust_name).ok();
            }
            continue;
        }

        // Null/padding with bitfield encoding — read and discard
        if (f.is_null_mem || f.constant.is_some()) && f.bit_width.is_some() {
            let bw = f.bit_width.unwrap();
            writeln!(out, "        // padding {} bits", bw).ok();
            writeln!(out, "        {{ if _bit_pos >= 8 {{ if buf.is_empty() {{ return None; }} _bit_byte = buf[0]; *buf = &buf[1..]; _bit_pos = 0; }} _bit_pos += {}; }}", bw).ok();
            continue;
        }

        // Bitfield field
        if let Some(bw) = f.bit_width {
            if !f.is_null_mem {
                writeln!(out, "        if _bit_pos >= 8 {{ if buf.is_empty() {{ return None; }} _bit_byte = buf[0]; *buf = &buf[1..]; _bit_pos = 0; }}").ok();
                let mask: u8 = if bw >= 8 { 0xFF } else { (1u8 << bw) - 1 };
                let mem_type = f.in_mem_rust_type.as_deref().unwrap_or("u8");
                if let Some(ref et) = f.enum_type {
                    // enum bitfield: extract bits then convert to enum
                    writeln!(out, "        let {} = {}::try_from((_bit_byte >> _bit_pos) & 0x{:02X}u8).unwrap_or_default();", f.rust_name, et, mask).ok();
                } else if mem_type == "bool" {
                    writeln!(out, "        let {} = (_bit_byte >> _bit_pos) & 0x01 != 0;", f.rust_name).ok();
                } else {
                    writeln!(out, "        let {} = ((_bit_byte >> _bit_pos) & 0x{:02X}u8) as {};", f.rust_name, mask, mem_type).ok();
                }
                writeln!(out, "        _bit_pos += {};", bw).ok();
            }
            continue;
        }

        // Non-bitfield — reset bit reader
        writeln!(out, "        _bit_pos = 8; // reset bit reader").ok();

        // Null/padding byte-aligned
        if f.is_null_mem {
            if let Some(ref enc) = f.encoded_rust_type {
                let bytes = match enc.as_str() {
                    "u8" | "i8" => 1usize,
                    "u16" | "i16" => 2,
                    "u32" | "i32" | "f32" => 4,
                    _ => 1,
                };
                writeln!(out, "        if buf.len() < {} {{ return None; }} *buf = &buf[{}..]; // padding", bytes, bytes).ok();
            }
            continue;
        }

        // Struct field
        if let Some(ref st) = f.struct_type {
            if f.depends_on.is_some() {
                writeln!(out, "        let {} = if _depends_cond {{ Some({}::decode(buf)?) }} else {{ None }};", f.rust_name, st).ok();
            } else if let Some(n) = f.array_size {
                // Variable array or fixed array of structs
                writeln!(out, "        let {} = {{ let mut a_ = Vec::new(); for _ in 0..{} {{ a_.push({}::decode(buf)?); }} a_ }};", f.rust_name, n, st).ok();
            } else {
                writeln!(out, "        let {} = {}::decode(buf)?;", f.rust_name, st).ok();
            }
            continue;
        }

        // Regular field
        if f.depends_on.is_some() {
            let mem_type = if let Some(ref et) = f.enum_type {
                et.clone()
            } else {
                f.in_mem_rust_type.clone().unwrap_or_else(|| "u8".into())
            };

            if let Some(n) = f.array_size {
                writeln!(out, "        let {} = if _depends_cond {{", f.rust_name).ok();
                writeln!(out, "            let mut arr = [<{} as Default>::default(); {}];", mem_type, n).ok();
                writeln!(out, "            for i in 0..{} {{", n).ok();
                let elem_dec = decode_elem(f, out);
                writeln!(out, "                arr[i] = {};", elem_dec).ok();
                writeln!(out, "            }}").ok();
                writeln!(out, "            Some(arr)").ok();
                writeln!(out, "        }} else {{ None }};").ok();
            } else {
                writeln!(out, "        let {} = if _depends_cond {{", f.rust_name).ok();
                let elem_dec = decode_elem(f, out);
                writeln!(out, "            Some({})", elem_dec).ok();
                writeln!(out, "        }} else {{ None }};").ok();
            }
        } else if let Some(n) = f.array_size {
            let elem = gen_decode_elem_body(f);
            if f.variable_array_len_field.is_some() {
                // Variable array: use the length field already decoded
                let len_field = f.variable_array_len_field.as_ref().unwrap();
                writeln!(out, "        let {} = {{", f.rust_name).ok();
                writeln!(out, "            let mut v_ = Vec::new();").ok();
                writeln!(out, "            for _ in 0..{} {{", to_snake(len_field)).ok();
                writeln!(out, "                v_.push({});", elem).ok();
                writeln!(out, "            }}").ok();
                writeln!(out, "            v_").ok();
                writeln!(out, "        }};").ok();
            } else {
                // Fixed array
                let mem_type = f.in_mem_rust_type.clone().unwrap_or_else(|| "u8".into());
                writeln!(out, "        let {} = {{", f.rust_name).ok();
                writeln!(out, "            let mut arr_ = [<{} as Default>::default(); {}];", mem_type, n).ok();
                writeln!(out, "            for i_ in 0..{} {{", n).ok();
                writeln!(out, "                arr_[i_] = {};", elem).ok();
                writeln!(out, "            }}").ok();
                writeln!(out, "            arr_").ok();
                writeln!(out, "        }};").ok();
            }
        } else {
            let val = decode_elem(f, out);
            writeln!(out, "        let {} = {};", f.rust_name, val).ok();
        }
    }
}

/// Inline version of decode for array elements — reads bytes directly
fn gen_decode_elem_body(f: &FieldDef) -> String {
    let enc = f.encoded_rust_type.as_deref().unwrap_or("u8");
    let mem = f.in_mem_rust_type.as_deref().unwrap_or("u8");
    let byte_count = match enc {
        "u8" | "i8" => 1usize,
        "u16" | "i16" => 2,
        "u32" | "i32" | "f32" => 4,
        "u64" | "i64" | "f64" => 8,
        _ => 1,
    };

    // Just return a zero default for now — the decode_elem function writes into `out`,
    // so we can't easily inline it. Return a placeholder.
    // TODO: proper inline decode for array elements
    match &f.scale {
        ScaleKind::None => {
            if f.enum_type.is_some() {
                format!("{{ if buf.len() < {} {{ return None; }} let v = {}::try_from(buf[0]).unwrap_or_default(); *buf = &buf[{}..]; v }}", byte_count, f.enum_type.as_ref().unwrap(), byte_count)
            } else if byte_count == 1 {
                format!("{{ if buf.len() < 1 {{ return None; }} let v = buf[0] as {}; *buf = &buf[1..]; v }}", mem)
            } else {
                format!("{{ if buf.len() < {sz} {{ return None; }} let mut a = [0u8; {sz}]; a.copy_from_slice(&buf[..{sz}]); *buf = &buf[{sz}..]; {enc}::from_be_bytes(a) as {mem} }}", sz=byte_count, enc=enc, mem=mem)
            }
        }
        ScaleKind::Scaler(s) => {
            format!("{{ if buf.len() < {sz} {{ return None; }} let mut a = [0u8; {sz}]; a.copy_from_slice(&buf[..{sz}]); *buf = &buf[{sz}..]; ({enc}::from_be_bytes(a) as f64 / {scaler:.10}_f64) as {mem} }}", sz=byte_count, enc=enc, scaler=s, mem=mem)
        }
        ScaleKind::MaxOnly(max) => {
            let mi = max_int_for(enc);
            format!("{{ if buf.len() < {sz} {{ return None; }} let mut a = [0u8; {sz}]; a.copy_from_slice(&buf[..{sz}]); *buf = &buf[{sz}..]; ({enc}::from_be_bytes(a) as f64 / {mi}_f64 * {max:.10}_f64) as {mem} }}", sz=byte_count, enc=enc, mi=mi, max=max, mem=mem)
        }
        ScaleKind::MinMax(min, max) => {
            let mi = max_int_for(enc);
            format!("{{ if buf.len() < {sz} {{ return None; }} let mut a = [0u8; {sz}]; a.copy_from_slice(&buf[..{sz}]); *buf = &buf[{sz}..]; ({enc}::from_be_bytes(a) as f64 / {mi}_f64 * {range:.10}_f64 + {min:.10}_f64) as {mem} }}", sz=byte_count, enc=enc, mi=mi, range=max-min, min=min, mem=mem)
        }
    }
}

fn gen_struct_fields_list(fields: &[FieldDef]) -> String {
    let mut s = String::new();
    for f in fields {
        if !has_struct_field(f) {
            continue;
        }
        let ft = field_rust_type(f);
        writeln!(s, "    pub {}: {},", f.rust_name, ft).ok();
    }
    s
}

fn gen_struct_init_list(fields: &[FieldDef]) -> String {
    let mut s = String::new();
    for f in fields {
        if !has_struct_field(f) {
            continue;
        }
        writeln!(s, "            {},", f.rust_name).ok();
    }
    s
}

fn generate_enum_code(e: &EnumDef, out: &mut String) {
    writeln!(out, "/// Orion enum `{}`", e.xml_name).ok();
    writeln!(out, "#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]").ok();
    writeln!(out, "#[repr(u8)]").ok();
    writeln!(out, "pub enum {} {{", e.rust_name).ok();
    let mut first_variant = String::new();
    for (vname, disc, is_alias) in &e.values {
        if *is_alias {
            continue;
        }
        if first_variant.is_empty() {
            first_variant = vname.clone();
        }
        writeln!(out, "    {} = 0x{:02X},", vname, disc).ok();
    }
    writeln!(out, "}}").ok();

    // Default impl
    if !first_variant.is_empty() {
        writeln!(out, "impl Default for {} {{", e.rust_name).ok();
        writeln!(out, "    fn default() -> Self {{ {}::{} }}", e.rust_name, first_variant).ok();
        writeln!(out, "}}").ok();
    }

    // TryFrom<u8>
    writeln!(out, "impl TryFrom<u8> for {} {{", e.rust_name).ok();
    writeln!(out, "    type Error = u8;").ok();
    writeln!(out, "    fn try_from(v: u8) -> Result<Self, Self::Error> {{").ok();
    writeln!(out, "        match v {{").ok();
    for (vname, disc, is_alias) in &e.values {
        if *is_alias {
            continue;
        }
        writeln!(out, "            0x{:02X} => Ok({}::{}),", disc, e.rust_name, vname).ok();
    }
    writeln!(out, "            _ => Err(v),").ok();
    writeln!(out, "        }}").ok();
    writeln!(out, "    }}").ok();
    writeln!(out, "}}").ok();
}

fn generate_struct_code(s: &StructDef, out: &mut String) {
    writeln!(out, "/// Orion structure `{}`", s.xml_name).ok();
    writeln!(out, "#[derive(Debug, Clone)]").ok();
    writeln!(out, "pub struct {} {{", s.rust_name).ok();
    out.push_str(&gen_struct_fields_list(&s.fields));
    writeln!(out, "}}").ok();

    // Default impl
    writeln!(out, "impl Default for {} {{", s.rust_name).ok();
    writeln!(out, "    fn default() -> Self {{ Self {{").ok();
    for f in &s.fields {
        if !has_struct_field(f) {
            continue;
        }
        writeln!(out, "        {}: Default::default(),", f.rust_name).ok();
    }
    writeln!(out, "    }} }}").ok();
    writeln!(out, "}}").ok();

    // encode method
    writeln!(out, "impl {} {{", s.rust_name).ok();
    writeln!(out, "    pub fn encode(&self, out: &mut Vec<u8>) {{").ok();
    gen_field_encode(&s.fields, out, "self.", false);
    writeln!(out, "    }}").ok();

    // decode method
    writeln!(out, "    pub fn decode(buf: &mut &[u8]) -> Option<Self> {{").ok();
    writeln!(out, "        let _depends_cond = false;").ok();
    gen_field_decode(&s.fields, out);
    out.push_str("        Some(Self {\n");
    out.push_str(&gen_struct_init_list(&s.fields));
    out.push_str("        })\n");
    writeln!(out, "    }}").ok();
    writeln!(out, "}}").ok();
}

fn generate_packet_code(p: &PacketDef, out: &mut String) {
    // Inline enums
    for e in &p.inline_enums {
        generate_enum_code(e, out);
    }
    // Inline structs
    for s in &p.inline_structs {
        generate_struct_code(s, out);
    }

    writeln!(out, "/// Orion packet `{}` (ID = 0x{:02X})", p.xml_name, p.id_value).ok();
    writeln!(out, "#[derive(Debug, Clone)]").ok();
    writeln!(out, "pub struct {} {{", p.rust_name).ok();
    out.push_str(&gen_struct_fields_list(&p.fields));
    writeln!(out, "}}").ok();

    // Default impl
    writeln!(out, "impl Default for {} {{", p.rust_name).ok();
    writeln!(out, "    fn default() -> Self {{ Self {{").ok();
    for f in &p.fields {
        if !has_struct_field(f) {
            continue;
        }
        writeln!(out, "        {}: Default::default(),", f.rust_name).ok();
    }
    writeln!(out, "    }} }}").ok();
    writeln!(out, "}}").ok();

    writeln!(out, "impl {} {{", p.rust_name).ok();
    writeln!(out, "    pub const ID: u8 = 0x{:02X};", p.id_value).ok();

    // encode
    writeln!(out, "    pub fn encode(&self) -> Vec<u8> {{").ok();
    writeln!(out, "        let mut out = Vec::new();").ok();
    {
        let mut body = String::new();
        gen_field_encode(&p.fields, &mut body, "self.", true);
        out.push_str(&body);
    }
    writeln!(out, "        out").ok();
    writeln!(out, "    }}").ok();

    // decode
    writeln!(out, "    pub fn decode(data: &[u8]) -> Option<Self> {{").ok();
    writeln!(out, "        let buf = &mut &data[..];").ok();
    writeln!(out, "        let _depends_cond = false;").ok();
    gen_field_decode(&p.fields, out);
    out.push_str("        Some(Self {\n");
    out.push_str(&gen_struct_init_list(&p.fields));
    out.push_str("        })\n");
    writeln!(out, "    }}").ok();
    writeln!(out, "}}").ok();
}

fn generate_code(doc: &roxmltree::Document) -> String {
    let mut out = String::new();
    writeln!(out, "// AUTO-GENERATED by build.rs from OrionPublicProtocol.xml — DO NOT EDIT").ok();
    writeln!(out).ok();

    let root = doc.root_element();
    let mut enum_consts: HashMap<String, i64> = HashMap::new();
    let mut enum_names: HashMap<String, String> = HashMap::new(); // xml_name → rust_name

    // ── 1. Collect all top-level enums first (for constant resolution) ──
    let global_enums = parse_enums(root, &mut enum_consts);
    for e in &global_enums {
        enum_names.insert(e.xml_name.clone(), e.rust_name.clone());
    }

    // ── 2. Also pre-scan packet inline enums so array sizes resolve ──
    for child in root.children() {
        if child.tag_name().name() == "Packet" {
            let _ = parse_enums(child, &mut enum_consts);
        }
    }

    // ── 3. Generate global enums ──
    writeln!(out, "// ═══════════════════════════ Enumerations ═══════════════════════════").ok();
    for e in &global_enums {
        generate_enum_code(e, &mut out);
        writeln!(out).ok();
    }

    // ── 4. Parse & generate global structures ──
    writeln!(out, "// ══════════════════════════ Structures ══════════════════════════════").ok();
    let global_structs = parse_structs(root, &enum_consts, &enum_names);
    for s in &global_structs {
        generate_struct_code(s, &mut out);
        writeln!(out).ok();
    }

    // ── 5. Parse & generate packets ──
    writeln!(out, "// ════════════════════════════ Packets ═══════════════════════════════").ok();
    let packets = parse_packets(root, &mut enum_consts, &mut enum_names);
    for p in &packets {
        generate_packet_code(p, &mut out);
        writeln!(out).ok();
    }

    out
}
