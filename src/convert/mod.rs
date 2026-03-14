pub mod to_cigi;
pub mod to_orion;

/// Maximum gimbal slew rate (rad/s) used for CIGI gain/level scaling — 60 °/s.
///
/// Shared by `to_cigi` (encode) and `to_orion` (decode) so both sides of the
/// normalisation always use the same scale factor.
pub const MAX_SLEW_RATE: f32 = 1.047_198; // 60°/s
