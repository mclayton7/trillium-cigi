pub mod to_cigi;
pub mod to_orion;

/// Maximum gimbal slew rate (rad/s) used for CIGI gain/level scaling — 60 °/s.
///
/// Shared by `to_cigi` (encode) and `to_orion` (decode) so both sides of the
/// normalisation always use the same scale factor.
pub const MAX_SLEW_RATE: f32 = std::f32::consts::FRAC_PI_3; // 60°/s = π/3
