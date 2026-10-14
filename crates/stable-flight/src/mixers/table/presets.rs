//! Preset tables for [`TableMixer`]; numbering and sign rules are in the
//! [module docs](super#presets). Diagrams are seen from above, nose up; `cw`/`ccw` is the
//! prop's spin.

use core::f32::consts::SQRT_2;

use super::{MixRow, MixerConfig, TableMixer};

/// tan 22.5°: an octocopter's inner roll/pitch factor once the outer one is normalized to 1.
const OCTO_INNER: f32 = SQRT_2 - 1.0;

const QUAD_X: [MixRow; 4] = [
    MixRow::motor(0, 1.0, 1.0, -1.0),
    MixRow::motor(1, -1.0, 1.0, 1.0),
    MixRow::motor(2, 1.0, -1.0, 1.0),
    MixRow::motor(3, -1.0, -1.0, -1.0),
];

const QUAD_PLUS: [MixRow; 4] = [
    MixRow::motor(0, 0.0, 1.0, -1.0),
    MixRow::motor(1, -1.0, 0.0, 1.0),
    MixRow::motor(2, 0.0, -1.0, -1.0),
    MixRow::motor(3, 1.0, 0.0, 1.0),
];

const HEX_X: [MixRow; 6] = [
    MixRow::motor(0, 0.5, 1.0, -1.0),
    MixRow::motor(1, -0.5, 1.0, 1.0),
    MixRow::motor(2, -1.0, 0.0, -1.0),
    MixRow::motor(3, -0.5, -1.0, 1.0),
    MixRow::motor(4, 0.5, -1.0, -1.0),
    MixRow::motor(5, 1.0, 0.0, 1.0),
];

const HEX_PLUS: [MixRow; 6] = [
    MixRow::motor(0, 0.0, 1.0, -1.0),
    MixRow::motor(1, -1.0, 0.5, 1.0),
    MixRow::motor(2, -1.0, -0.5, -1.0),
    MixRow::motor(3, 0.0, -1.0, 1.0),
    MixRow::motor(4, 1.0, -0.5, -1.0),
    MixRow::motor(5, 1.0, 0.5, 1.0),
];

const OCTO_X: [MixRow; 8] = [
    MixRow::motor(0, OCTO_INNER, 1.0, -1.0),
    MixRow::motor(1, -OCTO_INNER, 1.0, 1.0),
    MixRow::motor(2, -1.0, OCTO_INNER, -1.0),
    MixRow::motor(3, -1.0, -OCTO_INNER, 1.0),
    MixRow::motor(4, -OCTO_INNER, -1.0, -1.0),
    MixRow::motor(5, OCTO_INNER, -1.0, 1.0),
    MixRow::motor(6, 1.0, -OCTO_INNER, -1.0),
    MixRow::motor(7, 1.0, OCTO_INNER, 1.0),
];

const Y6: [MixRow; 6] = [
    MixRow::motor(0, 1.0, 0.5, -1.0),
    MixRow::motor(1, -1.0, 0.5, -1.0),
    MixRow::motor(2, 0.0, -1.0, -1.0),
    MixRow::motor(3, 1.0, 0.5, 1.0),
    MixRow::motor(4, -1.0, 0.5, 1.0),
    MixRow::motor(5, 0.0, -1.0, 1.0),
];

const X8: [MixRow; 8] = [
    MixRow::motor(0, 1.0, 1.0, -1.0),
    MixRow::motor(1, -1.0, 1.0, 1.0),
    MixRow::motor(2, -1.0, -1.0, -1.0),
    MixRow::motor(3, 1.0, -1.0, 1.0),
    MixRow::motor(4, 1.0, 1.0, 1.0),
    MixRow::motor(5, -1.0, 1.0, -1.0),
    MixRow::motor(6, -1.0, -1.0, 1.0),
    MixRow::motor(7, 1.0, -1.0, -1.0),
];

const TRI: [MixRow; 4] = [
    MixRow::motor(0, 1.0, 0.5, 0.0),
    MixRow::motor(1, -1.0, 0.5, 0.0),
    MixRow::motor(2, 0.0, -1.0, 0.0),
    MixRow::servo(0, 0.0, 0.0, 1.0),
];

const TAILSITTER_2P: [MixRow; 4] = [
    MixRow::motor(0, 0.0, 0.0, 1.0),
    MixRow::motor(1, 0.0, 0.0, -1.0),
    MixRow::servo(0, -1.0, 1.0, 0.0),
    MixRow::servo(1, 1.0, 1.0, 0.0),
];

const PLANE: [MixRow; 4] = [
    MixRow::motor(0, 0.0, 0.0, 0.0),
    MixRow::servo(0, 1.0, 0.0, 0.0),
    MixRow::servo(1, 0.0, 1.0, 0.0),
    MixRow::servo(2, 0.0, 0.0, 1.0),
];

const BI: [MixRow; 4] = [
    MixRow::motor(0, 1.0, 0.0, 0.0),
    MixRow::motor(1, -1.0, 0.0, 0.0),
    MixRow::servo(0, 0.0, 1.0, -1.0),
    MixRow::servo(1, 0.0, 1.0, 1.0),
];

impl TableMixer<4> {
    /// Quadcopter, X frame, in the RealFlight QuadCopterX order (not clockwise):
    ///
    /// ```text
    ///   0 cw    1 ccw
    ///       \  /
    ///        ><
    ///       /  \
    ///   2 ccw   3 cw
    /// ```
    pub fn quad_x() -> Self {
        Self::from_valid(QUAD_X, MixerConfig::default())
    }

    /// Quadcopter, + frame:
    ///
    /// ```text
    ///        0 cw
    ///         |
    ///  3 ccw -+- 1 ccw
    ///         |
    ///        2 cw
    /// ```
    pub fn quad_plus() -> Self {
        Self::from_valid(QUAD_PLUS, MixerConfig::default())
    }

    /// Tricopter with 3 motors and 1 servo; yaw comes from the tail servo alone:
    ///
    /// ```text
    ///   0 cw    1 ccw
    ///      \   /
    ///        |
    ///      2 cw   servo 0 tilts it
    /// ```
    ///
    /// Positive servo 0 tilts the tail motor's thrust to the left, pushing the tail left and
    /// the nose right. The thrust lost to the tilt (1/cos) and the tail prop's drag torque
    /// are not compensated.
    pub fn tri() -> Self {
        Self::from_valid(TRI, MixerConfig::default())
    }

    /// Bicopter with 2 motors and 2 servos: roll from differential thrust; pitch from tilting
    /// both rotors together, yaw from tilting them opposite ways:
    ///
    /// ```text
    ///   0 cw ---+--- 1 ccw
    ///   servo 0      servo 1
    /// ```
    ///
    /// Positive servo tilts its rotor's thrust aft (the top toward the tail), which pitches
    /// the nose up with rotors above the center of gravity; servo 0 tilts the left rotor,
    /// servo 1 the right.
    pub fn bi() -> Self {
        Self::from_valid(BI, MixerConfig::default())
    }

    /// Two-motor tailsitter with elevons, in hover (nose up) and forward flight alike: its
    /// factors are in the airplane frame, which both regimes' moments come in. Seen from above
    /// in forward flight (airplane FRD: x the nose, z the belly):
    ///
    /// ```text
    ///          nose
    ///   0 ------+------ 1
    ///   servo 0   servo 1     elevons on the trailing edge
    /// ```
    ///
    /// - **Yaw** (about z; in hover, tilting sideways): differential thrust, more on the left
    ///   motor to yaw right.
    /// - **Pitch** (about y): both elevons together, in the prop wash (or the airflow, flying
    ///   forward).
    /// - **Roll** (about x; in hover, turning about the vertical): the elevons opposite ways,
    ///   like ailerons.
    ///
    /// Positive servo raises its elevon's trailing edge; servo 0 is the left elevon, servo 1
    /// the right. The props' reaction torque from differential thrust (about x) is not
    /// compensated, nor is surface authority scaled with prop wash (the plane stabilizer
    /// scales its moments with airspeed).
    pub fn tailsitter_2p() -> Self {
        Self::from_valid(TAILSITTER_2P, MixerConfig::default())
    }

    /// Conventional airplane: one tractor motor and three surfaces, each carrying one moment.
    /// Seen from above (airplane FRD: x the nose, z the belly):
    ///
    /// ```text
    ///            0
    ///   servo 0 -+- servo 0     ailerons
    ///            |
    ///   servo 1 -+- servo 1     elevator, servo 2 rudder
    /// ```
    ///
    /// Each positive servo deflects its surface the way that makes a positive moment: servo 0
    /// (ailerons) rolls right, servo 1 (elevator, trailing edge up) raises the nose, servo 2
    /// (rudder, trailing edge right) turns the nose right. The motor carries thrust only, so
    /// full throttle drives it to the mixer's `thrust_factor` (0.9 by default).
    pub fn plane() -> Self {
        Self::from_valid(PLANE, MixerConfig::default())
    }
}

impl TableMixer<6> {
    /// Hexacopter, X frame (no motor on the nose):
    ///
    /// ```text
    ///    0 cw   1 ccw
    ///  5 ccw  +  2 cw
    ///    4 cw   3 ccw
    /// ```
    pub fn hex_x() -> Self {
        Self::from_valid(HEX_X, MixerConfig::default())
    }

    /// Hexacopter, + frame (a motor on the nose):
    ///
    /// ```text
    ///        0 cw
    ///  5 ccw      1 ccw
    ///         +
    ///  4 cw       2 cw
    ///        3 ccw
    /// ```
    pub fn hex_plus() -> Self {
        Self::from_valid(HEX_PLUS, MixerConfig::default())
    }

    /// Y6: three coaxial arms (two forward at ±60°, one aft). Top props 0–2 spin clockwise,
    /// bottom props 3–5 (under 0–2) counterclockwise:
    ///
    /// ```text
    ///   0/3     1/4
    ///      \   /
    ///        |
    ///       2/5
    /// ```
    pub fn y6() -> Self {
        Self::from_valid(Y6, MixerConfig::default())
    }
}

impl TableMixer<8> {
    /// Octocopter, X frame (no motor on the nose):
    ///
    /// ```text
    ///        0 cw  1 ccw
    ///  7 ccw            2 cw
    ///            +
    ///  6 cw             3 ccw
    ///        5 ccw 4 cw
    /// ```
    pub fn octo_x() -> Self {
        Self::from_valid(OCTO_X, MixerConfig::default())
    }

    /// X8: an X quad with coaxial arms. Top props 0–3 (clockwise from front-left) spin like an
    /// X quad's; bottom props 4–7 (under 0–3) spin the other way:
    ///
    /// ```text
    ///   0 cw/4 ccw    1 ccw/5 cw
    ///             ><
    ///   3 ccw/7 cw    2 cw/6 ccw
    /// ```
    pub fn x8() -> Self {
        Self::from_valid(X8, MixerConfig::default())
    }
}
