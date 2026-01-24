//! Various `wpimath` data structures.

#[cfg(feature = "protobuf")]
use crate::protobuf::{ProtobufData, controller::*, geometry2d::*, geometry3d::*, kinematics::*, plant::*, spline::*, system::*, trajectory::*, wpimath::*};

#[cfg(feature = "struct")]
use crate::r#struct::{StructData, byte::{ByteBuffer, ByteReader}};

macro_rules! data {
    ($(#[$m: meta])* $vis: vis struct $ident: ident ( $n: literal , $s: literal ) for $proto: ident { $($(#[$fm: meta])* $f: ident ( $p: ident ) : $ty: tt ),+ $(,)? }) => {
        $(#[$m])*
        #[derive(Debug, Clone, Copy, PartialEq)]
        $vis struct $ident {
        $(
            $(#[$fm])*
            pub $f: data!(@ty $ty),
        )+
        }

        #[cfg(feature = "struct")]
        impl $crate::r#struct::StructData for $ident {
            fn schema() -> String {
                $s.to_owned()
            }

            fn struct_type_name() -> String {
                $n.to_owned()
            }

            fn pack(self, buf: &mut $crate::r#struct::byte::ByteBuffer) {
                $(
                data!(@pack(buf) $ty, self.$f);
                )+
            }

            fn unpack(read: &mut $crate::r#struct::byte::ByteReader) -> Option<Self> {
                $(
                let $f = data!(@unpack(read) $ty);
                )+
                Some(Self {
                $(
                    $f,
                )+
                })
            }
        }

        #[cfg(feature = "protobuf")]
        impl $crate::protobuf::ProtobufData for $ident {
            type Proto = $proto;

            fn from_proto(proto: Self::Proto) -> Option<Self> {
                Some(Self {
                $(
                    $f: data!(@proto_into(proto, $p) $ty),
                )+
                })
            }

            fn into_proto(self) -> Self::Proto {
                $proto {
                $(
                    $p: data!(@proto_from(self, $f) $ty),
                )+
                    ..Default::default()
                }
            }
        }
    };

    // TYPE MATCHING
    (@ty $ty: ty) => { $ty };


    // PACK MATCHING
    (@pack($b: ident) i8 , $v: expr) => {
        $b.write_i8($v);
    };
    (@pack($b: ident) i16 , $v: expr) => {
        $b.write_i16($v);
    };
    (@pack($b: ident) i32 , $v: expr) => {
        $b.write_i32($v);
    };
    (@pack($b: ident) i64 , $v: expr) => {
        $b.write_i64($v);
    };
    (@pack($b: ident) isize , $v: expr) => {
        $b.write_isize($v);
    };

    (@pack($b: ident) f32 , $v: expr) => {
        $b.write_f32($v);
    };
    (@pack($b: ident) f64 , $v: expr) => {
        $b.write_f64($v);
    };

    (@pack($b: ident) $ty: ty , $v: expr) => {
        $v.pack($b);
    };


    // UNPACK MATCHING
    (@unpack($b: ident) i8) => {
        $b.read_i8()?
    };
    (@unpack($b: ident) i16) => {
        $b.read_i16()?
    };
    (@unpack($b: ident) i32) => {
        $b.read_i32()?
    };
    (@unpack($b: ident) i64) => {
        $b.read_i64()?
    };
    (@unpack($b: ident) isize) => {
        $b.read_isize()?
    };

    (@unpack($b: ident) f32) => {
        $b.read_f32()?
    };
    (@unpack($b: ident) f64) => {
        $b.read_f64()?
    };

    (@unpack($b: ident) $ty: ty) => {
        <$ty>::unpack($b)?
    };


    // PROTOBUF MATCHING
    (@proto_into($p: ident, $i: ident) f64) => {
        $p.$i
    };
    (@proto_into($p: ident, $i: ident) $ty: ty) => {
        <$ty as $crate::protobuf::ProtobufData>::from_proto($p.$i.into_option().unwrap_or_default())?
    };

    (@proto_from($p: ident, $i: ident) f64) => {
        $p.$i
    };
    (@proto_from($p: ident, $i: ident) $ty: ty) => {
        ::protobuf::MessageField::some(<$ty as $crate::protobuf::ProtobufData>::into_proto($p.$i))
    };
}

data! {
    /// Feedforward constants that model a simple arm.
    pub struct ArmFeedforward("ArmFeedforward", "double ks;double kg;double ka;double dt") for ProtobufArmFeedforward {
        /// The static gain in volts.
        k_s(ks): f64,
        /// The gravity gain in volts.
        k_g(kg): f64,
        /// The velocity gain in V/rad/s.
        k_v(kv): f64,
        /// The acceleration gain in V/rad/s².
        k_a(ka): f64,
        /// The period in seconds.
        d_t(dt): f64,
    }
}

data! {
    /// Robot chassis speeds.
    pub struct ChassisSpeeds("ChassisSpeeds", "double vx;double vy;double omega") for ProtobufChassisSpeeds {
        /// The x velocity in m/s.
        velocity_x(vx): f64,
        /// The y velocity in m/s.
        velocity_y(vy): f64,
        /// The angular velocity in rad/s.
        omega(omega): f64,
    }
}

/// Represents a hermite spline of degree 3.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CubicHermiteSpline {
    /// The control vector for the initial point in the x dimension.
    pub x_initial_control_vector: [f64; 2],
    /// The control vector for the final point in the x dimension.
    pub x_final_control_vector: [f64; 2],
    /// The control vector for the initial point in the y dimension.
    pub y_initial_control_vector: [f64; 2],
    /// The control vector for the final point in the y dimension.
    pub y_final_control_vector: [f64; 2],
}

#[cfg(feature = "struct")]
impl StructData for CubicHermiteSpline {
    fn schema() -> String {
        "double xInitial[2];double xFinal[2];double yInitial[2];double yFinal[2]".to_owned()
    }

    fn struct_type_name() -> String {
        "CubicHermiteSpline".to_owned()
    }

    fn pack(self, buf: &mut ByteBuffer) {
        buf.write_f64(self.x_initial_control_vector[0]);
        buf.write_f64(self.x_initial_control_vector[1]);

        buf.write_f64(self.x_final_control_vector[0]);
        buf.write_f64(self.x_final_control_vector[1]);

        buf.write_f64(self.y_initial_control_vector[0]);
        buf.write_f64(self.y_initial_control_vector[1]);

        buf.write_f64(self.y_final_control_vector[0]);
        buf.write_f64(self.y_final_control_vector[1]);
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let x_initial_control_vector = [read.read_f64()?, read.read_f64()?];
        let x_final_control_vector = [read.read_f64()?, read.read_f64()?];
        let y_initial_control_vector = [read.read_f64()?, read.read_f64()?];
        let y_final_control_vector = [read.read_f64()?, read.read_f64()?];

        Some(Self {
            x_initial_control_vector,
            x_final_control_vector,
            y_initial_control_vector,
            y_final_control_vector,
        })
    }
}

#[cfg(feature = "protobuf")]
impl ProtobufData for CubicHermiteSpline {
    type Proto = ProtobufCubicHermiteSpline;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            x_initial_control_vector: proto.x_initial.try_into().ok()?,
            x_final_control_vector: proto.x_final.try_into().ok()?,
            y_initial_control_vector: proto.y_initial.try_into().ok()?,
            y_final_control_vector: proto.y_final.try_into().ok()?,
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            x_initial: self.x_initial_control_vector.into(),
            x_final: self.x_final_control_vector.into(),
            y_initial: self.y_initial_control_vector.into(),
            y_final: self.y_final_control_vector.into(),
            ..Default::default()
        }
    }
}

data! {
    /// Constants for a DC motor.
    pub struct DCMotor("DCMotor", "double ks;double kv;double ka;double dt") for ProtobufDCMotor {
        /// Voltage at which the motor constants were measured in volts.
        nominal_voltage(nominal_voltage): f64,
        /// Torque when stalled in newton meters.
        stall_torque(stall_torque): f64,
        /// Current draw when stalled in amps.
        stall_current(stall_current): f64,
        /// Current draw under no load in amps.
        free_current(free_current): f64,
        /// Angular velocity under no load in rad/s.
        free_speed(free_speed): f64,
    }
}

data! {
    /// Feedforward constants that model a differential drive drivetrain.
    pub struct DifferentialDriveFeedforward(
        "DifferentialDriveFeedforward",
        "double kVLinear;double kALinear;double kVAngular;double kAAngular"
    ) for ProtobufDifferentialDriveFeedforward {
        /// The linear velocity gain in V/(m/s).
        velocity_linear(kv_linear): f64,
        /// The linear acceleration gain in V/(m/s²).
        acceleration_linear(ka_linear): f64,
        /// The angular velocity gain in V/(rad/s).
        velocity_angular(kv_angular): f64,
        /// The angular acceleration gain in V/(rad/s²).
        acceleration_angular(ka_angular): f64,
    }
}

data! {
    /// Kinematics for a differential drive.
    pub struct DifferentialDriveKinematics("DifferentialDriveKinematics", "double track_width") for ProtobufDifferentialDriveKinematics {
        /// The differential drive track width in meters.
        track_width(track_width): f64,
    }
}

data! {
    /// Represents wheel positions for a differential drive drivetrain.
    pub struct DifferentialDriveWheelPositions("DifferentialDriveWheelPositions", "double left;double right") for ProtobufDifferentialDriveWheelPositions {
        /// Distance measured by the left side.
        left(left): f64,
        /// Distance measured by the right side.
        right(right): f64,
    }
}

data! {
    /// Represents the wheel speeds for a differential drive drivetrain.
    pub struct DifferentialDriveWheelSpeeds("DifferentialDriveWheelSpeeds", "double left;double right") for ProtobufDifferentialDriveWheelSpeeds {
        /// Speed of the left side of the robot in m/s.
        left(left): f64,
        /// Speed of the right side of the robot in m/s.
        right(right): f64,
    }
}

data! {
    /// Represents the motor voltages for a differential drive drivetrain.
    pub struct DifferentialDriveWheelVoltages("DifferentialDriveWheelVoltages", "double left;double right") for ProtobufDifferentialDriveWheelVoltages {
        /// Left wheel voltage.
        left(left): f64,
        /// Right wheel voltage.
        right(right): f64,
    }
}

data! {
    /// Feedforward constants that model a simple elevator.
    pub struct ElevatorFeedforward("ElevatorFeedforward", "double ks;double kg;double kv;double ka;double dt") for ProtobufElevatorFeedforward {
        /// The static gain in volts.
        k_s(ks): f64,
        /// The gravity gain in volts.
        k_g(kg): f64,
        /// The velocity gain in V/(m/s).
        k_v(kv): f64,
        /// The acceleration gain in V/(m/s²).
        k_a(ka): f64,
        /// The period in seconds.
        d_t(dt): f64,
    }
}

data! {
    /// Represents a 2D ellipse space containing translational, rotational, and scaling components.
    pub struct Ellipse2d("Ellipse2d", "Pose2d center;double xSemiAxis;double ySemiAxis") for ProtobufEllipse2d {
        /// The center of the ellipse.
        center(center): Pose2d,
        /// The x semi-axis.
        x_semi_axis(xSemiAxis): f64,
        /// The y semi-axis.
        y_semi_axis(ySemiAxis): f64,
    }
}

/// Exponential profile state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExponentialProfileState {
    /// The position at this state
    pub position: f64,
    /// The velocity at this state
    pub velocity: f64,
}

#[cfg(feature = "struct")]
impl StructData for ExponentialProfileState {
    fn schema() -> String {
        "double position;double velocity".to_owned()
    }

    fn struct_type_name() -> String {
        "ExponentialProfileState".to_owned()
    }

    fn pack(self, buf: &mut ByteBuffer) {
        buf.write_f64(self.position);
        buf.write_f64(self.velocity);
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        Some(Self {
            position: read.read_f64()?,
            velocity: read.read_f64()?,
        })
    }
}

/// Represents a plant defined using state-space notation.
///
/// A plant is a mathematical model of a system's dynamics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinearSystem<const S: usize, const I: usize, const O: usize> {
    /// The system matrix A.
    pub a: Matrix<S, S>,
    /// The system matrix B.
    pub b: Matrix<S, I>,
    /// The system matrix C.
    pub c: Matrix<O, S>,
    /// The system matrix D.
    pub d: Matrix<O, I>,
}

#[cfg(feature = "struct")]
impl<const S: usize, const I: usize, const O: usize> StructData for LinearSystem<S, I, O,> {
    fn schema() -> String {
        format!(
            "{} a;{} b;{} c;{} d",
            <Matrix<S, S>>::struct_type_name(),
            <Matrix<S, I>>::struct_type_name(),
            <Matrix<O, S>>::struct_type_name(),
            <Matrix<O, I>>::struct_type_name(),
        )
    }

    fn struct_type_name() -> String {
        format!("LinearSystem__{}_{}_{}", S, I, O)
    }

    fn pack(self, buf: &mut ByteBuffer) {
        self.a.pack(buf);
        self.b.pack(buf);
        self.c.pack(buf);
        self.d.pack(buf);
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> where Self: Sized {
        let a = Matrix::unpack(read)?;
        let b = Matrix::unpack(read)?;
        let c = Matrix::unpack(read)?;
        let d = Matrix::unpack(read)?;

        Some(Self { a, b, c, d })
    }
}

#[cfg(feature = "protobuf")]
impl<const S: usize, const I: usize, const O: usize> ProtobufData for LinearSystem<S, I, O> {
    type Proto = ProtobufLinearSystem;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            a: Matrix::from_proto(proto.a.into_option().unwrap_or_default())?,
            b: Matrix::from_proto(proto.b.into_option().unwrap_or_default())?,
            c: Matrix::from_proto(proto.c.into_option().unwrap_or_default())?,
            d: Matrix::from_proto(proto.d.into_option().unwrap_or_default())?,
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            num_states: S as u32,
            num_inputs: I as u32,
            num_outputs: O as u32,
            a: protobuf::MessageField::some(self.a.into_proto()),
            b: protobuf::MessageField::some(self.a.into_proto()),
            c: protobuf::MessageField::some(self.a.into_proto()),
            d: protobuf::MessageField::some(self.a.into_proto()),
            ..Default::default()
        }
    }
}

/// Represents a RxC matrix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix<const R: usize, const C: usize> {
    /// The data of the matrix, indexed as `data[row][col]`.
    pub data: [[f64; C]; R],
}

#[cfg(feature = "struct")]
impl<const R: usize, const C: usize> StructData for Matrix<R, C> {
    fn schema() -> String {
        format!("double data[{}]", R * C)
    }

    fn struct_type_name() -> String {
        format!("Matrix__{}_{}", R, C)
    }

    fn pack(self, buf: &mut ByteBuffer) {
        self.data.into_iter().flatten().for_each(|value| {
            buf.write_f64(value);
        });
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let mut data = [[0.0; C]; R];
        for value in data.iter_mut().flatten() {
            *value = read.read_f64()?;
        }
        Some(Self { data })
    }
}

#[cfg(feature = "protobuf")]
impl<const R: usize, const C: usize> ProtobufData for Matrix<R, C> {
    type Proto = ProtobufMatrix;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        let mut data = [[0.0; C]; R];
        if proto.data.len() != C * R { return None; };
        proto.data.into_iter()
            .enumerate()
            .for_each(|(i, num)| {
                data[i / C][i % C] = num;
            });
        Some(Self {
            data,
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            num_rows: R as u32,
            num_cols: C as u32,
            data: self.data.into_iter().flatten().collect(),
            ..Default::default()
        }
    }
}

data! {
    /// Kinematics for a mecanum drive.
    pub struct MecanumDriveKinematics(
        "MecanumDriveKinematics",
        "Translation2d front_left;Translation2d front_right;Translation2d rear_left;Translation2d rear_right"
    ) for ProtobufMecanumDriveKinematics {
        /// The front-left wheel translation.
        front_left(front_left): Translation2d,
        /// The front-right wheel translation.
        front_right(front_right): Translation2d,
        /// The rear-right wheel translation.
        rear_left(rear_left): Translation2d,
        /// The rear-left wheel translation.
        rear_right(rear_right): Translation2d,
    }
}

data! {
    /// Represents the wheel positions for a mecanum drive drivetrain.
    pub struct MecanumDriveWheelPositions(
        "MecanumDriveWheelPositions",
        "double front_left;double front_right;double rear_left;double rear_right"
    ) for ProtobufMecanumDriveWheelPositions {
        /// Distance measured by the front-left wheel in meters.
        front_left(front_left): f64,
        /// Distance measured by the front-right wheel in meters.
        front_right(front_right): f64,
        /// Distance measured by the rear-left wheel in meters.
        rear_left(rear_left): f64,
        /// Distance measured by the rear-right wheel in meters.
        rear_right(rear_right): f64,
    }
}

data! {
    /// Represents the wheel speeds for a mecanum drive drivetrain.
    pub struct MecanumDriveWheelSpeeds("MecanumDriveWheelSpeeds", "double front_left;double front_right;double rear_left;double rear_right") for ProtobufMecanumDriveWheelSpeeds {
        /// Speed of the front-left wheel in m/s.
        front_left(front_left): f64,
        /// Speed of the front-right wheel in m/s.
        front_right(front_right): f64,
        /// Speed of the rear-left wheel in m/s.
        rear_left(rear_left): f64,
        /// Speed of the rear-right wheel in m/s.
        rear_right(rear_right): f64,
    }
}

data! {
    /// Represents a 2D pose containing translational and rotational elements.
    pub struct Pose2d("Pose2d", "Translation2d translation;Rotation2d rotation") for ProtobufPose2d {
        /// The translation component of the transformation.
        translation(translation): Translation2d,
        /// The rotational component of the transformation.
        rotation(rotation): Rotation2d,
    }
}

data! {
    /// Represents a 3D pose containing translational and rotational elements.
    pub struct Pose3d("Pose3d", "Translation3d translation;Rotation3d rotation") for ProtobufPose3d {
        /// The translation component of the transformation.
        translation(translation): Translation3d,
        /// The rotational component of the transformation.
        rotation(rotation): Rotation3d,
    }
}

data! {
    /// Represents a quaternion.
    pub struct Quaternion("Quaternion", "double w;double x;double y;double z") for ProtobufQuaternion {
        /// The w component of the quaternion.
        w(w): f64,
        /// The x component of the quaternion.
        x(x): f64,
        /// The y component of the quaternion.
        y(y): f64,
        /// The z component of the quaternion.
        z(z): f64,
    }
}

/// Represents a hermite spline of degree 5.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuinticHermiteSpline {
    /// The control vector for the initial point in the x dimension.
    pub x_initial: [f64; 3],
    /// The control vector for the final point in the x dimension.
    pub x_final: [f64; 3],
    /// The control vector for the initial point in the y dimension.
    pub y_initial: [f64; 3],
    /// The control vector for the final point in the y dimension.
    pub y_final: [f64; 3],
}

#[cfg(feature = "struct")]
impl StructData for QuinticHermiteSpline {
    fn schema() -> String {
        "double xInitial[3];double xFinal[3];double yInitial[3];double yFinal[3]".to_owned()
    }

    fn struct_type_name() -> String {
        "QuinticHermiteSpline".to_owned()
    }

    fn pack(self, buf: &mut ByteBuffer) {
        buf.write_f64(self.x_initial[0]);
        buf.write_f64(self.x_initial[1]);
        buf.write_f64(self.x_initial[2]);

        buf.write_f64(self.x_final[0]);
        buf.write_f64(self.x_final[1]);
        buf.write_f64(self.x_final[2]);

        buf.write_f64(self.y_initial[0]);
        buf.write_f64(self.y_initial[1]);
        buf.write_f64(self.y_initial[2]);

        buf.write_f64(self.y_final[0]);
        buf.write_f64(self.y_final[1]);
        buf.write_f64(self.y_final[2]);
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let x_initial = [read.read_f64()?, read.read_f64()?, read.read_f64()?];
        let x_final = [read.read_f64()?, read.read_f64()?, read.read_f64()?];
        let y_initial = [read.read_f64()?, read.read_f64()?, read.read_f64()?];
        let y_final = [read.read_f64()?, read.read_f64()?, read.read_f64()?];

        Some(Self {
            x_initial,
            x_final,
            y_initial,
            y_final,
        })
    }
}

#[cfg(feature = "protobuf")]
impl ProtobufData for QuinticHermiteSpline {
    type Proto = ProtobufQuinticHermiteSpline;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            x_initial: proto.x_initial.try_into().ok()?,
            x_final: proto.x_final.try_into().ok()?,
            y_initial: proto.y_initial.try_into().ok()?,
            y_final: proto.y_final.try_into().ok()?,
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            x_initial: self.x_initial.into(),
            x_final: self.x_final.into(),
            y_initial: self.y_initial.into(),
            y_final: self.y_final.into(),
            ..Default::default()
        }
    }
}

data! {
    /// Represents a 2D rectangular space containing translational, rotational, and scaling components.
    pub struct Rectangle2d("Rectangle2d", "Pose2d center;double xWidth;double yWidth") for ProtobufRectangle2d {
        /// The center of the rectangle.
        center(center): Pose2d,
        /// The x size component of the rectangle.
        x_width(xWidth): f64,
        /// The y size component of the rectangle.
        y_width(yWidth): f64,
    }
}

data! {
    /// Represents a rotation in a 2D coordinate frame representd by a point on the unit circle.
    pub struct Rotation2d("Rotation2d", "double value") for ProtobufRotation2d {
        /// The rotation in radians.
        value(value): f64,
    }
}

data! {
    /// Represents a rotation in a 2D coordinate frame representd by a point on the unit circle.
    pub struct Rotation3d("Rotation3d", "Quaternion q") for ProtobufRotation3d {
        /// The quaternion representation of the rotation.
        quaternion(q): Quaternion,
    }
}

data! {
    /// Feedforward constants that model a simple permanent-magnet DC motor.
    pub struct SimpleMotorFeedforward("SimpleMotorFeedforward", "double ks;double kv;double ka;double dt") for ProtobufSimpleMotorFeedforward {
        /// The static gain in volts.
        k_s(ks): f64,
        /// The velocity gain in V/(units/s).
        k_v(kv): f64,
        /// The acceleration gain in V/(units/s²).
        k_a(ka): f64,
        /// The period in seconds.
        d_t(dt): f64,
    }
}

/// Kinematics for a swerve drive with N modules.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwerveDriveKinematics<const N: usize> {
    /// The swerve module locations.
    pub modules: [Translation2d; N],
}

#[cfg(feature = "struct")]
impl<const N: usize> StructData for SwerveDriveKinematics<N> {
    fn schema() -> String {
        format!("Translation2d modules[{N}]")
    }

    fn struct_type_name() -> String {
        format!("SwerveDriveKinematics__{}", N)
    }

    fn pack(self, buf: &mut ByteBuffer) {
        Translation2d::pack_iter(self.modules, buf);
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let modules = Translation2d::unpack_array(read)?;
        Some(Self { modules })
    }
}

#[cfg(feature = "protobuf")]
impl<const N: usize> ProtobufData for SwerveDriveKinematics<N> {
    type Proto = ProtobufSwerveDriveKinematics;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            modules: proto.modules.into_iter()
                .map(Translation2d::from_proto)
                .collect::<Option<Vec<_>>>()?
                .try_into()
                .ok()?,
        })
    }
    
    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            modules: self.modules.into_iter()
                .map(|module| module.into_proto())
                .collect(),
            ..Default::default()
        }
    }
}

data! {
    /// Represents the state of one swerve module.
    pub struct SwerveModulePosition("SwerveModulePosition", "double distance;Rotation2d angle") for ProtobufSwerveModulePosition {
        /// Distance measured by the wheel of the module in meters.
        distance(distance): f64,
        /// Angle of the module.
        angle(angle): Rotation2d,
    }
}

data! {
    /// Represents the state of one swerve module.
    pub struct SwerveModuleState("SwerveModuleState", "double speed;Rotation2d angle") for ProtobufSwerveModuleState {
        /// The speed of the wheel of the module in m/s.
        speed(speed): f64,
        /// The angle of the module.
        angle(angle): Rotation2d,
    }
}

/// Represents a time-parameterized trajectory. The trajectory contains of various States that represent the pose, curvature, time elapsed, velocity, and acceleration at that point.
#[derive(Debug, Clone, PartialEq)]
pub struct Trajectory {
    /// The states of the trajectory.
    pub states: Vec<TrajectoryState>,
}

#[cfg(feature = "protobuf")]
impl ProtobufData for Trajectory {
    type Proto = ProtobufTrajectory;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            states: proto.states.into_iter()
                .map(TrajectoryState::from_proto)
                .collect::<Option<_>>()?
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            states: self.states.into_iter()
                .map(|state| state.into_proto())
                .collect(),
            ..Default::default()
        }
    }
}

/// Represents a state at a point in a trajectory.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrajectoryState {
    /// The time elapsed since the beginning of the trajectory in seconds.
    pub time: f64,
    /// The speed at that point of the trajector in m/s.
    pub velocity: f64,
    /// The acceleration at that point of the trajectory in m/s².
    pub acceleration: f64,
    /// The pose at that point of the trajectory in meters.
    pub pose: Pose2d,
    /// The curvature at that point of the trajectory in rad/m.
    pub curvature: f64,
}

#[cfg(feature = "protobuf")]
impl ProtobufData for TrajectoryState {
    type Proto = ProtobufTrajectoryState;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            time: proto.time,
            velocity: proto.velocity,
            acceleration: proto.acceleration,
            pose: Pose2d::from_proto(proto.pose.into_option().unwrap_or_default())?,
            curvature: proto.curvature,
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            time: self.time,
            velocity: self.velocity,
            acceleration: self.acceleration,
            pose: protobuf::MessageField::some(self.pose.into_proto()),
            curvature: self.curvature,
            ..Default::default()
        }
    }
}

data! {
    /// Represents a transformation for a Pose2d in the pose's frame.
    pub struct Transform2d("Transform2d", "Translation2d translation;Rotation2d rotation") for ProtobufTransform2d {
        /// The translation component of the transformation.
        translation(translation): Translation2d,
        /// The rotational component of the transformation.
        rotation(rotation): Rotation2d,
    }
}

data! {
    /// Represents a transformation for a Pose3d in the pose's frame.
    pub struct Transform3d("Transform3d", "Translation3d translation;Rotation3d rotation") for ProtobufTransform3d {
        /// The translation component of the transformation.
        translation(translation): Translation3d,
        /// The rotational component of the transformation.
        rotation(rotation): Rotation3d,
    }
}

data! {
    /// Represents a translation in 2D space.
    pub struct Translation2d("Translation2d", "double x;double y") for ProtobufTranslation2d {
        /// The x component of the translation.
        x(x): f64,
        /// The y component of the translation.
        y(y): f64,
    }
}

data! {
    /// Represents a translation in 2D space.
    pub struct Translation3d("Translation3d", "double x;double y;double z") for ProtobufTranslation3d {
        /// The x component of the translation.
        x(x): f64,
        /// The y component of the translation.
        y(y): f64,
        /// The z component of the translation.
        z(z): f64,
    }
}

/// Trapezoid profile state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrapezoidProfileState {
    /// The position at this state.
    pub position: f64,
    /// The velocity at this state.
    pub velocity: f64,
}

#[cfg(feature = "struct")]
impl StructData for TrapezoidProfileState {
    fn schema() -> String {
        "double position;double velocity".to_owned()
    }

    fn struct_type_name() -> String {
        "TrapezoidProfileState".to_owned()
    }

    fn pack(self, buf: &mut ByteBuffer) {
        buf.write_f64(self.position);
        buf.write_f64(self.velocity);
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        Some(Self {
            position: read.read_f64()?,
            velocity: read.read_f64()?,
        })
    }
}

data! {
    /// Represents a change in distance along a 2D arc.
    pub struct Twist2d("Twist2d", "double dx;double dy;double dtheta") for ProtobufTwist2d {
        /// The linear "dx" component.
        dx(dx): f64,
        /// The linear "dy" component.
        dy(dy): f64,
        /// The linear "dtheta" component in radians.
        dtheta(dtheta): f64,
    }
}

data! {
    /// Represents a change in distance along a 3D arc.
    pub struct Twist3d("Twist3d", "double dx;double dy;double dz;double rx;double ry;double rz") for ProtobufTwist3d {
        /// The linear "dx" component.
        dx(dx): f64,
        /// The linear "dy" component.
        dy(dy): f64,
        /// The linear "dz" component.
        dz(dz): f64,
        /// The rotation vector x component in radians.
        rx(rx): f64,
        /// The rotation vector y component in radians.
        ry(ry): f64,
        /// The rotation vector z component in radians.
        rz(rz): f64,
    }
}

/// Represents an N-dimensional Vector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector<const N: usize> {
    /// The rows of the vector.
    pub rows: [f64; N],
}

#[cfg(feature = "struct")]
impl<const N: usize> StructData for Vector<N> {
    fn schema() -> String {
        format!("double data[{N}]")
    }

    fn struct_type_name() -> String {
        format!("Vector__{}", N)
    }

    fn pack(self, buf: &mut ByteBuffer) {
        for value in self.rows {
            buf.write_f64(value);
        }
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let mut rows = [0.0; N];
        for value in rows.iter_mut() {
            *value = read.read_f64()?;
        }
        Some(Self { rows })
    }
}

#[cfg(feature = "protobuf")]
impl<const N: usize> ProtobufData for Vector<N> {
    type Proto = ProtobufVector;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            rows: proto.rows.try_into().ok()?,
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            rows: self.rows.into(),
            ..Default::default()
        }
    }
}

