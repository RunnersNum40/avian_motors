use avian3d::prelude::*;
use bevy::prelude::*;

// Conditional compilation for f32 and f64 support
#[cfg(feature = "f64")]
use bevy::math::DVec3 as Vector3;
#[cfg(not(feature = "f64"))]
use bevy::math::Vec3 as Vector3;

#[cfg(not(feature = "f64"))]
use std::f32::consts::PI;
#[cfg(feature = "f64")]
use std::f64::consts::PI;

#[cfg(feature = "f64")]
type Scalar = f64;
#[cfg(not(feature = "f64"))]
type Scalar = f32;

#[derive(Component, Debug, Clone)]
pub struct RevoluteMotor;

#[derive(Component, Debug, Clone)]
pub struct AngularVelocityTarget(pub Scalar);

#[derive(Component, Debug, Clone)]
pub struct TargetRotation(pub Scalar);

#[derive(Component, Debug, Clone)]
pub struct MotorProportionalGain(pub Scalar);

#[derive(Component, Debug, Clone)]
pub struct MotorDerivativeGain(pub Scalar);

#[derive(Component, Debug, Clone)]
pub struct MotorIntegralGain(pub Scalar);

#[derive(Component, Debug, Clone)]
pub struct MotorMaxTorque(pub Option<Scalar>);

#[derive(Component, Debug, Clone)]
pub struct MotorMaxAngularVelocity(pub Option<Scalar>);

#[derive(Component, Debug, Clone)]
pub struct MotorTotalRotation(pub Scalar);

#[derive(Component, Debug, Clone)]
pub struct MotorAngularVelocity(pub Scalar);

#[derive(Bundle, Debug, Clone)]
pub struct RevoluteMotorBundle {
    pub motor: RevoluteMotor,
    pub stiffness: MotorProportionalGain,
    pub damping: MotorDerivativeGain,
    pub integral_gain: MotorIntegralGain,
    pub max_torque: MotorMaxTorque,
    pub max_angular_velocity: MotorMaxAngularVelocity,
    pub total_rotation: MotorTotalRotation,
    pub angular_velocity: MotorAngularVelocity,
}

impl Default for RevoluteMotorBundle {
    fn default() -> Self {
        Self {
            motor: RevoluteMotor,
            stiffness: MotorProportionalGain(1e-7),
            damping: MotorDerivativeGain(0.0),
            integral_gain: MotorIntegralGain(0.0),
            max_torque: MotorMaxTorque(None),
            max_angular_velocity: MotorMaxAngularVelocity(None),
            total_rotation: MotorTotalRotation(0.0),
            angular_velocity: MotorAngularVelocity(0.0),
        }
    }
}

pub struct MotorPlugin {
    pub remove_dampning: bool,
    pub substep_count: Option<u32>,
}

impl Plugin for MotorPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(FixedUpdate, multi_target_warning)
            .add_systems(FixedUpdate, warn_if_axis_not_normalized)
            .add_systems(FixedUpdate, enforce_velocity_constraints)
            .add_systems(FixedUpdate, update_motor_rotation_state)
            .add_systems(FixedUpdate, update_motor_angular_velocity_state)
            .add_systems(FixedUpdate, apply_velocity_based_torque)
            .add_systems(FixedUpdate, apply_rotation_based_torque);

        if self.remove_dampning {
            app.add_systems(FixedUpdate, disable_damping);
        }

        if let Some(substep_count) = self.substep_count {
            app.insert_resource(SubstepCount(substep_count));
        }
    }
}

impl Default for MotorPlugin {
    fn default() -> Self {
        Self {
            remove_dampning: true,
            substep_count: Some(1000),
        }
    }
}

fn warn_if_axis_not_normalized(query: Query<&RevoluteJoint, Added<RevoluteMotor>>) {
    for joint in query.iter() {
        if joint.aligned_axis.length() != 1.0 {
            error!("Aligned axis must be normalized for motor to work")
        }
    }
}

fn multi_target_warning(
    query: Query<
        (&AngularVelocityTarget, &TargetRotation),
        Or<(Added<AngularVelocityTarget>, Added<TargetRotation>)>,
    >,
) {
    for _ in query.iter() {
        warn!("Multiple motor targets detected. Only one target per motor is supported.");
    }
}

fn disable_damping(mut query: Query<&mut RevoluteJoint, Added<RevoluteMotor>>) {
    for mut joint in query.iter_mut() {
        joint.damping_linear = 0.0;
        joint.damping_angular = 0.0;
    }
}

fn enforce_velocity_constraints(
    body_query: Query<&RigidBody>,
    max_velocity_query: Query<(&RevoluteJoint, &MotorMaxAngularVelocity)>,
    mut param_set: ParamSet<(
        Query<&AngularVelocity, With<RigidBody>>,
        Query<&mut AngularVelocity, With<RigidBody>>,
    )>,
) {
    for (joint, max_angular_velocity) in max_velocity_query.iter() {
        if let Some((entity1, entity2)) = get_entity_pair(joint, &body_query) {
            if let Some(relative_velocity) =
                get_relative_angular_velocity(entity1, entity2, &param_set.p0())
            {
                if let Some(max_angular_velocity) = max_angular_velocity.0 {
                    if relative_velocity.length() > max_angular_velocity {
                        let velocity_factor = max_angular_velocity / relative_velocity.length();

                        debug!(
                            "Clamping angular velocity to {:.4} from {:.4}",
                            max_angular_velocity,
                            relative_velocity.length()
                        );

                        if let Ok(mut entity1_velocity) = param_set.p1().get_mut(entity1) {
                            entity1_velocity.0 *= velocity_factor;
                        }
                        if let Ok(mut entity2_velocity) = param_set.p1().get_mut(entity2) {
                            entity2_velocity.0 *= velocity_factor;
                        }
                    }
                }
            }
        }
    }
}

fn apply_velocity_based_torque(
    mut motor_query: Query<
        (
            &RevoluteJoint,
            &AngularVelocityTarget,
            &MotorProportionalGain,
            &MotorDerivativeGain,
            &MotorIntegralGain,
            &MotorMaxTorque,
            &MotorAngularVelocity,
        ),
        Without<TargetRotation>,
    >,
    body_query: Query<&RigidBody>,
    mut torque_query: Query<&mut ExternalTorque, With<RigidBody>>,
    mut integral_accum: Local<Scalar>,
    mut prev_velocity_error: Local<Option<Scalar>>,
    time: Res<Time>,
) {
    for (
        joint,
        target_velocity,
        proportional_gain,
        derivative_gain,
        integral_gain,
        max_torque,
        relative_velocity,
    ) in motor_query.iter_mut()
    {
        if let Some((entity1, entity2)) = get_entity_pair(joint, &body_query) {
            let velocity_error = target_velocity.0 - relative_velocity.0;

            let dt = time.delta_seconds() as Scalar;

            *integral_accum += velocity_error * dt;
            if let Some(max_torque_value) = max_torque.0 {
                let max_integral = max_torque_value / (integral_gain.0 + Scalar::EPSILON);
                *integral_accum = integral_accum.clamp(-max_integral, max_integral);
            }

            let proportional = proportional_gain.0 * velocity_error;
            let integral = integral_gain.0 * *integral_accum;

            let derivative = if let Some(prev_error) = *prev_velocity_error {
                derivative_gain.0 * (velocity_error - prev_error) / dt
            } else {
                0.0
            };

            *prev_velocity_error = Some(velocity_error);

            let mut torque_magnitude = proportional + integral - derivative;

            if let Some(max_torque_value) = max_torque.0 {
                if torque_magnitude > max_torque_value {
                    torque_magnitude = max_torque_value;
                }
            }

            let torque = joint.aligned_axis * torque_magnitude;

            debug!("Torque: {:.4}", torque);

            if let Ok(mut entity1_torque) = torque_query.get_mut(entity1) {
                entity1_torque.set_torque(-torque);
            }

            if let Ok(mut entity2_torque) = torque_query.get_mut(entity2) {
                entity2_torque.set_torque(torque);
            }
        }
    }
}

fn apply_rotation_based_torque(
    mut motor_query: Query<
        (
            &RevoluteJoint,
            &TargetRotation,
            &MotorTotalRotation,
            &MotorProportionalGain,
            &MotorDerivativeGain,
            &MotorIntegralGain,
            &MotorMaxTorque,
        ),
        Without<AngularVelocityTarget>,
    >,
    mut torque_query: Query<&mut ExternalTorque, With<RigidBody>>,
    body_query: Query<&RigidBody>,
    mut integral_accum: Local<Scalar>,
    mut prev_error: Local<Option<Scalar>>,
    time: Res<Time>,
) {
    for (
        joint,
        target_rotation,
        current_rotation,
        proportional_gain,
        derivative_gain,
        integral_gain,
        max_torque,
    ) in motor_query.iter_mut()
    {
        let position_error = target_rotation.0 - current_rotation.0;

        let dt = time.delta_seconds() as Scalar;

        *integral_accum += position_error * dt;
        if let Some(max_torque_value) = max_torque.0 {
            let max_integral = max_torque_value / (integral_gain.0 + Scalar::EPSILON);
            *integral_accum = integral_accum.clamp(-max_integral, max_integral);
        }

        let derivative = if let Some(prev_err) = *prev_error {
            (prev_err - position_error) / dt
        } else {
            0.0
        };

        *prev_error = Some(position_error);

        let proportional = proportional_gain.0 * position_error;
        let integral = integral_gain.0 * *integral_accum;
        let derivative = derivative_gain.0 * derivative;

        let mut torque_magnitude = proportional + integral - derivative;

        if let Some(max_torque_value) = max_torque.0 {
            if torque_magnitude > max_torque_value {
                torque_magnitude = max_torque_value;
            }
        }

        let torque = joint.aligned_axis * torque_magnitude;

        debug!("Torque: {:.4}", torque);

        if let Some((entity1, entity2)) = get_entity_pair(joint, &body_query) {
            if let Ok(mut entity1_torque) = torque_query.get_mut(entity1) {
                entity1_torque.set_torque(-torque);
            }
            if let Ok(mut entity2_torque) = torque_query.get_mut(entity2) {
                entity2_torque.set_torque(torque);
            }
        }
    }
}

fn update_motor_rotation_state(
    mut query: Query<(
        &RevoluteJoint,
        &mut MotorTotalRotation,
        &MotorAngularVelocity,
    )>,
    body_query: Query<&RigidBody>,
    rotation_query: Query<&Rotation, With<RigidBody>>,
    time: Res<Time>,
) {
    for (joint, mut rotation, initial_velocity) in query.iter_mut() {
        if let Some((entity1, entity2)) = get_entity_pair(joint, &body_query) {
            let delta_time = time.delta_seconds() as Scalar;
            let midpoint_velocity = initial_velocity.0 + (0.5 * delta_time * initial_velocity.0);
            let delta_position = midpoint_velocity * delta_time;
            rotation.0 += delta_position;

            if let Some(relative_rotation) =
                get_relative_rotation(entity1, entity2, &rotation_query)
            {
                let rotation_error = rotation.0 - relative_rotation.dot(joint.aligned_axis);
                let rotation_error = (rotation_error + PI) % (2.0 * PI) - PI;
                if rotation_error.abs() < PI {
                    // Avoid singularity
                    rotation.0 -= rotation_error;
                }
            }
        }
    }
}

fn update_motor_angular_velocity_state(
    mut query: Query<(&RevoluteJoint, &mut MotorAngularVelocity)>,
    body_query: Query<&RigidBody>,
    velocity_query: Query<&AngularVelocity, With<RigidBody>>,
) {
    for (joint, mut angular_velocity) in query.iter_mut() {
        if let Some((entity1, entity2)) = get_entity_pair(joint, &body_query) {
            if let Some(relative_velocity) =
                get_relative_angular_velocity(entity1, entity2, &velocity_query)
            {
                angular_velocity.0 = relative_velocity.dot(joint.aligned_axis);
            }
        }
    }
}

pub fn get_entity_pair(
    joint: &RevoluteJoint,
    body_query: &Query<&RigidBody>,
) -> Option<(Entity, Entity)> {
    let (entity1, entity2) = (joint.entity1, joint.entity2);
    if body_query.get(entity1).is_ok() && body_query.get(entity2).is_ok() {
        Some((entity1, entity2))
    } else {
        None
    }
}

pub fn get_relative_angular_velocity(
    entity1: Entity,
    entity2: Entity,
    velocity_query: &Query<&AngularVelocity, With<RigidBody>>,
) -> Option<Vector3> {
    let entity1_velocity = velocity_query.get(entity1).ok()?;
    let entity2_velocity = velocity_query.get(entity2).ok()?;

    Some(entity2_velocity.0 - entity1_velocity.0)
}

pub fn get_relative_rotation(
    entity1: Entity,
    entity2: Entity,
    rotation_query: &Query<&Rotation, With<RigidBody>>,
) -> Option<Vector3> {
    let entity1_rotation = rotation_query.get(entity1).ok()?;
    let entity2_rotation = rotation_query.get(entity2).ok()?;

    let relative_rotation = entity2_rotation.0 * entity1_rotation.0.inverse();
    Some(relative_rotation.to_scaled_axis())
}
