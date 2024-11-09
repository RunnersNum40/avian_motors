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
pub struct TargetVelocity(pub Vector3);

#[derive(Component, Debug, Clone)]
pub struct TargetRotation(pub Vector3);

#[derive(Component, Debug, Clone)]
pub struct MotorStiffness(pub Scalar);

#[derive(Component, Debug, Clone)]
pub struct MotorDamping(pub Scalar);

#[derive(Component, Debug, Clone)]
pub struct MotorIntegralGain(pub Scalar);

#[derive(Component, Debug, Clone)]
pub struct MotorMaxTorque(pub Option<Scalar>);

#[derive(Component, Debug, Clone)]
pub struct MotorMaxAngularVelocity(pub Option<Scalar>);

#[derive(Component, Debug, Clone)]
pub struct MotorRotation(pub Vector3);

#[derive(Bundle, Debug, Clone)]
pub struct MotorBundle {
    pub stiffness: MotorStiffness,
    pub damping: MotorDamping,
    pub integral_gain: MotorIntegralGain,
    pub max_torque: MotorMaxTorque,
    pub max_angular_velocity: MotorMaxAngularVelocity,
    pub position: MotorRotation,
}

impl Default for MotorBundle {
    fn default() -> Self {
        Self {
            stiffness: MotorStiffness(0.0000001),
            damping: MotorDamping(0.0),
            integral_gain: MotorIntegralGain(0.0),
            max_torque: MotorMaxTorque(None),
            max_angular_velocity: MotorMaxAngularVelocity(None),
            position: MotorRotation(Vector3::ZERO),
        }
    }
}

pub struct MotorPlugin {
    pub remove_dampening: bool,
}

impl Plugin for MotorPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(SubstepCount(1000))
            .add_systems(FixedUpdate, enforce_velocity_limits)
            .add_systems(FixedUpdate, update_motor_rotation)
            .add_systems(FixedUpdate, apply_motor_torque_velocity)
            .add_systems(FixedUpdate, apply_motor_torque_rotation);

        if self.remove_dampening {
            app.add_systems(FixedUpdate, remove_dampening);
        }
    }
}

impl Default for MotorPlugin {
    fn default() -> Self {
        Self {
            remove_dampening: true,
        }
    }
}

fn remove_dampening(mut query: Query<&mut RevoluteJoint, Added<RevoluteJoint>>) {
    for mut joint in query.iter_mut() {
        joint.damping_linear = 0.0;
        joint.damping_angular = 0.0;
    }
}

fn get_relative_rotation(
    entity1: Entity,
    entity2: Entity,
    rotation_query: &Query<&Rotation, With<RigidBody>>,
) -> Option<Vector3> {
    let anchor_rotation = rotation_query.get(entity1).ok()?;
    let body_rotation = rotation_query.get(entity2).ok()?;

    let relative_rotation = body_rotation.0 * anchor_rotation.0.inverse();
    Some(relative_rotation.to_scaled_axis())
}

fn enforce_velocity_limits(
    body_query: Query<&RigidBody>,
    max_velocity_query: Query<(&RevoluteJoint, &MotorMaxAngularVelocity)>,
    mut param_set: ParamSet<(
        Query<&AngularVelocity, With<RigidBody>>,
        Query<&mut AngularVelocity, With<RigidBody>>,
    )>,
) {
    for (joint, max_angular_velocity) in max_velocity_query.iter() {
        if let Some((anchor_entity, body_entity)) = get_entity_pair(joint, &body_query) {
            if let Some(relative_velocity) =
                get_relative_angular_velocity(anchor_entity, body_entity, &param_set.p0())
            {
                if let Some(max_angular_velocity) = max_angular_velocity.0 {
                    if relative_velocity.length() > max_angular_velocity {
                        let velocity_factor = max_angular_velocity / relative_velocity.length();
                        if let Ok(mut body_velocity) = param_set.p1().get_mut(body_entity) {
                            body_velocity.0 *= velocity_factor;
                        }
                        if let Ok(mut anchor_velocity) = param_set.p1().get_mut(anchor_entity) {
                            anchor_velocity.0 *= velocity_factor;
                        }
                    }
                }
            }
        }
    }
}

fn apply_motor_torque_velocity(
    mut motor_query: Query<(
        &RevoluteJoint,
        &TargetVelocity,
        &MotorStiffness,
        &MotorDamping,
        &MotorIntegralGain,
        &MotorMaxTorque,
    )>,
    body_query: Query<&RigidBody>,
    velocity_query: Query<&AngularVelocity, With<RigidBody>>,
    mut torque_query: Query<&mut ExternalTorque, With<RigidBody>>,
    mut integral_accum: Local<Vector3>,
    mut prev_velocity_error: Local<Option<Vector3>>,
    time: Res<Time>,
) {
    for (joint, target_velocity, stiffness, damping, integral_gain, max_torque) in
        motor_query.iter_mut()
    {
        if let Some((anchor_entity, body_entity)) = get_entity_pair(joint, &body_query) {
            if let Some(relative_velocity) =
                get_relative_angular_velocity(anchor_entity, body_entity, &velocity_query)
            {
                let velocity_error = target_velocity.0 - relative_velocity;

                let dt = time.delta_seconds() as Scalar;

                *integral_accum += velocity_error * dt;
                if let Some(max_torque_value) = max_torque.0 {
                    let max_integral = max_torque_value / (integral_gain.0 + Scalar::EPSILON);
                    *integral_accum = integral_accum.clamp_length_max(max_integral);
                }

                let proportional = stiffness.0 * velocity_error;
                let integral = integral_gain.0 * *integral_accum;

                let derivative = if let Some(prev_error) = *prev_velocity_error {
                    damping.0 * (velocity_error - prev_error) / dt
                } else {
                    Vector3::ZERO
                };

                *prev_velocity_error = Some(velocity_error);

                let mut torque = proportional + integral - derivative;

                if let Some(max_torque_value) = max_torque.0 {
                    if torque.length() > max_torque_value {
                        torque = torque.normalize() * max_torque_value;
                    }
                }

                if let Ok(mut body_torque) = torque_query.get_mut(body_entity) {
                    body_torque.set_torque(torque);
                }

                if let Ok(mut anchor_torque) = torque_query.get_mut(anchor_entity) {
                    anchor_torque.set_torque(-torque);
                }
            }
        }
    }
}

fn apply_motor_torque_rotation(
    mut motor_query: Query<(
        &RevoluteJoint,
        &TargetRotation,
        &MotorRotation,
        &MotorStiffness,
        &MotorDamping,
        &MotorIntegralGain,
        &MotorMaxTorque,
    )>,
    mut torque_query: Query<&mut ExternalTorque, With<RigidBody>>,
    body_query: Query<&RigidBody>,
    mut integral_accum: Local<Vector3>,
    mut prev_error: Local<Option<Vector3>>,
    time: Res<Time>,
) {
    for (
        joint,
        target_rotation,
        current_rotation,
        stiffness,     // Kp
        damping,       // Kd
        integral_gain, // Ki
        max_torque,
    ) in motor_query.iter_mut()
    {
        let position_error = target_rotation.0 - current_rotation.0;

        let dt = time.delta_seconds() as Scalar;

        *integral_accum += position_error * dt;
        if let Some(max_torque_value) = max_torque.0 {
            let max_integral = max_torque_value / (integral_gain.0 + Scalar::EPSILON);
            *integral_accum = integral_accum.clamp_length_max(max_integral);
        }

        let derivative = if let Some(prev_err) = *prev_error {
            (prev_err - position_error) / dt
        } else {
            Vector3::ZERO
        };

        *prev_error = Some(position_error);

        let proportional = stiffness.0 * position_error;
        let integral = integral_gain.0 * *integral_accum;
        let derivative = damping.0 * derivative;

        let mut torque = proportional + integral - derivative;

        if let Some(max_torque_value) = max_torque.0 {
            if torque.length() > max_torque_value {
                torque = torque.normalize() * max_torque_value;
            }
        }

        if let Some((anchor_entity, body_entity)) = get_entity_pair(joint, &body_query) {
            if let Ok(mut body_torque) = torque_query.get_mut(body_entity) {
                body_torque.set_torque(torque);
            }
            if let Ok(mut anchor_torque) = torque_query.get_mut(anchor_entity) {
                anchor_torque.set_torque(-torque);
            }
        }
    }
}

fn update_motor_rotation(
    mut query: Query<(&RevoluteJoint, &mut MotorRotation)>,
    body_query: Query<&RigidBody>,
    rotation_query: Query<&Rotation, With<RigidBody>>,
    velocity_query: Query<&AngularVelocity, With<RigidBody>>,
    time: Res<Time>,
) {
    for (joint, mut rotation) in query.iter_mut() {
        if let Some((anchor_entity, body_entity)) = get_entity_pair(joint, &body_query) {
            if let Some(initial_velocity) =
                get_relative_angular_velocity(anchor_entity, body_entity, &velocity_query)
            {
                let delta_time = time.delta_seconds() as Scalar;
                let midpoint_velocity = initial_velocity + (0.5 * delta_time * initial_velocity);
                let delta_position = midpoint_velocity * delta_time;
                rotation.0 += delta_position;
            }

            if let Some(relative_rotation) =
                get_relative_rotation(anchor_entity, body_entity, &rotation_query)
            {
                let rotation_error = rotation.0 - relative_rotation;
                let rotation_error = (rotation_error + PI) % (2.0 * PI) - PI;
                if rotation_error.length() < PI {
                    // Avoid singularity
                    rotation.0 -= rotation_error;
                }
            }
        }
    }
}

pub fn get_entity_pair(
    joint: &RevoluteJoint,
    body_query: &Query<&RigidBody>,
) -> Option<(Entity, Entity)> {
    let (anchor_entity, body_entity) = (joint.entity1, joint.entity2);
    if body_query.get(anchor_entity).is_ok() && body_query.get(body_entity).is_ok() {
        Some((anchor_entity, body_entity))
    } else {
        None
    }
}

pub fn get_relative_angular_velocity(
    entity1: Entity,
    entity2: Entity,
    velocity_query: &Query<&AngularVelocity, With<RigidBody>>,
) -> Option<Vector3> {
    let anchor_velocity = match velocity_query.get(entity1) {
        Ok(anchor_velocity) => anchor_velocity.0,
        Err(_) => return None,
    };

    if let Ok(body_velocity) = velocity_query.get(entity2) {
        Some(body_velocity.0 - anchor_velocity)
    } else {
        None
    }
}
