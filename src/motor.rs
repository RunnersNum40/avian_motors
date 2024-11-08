use avian3d::prelude::*;
use bevy::{math::DVec3, prelude::*};
use std::f64::consts::PI;

#[derive(Component, Debug, Clone)]
pub struct TargetVelocity(pub DVec3);

#[derive(Component, Debug, Clone)]
pub struct MotorStiffness(pub f64);

#[derive(Component, Debug, Clone)]
pub struct MotorDamping(pub f64);

#[derive(Component, Debug, Clone)]
pub struct MotorIntegralGain(pub f64);

#[derive(Component, Debug, Clone)]
pub struct MotorMaxTorque(pub Option<f64>);

#[derive(Component, Debug, Clone)]
pub struct MotorRotation(pub DVec3);

#[derive(Bundle, Debug, Clone)]
pub struct MotorBundle {
    pub target_velocity: TargetVelocity,
    pub stiffness: MotorStiffness,
    pub damping: MotorDamping,
    pub integral_gain: MotorIntegralGain,
    pub max_torque: MotorMaxTorque,
    pub position: MotorRotation,
}

impl Default for MotorBundle {
    fn default() -> Self {
        Self {
            target_velocity: TargetVelocity(DVec3::ZERO),
            stiffness: MotorStiffness(0.0000001),
            damping: MotorDamping(0.0),
            integral_gain: MotorIntegralGain(0.0),
            max_torque: MotorMaxTorque(None),
            position: MotorRotation(DVec3::ZERO),
        }
    }
}

pub struct MotorPlugin {
    pub remove_dampening: bool,
}

impl Plugin for MotorPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(SubstepCount(500))
            .add_systems(FixedUpdate, apply_motor_torque)
            .add_systems(FixedUpdate, update_motor_rotation);

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

fn apply_motor_torque(
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
    mut integral_accum: Local<DVec3>,
    mut prev_velocity_error: Local<Option<DVec3>>,
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

                let dt = time.delta_seconds() as f64;

                *integral_accum += velocity_error * dt;
                if let Some(max_torque_value) = max_torque.0 {
                    let max_integral = max_torque_value / (integral_gain.0 + f64::EPSILON);
                    *integral_accum = integral_accum.clamp_length_max(max_integral);
                }

                let proportional = stiffness.0 * velocity_error;
                let integral = integral_gain.0 * *integral_accum;

                let derivative = if let Some(prev_error) = *prev_velocity_error {
                    damping.0 * (velocity_error - prev_error) / dt
                } else {
                    DVec3::ZERO
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
                let delta_time = time.delta_seconds() as f64;
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
) -> Option<DVec3> {
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

pub fn get_relative_rotation(
    entity1: Entity,
    entity2: Entity,
    rotation_query: &Query<&Rotation, With<RigidBody>>,
) -> Option<DVec3> {
    let anchor_rotation = rotation_query.get(entity1).ok()?;
    let body_rotation = rotation_query.get(entity2).ok()?;

    let relative_rotation = body_rotation.0 * anchor_rotation.0.inverse();
    Some(relative_rotation.to_scaled_axis())
}
