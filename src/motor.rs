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
pub struct AngularVelocityTarget(pub Vector3);

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
pub struct MotorTotalRotation(pub Vector3);

#[derive(Bundle, Debug, Clone)]
pub struct RevoluteMotorBundle {
    pub stiffness: MotorStiffness,
    pub damping: MotorDamping,
    pub integral_gain: MotorIntegralGain,
    pub max_torque: MotorMaxTorque,
    pub max_angular_velocity: MotorMaxAngularVelocity,
    pub total_rotation: MotorTotalRotation,
}

impl Default for RevoluteMotorBundle {
    fn default() -> Self {
        Self {
            stiffness: MotorStiffness(0.0000001),
            damping: MotorDamping(0.0),
            integral_gain: MotorIntegralGain(0.0),
            max_torque: MotorMaxTorque(None),
            max_angular_velocity: MotorMaxAngularVelocity(None),
            total_rotation: MotorTotalRotation(Vector3::ZERO),
        }
    }
}

pub struct MotorPlugin {
    pub remove_dampening: bool,
    pub substep_count: Option<u32>,
}

impl Plugin for MotorPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(FixedUpdate, multi_target_warning)
            .add_systems(FixedUpdate, enforce_velocity_constraints)
            .add_systems(FixedUpdate, update_motor_rotation_state)
            .add_systems(FixedUpdate, apply_velocity_based_torque)
            .add_systems(FixedUpdate, apply_rotation_based_torque);

        if self.remove_dampening {
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
            remove_dampening: true,
            substep_count: Some(1000),
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

fn disable_damping(mut query: Query<&mut RevoluteJoint, Added<RevoluteJoint>>) {
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
            &MotorStiffness,
            &MotorDamping,
            &MotorIntegralGain,
            &MotorMaxTorque,
        ),
        Without<TargetRotation>,
    >,
    body_query: Query<&RigidBody>,
    velocity_query: Query<&AngularVelocity, With<RigidBody>>,
    mut torque_query: Query<&mut ExternalTorque, With<RigidBody>>,
    mut integral_accum: Local<Vector3>,
    mut prev_velocity_error: Local<Option<Vector3>>,
    time: Res<Time>,
) {
    for (joint, target_velocity, proportional_gain, derivative_gain, integral_gain, max_torque) in
        motor_query.iter_mut()
    {
        if let Some((entity1, entity2)) = get_entity_pair(joint, &body_query) {
            if let Some(relative_velocity) =
                get_relative_angular_velocity(entity1, entity2, &velocity_query)
            {
                let velocity_error = target_velocity.0 - relative_velocity;

                let dt = time.delta_seconds() as Scalar;

                *integral_accum += velocity_error * dt;
                if let Some(max_torque_value) = max_torque.0 {
                    let max_integral = max_torque_value / (integral_gain.0 + Scalar::EPSILON);
                    *integral_accum = integral_accum.clamp_length_max(max_integral);
                }

                let proportional = proportional_gain.0 * velocity_error;
                let integral = integral_gain.0 * *integral_accum;

                let derivative = if let Some(prev_error) = *prev_velocity_error {
                    derivative_gain.0 * (velocity_error - prev_error) / dt
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

                if let Ok(mut entity1_torque) = torque_query.get_mut(entity1) {
                    entity1_torque.set_torque(-torque);
                }

                if let Ok(mut entity2_torque) = torque_query.get_mut(entity2) {
                    entity2_torque.set_torque(torque);
                }
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
            &MotorStiffness,
            &MotorDamping,
            &MotorIntegralGain,
            &MotorMaxTorque,
        ),
        Without<AngularVelocityTarget>,
    >,
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
            *integral_accum = integral_accum.clamp_length_max(max_integral);
        }

        let derivative = if let Some(prev_err) = *prev_error {
            (prev_err - position_error) / dt
        } else {
            Vector3::ZERO
        };

        *prev_error = Some(position_error);

        let proportional = proportional_gain.0 * position_error;
        let integral = integral_gain.0 * *integral_accum;
        let derivative = derivative_gain.0 * derivative;

        let mut torque = proportional + integral - derivative;

        if let Some(max_torque_value) = max_torque.0 {
            if torque.length() > max_torque_value {
                torque = torque.normalize() * max_torque_value;
            }
        }

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
    mut query: Query<(&RevoluteJoint, &mut MotorTotalRotation)>,
    body_query: Query<&RigidBody>,
    rotation_query: Query<&Rotation, With<RigidBody>>,
    velocity_query: Query<&AngularVelocity, With<RigidBody>>,
    time: Res<Time>,
) {
    for (joint, mut rotation) in query.iter_mut() {
        if let Some((entity1, entity2)) = get_entity_pair(joint, &body_query) {
            if let Some(initial_velocity) =
                get_relative_angular_velocity(entity1, entity2, &velocity_query)
            {
                let delta_time = time.delta_seconds() as Scalar;
                let midpoint_velocity = initial_velocity + (0.5 * delta_time * initial_velocity);
                let delta_position = midpoint_velocity * delta_time;
                rotation.0 += delta_position;
            }

            if let Some(relative_rotation) =
                get_relative_rotation(entity1, entity2, &rotation_query)
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
    let anchor_velocity = match velocity_query.get(entity1) {
        Ok(entity1_velocity) => entity1_velocity.0,
        Err(_) => return None,
    };

    if let Ok(entity2_velocity) = velocity_query.get(entity2) {
        Some(entity2_velocity.0 - anchor_velocity)
    } else {
        None
    }
}
