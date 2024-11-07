use avian3d::prelude::*;
use bevy::{math::DVec3, prelude::*};

#[derive(Component, Debug, Clone)]
pub struct TargetVelocity(pub DVec3);

#[derive(Component, Debug, Clone)]
pub struct MotorStiffness(pub f64);

#[derive(Component, Debug, Clone)]
pub struct MotorDamping(pub f64);

#[derive(Component, Debug, Clone)]
pub struct MotorMaxTorque(pub Option<f64>);

#[derive(Bundle, Debug, Clone)]
pub struct MotorBundle {
    pub target_velocity: TargetVelocity,
    pub stiffness: MotorStiffness,
    pub damping: MotorDamping,
    pub max_torque: MotorMaxTorque,
}

impl Default for MotorBundle {
    fn default() -> Self {
        Self {
            target_velocity: TargetVelocity(DVec3::ZERO),
            stiffness: MotorStiffness(0.0000001),
            damping: MotorDamping(0.0),
            max_torque: MotorMaxTorque(None),
        }
    }
}

pub struct MotorPlugin {
    pub remove_dampening: bool,
}

impl Plugin for MotorPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(SubstepCount(500))
            .add_systems(FixedUpdate, apply_motor_torque);

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
    mut query: Query<(
        &RevoluteJoint,
        &TargetVelocity,
        &MotorStiffness,
        &MotorDamping,
        &MotorMaxTorque,
    )>,
    mut body_query: Query<(&AngularVelocity, &mut ExternalTorque), With<RigidBody>>,
) {
    for (joint, target_velocity, stiffness, damping, max_torque) in query.iter_mut() {
        if let Some((relative_velocity, anchor_entity, body_entity)) =
            get_relative_velocity(joint, &mut body_query)
        {
            let torque = compute_motor_torque(
                target_velocity.0,
                relative_velocity,
                stiffness.0,
                damping.0,
                max_torque.0,
            );
            if let Ok((_, mut body_torque)) = body_query.get_mut(body_entity) {
                body_torque.set_torque(torque);
            }

            if let Ok((_, mut anchor_torque)) = body_query.get_mut(anchor_entity) {
                anchor_torque.set_torque(-torque);
            }
        }
    }
}

pub fn get_relative_velocity(
    joint: &RevoluteJoint,
    body_query: &mut Query<(&AngularVelocity, &mut ExternalTorque), With<RigidBody>>,
) -> Option<(DVec3, Entity, Entity)> {
    let (anchor_entity, body_entity) = (joint.entity1, joint.entity2);

    let anchor_velocity = match body_query.get_mut(anchor_entity) {
        Ok((anchor_velocity, _)) => anchor_velocity.0,
        Err(_) => return None,
    };

    if let Ok((body_velocity, _)) = body_query.get_mut(body_entity) {
        Some((
            body_velocity.0 - anchor_velocity,
            anchor_entity,
            body_entity,
        ))
    } else {
        None
    }
}

fn compute_motor_torque(
    target_velocity: DVec3,
    relative_velocity: DVec3,
    stiffness: f64,
    damping: f64,
    max_torque: Option<f64>,
) -> DVec3 {
    let velocity_error = target_velocity - relative_velocity;
    let mut torque = stiffness * velocity_error - damping * relative_velocity;

    if let Some(max_torque_value) = max_torque {
        if torque.length() > max_torque_value {
            torque = torque.normalize() * max_torque_value;
        }
    }

    torque
}
