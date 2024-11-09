use avian3d::prelude::*;
use avian_motors::motor::{
    AngularVelocityTarget, MotorAngularVelocity, MotorDerivativeGain, MotorIntegralGain,
    MotorMaxAngularVelocity, MotorPlugin, MotorProportionalGain, MotorTotalRotation, RevoluteMotorBundle,
};
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin};

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            EguiPlugin,
            MotorPlugin::default(),
        ))
        .add_systems(Startup, setup)
        .add_systems(Update, ui_controls)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let directional_light = DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 5000.0,
            shadows_enabled: true,
            ..Default::default()
        },
        transform: Transform::from_rotation(Quat::from_rotation_x(
            -std::f32::consts::FRAC_PI_2 * 0.8,
        )),
        ..Default::default()
    };

    commands.spawn(directional_light);

    let camera = (Camera3dBundle {
        transform: Transform::from_xyz(0.3, 0.3, 0.3).looking_at(Vec3::ZERO, Vec3::Y),
        ..Default::default()
    },);

    commands.spawn(camera);

    let offset = Vec3::new(0.0, 0.1, 0.0);

    let anchor_entity = {
        commands
            .spawn((
                RigidBody::Static,
                Transform::from_translation(offset / -2.0),
            ))
            .id()
    };

    let entity1 = {
        commands
            .spawn((
                RigidBody::Dynamic,
                PbrBundle {
                    mesh: meshes.add(Mesh::from(Cuboid::new(0.1, 0.05, 0.1))),
                    material: materials.add(Color::srgb(0.6, 0.5, 0.5)),
                    transform: Transform::from_translation(offset / -2.0),
                    ..Default::default()
                },
                ColliderConstructor::ConvexDecompositionFromMesh,
            ))
            .id()
    };

    let entity2 = {
        commands
            .spawn((
                RigidBody::Dynamic,
                PbrBundle {
                    mesh: meshes.add(Mesh::from(Cuboid::new(0.1, 0.05, 0.1))),
                    material: materials.add(Color::srgb(0.6, 0.5, 0.5)),
                    transform: Transform::from_translation(offset / 2.0),
                    ..Default::default()
                },
                ColliderConstructor::ConvexDecompositionFromMesh,
            ))
            .id()
    };

    let joint1 = RevoluteJoint::new(anchor_entity, entity1)
        .with_compliance(0.0)
        .with_angular_velocity_damping(0.0)
        .with_linear_velocity_damping(0.0)
        .with_aligned_axis(offset.into());

    let joint2 = RevoluteJoint::new(entity1, entity2)
        .with_compliance(0.0)
        .with_aligned_axis(offset.into())
        .with_local_anchor_1(offset.into());

    commands.spawn(joint1);

    commands.spawn((
        joint2,
        AngularVelocityTarget(Vec3::ZERO.into()),
        RevoluteMotorBundle {
            stiffness: MotorProportionalGain(0.00001),
            ..Default::default()
        },
    ));
}

fn ui_controls(
    mut contexts: EguiContexts,
    mut query: Query<(
        &mut AngularVelocityTarget,
        &mut MotorProportionalGain,
        &mut MotorDerivativeGain,
        &mut MotorIntegralGain,
        &mut MotorMaxAngularVelocity,
        &MotorTotalRotation,
        &MotorAngularVelocity,
    )>,
) {
    egui::Window::new("Motor Control").show(contexts.ctx_mut(), |ui| {
        for (
            mut target_angular_velocity,
            mut stiffness,
            mut damping,
            mut integral_gain,
            mut max_angular_velocity,
            rotation,
            velocity,
        ) in query.iter_mut()
        {
            ui.horizontal(|ui| {
                ui.label("Target Velocity Y:");
                ui.add(egui::Slider::new(
                    &mut target_angular_velocity.0.y,
                    -20.0..=20.0,
                ));
            });

            ui.separator();
            ui.label("Motor Parameters:");
            ui.horizontal(|ui| {
                ui.label("Stiffness:");
                ui.add(egui::Slider::new(&mut stiffness.0, 0.0..=0.00002));
            });
            ui.horizontal(|ui| {
                ui.label("Damping:");
                ui.add(egui::Slider::new(&mut damping.0, 0.0..=0.0000002));
            });
            ui.horizontal(|ui| {
                ui.label("Integral Gain:");
                ui.add(egui::Slider::new(&mut integral_gain.0, 0.0..=0.000002));
            });
            if let Some(ref mut max_angular_velocity) = max_angular_velocity.0 {
                ui.horizontal(|ui| {
                    ui.label("Max Angular Velocity:");
                    ui.add(egui::Slider::new(max_angular_velocity, 0.0..=50.0));
                });
            }

            ui.separator();
            ui.label("Current Velocity:");
            ui.horizontal(|ui| {
                ui.label(format!("{:.2}", velocity.0));
            });

            ui.separator();
            ui.label("Motor Position:");
            ui.horizontal(|ui| {
                ui.label(format!("{:.2}", rotation.0));
            });
        }
    });
}
