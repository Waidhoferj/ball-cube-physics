use bevy::animation::{AnimationTarget, AnimationTargetId, animated_field};
use bevy::math::bounding::Aabb3d;
use bevy::prelude::*;
use bevy::window::PresentMode;
use bevy::{
    core_pipeline::tonemapping::Tonemapping, ecs::resource::Resource, post_process::bloom::Bloom,
    reflect::Reflect,
};
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::quick::ResourceInspectorPlugin;
use bevy_inspector_egui::{DefaultInspectorConfigPlugin, InspectorOptions, prelude::*};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};

#[derive(Reflect, Default, Clone, Copy)]
enum AttractorDirection {
    Out,
    #[default]
    In,
}

#[derive(Resource, Reflect, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
struct Configuration {
    attractor_direction: AttractorDirection,
    #[inspector(min = 0.01, max = 50.0)]
    attractor_strength: f32,
    #[inspector(min = 1.0, max = 100.0)]
    radius: f32,
    #[inspector(min = 0.01, max = 10.0)]
    speed: f32,
    boundaries: Vec3,
    #[inspector(min = 0.01, max = 1.0)]
    dampening: f32,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            radius: 10.0,
            speed: 5.,
            boundaries: Vec3::splat(100.0),
            dampening: 0.5,
            attractor_strength: 1.,
            attractor_direction: AttractorDirection::In,
        }
    }
}

#[derive(Component)]
struct Ball;

#[derive(Component, Default)]
struct Motion {
    velocity: Vec3,
    acceleration: Vec3,
}

impl Motion {
    fn with_acc(acc: Vec3) -> Self {
        Motion {
            velocity: Vec3::splat(0.0),
            acceleration: acc,
        }
    }
}

// Holds information about the animation we programmatically create.
struct AnimationInfo {
    // The name of the animation target (in this case, the text).
    target_name: Name,
    // The ID of the animation target, derived from the name.
    target_id: AnimationTargetId,
    // The animation graph asset.
    graph: Handle<AnimationGraph>,
    // The index of the node within that graph.
    node_index: AnimationNodeIndex,
}

impl AnimationInfo {
    // Programmatically creates the UI animation.
    fn create(
        animation_target_name: Name,
        animation_curve: impl AnimationCurve,
        animation_graphs: &mut Assets<AnimationGraph>,
        animation_clips: &mut Assets<AnimationClip>,
    ) -> AnimationInfo {
        // Create an ID that identifies the text node we're going to animate.
        let animation_target_id = AnimationTargetId::from_name(&animation_target_name);

        let mut animation_clip = AnimationClip::default();
        animation_clip.add_curve_to_target(animation_target_id, animation_curve);

        // Save our animation clip as an asset.
        let animation_clip_handle = animation_clips.add(animation_clip);

        // Create an animation graph with that clip.
        let (animation_graph, animation_node_index) =
            AnimationGraph::from_clip(animation_clip_handle);
        let animation_graph_handle = animation_graphs.add(animation_graph);

        AnimationInfo {
            target_name: animation_target_name,
            target_id: animation_target_id,
            graph: animation_graph_handle,
            node_index: animation_node_index,
        }
    }
}

#[derive(Resource)]
struct ForceEmitterHandles {
    mesh: Handle<Mesh>,
    attractor_material: Handle<StandardMaterial>,
    repulser_material: Handle<StandardMaterial>,
    animation: AnimationInfo,
}

#[derive(Component)]
struct ForceEmitter {
    strength: f32,
    force_direction: AttractorDirection,
}

impl ForceEmitter {
    pub fn spawn(
        commands: &mut Commands,
        strength: f32,
        pos: Vec3,
        handles: &ForceEmitterHandles,
        direction: AttractorDirection,
    ) {
        let (material, speed) = match direction {
            AttractorDirection::In => (handles.attractor_material.clone(), -1.0),
            AttractorDirection::Out => (handles.repulser_material.clone(), 1.0),
        };
        let mut animation_player = AnimationPlayer::default();
        animation_player
            .play(handles.animation.node_index)
            .set_speed(speed)
            .repeat();
        let entity = commands
            .spawn((
                ForceEmitter {
                    strength,
                    force_direction: direction,
                },
                Transform::from_translation(pos),
                Mesh3d(handles.mesh.clone()),
                MeshMaterial3d(material),
                AnimationGraphHandle(handles.animation.graph.clone()),
                animation_player,
            ))
            .observe(delete_attractor)
            .id();

        commands.entity(entity).insert(children![(
            AnimationTarget {
                id: handles.animation.target_id,
                player: entity,
            },
            handles.animation.target_name.clone(),
            Transform::from_translation(Vec3::splat(0.0)),
            Mesh3d(handles.mesh.clone()),
            MeshMaterial3d(handles.attractor_material.clone()),
        )]);
    }
}

#[derive(Component)]
struct ContainerBox;

fn main() {
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                // This tells Wasm to resize the window according to the available canvas
                fit_canvas_to_parent: true,
                // You might also want to set PresentMode for smoother rendering
                present_mode: PresentMode::AutoVsync,
                ..default()
            }),
            ..default()
        }),
        EguiPlugin::default(),
        DefaultInspectorConfigPlugin,
    ))
    .add_plugins((PanOrbitCameraPlugin, MeshPickingPlugin))
    .init_resource::<Configuration>()
    .register_type::<Configuration>()
    .add_plugins(ResourceInspectorPlugin::<Configuration>::default())
    .add_systems(Startup, setup)
    .add_systems(
        Update,
        (apply_attractor_force, apply_boundaries, cascade_vectors).chain(),
    )
    .add_systems(Update, (adjust_ball_size, adjust_boundary_size));

    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut animations: ResMut<Assets<AnimationClip>>,
    mut graphs: ResMut<Assets<AnimationGraph>>,
    config: Res<Configuration>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(Vec3::new(0.0, 1.5, config.boundaries.z * 1.3)),
        PanOrbitCamera::default(),
        Tonemapping::TonyMcMapface,
        Bloom::default(),
    ));

    // Create Force emitter animation
    let animation_domain = interval(0.0, 1.0).expect("This is a valid hard coded domain.");
    let scale_curve = EasingCurve::new(
        Vec3::splat(0.0),
        Vec3::splat(1.0),
        EaseFunction::QuadraticInOut,
    )
    .reparametrize_linear(animation_domain)
    .expect("This is statically initialized.");
    let curve = AnimatableCurve::new(animated_field!(Transform::scale), scale_curve);
    let animation_info =
        AnimationInfo::create(Name::from("Emitter"), curve, &mut graphs, &mut animations);

    commands.insert_resource(ForceEmitterHandles {
        mesh: meshes.add(Sphere::new(5.0)),
        attractor_material: materials.add(Color::hsla(0.0, 0.0, 0.0, 0.5)),
        repulser_material: materials.add(Color::linear_rgba(1.0, 0.0, 0.0, 0.5)),
        animation: animation_info,
    });

    // Lighting

    commands.spawn((
        PointLight {
            intensity: 10_000_000.,
            range: 100.0,
            shadow_depth_bias: 0.2,
            ..default()
        },
        Transform::from_xyz(0., 0., 0.),
    ));

    commands.spawn(DirectionalLight {
        color: Color::hsl(79., 1., 0.6),
        shadows_enabled: true,
        illuminance: 150.,
        ..default()
    });

    // Ball
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::hsl(140.0, 0.7, 0.7),
            ..Default::default()
        })),
        Motion::with_acc(Vec3::new(0.1, 0.2, 0.12)),
        Transform::from_translation(Vec3::splat(0.0)),
        Ball,
    ));

    // Container
    commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
            Transform::from_translation(Vec3::splat(0.0)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::hsla(140.0, 0.9, 0.9, 0.3),
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            ContainerBox,
        ))
        .observe(place_attractor);
}

fn adjust_ball_size(mut query: Query<&mut Transform, With<Ball>>, config: Res<Configuration>) {
    if !config.is_changed() {
        return;
    }
    for mut transform in query.iter_mut() {
        transform.scale = Vec3::splat(config.radius);
    }
}

fn adjust_boundary_size(
    mut query: Query<&mut Transform, With<ContainerBox>>,
    config: Res<Configuration>,
) {
    if !config.is_changed() {
        return;
    }
    for mut transform in query.iter_mut() {
        transform.scale = config.boundaries;
    }
}

fn cascade_vectors(
    mut query: Query<(&mut Motion, &mut Transform)>,
    time: Res<Time>,
    config: Res<Configuration>,
) {
    for (mut motion, mut transform) in query.iter_mut() {
        let acc = motion.acceleration;
        let clamp_boundary = config.boundaries / 2.0 - config.radius;
        let delta = time.delta().as_secs_f32();
        motion.velocity += acc * delta * config.speed;
        transform.translation += motion.velocity * delta * config.speed;
        transform.translation = transform.translation.clamp(-clamp_boundary, clamp_boundary);
    }
}

struct Plane {
    origin: Vec3,
    norm: Vec3,
}

impl Plane {
    pub fn get_closest_point(&self, p: Vec3) -> Vec3 {
        p - ((p - self.origin).dot(self.norm) / self.norm.length().powf(2.0)) * self.norm
    }

    pub fn from_bounding_box(bounds: &Aabb3d) -> [Plane; 6] {
        return [
            Plane {
                origin: Vec3::new(bounds.min.x, 0.0, 0.0),
                norm: Vec3::X, // Pointing inwards
            },
            Plane {
                origin: Vec3::new(bounds.max.x, 0.0, 0.0),
                norm: Vec3::NEG_X, // Pointing inwards
            },
            Plane {
                origin: Vec3::new(0.0, bounds.min.y, 0.0),
                norm: Vec3::Y, // Pointing inwards
            },
            Plane {
                origin: Vec3::new(0.0, bounds.max.y, 0.0),
                norm: Vec3::NEG_Y, // Pointing inwards
            },
            Plane {
                origin: Vec3::new(0.0, 0.0, bounds.min.z),
                norm: Vec3::Z, // Pointing inwards
            },
            Plane {
                origin: Vec3::new(0.0, 0.0, bounds.max.z),
                norm: Vec3::NEG_Z, // Pointing inwards
            },
        ];
    }
}

fn apply_boundaries(
    mut ball_query: Query<(&mut Motion, &Transform), With<Ball>>,
    container_query: Query<&Transform, With<ContainerBox>>,
    config: Res<Configuration>,
) {
    let Ok(box_transform) = container_query.single() else {
        return;
    };
    let dampen_scalar = 1. - config.dampening;
    let bounds = Aabb3d::new(Vec3::ZERO, box_transform.scale / 2.0);
    for (mut motion, transform) in ball_query.iter_mut() {
        let ball_radius = transform.scale.x;
        // let ball_bounds = BoundingSphere::new(transform.translation, ball_radius).aabb_3d();
        // if bounds.contains(&ball_bounds) {
        //     continue;
        // }
        // For each plane, project the point onto the plane
        let planes = Plane::from_bounding_box(&bounds);
        // Subtract plane point from actual point Find vector with the minimum magnitude. find norm of that plane
        let colliding_planes: Vec<Vec3> = planes
            .into_iter()
            .filter_map(|plane| {
                let ball_pos = transform.translation;
                let closest_point = plane.get_closest_point(ball_pos);
                let diff = (closest_point - ball_pos).length();
                (diff <= ball_radius).then(|| plane.norm)
            })
            .collect();

        if !colliding_planes.is_empty() {
            // Sum all collision normals and normalize to get the average collision normal.
            let combined_norm: Vec3 = colliding_planes.into_iter().sum();
            let plane_norm = combined_norm.normalize_or_zero();

            // Reflect the velocity and acceleration vectors off the plane normal
            motion.velocity = motion.velocity.reflect(plane_norm);
            motion.acceleration = motion.acceleration.reject_from(plane_norm);
            // Apply dampening to simulate energy loss
            motion.velocity *= dampen_scalar;
        }
    }
}

fn delete_attractor(event: On<Pointer<Release>>, mut commands: Commands) {
    commands.entity(event.entity).despawn();
}

fn apply_attractor_force(
    attractors: Query<(&ForceEmitter, &Transform)>,
    mut balls: Query<(&mut Motion, &Transform), With<Ball>>,
) {
    for (attractor, attractor_t) in attractors.iter() {
        for (mut ball_motion, ball_t) in balls.iter_mut() {
            let attractor_direction = match attractor.force_direction {
                AttractorDirection::In => {
                    (attractor_t.translation - ball_t.translation).normalize()
                }
                AttractorDirection::Out => {
                    (ball_t.translation - attractor_t.translation).normalize()
                }
            };
            ball_motion.acceleration += attractor_direction * attractor.strength
        }
    }
}

fn place_attractor(
    event: On<Pointer<Release>>,
    mut commands: Commands,
    handles: Res<ForceEmitterHandles>,
    config: Res<Configuration>,
) {
    if let Some(pos) = event.hit.position {
        ForceEmitter::spawn(
            &mut commands,
            config.attractor_strength,
            pos,
            &handles,
            config.attractor_direction.clone(),
        );
    }
}

#[cfg(test)]
mod tests {
    use bevy::math::{
        Vec3,
        bounding::{Aabb3d, BoundingSphere, BoundingVolume},
    };

    use crate::Plane;

    #[test]
    fn verify_that_bounding_spheres_make_sense() {
        let bounds = Aabb3d::new(Vec3::ZERO, Vec3::splat(10.0) / 2.0);
        let ball_bounds = BoundingSphere::new(Vec3::new(0.0, 4.0, 0.0), 1.0);
        assert!(bounds.contains(&ball_bounds.aabb_3d()));
    }

    #[test]
    fn test_closest_point() {
        let origin = Vec3::new(3., 0., 0.);
        let p = Plane {
            origin: origin.clone(),
            norm: Vec3::ZERO.with_x(-1.),
        };

        assert_eq!(p.get_closest_point(Vec3::ZERO), origin);
        assert_eq!(
            p.get_closest_point(Vec3::new(1., 2., 3.)),
            Vec3::new(3., 2., 3.)
        );

        let origin = Vec3::new(-3., 0., 0.);
        let p = Plane {
            origin: origin.clone(),
            norm: Vec3::ZERO.with_x(-1.),
        };
        assert_eq!(p.get_closest_point(Vec3::ZERO), origin);
    }

    #[test]
    fn test_projection() {
        let acc = Vec3::new(2., 1., 3.);
        let norm = Vec3::new(-1.0, 0.0, 0.0);
        let res = acc.project_onto(norm);
        assert_eq!(res.length(), 2.0);
        assert_eq!(res.x, 2.0);
    }
}
