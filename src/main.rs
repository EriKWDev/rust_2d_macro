use std::collections::HashMap;

use bitsets::{BitSet, Flag};
use macroquad::prelude::*;
use rapier2d::prelude::*;

use slotmap::{new_key_type, DenseSlotMap, SecondaryMap, SparseSecondaryMap};

pub mod bitsets;
pub mod utils;

new_key_type! {
    pub struct Entity;
}

#[rustfmt::skip]
pub mod components {
    use crate::bitsets::{BitSet, Flag};

    pub type Query = BitSet;

    pub const TEXTURE: Flag         = 1 << 0;
    pub const RIGIDBODY: Flag       = 1 << 1;
    pub const COLLIDER: Flag        = 1 << 2;
    pub const FIXED_COLLIDER: Flag  = 1 << 3;
    pub const PLAYER: Flag          = 1 << 4;

    pub const NUM_COMPONENTS: usize =      5;

    pub fn every_component() -> impl Iterator<Item=Flag> + 'static  {
        (0..NUM_COMPONENTS).into_iter().map(|i| 1 << i)
    }
}

use components::Query;

struct TextureComponent {
    texture: Texture2D,
    size: Vec2,
    color: Color,
}

struct RigidbodyComponent {
    rigidbody_handle: RigidBodyHandle,
}

struct ColliderComponent {
    collider_handle: ColliderHandle,
}

#[derive(Default)]
struct PlayerComponent {}

#[derive(PartialEq, Eq, Hash)]
enum Actions {
    QuitImmediately,

    MoveRight,
    MoveLeft,
    MoveUp,
    MoveDown,
}

pub mod constants {
    pub const MAX_ENTITIES: usize = 1_000;

    pub const SINGLE_COMPONENT: usize = 1;
    pub const BARELY_ANY_COMPONENTS: usize = 4;
    pub const NOT_SO_MANY_COMPONENTS: usize = 100;
    pub const MANY_COMPONENTS: usize = MAX_ENTITIES;

    pub const GOAL_DELTA_TIME: f64 = 1.0 / 60.0;
}

use constants::*;

use crate::utils::lerp;

type EntityMap = DenseSlotMap<Entity, BitSet>;
type SparseComponentMap<T> = SparseSecondaryMap<Entity, T>;
type DenseComponentMap<T> = SecondaryMap<Entity, T>;

struct Game {
    // Ecs
    entities: EntityMap,

    label_container: DenseComponentMap<&'static str>,

    texture_container: SparseComponentMap<TextureComponent>,
    rigidbody_container: DenseComponentMap<RigidbodyComponent>,
    collider_container: DenseComponentMap<ColliderComponent>,

    player_container: DenseComponentMap<PlayerComponent>,

    // Other
    zoom: f32,
    camera: Camera2D,

    keys: HashMap<Actions, KeyCode>,

    // Physics
    gravity: nalgebra::Vector2<f32>,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    physics_hooks: (),
    event_handler: (),
}

impl Default for Game {
    fn default() -> Self {
        let rigid_body_set = RigidBodySet::new();
        let collider_set = ColliderSet::new();

        let integration_parameters = IntegrationParameters::default();
        let physics_pipeline = PhysicsPipeline::new();
        let island_manager = IslandManager::new();
        let broad_phase = BroadPhase::new();
        let narrow_phase = NarrowPhase::new();
        let impulse_joint_set = ImpulseJointSet::new();
        let multibody_joint_set = MultibodyJointSet::new();
        let ccd_solver = CCDSolver::new();

        const ZOOM: f32 = -0.002;

        Self {
            // Ecs
            entities: EntityMap::with_capacity_and_key(MAX_ENTITIES),

            label_container: DenseComponentMap::with_capacity(MANY_COMPONENTS),

            texture_container: SparseComponentMap::with_capacity(NOT_SO_MANY_COMPONENTS),
            rigidbody_container: DenseComponentMap::with_capacity(MANY_COMPONENTS),
            collider_container: DenseComponentMap::with_capacity(MANY_COMPONENTS),

            player_container: DenseComponentMap::with_capacity(SINGLE_COMPONENT),

            // Other
            zoom: ZOOM,
            camera: Camera2D {
                zoom: vec2(ZOOM, ZOOM * screen_width() / screen_height()),

                target: vec2(500.0, 500.0),
                offset: vec2(0.0, 0.0),

                ..Default::default()
            },

            keys: HashMap::from([
                (Actions::QuitImmediately, KeyCode::Escape),
                (Actions::MoveRight, KeyCode::D),
                (Actions::MoveLeft, KeyCode::A),
                (Actions::MoveUp, KeyCode::W),
                (Actions::MoveDown, KeyCode::S),
            ]),

            // Physics
            gravity: vector![0.0, 569.1337],

            rigid_body_set,
            collider_set,
            integration_parameters,
            physics_pipeline,
            island_manager,
            broad_phase,
            narrow_phase,
            impulse_joint_set,
            multibody_joint_set,
            ccd_solver,
            physics_hooks: (),
            event_handler: (),
        }
    }
}

impl Game {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self::default()
    }
}

// Ecs Api
impl Game {
    pub fn new_entity(&mut self, label: &'static str) -> Entity {
        let entity = self.entities.insert(BitSet::empty());

        self.label_container.insert(entity, label);

        entity
    }

    pub fn remove_entity(&mut self, entity: Entity) {
        self.entities.remove(entity);
    }

    pub fn add_flag(&mut self, entity: Entity, flag: Flag) {
        self.entities.get_mut(entity).unwrap().include_flag(flag);
    }

    pub fn remove_flag(&mut self, entity: Entity, flag: Flag) {
        self.entities.get_mut(entity).unwrap().exclude_flag(flag);
    }

    #[inline]
    pub fn add_texture(&mut self, entity: Entity, component: TextureComponent) {
        self.texture_container.insert(entity, component);
        self.add_flag(entity, components::TEXTURE);
    }

    #[inline]
    pub fn add_player_component(&mut self, entity: Entity, component: PlayerComponent) {
        self.player_container.insert(entity, component);
        self.add_flag(entity, components::PLAYER);
    }

    pub fn add_physics(&mut self, entity: Entity, rigid_body: RigidBody, collider: Collider) {
        let rigidbody_handle = self.rigid_body_set.insert(rigid_body);

        let collider_handle = self.collider_set.insert_with_parent(
            collider,
            rigidbody_handle,
            &mut self.rigid_body_set,
        );

        self.rigidbody_container
            .insert(entity, RigidbodyComponent { rigidbody_handle });

        self.collider_container
            .insert(entity, ColliderComponent { collider_handle });

        self.add_flag(entity, components::RIGIDBODY);
        self.add_flag(entity, components::COLLIDER);
    }

    #[inline]
    pub fn add_fixed_collider(&mut self, entity: Entity, collider: Collider) {
        let rigid_body = RigidBodyBuilder::fixed().build();
        self.add_physics(entity, rigid_body, collider);

        self.add_flag(entity, components::FIXED_COLLIDER);
    }
}

// Logic Systems
impl Game {
    pub fn player_movement_system(&mut self, delta: f32) {
        let mut force = vector![0.0, 0.0];
        let jump = is_key_pressed(self.keys[&Actions::MoveUp]);

        if is_key_down(self.keys[&Actions::MoveRight]) {
            force.x -= 1.0;
        }

        if is_key_down(self.keys[&Actions::MoveLeft]) {
            force.x += 1.0;
        }

        if is_key_down(self.keys[&Actions::MoveDown]) {
            force.y += 1.0;
        }

        const PLAYER_SPEED: f32 = 10_00.0;
        force = force.try_normalize(0.1).unwrap_or(vector![0.0, 0.0]) * PLAYER_SPEED * delta;

        self.player_container
            .iter()
            .for_each(|(entity, _player_component)| {
                /*
                    SAFETY: We work on the premise that an entity with a PlayerComponent
                            necessarily has a RigidBodyComponent and a ColliderComponent
                */

                let rigidbody = self
                    .rigid_body_set
                    .get_mut(
                        unsafe { self.rigidbody_container.get_unchecked(entity) }.rigidbody_handle,
                    )
                    .unwrap();

                let collider = self
                    .collider_set
                    .get_mut(
                        unsafe { self.collider_container.get_unchecked(entity) }.collider_handle,
                    )
                    .unwrap();

                let linvel = rigidbody.linvel();
                let new_linvel = vector![
                    linvel.x + force.x,
                    if jump { -800.0 } else { linvel.y } + force.y
                ];

                rigidbody.set_linvel(new_linvel, true);

                let is_falling = new_linvel.y > 0.0;

                // collider.set_mass(if is_falling { 300.0 } else { 10.0 });
                //let mass = collider.mass();

                // println!("{mass}");

                let isom = rigidbody.position();
                let pos = isom.translation;

                let t = delta * 5.0;

                self.camera.target.x = lerp(self.camera.target.x, pos.x, t * 2.0);
                self.camera.target.y = lerp(self.camera.target.y, pos.y, t);
            });
    }

    pub fn physics_system(&mut self, delta: f32) {
        self.integration_parameters.dt = delta;

        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &self.physics_hooks,
            &self.event_handler,
        );
    }

    pub fn run_logic_systems(&mut self, delta: f32) {
        self.player_movement_system(delta);
        self.physics_system(delta);

        if is_key_pressed(self.keys[&Actions::QuitImmediately]) {
            std::process::exit(0);
        }
    }
}

// Rendering Systems
impl Game {
    pub fn render_sprites_system(&self) {
        const QUERY: Query = Query::new(components::RIGIDBODY | components::TEXTURE);

        self.entities
            .iter()
            .filter(|(_entity, bitset)| QUERY.is_subset_of(bitset))
            .for_each(|(entity, _bitset)| {
                let tex = unsafe { self.texture_container.get_unchecked(entity) };
                let rigidbody = unsafe {
                    self.rigid_body_set
                        .get(
                            self.rigidbody_container
                                .get_unchecked(entity)
                                .rigidbody_handle,
                        )
                        .unwrap()
                };

                let isom = rigidbody.position();
                let pos = isom.translation;
                let rot = isom.rotation;

                draw_texture_ex(
                    tex.texture,
                    pos.x - tex.size.x / 2.0,
                    pos.y - tex.size.y / 2.0,
                    tex.color,
                    DrawTextureParams {
                        dest_size: Some(tex.size),
                        rotation: rot.angle(),

                        ..Default::default()
                    },
                );
            });
    }

    pub fn render_fixed_colliders(&self) {
        const QUERY: Query = Query::new(components::FIXED_COLLIDER | components::RIGIDBODY);

        self.entities
            .iter()
            .filter(|(_entity, bitset)| QUERY.is_subset_of(bitset))
            .for_each(|(entity, _bitset)| {
                let collider = self
                    .collider_set
                    .get(unsafe { self.collider_container.get_unchecked(entity) }.collider_handle)
                    .unwrap();

                let aabb = collider.compute_aabb();

                let extends = aabb.extents();
                let center = aabb.center();

                draw_rectangle_lines(center.x, center.y, extends.x, extends.y, 0.0, RED);
            });
    }

    #[cfg(feature = "editor")]
    pub fn render_gui_system(&mut self) {
        egui_macroquad::ui(|egui_ctx| {
            let r = 8.0;

            egui_ctx.set_style(egui::Style {
                visuals: egui::Visuals {
                    dark_mode: true,
                    window_rounding: egui::Rounding {
                        nw: r,
                        ne: r,
                        sw: r,
                        se: r,
                    },
                    ..Default::default()
                },

                ..Default::default()
            });

            egui::Window::new("egui ❤ macroquad").show(egui_ctx, |ui| {
                ui.heading("Camera");

                ui.vertical(|ui| {
                    ui.label("pos");
                    ui.horizontal(|ui| {
                        ui.label("x:");
                        ui.add(egui::DragValue::new(&mut self.camera.target.x));

                        ui.label("y:");
                        ui.add(egui::DragValue::new(&mut self.camera.target.y));
                    });

                    ui.label("zoom");
                    ui.horizontal(|ui| {
                        ui.add(egui::Slider::new(&mut self.zoom, -3.0..=3.0));
                    });
                });
            });
        });

        egui_macroquad::draw();
    }

    pub fn run_rendering_systems(&mut self, _delta: f32) {
        clear_background(BLACK);

        self.camera.zoom = vec2(self.zoom, self.zoom * screen_width() / screen_height());
        set_camera(&self.camera);

        self.render_fixed_colliders();
        self.render_sprites_system();

        #[cfg(feature = "editor")]
        {
            set_default_camera();
            self.render_gui_system();
        }
    }
}

pub struct Application {
    game: Game,

    lag: f64,
    prev_time: f64,
}

impl Default for Application {
    fn default() -> Self {
        Self {
            game: Game::new(),

            lag: 0.0,
            prev_time: get_time(),
        }
    }
}

impl Application {
    pub fn new() -> Self {
        let mut result = Self::default();

        let bytes = include_bytes!("../assets/it.png");
        let texture = Texture2D::from_file_with_format(bytes, Some(ImageFormat::Png));

        let ecs = &mut result.game;

        // ground

        let ground_entity = ecs.new_entity("Ground");
        let collider = ColliderBuilder::cuboid(800.0, 10.0)
            .rotation(0.0)
            .translation(vector![500.0, 700.0])
            .build();
        ecs.add_fixed_collider(ground_entity, collider);

        let ground_entity = ecs.new_entity("Ground");
        let collider = ColliderBuilder::cuboid(100.0, 10.0)
            .rotation(0.0)
            .translation(vector![500.0, 500.0])
            .build();
        ecs.add_fixed_collider(ground_entity, collider);

        // entities

        for i in 0..50 {
            for j in 0..30 {
                let entity = ecs.new_entity("Ball");

                let (x, y) = ((i as f32 + 30.0) * 10.0, (j as f32) * 10.0);

                ecs.add_texture(
                    entity,
                    TextureComponent {
                        texture,
                        size: vec2(10.0, 10.0),
                        color: Color::from_rgba(255, 255, 255, 255),
                    },
                );

                ecs.add_physics(
                    entity,
                    RigidBodyBuilder::dynamic()
                        .translation(vector![x, y])
                        .build(),
                    ColliderBuilder::ball(5.0)
                        .restitution(0.8)
                        .mass(1.0)
                        .build(),
                );
            }
        }

        // player

        let player_entity = ecs.new_entity("Player");

        ecs.add_texture(
            player_entity,
            TextureComponent {
                texture,
                size: vec2(20.0, 40.0),
                color: Color::from_rgba(125, 72, 252, 255),
            },
        );

        ecs.add_physics(
            player_entity,
            RigidBodyBuilder::dynamic()
                .translation(vector![500.0, 200.0])
                .linear_damping(0.99)
                .lock_rotations()
                .build(),
            ColliderBuilder::round_cuboid(10.0, 20.0, 3.0)
                .restitution(1.0)
                .friction(0.9)
                .build(),
        );

        ecs.add_player_component(player_entity, PlayerComponent::default());

        result
    }

    pub async fn run(&mut self) {
        loop {
            let time = get_time();
            let delta = time - self.prev_time;

            /*
                NOTE(Erik): Ensure that logic systems run on a fixed delta while
                            still calling the rendering functions as quickly as possible.
            */
            self.lag += delta;
            while self.lag >= GOAL_DELTA_TIME {
                self.game.run_logic_systems(GOAL_DELTA_TIME as f32);
                self.lag -= GOAL_DELTA_TIME;
            }

            self.game.run_rendering_systems(delta as f32);
            self.prev_time = time;

            next_frame().await
        }
    }
}

#[macroquad::main("egui with macroquad")]
async fn main() {
    let mut game = Application::new();
    game.run().await;
}
