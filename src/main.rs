// https://people.ece.cornell.edu/land/courses/ece4760/labs/s2021/Boids/Boids.html
// https://vergenet.net/~conrad/boids/pseudocode.html
use rand::prelude::*;
use raylib::{ffi::atan2f, prelude::*};
use rayon::prelude::*;

#[derive(Debug, Default, Clone, Copy, PartialEq)]
struct Boid {
    pos: Vector2,
    vel: Vector2,
}

struct SimulationParameters {
    width: f32,
    height: f32,
    num_boids: usize,
    speed_max: f32,
    speed_min: f32,
    visual_range: f32,
    protected_range: f32,
    view_angle: f32,
    cohesion_factor: f32,
    alignment_factor: f32,
    separation_factor: f32,
    turn_factor: f32,
    show_ranges: bool,
}

fn setup_random_boids(sim_params: &SimulationParameters, rng_obj: &mut ThreadRng) -> Vec<Boid> {
    (0..sim_params.num_boids)
        .map(|_| Boid {
            pos: Vector2::from(rng_obj.gen::<(f32, f32)>()) * (sim_params.width * 0.9)
                + (sim_params.width * 0.05),
            vel: (Vector2::from(rng_obj.gen::<(f32, f32)>()) * sim_params.speed_max)
                - (sim_params.speed_max / 2.),
        })
        .collect()
}

fn population_step(boid_pop: Vec<Boid>, sim_params: &SimulationParameters) -> Vec<Boid> {
    boid_pop
        .par_iter()
        .map(|b1| {
            // Store accumulators for neighbour behaviour
            let mut num_neighbours: f32 = 0.;
            let mut close_boid_position_difference = Vector2::zero();
            let mut neighbour_boid_positions = Vector2::zero();
            let mut neighbour_boid_velocities = Vector2::zero();

            // Iterate over all other boid_pop
            for b2 in &boid_pop {
                if b1 != b2 && b1.vel.dot(b2.vel) < sim_params.view_angle {
                    let distance = f32::powf(
                        f32::powf(b2.pos.x - b1.pos.x, 2.) + f32::powf(b2.pos.y - b1.pos.y, 2.),
                        0.5,
                    );
                    if distance < sim_params.protected_range {
                        close_boid_position_difference += b1.pos - b2.pos;
                    } else if distance < sim_params.visual_range {
                        neighbour_boid_positions += b2.pos;
                        neighbour_boid_velocities += b2.vel;
                        num_neighbours += 1.;
                    }
                }
            }

            let mut new_boid = b1.clone();
            // Include neighbour effects on velocity
            if num_neighbours > 0. {
                let separation_vel = close_boid_position_difference * sim_params.separation_factor;
                let alignment_vel = ((neighbour_boid_velocities / num_neighbours) - b1.vel)
                    * sim_params.alignment_factor;
                let cohesion_vel = ((neighbour_boid_positions / num_neighbours) - b1.pos)
                    * sim_params.cohesion_factor;
                new_boid.vel += separation_vel + alignment_vel + cohesion_vel;
            }

            // Edge-avoidance velocity
            if new_boid.pos.x > sim_params.width * 0.95 {
                new_boid.vel.x -= sim_params.turn_factor;
            }
            if new_boid.pos.x < sim_params.width * 0.05 {
                new_boid.vel.x += sim_params.turn_factor;
            }
            if new_boid.pos.y > sim_params.height * 0.95 {
                new_boid.vel.y -= sim_params.turn_factor;
            }
            if new_boid.pos.y < sim_params.height * 0.05 {
                new_boid.vel.y += sim_params.turn_factor;
            }

            // Speed Check
            let new_speed = new_boid.vel.length();
            if new_speed < sim_params.speed_min {
                new_boid.vel = (new_boid.vel / new_speed) * sim_params.speed_min;
            }
            if new_speed > sim_params.speed_max {
                new_boid.vel = (new_boid.vel / new_speed) * sim_params.speed_max;
            }

            // Update boid states
            new_boid.pos = new_boid.pos + new_boid.vel;

            new_boid
        })
        .collect()
}

fn rotate2d(vec: Vector2, angle: f32) -> Vector2 {
    let mut new_vec = vec;
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    new_vec.x = vec.x * cos_a - vec.y * sin_a;
    new_vec.y = vec.x * sin_a + vec.y * cos_a;
    new_vec
}

fn scale_color(value: f32, params: &SimulationParameters) -> u8 {
    (((value.abs() + params.speed_min) / (params.speed_max * 0.5)) * 255.) as u8
}

fn main() {
    let mut rng: ThreadRng = rand::thread_rng();
    let mut params = SimulationParameters {
        width: 800.,
        height: 800.,
        num_boids: 3000,
        speed_max: 5.,
        speed_min: 1.,
        visual_range: 30.,
        protected_range: 5.,
        view_angle: ((2. * PI) / 3.) as f32,
        cohesion_factor: 0.0005,
        alignment_factor: 0.02,
        separation_factor: 0.01,
        turn_factor: 0.1,
        show_ranges: false,
    };

    // Create boid population
    let mut boids = setup_random_boids(&params, &mut rng);

    // Setup raylib window
    let (mut rl, thread) = raylib::init()
        .size(params.width as i32, params.height as i32)
        .title("Boids Simulation")
        .build();

    // Run display loop
    rl.set_target_fps(60);
    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::BLACK);
        d.draw_fps((params.width - 30.) as i32, 0);

        // Update population
        boids = population_step(boids, &params);

        // Visualise boids
        if params.show_ranges {
            for b in &boids {
                unsafe {
                    let direction = atan2f(b.vel.x, b.vel.y).to_degrees();
                    let angle = params.view_angle.to_degrees();
                    d.draw_circle_sector_lines(
                        b.pos,
                        params.visual_range,
                        direction - angle,
                        direction + angle,
                        0,
                        Color::WHITE,
                    );
                    d.draw_circle_sector_lines(
                        b.pos,
                        params.protected_range,
                        direction - angle,
                        direction + angle,
                        0,
                        Color::GRAY,
                    );
                }
            }
        }
        for b in &boids {
            let vel_color: Color = rcolor(
                scale_color(b.vel.x, &params),
                0,
                scale_color(b.vel.y, &params),
                255,
            );
            let direction_vec = b.vel.normalized();
            let angle = (PI / 3.) as f32;
            d.draw_triangle_lines(
                b.pos + direction_vec * 10.,
                b.pos + rotate2d(direction_vec * 2., angle),
                b.pos + rotate2d(direction_vec * 2., -angle),
                vel_color,
            )
        }

        // User Interface
        let should_quit = d.gui_button(
            rrect(params.width - 175., params.height - 20., 50., 20.),
            Some(rstr!("Quit")),
        );
        if should_quit {
            break;
        };

        let should_reset = d.gui_button(
            rrect(params.width - 125., params.height - 20., 50., 20.),
            Some(rstr!("Reset")),
        );
        if should_reset {
            boids = setup_random_boids(&params, &mut rng);
        }

        params.show_ranges = d.gui_toggle(
            rrect(params.width - 75., params.height - 20., 75., 20.),
            Some(rstr!("Show Ranges")),
            params.show_ranges,
        );
        params.visual_range = d.gui_slider(
            rrect(0., params.height - 20., 200., 20.),
            Some(rstr!("")),
            Some(rstr!("Visual Range")),
            params.visual_range,
            1.,
            100.,
        );
        params.protected_range = d.gui_slider(
            rrect(0., params.height - 40., 200., 20.),
            Some(rstr!("")),
            Some(rstr!("Protected Range")),
            params.protected_range,
            1.,
            100.,
        );
        params.cohesion_factor = d.gui_slider(
            rrect(0., params.height - 60., 200., 20.),
            Some(rstr!("")),
            Some(rstr!("Cohesion")),
            params.cohesion_factor,
            0.0005,
            0.001,
        );
        params.separation_factor = d.gui_slider(
            rrect(0., params.height - 80., 200., 20.),
            Some(rstr!("")),
            Some(rstr!("Separation")),
            params.separation_factor,
            0.01,
            0.1,
        );
        params.alignment_factor = d.gui_slider(
            rrect(0., params.height - 100., 200., 20.),
            Some(rstr!("")),
            Some(rstr!("Alignment")),
            params.alignment_factor,
            0.01,
            0.1,
        );
    }
}
