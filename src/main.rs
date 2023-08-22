// https://vergenet.net/~conrad/boids/pseudocode.html
use std::{thread, time};
use raylib::prelude::*;
use rand::prelude::*;

#[derive(Debug, Default, Clone, PartialEq)]
struct Vec2 {
    x: f32,
    y: f32
}
impl Vec2 {
    fn add(self, other_vec: Vec2) -> Self {
        Self { x: self.x + other_vec.x, y: self.y + other_vec.y}
    }
    fn sub(self, other_vec: Vec2) -> Self {
        Self { x: self.x - other_vec.x, y: self.y - other_vec.y}
    }
    fn div(self, value: f32) -> Self {
        Self { x: self.x / value, y: self.y / value}
    }
    fn max_filter(self, max_value: f32) -> Self {
        Self { x: self.x.min(max_value), y: self.y.min(max_value)}
    }
    fn random(rng_obj: &mut ThreadRng, scale_factor: f32, offset: f32) -> Self {
        Self { x: (rng_obj.gen::<f32>() * scale_factor) + offset, y: (rng_obj.gen::<f32>() * scale_factor) + offset } 
    }
    fn modulus(self, mod_val: f32) -> Self {
        Self { x: self.x % mod_val, y: self.y % mod_val }
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Boid {
    pos: Vec2,
    vel: Vec2,
}


fn setup_random_boids(num_boids: usize, rng_obj: &mut ThreadRng) -> Vec<Boid> {
    let mut boids: Vec<Boid> = Vec::with_capacity(num_boids);
    for _ in 0..boids.capacity() {
        boids.push(
            Boid {
                pos: Vec2::random(rng_obj, 600., 0.), 
                vel: Vec2::random(rng_obj, 10., -5.) 
            }
        );
    }
    boids
}

fn population_step(boid_pop: Vec<Boid>, obs_range: f32, vel_limit: f32, cohesion: f32, separation: f32, alignment: f32) -> Vec<Boid> {
    let mut new_boids: Vec<Boid> = boid_pop.clone();

    for (i1, b1) in boid_pop.iter().enumerate() {
        // Store accumulators for neighbour behaviour 
        let mut num_neighbours: f32 = 0.;
        let mut neighbour_centre_of_mass: Vec2 = Vec2::default();
        let mut neighbour_displacements: Vec2 = Vec2::default();
        let mut neighbour_velocities: Vec2 = Vec2::default();

        // Iterate over all other boid_pop 
        for b2 in &boid_pop {
            if b1 != b2 {
                let distance = f32::powf(f32::powf(b2.pos.x - b1.pos.x, 2.) + f32::powf(b2.pos.y - b1.pos.y, 2.), 0.5);
                if distance < obs_range {
                    num_neighbours += 1.;
                    neighbour_centre_of_mass = neighbour_centre_of_mass.add(b2.pos.clone()); // Cohesion
                    neighbour_displacements = neighbour_displacements.sub(b2.pos.clone().sub(b1.pos.clone())); // Separation 
                    neighbour_velocities = neighbour_velocities.add(b2.vel.clone()); // Alignment 
                }
            }
        }
        
        // Update velocity
        if num_neighbours > 0. {
            // let v1: Vec2 = neighbour_centre_of_mass.div(num_neighbours).sub(b1.pos.clone()).div(10.); // Cohesion
            // let v2: Vec2 = neighbour_displacements.div(50.); // Separation 
            // let v3: Vec2 = neighbour_velocities.div(num_neighbours).sub(b1.vel.clone()).div(1.); // Alignment 
            let v1: Vec2 = neighbour_centre_of_mass.div(num_neighbours).sub(b1.pos.clone()).div(cohesion); // Cohesion
            let v2: Vec2 = neighbour_displacements.div(separation); // Separation 
            let v3: Vec2 = neighbour_velocities.div(num_neighbours).sub(b1.vel.clone()).div(alignment); // Alignment 
            new_boids[i1].vel = new_boids[i1].vel.clone().add(v1).add(v2).add(v3);
            new_boids[i1].vel = new_boids[i1].vel.clone().max_filter(vel_limit);
        }
        
        // Update positions
        new_boids[i1].pos = new_boids[i1].pos.clone().add(new_boids[i1].vel.clone());
        new_boids[i1].pos = new_boids[i1].pos.clone().modulus(600.);
    };
    new_boids
}

fn main() {

    let mut rng: ThreadRng = rand::thread_rng();
    const NUM_BOIDS: usize = 1000;
    const VELOCITY_LIMIT: f32 = 10.;

    let mut cohesion_val = 10.;
    let mut separation_val = 50.;
    let mut alignment_val = 1.;
    let mut observation_range = 20.;

    // Create boid population
    let mut boids = setup_random_boids(NUM_BOIDS, &mut rng);
    
    // Setup raylib window
    let (mut rl, thread) = raylib::init()
        .size(600, 600)
        .title("Boids Simulation")
        .build();

    // Run display loop
    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::BLACK);

        boids = population_step(boids, observation_range, VELOCITY_LIMIT, cohesion_val, separation_val, alignment_val);

        for b in &boids {
            d.draw_circle_lines(b.pos.x as i32, b.pos.y as i32, observation_range, Color::SKYBLUE);
        }
        for b in &boids {
            d.draw_circle(b.pos.x as i32, b.pos.y as i32, 5., Color::BLUE);
        }


        // User Interface
        let should_quit = d.gui_button(Rectangle::new(20., 10., 100., 20.), Some(rstr!("Quit")));
        if should_quit {
            break;
        }

        cohesion_val = d.gui_slider(Rectangle::new(20., 30., 200., 20.), Some(rstr!("Low")), Some(rstr!("High")), cohesion_val, 0., 100.);
        separation_val = d.gui_slider(Rectangle::new(20., 50., 200., 20.), Some(rstr!("Low")), Some(rstr!("High")), separation_val, 0., 100.);
        alignment_val = d.gui_slider(Rectangle::new(20., 70., 200., 20.), Some(rstr!("Low")), Some(rstr!("High")), alignment_val, 0., 1.);
        observation_range = d.gui_slider(Rectangle::new(20., 90., 200., 20.), Some(rstr!("Low")), Some(rstr!("High")), observation_range, 0., 100.);

        let should_reset = d.gui_button(Rectangle::new(120., 10., 100., 20.), Some(rstr!("Reset")));
        if should_reset {
            boids = setup_random_boids(NUM_BOIDS, &mut rng);
        }

        thread::sleep(time::Duration::from_millis(10));
    }
}
