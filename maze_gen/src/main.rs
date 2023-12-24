extern crate olc_pixel_game_engine;

use std::thread;
use std::time::Duration;
use olc_pixel_game_engine::{fill_rect, GREEN, RED};
use olc_pixel_game_engine::draw;
use olc_pixel_game_engine::BLUE;
use olc_pixel_game_engine::BLACK;
use olc_pixel_game_engine::screen_height;
use olc_pixel_game_engine::screen_width;
use olc_pixel_game_engine::WHITE;
use rand::thread_rng;
use crate::olc_pixel_game_engine as olc;
use rand::rngs::ThreadRng;
use rand::Rng;


const CELL_VISITED: u32 = 0x01;
const CELL_PATH_S: u32 = 0x02;
const CELL_PATH_N: u32 = 0x04;
const CELL_PATH_E: u32 = 0x08;
const CELL_PATH_W: u32 = 0x10;


const NORTH_ID: u8 = 0;
const EAST_ID: u8 = 1;
const SOUTH_ID: u8 = 2;
const WEST_ID: u8 = 3;


struct MazeStruct {
    maze_width: i32,
    maze_height: i32,
    maze: Vec<u32>,
    visited_cell_count: i32,

    // this represents x & y coordinates respectively.
    stack: Vec<(i32, i32)>,
    path_width: i32,
    rng: ThreadRng,
}

impl olc::Application for MazeStruct {


    fn on_user_create(&mut self) -> Result<(), olc::Error> {
        // Mirrors `olcPixelGameEngine::onUserCreate`.
        // Your code goes here.

        self.path_width = 3;
        self.maze_width = 40;
        self.maze_height = 25;
        // we allocate all of our cells and set their value to zero.
        let size = self.maze_width * self.maze_height;
        self.maze = vec![0; size as usize];
        // init our stack
        self.stack = Vec::new();
        // push our first square onto the stack. This is the top left square
        self.stack.push((thread_rng().gen_range(0..self.maze_width), thread_rng().gen_range(0..self.maze_height)));

        // self.maze[0] = CELL_VISITED;
        self.visited_cell_count = 1;

        Ok(())
    }

    fn on_user_update(&mut self, _elapsed_time: f32) -> Result<(), olc::Error> {
        thread::sleep(Duration::from_millis(10));
        // maze algorithm


        // run our maze algo only until we have visited all cells...
        if self.visited_cell_count < (self.maze_width * self.maze_height)  {
            // step one create a set of unvisited neighbors

            // pre-fetch the top of our stack so we don't get borrow errors. Also this is easier.
            let top_stack = *self.stack.last().unwrap();
            let offset = |x: i32, y: i32| -> usize {
                let off =  (top_stack.1 + y) * self.maze_width + (top_stack.0 + x);
                println!("offset: {} input x:{}, y:{}, topStack x: {}, y: {}", off, x, y, top_stack.0, top_stack.1);
                return off as usize;
            };

            let mut neighbors: Vec<u8> = Vec::new();

            // North Neighbour
            if top_stack.1 > 0 && (self.maze[offset(0, -1)] & CELL_VISITED) == 0 {
                neighbors.push(NORTH_ID)
            }

            // East Neighbour
            if top_stack.0 < self.maze_width - 1 && (self.maze[offset(1, 0)] & CELL_VISITED) == 0 {
                neighbors.push(EAST_ID)
            }

            // South Neighbour
            if top_stack.1 < self.maze_height - 1 && self.maze[offset(0, 1)] & CELL_VISITED == 0 {
                neighbors.push(SOUTH_ID)
            }

            // West neighbor
            if top_stack.0 > 0 && (self.maze[offset(-1, 0)] & CELL_VISITED) == 0 {
                neighbors.push(WEST_ID)
            }


            // are there any neighbors available?
            if !neighbors.is_empty() {

                // choose a neighbor to explore at random
                let random_index = self.rng.gen_range(0..neighbors.len());
                let next_cell_direction = neighbors[random_index];

                // create a path between the neighbour and the current cell

                match next_cell_direction {
                    NORTH_ID => {
                        self.maze[offset(0, -1)] |= CELL_VISITED | CELL_PATH_S;
                        self.maze[offset(0,0)] |= CELL_PATH_N;
                        self.stack.push((top_stack.0 + 0, top_stack.1 - 1))
                    }
                    EAST_ID => {
                        self.maze[offset(1, 0)] |= CELL_VISITED | CELL_PATH_W;
                        self.maze[offset(0,0)] |= CELL_PATH_E;
                        self.stack.push((top_stack.0 + 1, top_stack.1 + 0))
                    }
                    SOUTH_ID => {
                        self.maze[offset(0, 1)] |= CELL_VISITED | CELL_PATH_N;
                        self.maze[offset(0,0)] |= CELL_PATH_S;
                        self.stack.push((top_stack.0 + 0, top_stack.1 + 1))
                    }
                    WEST_ID => {
                        self.maze[offset(-1, 0)] |= CELL_VISITED | CELL_PATH_E;
                        self.maze[offset(0,0)] |= CELL_PATH_W;
                        self.stack.push((top_stack.0 - 1, top_stack.1 + 0))
                    }
                    _ => todo!("WE SHOULD NEVER GET HERE. INVALID STATE")
                }

                self.maze[offset(0,0)] |= CELL_VISITED;
                self.visited_cell_count += 1;
                println!("visited count {}", self.visited_cell_count)
            } else {
                self.stack.pop().unwrap();
            }
        }


        // reset our render to be blank
        fill_rect(0, 0, screen_width(), screen_height(), BLACK);

        // render our initial maze
        for x in 0..self.maze_width {
            for y in 0..self.maze_height {

                let index = y as usize * self.maze_width as usize + x as usize;
                // Each cell is inflated by path_width, so fill it in
                for py in 0..self.path_width {
                    for px in 0..self.path_width {
                        if self.maze[index] & CELL_VISITED != 0 {
                            draw(x * (self.path_width + 1) + px, y * (self.path_width + 1) + py, WHITE);
                        } else {
                            draw(x * (self.path_width + 1) + px, y * (self.path_width + 1) + py, BLUE);
                        }
                    }
                }

                // Draw passageways between cells
                for p in 0..self.path_width {
                    // println!("south: {}, east: {}", self.maze[index] & CELL_PATH_S, self.maze[index] & CELL_PATH_E);

                    if self.maze[index] & CELL_PATH_S != 0 {
                        draw(x * (self.path_width + 1) + p, y * (self.path_width + 1) + self.path_width, WHITE);
                    }

                    if self.maze[index] & CELL_PATH_E != 0 {
                        draw(x * (self.path_width + 1) + self.path_width, y * (self.path_width + 1) + p, WHITE);
                    }
                }
            }
        }

        // Draw the current exploration square - the top of the stack
        for py in 0..self.path_width {
            for px in 0..self.path_width {
                let top_stack = *self.stack.last().unwrap();
                draw(top_stack.0 * (self.path_width + 1) + px, top_stack.1 * (self.path_width + 1) + py, GREEN);
            }
        }

        Ok(())
    }

    fn on_user_destroy(&mut self) -> Result<(), olc::Error> {
        // Mirrors `olcPixelGameEngine::onUserDestroy`.
        // Your code goes here.
        Ok(())
    }
}

fn main() {
    let mut maze = MazeStruct {
        maze_width: 0,
        maze_height: 0,
        maze: vec![],
        visited_cell_count: 0,
        stack: vec![],
        path_width: 0,
        rng: thread_rng()
    };

    // Launches the program in 256x240 "pixels" screen, where each "pixel" is 4x4 pixel square,
    // and starts the main game loop.
    olc::start("Maze Generator", &mut maze, 160, 100, 8, 8).unwrap();
}