extern crate olc_pixel_game_engine;

use olc::Application;
use olc_pixel_game_engine::{get_key, RED};
use olc_pixel_game_engine::GREY;
use olc_pixel_game_engine::GREEN;
use olc_pixel_game_engine::get_mouse_y;
use olc_pixel_game_engine::get_mouse_x;
use olc_pixel_game_engine::get_mouse;
use olc_pixel_game_engine::fill_rect;
use olc_pixel_game_engine::Error;
use olc_pixel_game_engine::draw_line;
use olc_pixel_game_engine::DARK_BLUE;
use olc_pixel_game_engine::BLUE;
use olc_pixel_game_engine::BLACK;
use olc_pixel_game_engine::Key::{CTRL, SHIFT};
use olc_pixel_game_engine::screen_height;
use olc_pixel_game_engine::screen_width;
use olc_pixel_game_engine::VERY_DARK_BLUE;
use olc_pixel_game_engine::YELLOW;
use crate::olc_pixel_game_engine as olc;


#[derive(Clone, Debug, PartialEq, Eq, Hash)]
struct Node {
    // distance to goal if we took alternative route
    x: i32,
    // nodes pos in 2D space
    y: i32,

    obstacle: bool,
    // is the node an obstruction
    visited: bool,
    // have we searched this node before?
    global_goal: i32,
    // distance to gaol so far
    local_goal: i32,
    parent: Option<usize>,
}

const MAP_WIDTH: i32 = 16;
const MAP_HEIGHT: i32 = 16;


const NODE_SIZE: i32 = 9;
const NODE_BORDER: i32 = 6;


impl Node {
    /**
    The pythagorean theorem to get the distance between two points "as the crow flies" heuristic to aid our A* search
     */
    fn distance(&self, goal: &Node) -> i32 {
        let square_dif_x: f32 = ((self.x - goal.x) * (self.x - goal.x)) as f32;
        let square_dif_y: f32 = ((self.y - goal.y) * (self.y - goal.y)) as f32;
        return (square_dif_x + square_dif_y).sqrt() as i32;
    }

    /**
     * This is how the A* algorithm is fast and efficient. Basically a better heuristic means a more efficient search.
     */
    fn heuristic(&self, goal: &Node) -> i32 {
        return self.distance(goal);
    }

}

struct AppStruct {
    nodes: Vec<Node>,
    node_start_index: Option<usize>,
    // this represents the start of our search
    node_end_index: Option<usize>, // this represents the destination of our search.

    // this is a simple flag we use to determine when we need re-run our a-star algorithm. we re-run when our obstacles
    // change or our start/end gaol changes. This saves lots of CPU cycles.
    needs_a_star_run: bool,
}


impl Application for AppStruct {
    fn on_user_create(&mut self) -> Result<(), Error> {
        self.nodes = Vec::new();

        for y in 0..MAP_HEIGHT {
            for x in 0..MAP_WIDTH {
                self.nodes.push(Node {
                    obstacle: false,
                    visited: false,
                    global_goal: i32::MAX,
                    local_goal: i32::MAX,
                    x,
                    y,
                    parent: None,
                })
            }
        }

        // assign defaults to our start and end locations.
        self.node_start_index = Some(((MAP_HEIGHT / 2) * MAP_WIDTH + 1) as usize);
        self.node_end_index = Some(((MAP_HEIGHT / 2) * MAP_WIDTH + MAP_WIDTH - 2) as usize);

        Ok(())
    }


    fn on_user_update(&mut self, _elapsed_time: f32) -> Result<(), Error> {
        self.check_mouse_keyboard_events();

        // fill our view with black by default. This will set the background color
        fill_rect(0, 0, screen_width(), screen_height(), BLACK);
        self.render_node_edges();

        // we want to render our active path behind the nodes.
        self.render_active_path();

        // render our squares
        self.render_nodes();

        Ok(())
    }

    fn on_user_destroy(&mut self) -> Result<(), Error> {
        Ok(())
    }
}

fn a_star(start_index: usize, goal_index: usize, nodes: &mut Vec<Node>) -> Option<Vec<usize>> {
    let mut open_set = std::collections::BinaryHeap::new();

    nodes[start_index].global_goal = 0;
    nodes[start_index].local_goal = nodes[start_index].heuristic(&nodes[goal_index]);
    open_set.push(std::cmp::Reverse((nodes[start_index].local_goal, start_index)));

    // In Rust, the std::collections::BinaryHeap is a max-heap by default, meaning it always pops the largest element
    // first. However, in many algorithms like A*, you typically need a min-heap, which pops the smallest element first.
    while let Some(std::cmp::Reverse((_, current_index))) = open_set.pop() {
        if current_index == goal_index {
            return Some(construct_path(nodes, goal_index));
        }

        for neighbor_index in get_neighbors(current_index, nodes) {
            // Check if the neighbor is an obstacle
            if nodes[neighbor_index].obstacle {
                continue;  // Skip this neighbor and proceed to the next one
            }

            let tentative_global_goal = nodes[current_index].global_goal + 1; // Assuming uniform cost for simplicity

            if tentative_global_goal < nodes[neighbor_index].global_goal {
                nodes[neighbor_index].parent = Some(current_index);
                nodes[neighbor_index].global_goal = tentative_global_goal;
                nodes[neighbor_index].local_goal = tentative_global_goal + nodes[neighbor_index].heuristic(&nodes[goal_index]);

                // A node (neighbor) is added to the open_set if it is not already present in it.
                // This check is performed by iterating over all nodes currently in the open_set and seeing if any of
                // them have the same index as the neighbor node (neighbor_index). The check is necessary because
                // adding the same node multiple times would be inefficient and could lead to incorrect behavior.
                if !open_set.iter().any(|std::cmp::Reverse((_, idx))| *idx == neighbor_index) {
                    open_set.push(std::cmp::Reverse((nodes[neighbor_index].local_goal, neighbor_index)));
                }
            }
        }
    }

    None
}


fn construct_path(nodes: &[Node], mut current_index: usize) -> Vec<usize> {
    let mut path = vec![current_index];
    while let Some(parent_index) = nodes[current_index].parent {
        path.push(parent_index);
        current_index = parent_index;
    }
    path.reverse();
    path
}

fn get_neighbors(index: usize, nodes: &Vec<Node>) -> Vec<usize> {
    let mut neighbors = Vec::new();
    let x = index % (MAP_WIDTH as usize);
    let y = index / (MAP_WIDTH as usize);

    // north
    if y > 0 {
        let north_idx = (y - 1) * (MAP_WIDTH as usize) + x;
        if !nodes[north_idx].obstacle {
            neighbors.push(north_idx);
        }
    }
    // south
    if y < (MAP_HEIGHT - 1) as usize {
        let south_idx = (y + 1) * (MAP_WIDTH as usize) + x;
        if !nodes[south_idx].obstacle {
            neighbors.push(south_idx);
        }
    }
    // west
    if x > 0 {
        let west_idx = y * (MAP_WIDTH as usize) + (x - 1);
        if !nodes[west_idx].obstacle {
            neighbors.push(west_idx);
        }
    }
    // east
    if x < (MAP_WIDTH - 1) as usize {
        let east_idx = y * (MAP_WIDTH as usize) + (x + 1);
        if !nodes[east_idx].obstacle {
            neighbors.push(east_idx);
        }
    }

    neighbors
}
fn reset_and_clone_nodes(nodes: &[Node]) -> Vec<Node> {
    nodes.iter().map(|node| Node {
        x: node.x,
        y: node.y,
        obstacle: node.obstacle,
        local_goal: i32::MAX,
        global_goal: i32::MAX,
        parent: None,
        visited: node.visited,
    }).collect()
}


impl AppStruct {

    /**
    Renders the node edges that connect the nodes together.
     */
    fn render_node_edges(&mut self) {
        for y in 0..MAP_HEIGHT {
            for x in 0..MAP_WIDTH {
                let index = (y * MAP_WIDTH + x) as usize;

                let neighbors = get_neighbors(index, &self.nodes);
                for neighbor_index in neighbors {
                    let neighbor = &self.nodes[neighbor_index];

                    // I'm not sure why we have to offset this by - 4... my original theory was I wasn't taking
                    // the border into account, but 4 is not a factor of the current border value, so I'm at a loss, but
                    // this seems to work.
                    draw_line(x * NODE_SIZE + NODE_SIZE - 4 / 2,
                              y * NODE_SIZE + NODE_SIZE - 4 / 2,
                              neighbor.x * NODE_SIZE + NODE_SIZE - 4 / 2,
                              neighbor.y * NODE_SIZE + NODE_SIZE - 4 / 2, VERY_DARK_BLUE)
                }
            }
        }
    }


    /**
    Renders the a* path
     */
    fn render_active_path(&mut self) {
        if let Some(start_idx, ) = self.node_start_index {
            if let Some(goal_idx) = self.node_end_index {
                if self.needs_a_star_run {
                    // reset our node values
                    let nodes = reset_and_clone_nodes(&self.nodes);
                    self.nodes = nodes;
                    a_star(start_idx, goal_idx, &mut self.nodes);
                    self.needs_a_star_run = false
                }
            }
        }

        if let Some(mut node_index) = self.node_end_index {
            while let Some(parent_index) = self.nodes[node_index].parent {
                let (node_x, node_y) = (self.nodes[node_index].x, self.nodes[node_index].y);
                let (parent_x, parent_y) = (self.nodes[parent_index].x, self.nodes[parent_index].y);

                draw_line(node_x * NODE_SIZE + NODE_SIZE - 4 / 2,
                          node_y * NODE_SIZE + NODE_SIZE - 4  / 2,
                          parent_x * NODE_SIZE + NODE_SIZE - 4 / 2,
                          parent_y * NODE_SIZE + NODE_SIZE - 4 / 2,
                          YELLOW);

                node_index = parent_index;
            }
        }
    }

    fn check_mouse_keyboard_events(&mut self) {
        let selected_node_x = get_mouse_x() / NODE_SIZE;
        let selected_node_y = get_mouse_y() / NODE_SIZE;

        // check what square we are clicking if any and update our node that's being clicked to
        // toggle the obstacle flag.
        if get_mouse(0).released {
            if selected_node_x >= 0 && selected_node_x < MAP_WIDTH {
                if selected_node_y >= 0 && selected_node_y < MAP_HEIGHT {
                    let index: usize = (selected_node_y * MAP_WIDTH + selected_node_x) as usize;
                    if index < self.nodes.len() { // check that our index is not out of bounds.
                        if get_key(SHIFT).held { // if we hold the shift key while clicking... we should set the end node
                            self.node_end_index = Some(index)
                        } else if get_key(CTRL).held { // if we hold the control key while clicking we should set the start node
                            self.node_start_index = Some(index)
                        } else { // otherwise just toggle an obstacle node.
                            self.nodes[index].obstacle = !self.nodes[index].obstacle;
                        }
                        self.needs_a_star_run = true
                    }
                }
            }
        }
    }

    /**
    Renders the nodes aka the squares.
     */
    fn render_nodes(&mut self) {
        for y in 0..MAP_HEIGHT {
            for x in 0..MAP_WIDTH {
                let index: usize = (y * MAP_WIDTH + x) as usize; // get the index of the current square being rendered.
                if index < self.nodes.len() { // check that our index is not out of bounds.
                    fill_rect(x * NODE_SIZE + NODE_BORDER,
                              y * NODE_SIZE + NODE_BORDER,
                              NODE_SIZE - NODE_BORDER,
                              NODE_SIZE - NODE_BORDER,
                              // we change the color of our square if it's obstacle value is true
                              if self.nodes[index].obstacle { GREY } else { DARK_BLUE });


                    if self.nodes[index].visited {
                        fill_rect(x * NODE_SIZE + NODE_BORDER,
                                  y * NODE_SIZE + NODE_BORDER,
                                  NODE_SIZE - NODE_BORDER,
                                  NODE_SIZE - NODE_BORDER,
                                  // we change the color of our square if it's obstacle value is true
                                  if self.nodes[index].obstacle { GREY } else { BLUE });
                    }


                    if let Some(start_index) = self.node_start_index {
                        if index == start_index {
                            fill_rect(x * NODE_SIZE + NODE_BORDER,
                                      y * NODE_SIZE + NODE_BORDER,
                                      NODE_SIZE - NODE_BORDER,
                                      NODE_SIZE - NODE_BORDER,
                                      // we change the color of our square if it's obstacle value is true
                                      GREEN);
                        }
                    }

                    if let Some(end_index) = self.node_end_index {
                        if index == end_index {
                            fill_rect(
                                x * NODE_SIZE + NODE_BORDER,
                                y * NODE_SIZE + NODE_BORDER,
                                NODE_SIZE - NODE_BORDER,
                                NODE_SIZE - NODE_BORDER,
                                RED, // assuming RED is previously defined
                            );
                        }
                    }
                }
            }
        }
    }
}


fn main() {
    let mut a_star = AppStruct {
        nodes: vec![],
        node_start_index: None,
        node_end_index: None,
        needs_a_star_run: true,
    };

    olc::start("A*", &mut a_star, 160, 160, 6, 6).unwrap();
}
