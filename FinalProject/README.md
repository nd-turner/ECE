# Formation Control with Follower Leader (Boid Model)

This project implements a simulation of formation control using the Boid model, where a group of boids (birds) follow a leader while maintaining a certain formation. The leader's movement is controlled by the mouse position, and followers maintain a constant distance from the leader while forming a lattice structure around it.

<video width="640" height="440" controls>
  <source src="Demo/video.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>



## Technologies Used

- **Python**: The main programming language used for this project.
- **Pygame**: Pygame is a set of Python modules designed for writing video games. It provides functionality for graphics rendering and event handling, making it suitable for simulations like this.

## Features

- **Leader Behavior**: The leader's movement is controlled by the mouse position. It moves towards the mouse cursor at a constant speed.
- **Follower Behavior**: Followers maintain a formation around the leader. They move towards their designated positions in the formation while avoiding obstacles and staying within the boundaries of the screen.
- **Obstacle Avoidance**: Boids detect obstacles in their path and adjust their movement to avoid collisions.
- **Boundary Conditions**: Boids stay within the boundaries of the screen to prevent them from going off-screen.
- **Customization**: Parameters such as the number of boids, number of leaders, boid speed, obstacle avoidance distance, etc., can be adjusted to customize the simulation.

## Parameters

- **Number of Boids**: The total number of boids in the simulation.
- **Number of Leaders**: The number of boids designated as leaders. Leaders are controlled by the mouse position.
- **Boid Size**: The size of each boid displayed on the screen.
- **Boid Speed**: The speed at which followers move towards their designated positions.
- **Leader Speed**: The speed at which leaders move towards the mouse cursor.
- **Formation Distance**: The desired distance between followers and the leader in the formation.
- **Formation Size**: The number of followers in each lattice formation around the leader.
- **Formation Radius**: The radius of the lattice formation around the leader.
- **Number of Obstacles**: The total number of obstacles present on the screen.
- **Obstacle Size**: The size of each obstacle displayed on the screen.
- **Obstacle Avoidance Distance**: The distance at which obstacle avoidance behavior is activated.
- **Obstacle Avoidance Force**: The magnitude of the force applied to avoid obstacles.
- **Boundary Margin**: The margin from the screen boundaries within which boids and obstacles are confined.

## Usage

1. Ensure you have Python and Pygame installed on your system.
2. Run the provided Python script.
3. The simulation window will open, displaying the boids and obstacles.
4. Use the mouse to control the movement of the leader(s).
5. Enjoy observing the formation control behavior of the boids!

## Future Improvements

- Add more complex formations and leader behaviors.
- Enhance obstacle avoidance strategies for smoother movement.
- Implement different types of obstacles with varying shapes and sizes.
- Improve performance optimizations for larger simulations.

