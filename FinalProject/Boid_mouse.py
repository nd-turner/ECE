import pygame
import random
import math

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Formation control with follower leader (boid model)")

# Colors
RED = (255, 0, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# Boid parameters
NUM_BOIDS = 200
NUM_LEADERS = 1
BOID_SIZE = 8
MAX_BOID_SPEED = 2
LEADER_SPEED = 4
TURN_SPEED = 0.05
FORMATION_SIZE = 4
FORMATION_RADIUS = 80
NEIGHBOR_RADIUS = 30

# Obstacle parameters
NUM_OBSTACLES = 0
OBSTACLE_SIZE = 20
OBSTACLE_AVOIDANCE_DISTANCE = 30
OBSTACLE_AVOIDANCE_FORCE = 5
KP = .01
KN = .5
NEIGHBOR_AVOIDANCE_FORCE = 1

# Boundary parameters
BOUNDARY_MARGIN = 50

# Helper function to generate random colors that aren't red
def random_non_red_color():
    while True:
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        if not (r > 200 and g < 100 and b < 100):
            return (r, g, b)

# Boid class
class Boid:
    def __init__(self, x, y, is_leader=False):
        self.x = x
        self.y = y
        self.angle = random.uniform(0, 2 * math.pi)
        self.vx = 0
        self.vy = 0
        self.is_leader = is_leader
        self.color = RED if is_leader else random_non_red_color()

        # If the boid is a leader, set an initial random angle and speed
        if is_leader:
            self.angle = random.uniform(0, 2 * math.pi)  # Random initial angle
            self.vx = LEADER_SPEED * math.cos(self.angle)
            self.vy = LEADER_SPEED * math.sin(self.angle)

    def update(self, boids, obstacles):
        if self.is_leader:
            # Incrementally change the leader's angle to create independent movement
            self.angle += random.uniform(-TURN_SPEED, TURN_SPEED)  # Small random turn
            
            # Update velocity based on the current angle
            self.vx = LEADER_SPEED * math.cos(self.angle)
            self.vy = LEADER_SPEED * math.sin(self.angle)

            # Check for boundary collisions
            if self.x <= BOUNDARY_MARGIN or self.x >= WIDTH - BOUNDARY_MARGIN:
                self.vx *= -1  # Invert horizontal velocity on collision
                if self.x <= BOUNDARY_MARGIN:
                    self.x = BOUNDARY_MARGIN  # Prevent moving out of bounds
                else:
                    self.x = WIDTH - BOUNDARY_MARGIN  # Prevent moving out of bounds

            if self.y <= BOUNDARY_MARGIN or self.y >= HEIGHT - BOUNDARY_MARGIN:
                self.vy *= -1  # Invert vertical velocity on collision
                if self.y <= BOUNDARY_MARGIN:
                    self.y = BOUNDARY_MARGIN  # Prevent moving out of bounds
                else:
                    self.y = HEIGHT - BOUNDARY_MARGIN  # Prevent moving out of bounds

        else:
            # Follower behavior
            leader = next(b for b in boids if b.is_leader)
            formation_index = boids.index(self) % FORMATION_SIZE
            formation_angle = (2 * math.pi / FORMATION_SIZE) * formation_index
            formation_x = leader.x + FORMATION_RADIUS * math.cos(formation_angle)
            formation_y = leader.y + FORMATION_RADIUS * math.sin(formation_angle)
            distance_to_formation = math.sqrt((formation_x - self.x) ** 2 + (formation_y - self.y) ** 2)

            # Proportional movement towards formation
            desired_angle = math.atan2(formation_y - self.y, formation_x - self.x)
            desired_speed = min(KP * distance_to_formation, MAX_BOID_SPEED)

            # Smoothly adjust angle towards desired direction
            angle_diff = desired_angle - self.angle
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            self.angle += max(-TURN_SPEED, min(TURN_SPEED, angle_diff))

            # Update velocity
            self.vx = desired_speed * math.cos(self.angle)
            self.vy = desired_speed * math.sin(self.angle)

            # Obstacle avoidance
            for obstacle in obstacles:
                distance_to_obstacle = math.sqrt((obstacle.x - self.x) ** 2 + (obstacle.y - self.y) ** 2)
                if distance_to_obstacle < OBSTACLE_AVOIDANCE_DISTANCE:
                    angle_to_obstacle = math.atan2(self.y - obstacle.y, self.x - obstacle.x)
                    avoidance_vx = OBSTACLE_AVOIDANCE_FORCE * math.cos(angle_to_obstacle)
                    avoidance_vy = OBSTACLE_AVOIDANCE_FORCE * math.sin(angle_to_obstacle)
                    self.vx += avoidance_vx
                    self.vy += avoidance_vy

            # Neighbor avoidance behavior
            avoid_direction = [0, 0]
            num_neighbors = 0
            for other_boid in boids:
                if other_boid != self:
                    distance_to_other = math.sqrt((other_boid.x - self.x) ** 2 + (other_boid.y - self.y) ** 2)
                    if distance_to_other < NEIGHBOR_RADIUS:
                        angle_to_other = math.atan2(self.y - other_boid.y, self.x - other_boid.x)
                        avoid_direction[0] += math.cos(angle_to_other)
                        avoid_direction[1] += math.sin(angle_to_other)
                        num_neighbors += 1
            
            if num_neighbors > 0:
                avoid_direction[0] /= num_neighbors
                avoid_direction[1] /= num_neighbors
                avoid_angle = math.atan2(avoid_direction[1], avoid_direction[0])
                self.vx += KN * NEIGHBOR_AVOIDANCE_FORCE * math.cos(avoid_angle)
                self.vy += KN * NEIGHBOR_AVOIDANCE_FORCE * math.sin(avoid_angle)

            # Apply velocity limits
            speed = math.sqrt(self.vx**2 + self.vy**2)
            if speed > MAX_BOID_SPEED:
                self.vx = (self.vx / speed) * MAX_BOID_SPEED
                self.vy = (self.vy / speed) * MAX_BOID_SPEED

        # Update position
        self.x += self.vx
        self.y += self.vy

    def draw(self):
        # Define the triangle shape
        tip = (self.x + BOID_SIZE * math.cos(self.angle), self.y + BOID_SIZE * math.sin(self.angle))
        left_wing = (self.x + BOID_SIZE * math.cos(self.angle + 2.5), self.y + BOID_SIZE * math.sin(self.angle + 2.5))
        right_wing = (self.x + BOID_SIZE * math.cos(self.angle - 2.5), self.y + BOID_SIZE * math.sin(self.angle - 2.5))

        # Draw the boid as a triangle with its color
        pygame.draw.polygon(screen, self.color, [tip, left_wing, right_wing])

# Obstacle class
class Obstacle:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def draw(self):
        pygame.draw.circle(screen, WHITE, (int(self.x), int(self.y)), OBSTACLE_SIZE)

# Create boids with leaders
boids = [Boid(random.randint(0, WIDTH), random.randint(0, HEIGHT), is_leader=(i < NUM_LEADERS)) for i in range(NUM_BOIDS)]

# Create obstacles
obstacles = [Obstacle(random.randint(0, WIDTH), random.randint(0, HEIGHT)) for _ in range(NUM_OBSTACLES)]

# Main loop
running = True
while running:
    screen.fill(BLACK)

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Update and draw boids
    for boid in boids:
        boid.update(boids, obstacles)
        boid.draw()

    # Draw obstacles
    for obstacle in obstacles:
        obstacle.draw()

    pygame.display.flip()

    # Control frame rate
    pygame.time.Clock().tick(60)

# Quit Pygame
pygame.quit()
