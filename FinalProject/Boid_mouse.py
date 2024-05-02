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
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Boid parameters
NUM_BOIDS = 100
NUM_LEADERS = 1
BOID_SIZE = 4
BOID_SPEED = 1
LEADER_SPEED = 3  # Leader's speed
FORMATION_DISTANCE = 5  # Distance from the leader for followers
FORMATION_SIZE = 5  # Number of followers in each lattice formation
FORMATION_RADIUS = 80  # Radius of the lattice formation
NEIGHBOR_RADIUS = 10

# Obstacle parameters
NUM_OBSTACLES = 20
OBSTACLE_SIZE = 20
OBSTACLE_AVOIDANCE_DISTANCE = 30  # Distance at which obstacle avoidance behavior is activated
OBSTACLE_AVOIDANCE_FORCE = 5  # Magnitude of obstacle avoidance force
KP = .01
KN = .2
NEIGHBOR_AVOIDANCE_FORCE = 1

# Boundary parameters
BOUNDARY_MARGIN = 50

# Boid class
class Boid:
    def __init__(self, x, y, is_leader=False):
        self.x = x
        self.y = y
        self.angle = random.uniform(0, math.pi * 2)
        self.is_leader = is_leader

    def update(self, boids, obstacles):
        if self.is_leader:
            # Leader behavior
            mouse_x, mouse_y = pygame.mouse.get_pos()
            angle_to_mouse = math.atan2(mouse_y - self.y, mouse_x - self.x)
            self.angle = angle_to_mouse
            self.x += LEADER_SPEED * math.cos(self.angle)
            self.y += LEADER_SPEED * math.sin(self.angle)
        else:
            # Follower behavior
            leader = [b for b in boids if b.is_leader][0]
            formation_index = boids.index(self) % FORMATION_SIZE
            formation_angle = (2 * math.pi / FORMATION_SIZE) * formation_index
            formation_x = leader.x + FORMATION_RADIUS * math.cos(formation_angle)
            formation_y = leader.y + FORMATION_RADIUS * math.sin(formation_angle)
            distance_to_formation = math.sqrt((formation_x - self.x) ** 2 + (formation_y - self.y) ** 2)
            BOID_SPEED = KP * distance_to_formation
            if distance_to_formation > FORMATION_DISTANCE:
                angle_to_formation = math.atan2(formation_y - self.y, formation_x - self.x)
                self.angle = angle_to_formation
                self.x += BOID_SPEED * math.cos(self.angle)
                self.y += BOID_SPEED * math.sin(self.angle)

           
              # Obstacle avoidance
                for obstacle in obstacles:
                    distance_to_obstacle = math.sqrt((obstacle.x - self.x) ** 2 + (obstacle.y - self.y) ** 2)
                    if distance_to_obstacle < OBSTACLE_AVOIDANCE_DISTANCE:
                        angle_to_obstacle = math.atan2(self.y - obstacle.y, self.x - obstacle.x)
                        self.x += OBSTACLE_AVOIDANCE_FORCE * math.cos(angle_to_obstacle)
                        self.y += OBSTACLE_AVOIDANCE_FORCE * math.sin(angle_to_obstacle)
           
            # Calculate average direction to nearby boids             
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
            
            # Avoidance behavior
            if num_neighbors > 0:
                avoid_direction[0] /= num_neighbors
                avoid_direction[1] /= num_neighbors
                avoid_angle = math.atan2(avoid_direction[1], avoid_direction[0])
                self.angle = avoid_angle
                self.x += KN * NEIGHBOR_AVOIDANCE_FORCE * math.cos(avoid_angle)
                self.y += KN * NEIGHBOR_AVOIDANCE_FORCE * math.sin(avoid_angle)

                
                # Boundary conditions
                if self.x < BOUNDARY_MARGIN:
                    self.x = BOUNDARY_MARGIN
                elif self.x > WIDTH - BOUNDARY_MARGIN:
                    self.x = WIDTH - BOUNDARY_MARGIN
                if self.y < BOUNDARY_MARGIN:
                    self.y = BOUNDARY_MARGIN
                elif self.y > HEIGHT - BOUNDARY_MARGIN:
                    self.y = HEIGHT - BOUNDARY_MARGIN

              

    def draw(self):
        if self.is_leader:
            pygame.draw.circle(screen, RED, (int(self.x), int(self.y)), BOID_SIZE)
        else:
            pygame.draw.circle(screen, WHITE, (int(self.x), int(self.y)), BOID_SIZE)

# Obstacle class
class Obstacle:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def draw(self):
        pygame.draw.circle(screen, BLUE, (int(self.x), int(self.y)), OBSTACLE_SIZE)

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
