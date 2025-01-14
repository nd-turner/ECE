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
NUM_LEADERS = 3
BOID_SIZE = 8
MAX_BOID_SPEED = 2
LEADER_SPEED = 2
TURN_SPEED = 0.05
FORMATION_DISTANCE = 5
FORMATION_SIZE = 4
FORMATION_RADIUS = 80
NEIGHBOR_RADIUS = 100
AVOID_DISTANCE = 15  # Minimum distance to avoid overlap

# Obstacle parameters
NUM_OBSTACLES = 0
OBSTACLE_SIZE = 20
OBSTACLE_AVOIDANCE_DISTANCE = 30
OBSTACLE_AVOIDANCE_FORCE = 5
KP = 0.01
KN = 0.5
NEIGHBOR_AVOIDANCE_FORCE = 10

# Boundary parameters
BOUNDARY_MARGIN = 50

# Helper function to generate random colors that aren't red
def random_non_red_color():
    while True:
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        if not (r > 200 and g < 100 and b < 100):  # Check that the color isn't too close to red
            return (r, g, b)

# Boid class
class Boid:
    def __init__(self, x, y, is_leader=False):
        self.x = x
        self.y = y
        self.angle = random.uniform(0, math.pi * 2)
        self.vx = random.uniform(-LEADER_SPEED, LEADER_SPEED) if is_leader else 0
        self.vy = random.uniform(-LEADER_SPEED, LEADER_SPEED) if is_leader else 0
        self.is_leader = is_leader
        self.change_direction_timer = 0  # Timer to periodically change direction
        self.color = RED if is_leader else random_non_red_color()

    def update(self, boids, obstacles):
        if self.is_leader:
            # Leader periodically changes direction
            self.change_direction_timer += 1
            if self.change_direction_timer > 120:  # Change direction every second (60 frames)
                self.change_direction_timer = 0
                random_angle_change = random.uniform(-math.pi/4, math.pi/4)
                self.angle += random_angle_change
                self.vx = LEADER_SPEED * math.cos(self.angle)
                self.vy = LEADER_SPEED * math.sin(self.angle)

            # Ensure leader's speed does not exceed LEADER_SPEED
            speed = math.sqrt(self.vx**2 + self.vy**2)
            if speed > LEADER_SPEED:
                self.vx = (self.vx / speed) * LEADER_SPEED
                self.vy = (self.vy / speed) * LEADER_SPEED

            # Move leader based on velocity
            self.x += (self.vx/speed)
            self.y += (self.vy/speed)

            # Update angle based on movement direction
            if self.vx != 0 or self.vy != 0:
                self.angle = math.atan2(self.vy, self.vx)

            # Bounce off boundaries
            if self.x <= BOUNDARY_MARGIN or self.x >= WIDTH - BOUNDARY_MARGIN:
                self.vx *= -1
            if self.y <= BOUNDARY_MARGIN or self.y >= HEIGHT - BOUNDARY_MARGIN:
                self.vy *= -1

        else:
            # Follower behavior
            leader = [b for b in boids if b.is_leader][0]
            formation_index = boids.index(self) % FORMATION_SIZE
            formation_angle = (2 * math.pi / FORMATION_SIZE) * formation_index
            formation_x = leader.x + FORMATION_RADIUS * math.cos(formation_angle)
            formation_y = leader.y + FORMATION_RADIUS * math.sin(formation_angle)
            distance_to_formation = math.sqrt((formation_x - self.x) ** 2 + (formation_y - self.y) ** 2)

            # Calculate distance to the leader
            distance_to_leader = math.sqrt((leader.x - self.x) ** 2 + (leader.y - self.y) ** 2)

            # Proportional movement towards formation
            desired_angle = math.atan2(formation_y - self.y, formation_x - self.x)

            # Increase speed based on distance to the leader
            desired_speed = min(KP * distance_to_formation + 0.02 * distance_to_leader, MAX_BOID_SPEED)

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

            # Apply neighbor avoidance
            for other_boid in boids:
                if other_boid != self:
                    distance_to_other = math.sqrt((other_boid.x - self.x) ** 2 + (other_boid.y - self.y) ** 2)
                    if distance_to_other < AVOID_DISTANCE:
                        # Calculate avoidance direction
                        avoid_angle = math.atan2(self.y - other_boid.y, self.x - other_boid.x)
                        self.vx += NEIGHBOR_AVOIDANCE_FORCE * math.cos(avoid_angle)
                        self.vy += NEIGHBOR_AVOIDANCE_FORCE * math.sin(avoid_angle)

            # Apply velocity limits
            speed = math.sqrt(self.vx**2 + self.vy**2)
            if speed > MAX_BOID_SPEED:
                self.vx = (self.vx / speed) * MAX_BOID_SPEED
                self.vy = (self.vy / speed) * MAX_BOID_SPEED

        # Update position
        self.x += self.vx
        self.y += self.vy

        # Ensure boids stay within bounds
        self.x = max(BOID_SIZE, min(WIDTH - BOID_SIZE, self.x))
        self.y = max(BOID_SIZE, min(HEIGHT - BOID_SIZE, self.y))

    def draw(self):
        # Define the triangle shape
        tip = (self.x + BOID_SIZE * math.cos(self.angle), self.y + BOID_SIZE * math.sin(self.angle))
        left_wing = (self.x + BOID_SIZE * math.cos(self.angle + 2.5), self.y + BOID_SIZE * math.sin(self.angle + 2.5))
        right_wing = (self.x + BOID_SIZE * math.cos(self.angle - 2.5), self.y + BOID_SIZE * math.sin(self.angle - 2.5))

        # Draw the boid as a triangle with its color
        pygame.draw.polygon(screen, self.color, [tip, left_wing, right_wing])

# Obstacle class (not used here)
class Obstacle:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def draw(self):
        pygame.draw.circle(screen, WHITE, (int(self.x), int(self.y)), OBSTACLE_SIZE)

# Create boids with leaders
boids = [Boid(random.randint(0, WIDTH), random.randint(0, HEIGHT), is_leader=(i < NUM_LEADERS)) for i in range(NUM_BOIDS)]

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
        boid.update(boids, [])
        boid.draw()

    pygame.display.flip()

    # Control frame rate
    pygame.time.Clock().tick(60)

# Quit Pygame
pygame.quit()
