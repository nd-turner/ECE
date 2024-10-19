import pygame
import random
import math

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 1200, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
pygame.display.set_caption("Formation control with follower leader (boid model)")

# Colors
RED = (255, 0, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# Boid parameters
NUM_BOIDS = 100
NUM_LEADERS = 1
BOID_SIZE = 10
MAX_BOID_SPEED = 2
LEADER_SPEED = .5
TURN_SPEED = 0.1
FORMATION_DISTANCE = 20
FORMATION_SIZE = 4
FORMATION_RADIUS = 100
NEIGHBOR_RADIUS = 10
AVOID_DISTANCE = 15  # Minimum distance to avoid overlap

# Obstacle parameters
NUM_OBSTACLES = 4
OBSTACLE_SIZE = 15
OBSTACLE_AVOIDANCE_DISTANCE = 30
OBSTACLE_AVOIDANCE_FORCE = 10
KP = 0.01
KN = 0.5
NEIGHBOR_AVOIDANCE_FORCE = 15

# Boundary parameters
BOUNDARY_MARGIN = 50

# Helper function to generate random colors that aren't red
def random_non_red_color():
    while True:
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        if not (r > 1 and g < 100 and b < 100):  # Check that the color isn't too close to red
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
                random_angle_change = random.uniform(-math.pi / 4, math.pi / 4)
                self.angle += random_angle_change
                self.vx = LEADER_SPEED * math.cos(self.angle)
                self.vy = LEADER_SPEED * math.sin(self.angle)

            # Ensure leader's speed does not exceed LEADER_SPEED
            speed = math.sqrt(self.vx**2 + self.vy**2)
            if speed > LEADER_SPEED:
                self.vx = (self.vx / speed) * LEADER_SPEED
                self.vy = (self.vy / speed) * LEADER_SPEED

            # Move leader based on velocity
            self.x += (self.vx / speed)
            self.y += (self.vy / speed)

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

            # Apply obstacle avoidance
            for obstacle in obstacles:
                distance_to_obstacle = math.sqrt((obstacle.x - self.x) ** 2 + (obstacle.y - self.y) ** 2)
                if distance_to_obstacle < OBSTACLE_AVOIDANCE_DISTANCE:
                    avoid_angle = math.atan2(self.y - obstacle.y, self.x - obstacle.x)
                    self.vx += OBSTACLE_AVOIDANCE_FORCE * math.cos(avoid_angle)
                    self.vy += OBSTACLE_AVOIDANCE_FORCE * math.sin(avoid_angle)

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

# Obstacle class
class Obstacle:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def draw(self):
        pygame.draw.circle(screen, WHITE, (int(self.x), int(self.y)), OBSTACLE_SIZE)

# Create obstacles at random positions
obstacles = [Obstacle(random.randint(OBSTACLE_SIZE, WIDTH - OBSTACLE_SIZE), random.randint(OBSTACLE_SIZE, HEIGHT - OBSTACLE_SIZE)) for _ in range(NUM_OBSTACLES)]

# Create boids with leaders
boids = [Boid(random.randint(0, WIDTH), random.randint(0, HEIGHT), is_leader=(i < NUM_LEADERS)) for i in range(NUM_BOIDS)]

# Timer for changing leaders
leader_change_timer = 0
LEADER_CHANGE_INTERVAL = 300  # Change leader every 5 seconds (300 frames)

# Main loop
running = True
while running:
    screen.fill(BLACK)

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.VIDEORESIZE:
            # Update screen size when window is resized
            WIDTH, HEIGHT = event.w, event.h
            screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)

    # Update and draw boids
    for boid in boids:
        boid.update(boids, obstacles)
        boid.draw()

    # Draw obstacles
    for obstacle in obstacles:
        obstacle.draw()

    # Check if it's time to change the leader
    leader_change_timer += 1
    if leader_change_timer >= LEADER_CHANGE_INTERVAL:
        # Reset the timer
        leader_change_timer = 0

        # Select a new leader from followers
        current_leaders = [b for b in boids if b.is_leader]
        
        # Make current leaders followers
        for leader in current_leaders:
            leader.is_leader = False  # Make current leader a follower
            leader.color = random_non_red_color()  # Change color to non-red

        # Get followers
        followers = [b for b in boids if not b.is_leader]
        
        # Ensure there are enough followers to change to leaders
        num_new_leaders = min(NUM_LEADERS, len(followers))
        if num_new_leaders > 0:
            # Randomly select new leaders from followers
            new_leaders = random.sample(followers, num_new_leaders)
            for new_leader in new_leaders:
                new_leader.is_leader = True  # Make this boid a leader
                new_leader.color = RED  # Change color to red

    pygame.display.flip()

    # Control frame rate
    pygame.time.Clock().tick(60)

# Quit Pygame
pygame.quit()
