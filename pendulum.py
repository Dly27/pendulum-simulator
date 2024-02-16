import numpy as np
import sys
import pygame

# Colours
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)


class Pendulum:
    def __init__(self, pivot_x, pivot_y, length, omega, mass):
        self.mass = mass
        self.pivot = (pivot_x, pivot_y)
        self.pivot_offset_x = 0
        self.pivot_offset_y = 0
        self.pivot_dragging = False
        self.length = length
        self.omega = omega
        self.theta = 0
        self.bob_offset_x = 0
        self.bob_offset_y = 0
        self.bob_dragging = False
        self.bob_radius = 20


    def make_pendulum(self, screen, x, y):
        pygame.draw.line(screen, WHITE, self.pivot, (self.pivot[0] + x, self.pivot[1] + y), 2)
        pygame.draw.circle(screen, RED, (self.pivot[0] + x, self.pivot[1] + y), 20)

    def bob_position(self):
        return (self.pivot[0] + int(self.length * np.sin(self.theta)), self.pivot[1] + int(self.length * np.cos(self.theta)))

# Calculate motion of pendulum
def pendulum_equation(omega, theta, g, L, damping):
    alpha = -g / L * np.sin(theta) - (damping * omega)
    return omega, alpha

def pendulum_simulator(screen, clock, pendulums):
    theta = 0
    g = 9.81
    dt = 0.1
    dragging = False
    damping = 0.05
    active_pendulum = None

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if not active_pendulum:
                    for pendulum in pendulums:
                        mouse_pos = pygame.mouse.get_pos()
                        bob_pos = (pendulum.pivot[0] + int(pendulum.length * np.sin(pendulum.theta)),
                                   pendulum.pivot[1] + int(pendulum.length * np.cos(pendulum.theta)))
                        # Check if bob is clicked
                        if pygame.math.Vector2(mouse_pos).distance_to(pygame.math.Vector2(bob_pos)) < 20:
                            active_pendulum = pendulum
                            active_pendulum.bob_dragging = True
                            active_pendulum.bob_offset_x = active_pendulum.pivot[0] + int(active_pendulum.length * np.sin(active_pendulum.theta)) - mouse_pos[0]
                            active_pendulum.bob_offset_y = active_pendulum.pivot[1] + int(active_pendulum.length * np.cos(active_pendulum.theta)) - mouse_pos[1]
                            break
                        # Check if pivot is clicked
                        elif pygame.math.Vector2(mouse_pos).distance_to(pygame.math.Vector2(pendulum.pivot)) < 40:
                            active_pendulum = pendulum
                            active_pendulum.pivot_dragging = True
                            active_pendulum.pivot_offset_x = active_pendulum.pivot[0] - mouse_pos[0]
                            active_pendulum.pivot_offset_y = active_pendulum.pivot[1] - mouse_pos[1]
                            break
            elif event.type == pygame.MOUSEBUTTONUP:
                if active_pendulum:
                    active_pendulum.bob_dragging = False
                    active_pendulum.pivot_dragging = False
                    active_pendulum = None

        # Pendulum dragging movement
        if active_pendulum:
            if active_pendulum.bob_dragging:
                mouse_pos = pygame.mouse.get_pos()
                dx = mouse_pos[0] + active_pendulum.bob_offset_x - active_pendulum.pivot[0]
                dy = mouse_pos[1] + active_pendulum.bob_offset_y - active_pendulum.pivot[1]
                active_pendulum.theta = np.arctan2(dx, dy)
            elif active_pendulum.pivot_dragging:
                mouse_pos = pygame.mouse.get_pos()
                active_pendulum.pivot = (mouse_pos[0] + active_pendulum.pivot_offset_x, mouse_pos[1] + active_pendulum.pivot_offset_y)

        # Calculate new state for each pendulum
        for pendulum in pendulums:
            omega, alpha = pendulum_equation(pendulum.omega, pendulum.theta, g, pendulum.length, damping)
            pendulum.omega += alpha * dt
            pendulum.theta += pendulum.omega * dt

        # Fill screen
        screen.fill(BLACK)

        # Make pendulums
        for pendulum in pendulums:
            pendulum.make_pendulum(screen, int(pendulum.length * np.sin(pendulum.theta)),
                                   int(pendulum.length * np.cos(pendulum.theta)))

        # Collision Detection
        for i in range(len(pendulums)):
            for j in range(i + 1, len(pendulums)):
                pendulum1 = pendulums[i]
                pendulum2 = pendulums[j]

                # Find distance between bobs
                distance = np.linalg.norm(
                    np.array(pendulum1.bob_position())
                    - np.array(pendulum2.bob_position())
                )
                if distance < pendulum1.bob_radius + pendulum2.bob_radius:
                    # Apply conservation of momentum when bob collides
                    v1 = pendulum1.omega * pendulum1.length
                    v2 = pendulum2.omega * pendulum2.length
                    m1 = pendulum1.mass
                    m2 = pendulum2.mass
                    new_v1 = (v1 * (m1 - m2) + 2 * m2 * v2) / (m1 + m2)
                    new_v2 = (v2 * (m2 - m1) + 2 * m1 * v1) / (m1 + m2)
                    pendulum1.omega = new_v1 / pendulum1.length
                    pendulum2.omega = new_v2 / pendulum2.length

        # Display update
        pygame.display.flip()

        # Framerate cap
        clock.tick(60)

def main():
    # Initialize Pygame
    pygame.init()

    # Screen setup
    screen_info = pygame.display.Info()
    screen_width, screen_height = screen_info.current_w, screen_info.current_h
    screen = pygame.display.set_mode((screen_width, screen_height - 60))
    pygame.display.set_caption("2D Pendulum Simulator")

    # Clock setup
    clock = pygame.time.Clock()

    # Create pendulums
    pendulum1 = Pendulum(630, 200, 300, omega=0, mass=0.1)
    pendulum2 = Pendulum(1000, 200, 300, omega=0, mass=0.01)
    pendulum3 = Pendulum(260, 200, 300, omega=0, mass=0.01)
    pendulums = [pendulum1, pendulum2, pendulum3]

    # Run simulation
    pendulum_simulator(screen, clock, pendulums)

if __name__ == "__main__":
    main()
