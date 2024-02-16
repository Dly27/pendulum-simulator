import numpy as np
import sys
import pygame

# Colours
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)


class Pendulum:
    def __init__(self, pivot_x, pivot_y, length, omega):
        self.pivot = (pivot_x, pivot_y)
        self.length = length
        self.omega = omega
        self.theta = 0
        self.offset_x = 0
        self.offset_y = 0
        self.dragging = False

    def make_pendulum(self, screen, x, y):
        pygame.draw.line(screen, WHITE, self.pivot, (self.pivot[0] + x, self.pivot[1] + y), 2)
        pygame.draw.circle(screen, RED, (self.pivot[0] + x, self.pivot[1] + y), 20)

def pendulum_equation(omega, theta, g, L, damping):
    alpha = -g / L * np.sin(theta) - (damping * omega)
    return omega, alpha

def pendulum_simulator(screen, clock, pendulums):
    g = 9.81
    dt = 0.1
    dragging = False
    damping = 0.05

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                for pendulum in pendulums:
                    mouse_pos = pygame.mouse.get_pos()
                    bob_pos = (pendulum.pivot[0] + int(pendulum.length * np.sin(pendulum.theta)),
                               pendulum.pivot[1] + int(pendulum.length * np.cos(pendulum.theta)))
                    if pygame.math.Vector2(mouse_pos).distance_to(pygame.math.Vector2(bob_pos)) < 20:
                        pendulum.dragging = True
                        pendulum.offset_x = pendulum.pivot[0] + int(pendulum.length * np.sin(pendulum.theta)) - mouse_pos[0]
                        pendulum.offset_y = pendulum.pivot[1] + int(pendulum.length * np.cos(pendulum.theta)) - mouse_pos[1]
            elif event.type == pygame.MOUSEBUTTONUP:
                for pendulum in pendulums:
                    pendulum.dragging = False

        # Pendulum dragging movement
            for pendulum in pendulums:
                if pendulum.dragging:
                    mouse_pos = pygame.mouse.get_pos()
                    dx = mouse_pos[0] + pendulum.offset_x - pendulum.pivot[0]
                    dy = mouse_pos[1] + pendulum.offset_y - pendulum.pivot[1]
                    pendulum.theta = np.arctan2(dx, dy)

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
    pendulum1 = Pendulum(630, 200, 300 ,omega=0)
    pendulum2 = Pendulum(1000, 200, 300, omega=0)
    pendulum3 = Pendulum(260, 200, 300, omega=0)
    pendulums = [pendulum1, pendulum2, pendulum3]

    # Run simulation
    pendulum_simulator(screen, clock, pendulums)


if __name__ == "__main__":
    main()
