import numpy as np
import sys
import pygame

# Colours
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)


def pendulum_equation(omega, theta, g, L, damping):
    alpha = -g / L * np.sin(theta) - (damping * omega)
    return omega, alpha

def pendulum_simulator(screen, clock, L, omega):
    theta = np.pi / 4
    g = 9.81
    dt = 0.1
    dragging = False
    damping=0.1

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Check if the mouse is clicked on the pendulum bob
                mouse_pos = pygame.mouse.get_pos()
                bob_pos = (630 + int(L * np.sin(theta)), 200 + int(L * np.cos(theta)))
                if pygame.math.Vector2(mouse_pos).distance_to(pygame.math.Vector2(bob_pos)) < 20:
                    dragging = True
                    offset_x = 630 + int(L * np.sin(theta)) - mouse_pos[0]
                    offset_y = 200 + int(L * np.cos(theta)) - mouse_pos[1]
            elif event.type == pygame.MOUSEBUTTONUP:
                dragging = False

        if dragging:
            mouse_pos = pygame.mouse.get_pos()
            dx = mouse_pos[0] + offset_x - 630
            dy = mouse_pos[1] + offset_y - 200
            theta = np.arctan2(dx, dy)

        # Calculate new state
        omega, alpha = pendulum_equation(omega, theta, g, L, damping)
        omega += alpha * dt
        theta += omega * dt

        # Convert polar into cartesian
        x = int(L * np.sin(theta))
        y = int(L * np.cos(theta))

        # Fill screen
        screen.fill(BLACK)

        # Draw pendulum string
        pygame.draw.line(screen, WHITE, (630, 200), (630 + x, 200 + y), 2)

        # Draw pendulum bob
        pygame.draw.circle(screen, RED, (630 + x, 200 + y), 20)

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

    # Run simulation
    pendulum_simulator(screen, clock, L=300, omega=0.2)


if __name__ == "__main__":
    main()
