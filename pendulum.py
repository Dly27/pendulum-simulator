import numpy as np
import sys
import pygame

# Constants
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

    def draw(self, screen):
        pygame.draw.line(screen, WHITE, self.pivot, self.bob_position(), 2)
        pygame.draw.circle(screen, RED, self.bob_position(), self.bob_radius)

    def bob_position(self):
        return (
            self.pivot[0] + int(self.length * np.sin(self.theta)),
            self.pivot[1] + int(self.length * np.cos(self.theta)),
        )


class Simulator:
    def __init__(self, screen, clock, pendulums):
        self.screen = screen
        self.clock = clock
        self.pendulums = pendulums
        self.active_pendulum = None
        self.g = 9.81
        self.dt = 0.1
        self.damping = 0.05

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.handle_mouse_down(event)
            elif event.type == pygame.MOUSEBUTTONUP:
                self.handle_mouse_up(event)

    def handle_mouse_down(self, event):
        if not self.active_pendulum:
            for pendulum in self.pendulums:
                mouse_pos = pygame.mouse.get_pos()
                bob_pos = pendulum.bob_position()
                if np.linalg.norm(np.array(mouse_pos) - np.array(bob_pos)) < pendulum.bob_radius:
                    self.active_pendulum = pendulum
                    self.active_pendulum.bob_dragging = True
                    self.active_pendulum.bob_offset_x = bob_pos[0] - mouse_pos[0]
                    self.active_pendulum.bob_offset_y = bob_pos[1] - mouse_pos[1]
                    break
                elif np.linalg.norm(np.array(mouse_pos) - np.array(pendulum.pivot)) < 40:
                    self.active_pendulum = pendulum
                    self.active_pendulum.pivot_dragging = True
                    self.active_pendulum.pivot_offset_x = pendulum.pivot[0] - mouse_pos[0]
                    self.active_pendulum.pivot_offset_y = pendulum.pivot[1] - mouse_pos[1]

    def handle_mouse_up(self, event):
        if self.active_pendulum:
            self.active_pendulum.bob_dragging = False
            self.active_pendulum.pivot_dragging = False
            self.active_pendulum = None

    def update_pendulums(self):
        for pendulum in self.pendulums:
            omega, alpha = self.pendulum_equation(
                pendulum.omega, pendulum.theta, self.g, pendulum.length, self.damping
            )
            pendulum.omega += alpha * self.dt
            pendulum.theta += pendulum.omega * self.dt

    @staticmethod
    def pendulum_equation(omega, theta, g, L, damping):
        alpha = -g / L * np.sin(theta) - (damping * omega)
        return omega, alpha

    def handle_dragging(self):
        if self.active_pendulum:
            if self.active_pendulum.bob_dragging:
                mouse_pos = pygame.mouse.get_pos()
                dx = mouse_pos[0] + self.active_pendulum.bob_offset_x - self.active_pendulum.pivot[0]
                dy = mouse_pos[1] + self.active_pendulum.bob_offset_y - self.active_pendulum.pivot[1]
                self.active_pendulum.theta = np.arctan2(dx, dy)
            elif self.active_pendulum.pivot_dragging:
                mouse_pos = pygame.mouse.get_pos()
                self.active_pendulum.pivot = (
                    mouse_pos[0] + self.active_pendulum.pivot_offset_x,
                    mouse_pos[1] + self.active_pendulum.pivot_offset_y,
                )

    def check_collisions(self):
        for i in range(len(self.pendulums)):
            for j in range(i + 1, len(self.pendulums)):
                pendulum1 = self.pendulums[i]
                pendulum2 = self.pendulums[j]
                distance = np.linalg.norm(
                    np.array(pendulum1.bob_position())
                    - np.array(pendulum2.bob_position())
                )
                if distance < pendulum1.bob_radius + pendulum2.bob_radius:
                    v1 = pendulum1.omega * pendulum1.length
                    v2 = pendulum2.omega * pendulum2.length
                    m1 = pendulum1.mass
                    m2 = pendulum2.mass
                    new_v1 = (v1 * (m1 - m2) + 2 * m2 * v2) / (m1 + m2)
                    new_v2 = (v2 * (m2 - m1) + 2 * m1 * v1) / (m1 + m2)
                    pendulum1.omega = new_v1 / pendulum1.length
                    pendulum2.omega = new_v2 / pendulum2.length

    def run(self):
        while True:
            self.handle_events()
            self.handle_dragging()
            self.update_pendulums()
            self.screen.fill(BLACK)
            for pendulum in self.pendulums:
                pendulum.draw(self.screen)
            self.check_collisions()
            pygame.display.flip()
            self.clock.tick(60)


def main():
    pygame.init()
    screen_info = pygame.display.Info()
    screen_width, screen_height = screen_info.current_w, screen_info.current_h
    screen = pygame.display.set_mode((screen_width, screen_height - 60))
    pygame.display.set_caption("2D Pendulum Simulator")
    clock = pygame.time.Clock()

    pendulum1 = Pendulum(630, 200, 300, omega=0, mass=0.1)
    pendulum2 = Pendulum(1000, 200, 300, omega=0, mass=0.01)
    pendulum3 = Pendulum(260, 200, 300, omega=0, mass=0.01)
    pendulums = [pendulum1, pendulum2, pendulum3]

    simulator = Simulator(screen, clock, pendulums)
    simulator.run()


if __name__ == "__main__":
    main()
