import pygame
import math

pygame.init()


WIDTH = 1400
HEIGHT = 800

win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("N-Body Simulator")


G = 100000

font = pygame.font.SysFont(None, 28)
clock = pygame.time.Clock()



# PLANET CLASS


class Planet:

    def __init__(self, name, mass, x, y, Vx, Vy, radius, color):

        self.name = name
        self.mass = mass

        # WORLD POSITION
        self.x = x
        self.y = y

        # VELOCITY
        self.Vx = Vx
        self.Vy = Vy

        self.radius = radius
        self.color = color

    def calculate_acceleration(self, planets):

        total_ax = 0
        total_ay = 0

        for other in planets:

            if other == self:
                continue

            dx = other.x - self.x
            dy = other.y - self.y

            r_squared = dx**2 + dy**2

            if r_squared == 0:
                continue

            r = math.sqrt(r_squared)

            # Magnitude of acceleration
            a = G * other.mass / r_squared

            # Direction
            ax = a * dx / r
            ay = a * dy / r

            # Add contribution
            total_ax += ax
            total_ay += ay

        return total_ax, total_ay



    # MOVE PLANET
    def move(self, dt, ax, ay):


        self.Vx += ax * dt
        self.Vy += ay * dt


        self.x += self.Vx * dt
        self.y += self.Vy * dt



    # DRAW PLANET
    def draw(self, win, camera):

        # Convert WORLD coordinates → SCREEN coordinates
        screen_x, screen_y = camera.world_to_screen(
            self.x,
            self.y
        )

        # Scale radius with zoom
        screen_radius = max(
            2,
            int(self.radius * camera.zoom)
        )

        # Draw planet
        pygame.draw.circle(
            win,
            self.color,
            (int(screen_x), int(screen_y)),
            screen_radius
        )

        # Draw name
        text = font.render(
            self.name,
            True,
            (255, 255, 255)
        )

        win.blit(
            text,
            (
                int(screen_x + screen_radius + 5),
                int(screen_y)
            )
        )


# ============================================================
# CAMERA CLASS
# ============================================================

class Camera:

    def __init__(self):

        # Position of camera in WORLD coordinates
        self.x = 500
        self.y = 400

        # Zoom
        self.zoom = 2.0

        # Mouse dragging
        self.dragging = False
        self.last_mouse_pos = (0, 0)


    # --------------------------------------------------------
    # WORLD → SCREEN
    # --------------------------------------------------------

    def world_to_screen(self, x, y):

        screen_x = (x - self.x) * self.zoom + WIDTH / 2
        screen_y = (y - self.y) * self.zoom + HEIGHT / 2

        return screen_x, screen_y


    # --------------------------------------------------------
    # HANDLE CAMERA EVENTS
    # --------------------------------------------------------

    def handle_event(self, event):

        # Start dragging
        if event.type == pygame.MOUSEBUTTONDOWN:

            if event.button == 2:

                self.dragging = True

                self.last_mouse_pos = event.pos


        # Stop dragging
        elif event.type == pygame.MOUSEBUTTONUP:

            if event.button == 2:

                self.dragging = False


        # Drag camera
        elif event.type == pygame.MOUSEMOTION:

            if self.dragging:

                mouse_x, mouse_y = event.pos

                dx = mouse_x - self.last_mouse_pos[0]
                dy = mouse_y - self.last_mouse_pos[1]

                # Move camera opposite to mouse
                self.x -= dx / self.zoom
                self.y -= dy / self.zoom

                self.last_mouse_pos = event.pos


        # Zoom
        elif event.type == pygame.MOUSEWHEEL:

            # Zoom in
            if event.y > 0:

                self.zoom *= 1.1

            # Zoom out
            elif event.y < 0:

                self.zoom /= 1.1

            # Prevent ridiculous zoom
            self.zoom = max(
                0.1,
                min(self.zoom, 5.0)
            )

# ============================================================
# RK4 INTEGRATOR
# ============================================================

def calculate_accelerations(states, planets):

    accelerations = []

    for i, planet in enumerate(planets):

        x, y, vx, vy = states[i]

        ax = 0.0
        ay = 0.0

        for j, other in enumerate(planets):

            if i == j:
                continue

            other_x, other_y, _, _ = states[j]

            dx = other_x - x
            dy = other_y - y

            r_squared = dx**2 + dy**2

            if r_squared == 0:
                continue

            r = math.sqrt(r_squared)

            # Gravitational acceleration
            factor = G * other.mass / (r_squared * r)

            ax += factor * dx
            ay += factor * dy

        accelerations.append((ax, ay))

    return accelerations


def derivatives(states, planets):

    accelerations = calculate_accelerations(states, planets)

    result = []

    for i, state in enumerate(states):

        x, y, vx, vy = state
        ax, ay = accelerations[i]

        # Derivatives of (x, y, vx, vy)
        result.append((vx, vy, ax, ay))

    return result


def add_scaled_states(states, derivatives_list, scale):

    result = []

    for state, derivative in zip(states, derivatives_list):

        result.append(tuple(
            value + scale * change
            for value, change in zip(state, derivative)
        ))

    return result


def rk4_step(planets, dt):

    # Initial state of every planet
    states = [
        (planet.x, planet.y, planet.Vx, planet.Vy)
        for planet in planets
    ]

    # Stage 1
    k1 = derivatives(states, planets)

    # Stage 2
    states_2 = add_scaled_states(
        states, k1, dt / 2
    )

    k2 = derivatives(states_2, planets)

    # Stage 3
    states_3 = add_scaled_states(
        states, k2, dt / 2
    )

    k3 = derivatives(states_3, planets)

    # Stage 4
    states_4 = add_scaled_states(
        states, k3, dt
    )

    k4 = derivatives(states_4, planets)

    # Combine all four stages
    for i, planet in enumerate(planets):

        x, y, vx, vy = states[i]

        dx1, dy1, dvx1, dvy1 = k1[i]
        dx2, dy2, dvx2, dvy2 = k2[i]
        dx3, dy3, dvx3, dvy3 = k3[i]
        dx4, dy4, dvx4, dvy4 = k4[i]

        planet.x = x + (dt / 6) * (
            dx1 + 2*dx2 + 2*dx3 + dx4
        )

        planet.y = y + (dt / 6) * (
            dy1 + 2*dy2 + 2*dy3 + dy4
        )

        planet.Vx = vx + (dt / 6) * (
            dvx1 + 2*dvx2 + 2*dvx3 + dvx4
        )

        planet.Vy = vy + (dt / 6) * (
            dvy1 + 2*dvy2 + 2*dvy3 + dvy4
        )

Sun = Planet("Sun", 100, 400, 400, 0, 0, 40, (255, 0, 0))
Earth = Planet("Earth", 1, 50, 50, -110, 110, 10, (0, 0, 255))
Mars = Planet("Mars", 0.5, 700, 200, 0, 100, 8, (255, 100, 0))
Jupiter = Planet("Jupiter", 10, 900, 500, -50, 80, 15, (200, 150, 100))
Venus = Planet("Venus", 0.8, -300, 300, 80, -50, 9, (255, 200, 100))
Saturn = Planet("Saturn", 5, 1500, 700, -70, -40, 14, (220, 180, 100))

planets = [Sun, Earth, Mars, Jupiter, Venus, Saturn]


trajectories = {}

for planet in planets:

    trajectories[planet.name] = []


camera = Camera()


run = True

while run:

    time_scale = 1

    dt = clock.tick(60) / 1000

    dt *= time_scale

    for event in pygame.event.get():

        if event.type == pygame.QUIT:

            run = False

        camera.handle_event(event)


    win.fill((0, 0, 0))

    # ============================================================
    # UPDATE PHYSICS USING RK4
    # ============================================================

    rk4_step(planets, dt)

    # STORE TRAJECTORIES

    for planet in planets:
        trajectories[planet.name].append(
            (planet.x, planet.y)
        )



    # ========================================================
    # DRAW TRAJECTORIES
    # ========================================================

    for planet in planets:

        trajectory = trajectories[planet.name]

        if len(trajectory) > 1:

            screen_trajectory = []

            for x, y in trajectory:

                screen_x, screen_y = camera.world_to_screen(
                    x,
                    y
                )

                screen_trajectory.append(
                    (
                        int(screen_x),
                        int(screen_y)
                    )
                )


            pygame.draw.lines(
                win,
                planet.color,
                False,
                screen_trajectory,
                2
            )


    # ========================================================
    # DRAW PLANETS
    # ========================================================

    for planet in planets:

        planet.draw(
            win,
            camera
        )


    # ========================================================
    # DISPLAY CAMERA INFO
    # ========================================================

    camera_text = font.render(
        f"Camera: ({camera.x:.1f}, {camera.y:.1f})   "
        f"Zoom: {camera.zoom:.2f}x",
        True,
        (255, 255, 255)
    )

    win.blit(
        camera_text,
        (20, 20)
    )



    pygame.display.update()


pygame.quit()