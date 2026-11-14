import math
import pygame

pygame.init()

# ============================================================
# DISPLAY SETTINGS
# ============================================================

WIDTH = 1400
HEIGHT = 800
FPS = 60

win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("N-Body Gravitational Simulator")

font = pygame.font.SysFont(None, 24)
clock = pygame.time.Clock()

# ============================================================
# SIMULATION SETTINGS
# ============================================================

# Astronomical simulation units:
# distance = AU, mass = solar masses, time = years,
# velocity = AU/year. In these units G = 4*pi^2.
G = 4 * math.pi**2

# Fixed physics timestep: one simulated day, expressed in years.
PHYSICS_DT = 1 / 365.25

# Simulate one year for every 30 seconds of real time.
# This converts rendering seconds into simulation years.
SIMULATION_YEARS_PER_SECOND = 1 / 30

# Limit stored trajectory points to prevent unlimited memory growth.
MAX_TRAIL_POINTS = 3000


# ============================================================
# PLANET CLASS
# ============================================================

class Planet:
    def __init__(self, name, mass, x, y, vx, vy, radius, color):
        self.name = name
        self.mass = mass

        # World position
        self.x = float(x)
        self.y = float(y)

        # Velocity
        self.Vx = float(vx)
        self.Vy = float(vy)

        self.radius = radius
        self.color = color

    def draw(self, surface, camera):
        screen_x, screen_y = camera.world_to_screen(self.x, self.y)
        screen_radius = max(2, int(self.radius * camera.zoom))

        pygame.draw.circle(
            surface,
            self.color,
            (int(screen_x), int(screen_y)),
            screen_radius
        )

        label = font.render(self.name, True, (255, 255, 255))
        surface.blit(
            label,
            (int(screen_x + screen_radius + 5), int(screen_y))
        )


# ============================================================
# CAMERA CLASS
# ============================================================

class Camera:
    def __init__(self):
        # View the Solar System barycentre.
        self.x = 0.0
        self.y = 0.0
        self.zoom = 40.0

        self.dragging = False
        self.last_mouse_pos = (0, 0)

    def world_to_screen(self, x, y):
        screen_x = (x - self.x) * self.zoom + WIDTH / 2
        screen_y = (y - self.y) * self.zoom + HEIGHT / 2
        return screen_x, screen_y

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 2:
                self.dragging = True
                self.last_mouse_pos = event.pos

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 2:
                self.dragging = False

        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                mouse_x, mouse_y = event.pos
                dx = mouse_x - self.last_mouse_pos[0]
                dy = mouse_y - self.last_mouse_pos[1]

                self.x -= dx / self.zoom
                self.y -= dy / self.zoom

                self.last_mouse_pos = event.pos

        elif event.type == pygame.MOUSEWHEEL:
            if event.y > 0:
                self.zoom *= 1.1
            elif event.y < 0:
                self.zoom /= 1.1

            self.zoom = max(5.0, min(self.zoom, 100.0))


# ============================================================
# N-BODY GRAVITATIONAL ACCELERATION
# ============================================================

def calculate_accelerations(states, planets):
    """
    Calculate the acceleration of every body due to every other body.

    states[i] = (x, y, vx, vy)
    Planet masses are read from the corresponding Planet objects.
    """

    accelerations = []

    for i, planet in enumerate(planets):
        x, y, _, _ = states[i]

        ax = 0.0
        ay = 0.0

        for j, other in enumerate(planets):
            if i == j:
                continue

            other_x, other_y, _, _ = states[j]

            dx = other_x - x
            dy = other_y - y

            r_squared = dx * dx + dy * dy

            # Avoid division by zero if two bodies occupy
            # exactly the same position.
            if r_squared == 0:
                continue

            r = math.sqrt(r_squared)

            # G * m / r^3, multiplied by the displacement vector.
            factor = G * other.mass / (r_squared * r)

            ax += factor * dx
            ay += factor * dy

        accelerations.append((ax, ay))

    return accelerations


# ============================================================
# RK4 DERIVATIVES AND STATE OPERATIONS
# ============================================================

def derivatives(states, planets):
    accelerations = calculate_accelerations(states, planets)
    result = []

    for i, state in enumerate(states):
        x, y, vx, vy = state
        ax, ay = accelerations[i]

        # d(position)/dt = velocity
        # d(velocity)/dt = acceleration
        result.append((vx, vy, ax, ay))

    return result


def add_scaled_states(states, derivative_values, scale):
    result = []

    for state, derivative in zip(states, derivative_values):
        result.append(tuple(
            value + scale * change
            for value, change in zip(state, derivative)
        ))

    return result


def rk4_step(planets, dt):
    """Advance the complete coupled N-body system by one RK4 step."""

    states = [
        (planet.x, planet.y, planet.Vx, planet.Vy)
        for planet in planets
    ]

    # Stage 1
    k1 = derivatives(states, planets)

    # Stage 2
    states_2 = add_scaled_states(states, k1, dt / 2)
    k2 = derivatives(states_2, planets)

    # Stage 3
    states_3 = add_scaled_states(states, k2, dt / 2)
    k3 = derivatives(states_3, planets)

    # Stage 4
    states_4 = add_scaled_states(states, k3, dt)
    k4 = derivatives(states_4, planets)

    # Combine the four stages and update all bodies.
    for i, planet in enumerate(planets):
        x, y, vx, vy = states[i]

        dx1, dy1, dvx1, dvy1 = k1[i]
        dx2, dy2, dvx2, dvy2 = k2[i]
        dx3, dy3, dvx3, dvy3 = k3[i]
        dx4, dy4, dvx4, dvy4 = k4[i]

        planet.x = x + (dt / 6) * (
            dx1 + 2 * dx2 + 2 * dx3 + dx4
        )

        planet.y = y + (dt / 6) * (
            dy1 + 2 * dy2 + 2 * dy3 + dy4
        )

        planet.Vx = vx + (dt / 6) * (
            dvx1 + 2 * dvx2 + 2 * dvx3 + dvx4
        )

        planet.Vy = vy + (dt / 6) * (
            dvy1 + 2 * dvy2 + 2 * dvy3 + dvy4
        )


# ============================================================
# INITIAL CONDITIONS - APPROXIMATE 2D SOLAR SYSTEM
# ============================================================

# Approximate mean orbital radii and planetary-to-solar mass ratios.

Sun = Planet("Sun", 1.0, 0, 0, 0, 0, 0.12, (255, 190, 40))

def make_planet(name, mass, orbital_radius, angle_deg, draw_radius, color):
    theta = math.radians(angle_deg)

    x = orbital_radius * math.cos(theta)
    y = orbital_radius * math.sin(theta)

    # Approximate circular heliocentric speed.
    speed = math.sqrt(G * (Sun.mass + mass) / orbital_radius)
    vx = -speed * math.sin(theta)
    vy = speed * math.cos(theta)

    return Planet(
        name, mass, x, y, vx, vy, draw_radius, color
    )


Venus = make_planet(
    "Venus", 2.447e-6, 0.723, 35, 0.045, (255, 190, 100)
)
Earth = make_planet(
    "Earth", 3.003e-6, 1.000, 115, 0.050, (70, 130, 255)
)
Mars = make_planet(
    "Mars", 3.227e-7, 1.524, 205, 0.040, (230, 90, 55)
)
Jupiter = make_planet(
    "Jupiter", 9.545e-4, 5.203, 285, 0.090, (205, 165, 120)
)
Saturn = make_planet(
    "Saturn", 2.858e-4, 9.537, 330, 0.080, (220, 195, 135)
)

planets = [Sun, Venus, Earth, Mars, Jupiter, Saturn]

# Shift the initial state to the Solar System barycentric frame.
# This preserves relative positions/velocities while making total
# centre-of-mass position and momentum zero.
total_mass = sum(planet.mass for planet in planets)

com_x = sum(p.mass * p.x for p in planets) / total_mass
com_y = sum(p.mass * p.y for p in planets) / total_mass
com_vx = sum(p.mass * p.Vx for p in planets) / total_mass
com_vy = sum(p.mass * p.Vy for p in planets) / total_mass

for planet in planets:
    planet.x -= com_x
    planet.y -= com_y
    planet.Vx -= com_vx
    planet.Vy -= com_vy


# ============================================================
# TRAJECTORY STORAGE AND CAMERA
# ============================================================

trajectories = {planet.name: [] for planet in planets}
camera = Camera()


# MAIN LOOP


run = True
accumulator = 0.0

while run:
    # Rendering frame time, in real seconds.
    frame_time = clock.tick(FPS) / 1000.0

    frame_time = min(frame_time, 0.05)

    accumulator += frame_time * SIMULATION_YEARS_PER_SECOND

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

        camera.handle_event(event)

    # --------------------------------------------------------
    # UPDATE PHYSICS AT A FIXED TIMESTEP
    # --------------------------------------------------------

    while accumulator >= PHYSICS_DT:
        rk4_step(planets, PHYSICS_DT)
        accumulator -= PHYSICS_DT

    # --------------------------------------------------------
    # STORE TRAJECTORIES
    # --------------------------------------------------------

    for planet in planets:
        trail = trajectories[planet.name]
        trail.append((planet.x, planet.y))

        if len(trail) > MAX_TRAIL_POINTS:
            del trail[0]

    # --------------------------------------------------------
    # DRAW BACKGROUND
    # --------------------------------------------------------

    win.fill((0, 0, 0))

    # --------------------------------------------------------
    # DRAW TRAJECTORIES
    # --------------------------------------------------------

    for planet in planets:
        trajectory = trajectories[planet.name]

        if len(trajectory) > 1:
            screen_trajectory = [
                tuple(map(int, camera.world_to_screen(x, y)))
                for x, y in trajectory
            ]

            pygame.draw.lines(
                win,
                planet.color,
                False,
                screen_trajectory,
                2
            )

    # --------------------------------------------------------
    # DRAW PLANETS
    # --------------------------------------------------------

    for planet in planets:
        planet.draw(win, camera)

    # --------------------------------------------------------
    # DISPLAY CAMERA AND SIMULATION INFORMATION
    # --------------------------------------------------------

    info_text = font.render(
        f"Camera: ({camera.x:.1f}, {camera.y:.1f})   "
        f"Zoom: {camera.zoom:.2f}x   "
        f"Physics dt: {PHYSICS_DT:.5f} yr   "
        f"Sim speed: {SIMULATION_YEARS_PER_SECOND:.4f} yr/s",
        True,
        (255, 255, 255)
    )

    win.blit(info_text, (20, 20))

    pygame.display.flip()


pygame.quit()
