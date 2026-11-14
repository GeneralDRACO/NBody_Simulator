# edgar
import pygame
import math
pygame.init()

# Setting the background
win = pygame.display.set_mode((1400,800))
pygame.display.set_caption("Demo planets")

#defining a general class
class Planet:
    def __init__(self, name,mass, x, y, Vx, Vy, radius,color):
        self.name = name
        self.mass = mass
        self.x = x
        self.y = y
        self.Vx = Vx
        self.Vy = Vy
        self.radius = radius
        self.color = color

    def draw(self,win,font):
        pygame.draw.circle(win, self.color , (self.x,self.y), self.radius)
        Surface = font.render(self.name,True, (255,255,255))
        win.blit(Surface, (self.x + 25, self.y + 10))

    def move(self, dt,ax,ay):
        self.Vx += ax * dt
        self.Vy += ay * dt

        self.x += self.Vx * dt
        self.y += self.Vy * dt

    def calculate_gravity(self, other):
        dx = other.x - self.x
        dy = other.y - self.y
        r = math.sqrt(dx**2 + dy**2)
        if r == 0:
            return 0,0
        a = G * other.mass / r**2
        ax = a * dx/r
        ay = a * dy/r
        return ax, ay

G = 100000
# Variables for 1 planet system
Earth = Planet("Earth", 1,50, 50, -110, 110, 25, (0,0,255))
# Variables for sun
Sun = Planet("Sun",100,400,400,0,0,100,(255,0,0))
planets = [Sun,Earth]

#trajectory
trajectory=[]

font = pygame.font.SysFont(None,36)

clock = pygame.time.Clock()

run = True
while run:
    time_scale = 10
    dt = clock.tick(60) / 1000
    dt *= time_scale
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
    win.fill((0, 0, 0))
    for planet in planets:
        if planet != Sun:
            ax,ay = planet.calculate_gravity(Sun)
            planet.move(dt,ax,ay)
            trajectory.append((Earth.x, Earth.y))
        planet.draw(win,font)
    if len(trajectory) > 1:
        pygame.draw.lines(win, (255, 255, 255), False, trajectory, 2)


    pygame.display.update()

# Cleanly close the window when the loop ends
pygame.quit()

