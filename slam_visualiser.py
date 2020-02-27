#Tutorial : how to use ThorPy with a pre-existing code - step 1
import pygame
import thorpy

class Robot:
    def __init__(self, p_screen):
        self.screen = p_screen
        self.x_pos = 10
        self.y_pos = 10
        self.img = pygame.image.load("roomba.jpg")
        self.velocity = [0, 0]  # (x_vel, y_vel) pixels/tick
        self.last_velocity = [0, 0]
        self.max_velocity = 6
        self.acceleration = 1

    def update(self):
        self.move_velocity()
        self.screen.blit(self.img, (self.x_pos, self.y_pos))

    def move_velocity(self):
        if not self.collision_detector():
            self.y_pos += self.velocity[1]
            self.x_pos += self.velocity[0]
        if self.velocity == self.last_velocity:
            if self.velocity[0] > 0:
                self.velocity[0] -= self.acceleration
            elif self.velocity[0] < 0:
                self.velocity[0] += self.acceleration
            if self.velocity[1] > 0:
                self.velocity[1] -= self.acceleration
            elif self.velocity[1] < 0:
                self.velocity[1] += self.acceleration
        self.last_velocity = self.velocity

    def change_velocity(self, direction):
        if self.velocity[1] < self.max_velocity and self.velocity[1] > -self.max_velocity:
            if direction == "DOWN":
                self.velocity[1] += self.acceleration * 2
            elif direction == "UP":
                self.velocity[1] -= self.acceleration * 2
        if self.velocity[0] < self.max_velocity and self.velocity[0] > -self.max_velocity:
            if direction == "LEFT":
                self.velocity[0] -= self.acceleration * 2
            elif direction == "RIGHT":
                self.velocity[0] += self.acceleration * 2
        self.last_velocity = [0, 0]

    def collision_detector(self):
        return False

    def get_size(self):
        return (self.img.get_width(), self.img.get_height())

pygame.init()
pygame.key.set_repeat(300, 30)
screen = pygame.display.set_mode((400, 400))
screen.fill((255, 255, 255))
rect = pygame.Rect((0, 0, 50, 50))
rect.center = screen.get_rect().center
clock = pygame.time.Clock()

background = pygame.Surface(screen.get_size())
background = background.convert()
background.fill((250, 250, 250))

pygame.draw.rect(screen, (255, 0, 0), rect)
pygame.display.flip()

#declaration of some ThorPy elements ...
slider = thorpy.SliderX(100, (12, 35), "My Slider")
button = thorpy.make_button("Quit", func=thorpy.functions.quit_func)
box = thorpy.Box(elements=[slider, button])
#we regroup all elements on a menu, even if we do not launch the menu
menu = thorpy.Menu(box)
#important : set the screen as surface for all elements
for element in menu.get_population():
    element.surface = screen
#use the elements normally...
box.set_topleft((100, 100))
box.blit()
box.update()

robot = Robot(screen)
robot.update()

playing_game = True
while playing_game:
    clock.tick(45)
    screen.blit(background, (0, 0))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            playing_game = False
            break
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_DOWN:
                robot.change_velocity("DOWN")
            if event.key == pygame.K_UP:
                robot.change_velocity("UP")
            if event.key == pygame.K_LEFT:
                robot.change_velocity("LEFT")
            if event.key == pygame.K_RIGHT:
                robot.change_velocity("RIGHT")

        menu.react(event) #the menu automatically integrate your elements
    robot.update()
    pygame.display.update()

pygame.quit()
