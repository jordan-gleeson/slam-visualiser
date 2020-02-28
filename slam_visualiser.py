#Tutorial : how to use ThorPy with a pre-existing code - step 1
import pygame
import time
import thorpy

class Robot:
    def __init__(self, p_screen):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load("roomba.png")
        self.og_image = self.image
        self.rect = self.image.get_rect()
        self.screen = p_screen
        self.velocity = [0, 0]  # (x_vel, y_vel) pixels/tick
        self.last_velocity = [0, 0]
        self.max_velocity = 6
        self.acceleration = 1
        self.cur_keys = []
        self.direction = 0

    def update(self):
        # self.move_velocity()
        # self.screen.blit(self.image, (self.rect.x, self.rect.y))
        pass

    def rotate(self):
        self.direction += 1
        if self.direction > 180:
            self.direction = -180
        if self.direction < -180:
            self.direction = 180
        self.image = pygame.transform.rotate(self.og_image, self.direction)
        # rot_rect = self.image.get_rect(center=rect.center)
        self.screen.blit(self.image, (self.rect.x, self.rect.y))
        # rotated_image = pygame.transform.rotate(self.og_image, self.direction)
        # new_rect = rotated_image.get_rect(center = self.og_image.get_rect(topleft = topleft).center)

        # self.screen.blit(rotated_image, new_rect.topleft)
        print(self.image.get_size())

    def move_velocity(self):
        self.rect.y += self.velocity[1]
        self.rect.x += self.velocity[0]
        if "RIGHT" not in self.cur_keys:
            if self.velocity[0] > 0:
                self.velocity[0] -= self.acceleration
        if "LEFT" not in self.cur_keys:
            if self.velocity[0] < 0:
                self.velocity[0] += self.acceleration
        if "DOWN" not in self.cur_keys:
            if self.velocity[1] > 0:
                self.velocity[1] -= self.acceleration
        if "UP" not in self.cur_keys:
            if self.velocity[1] < 0:
                self.velocity[1] += self.acceleration

    def change_velocity(self, keys):
        direction = self.convert_key(keys)
        if self.velocity[1] < self.max_velocity and self.velocity[1] > -self.max_velocity:
            if "DOWN" in direction:
                self.velocity[1] += self.acceleration * 2
            if "UP" in direction:
                self.velocity[1] -= self.acceleration * 2
        if self.velocity[0] < self.max_velocity and self.velocity[0] > -self.max_velocity:
            if "LEFT" in direction:
                self.velocity[0] -= self.acceleration * 2
            if "RIGHT" in direction:
                self.velocity[0] += self.acceleration * 2

    def convert_key(self, keys):
        _action = False
        _keys_to_check = [[pygame.K_LEFT, "LEFT"], [pygame.K_RIGHT, "RIGHT"], [pygame.K_UP, "UP"], [pygame.K_DOWN, "DOWN"]]
        for i in range(len(_keys_to_check)):
            if keys[_keys_to_check[i][0]]:
                if _keys_to_check[i][1] not in self.cur_keys:
                    self.cur_keys.append(_keys_to_check[i][1])
                _action = True
            else:
                try:
                    self.cur_keys.remove(_keys_to_check[i][1])
                except ValueError:
                    pass

        if _action:
            self.cur_keys = self.cur_keys[-2:]
        else:
            self.cur_keys = []

        print(self.cur_keys)
        return self.cur_keys

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

gui = pygame.Surface(screen.get_size())
gui.set_alpha(0)

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
    box.blit()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            playing_game = False
            break
        if event.type == pygame.KEYDOWN:
            pass
        menu.react(event) #the menu automatically integrate your elements
    # print(pygame.key.get_pressed())
    robot.rotate()
    # robot.change_velocity(pygame.key.get_pressed())
    robot.update()
    pygame.display.update()

pygame.quit()
