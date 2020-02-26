#Tutorial : how to use ThorPy with a pre-existing code - step 1
import pygame
import thorpy

class Robot:
    def __init__(self, p_screen):
        self.screen = p_screen
        self.x_pos = 10
        self.y_pos = 10
        self.img = pygame.image.load("roomba.jpg")
        self.velocity = 2  # pixels/input

    def update(self):
        self.screen.blit(self.img, (self.x_pos, self.y_pos))

    def move_distance(self, distance, direction):
        if direction == "DOWN":
            if self.y_pos < self.screen.get_height() - self.get_size()[1]:
                self.y_pos += distance
        elif direction == "UP":
            if self.y_pos > 0:
                self.y_pos -= distance
        elif direction == "LEFT":
            if self.x_pos > 0:
                self.x_pos -= distance
        elif direction == "RIGHT":
            if self.x_pos < self.screen.get_width() - self.get_size()[0]:
                self.x_pos += distance

    def get_size(self):
        return (50, 50)

pygame.init()
pygame.key.set_repeat(300, 30)
screen = pygame.display.set_mode((400, 400))
screen.fill((255, 255, 255))
rect = pygame.Rect((0, 0, 50, 50))
rect.center = screen.get_rect().center
clock = pygame.time.Clock()

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
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            playing_game = False
            break
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_DOWN:
                robot.move_distance(1 * robot.velocity, "DOWN")
            elif event.key == pygame.K_UP:
                robot.move_distance(1 * robot.velocity, "UP")
            elif event.key == pygame.K_LEFT:
                robot.move_distance(1 * robot.velocity, "LEFT")
            elif event.key == pygame.K_RIGHT:
                robot.move_distance(1 * robot.velocity, "RIGHT")

        menu.react(event) #the menu automatically integrate your elements
    robot.update()
    pygame.display.update()

pygame.quit()
