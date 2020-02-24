#Tutorial : how to use ThorPy with a pre-existing code - step 1
import pygame, thorpy

class Robot:
    def __init__(self, screen):
        self.screen = screen
        self.x = 10
        self.y = 10
        self.img = pygame.image.load("roomba.jpg")
        self.velocity = 2  # pixels/input

    def update(self):
        self.screen.blit(self.img, (self.x, self.y))

    def move_distance(self, distance, direction):
        if direction == "DOWN":
            if self.y < self.screen.get_height() - self.get_size()[1]:
                self.y += distance
        elif direction == "UP":
            if self.y > 0:
                self.y -= distance
        elif direction == "LEFT":
            if self.x > 0:
                self.x -= distance
        elif direction == "RIGHT":
            if self.x < self.screen.get_width() - self.get_size()[0]:
                self.x += distance

    def get_size(self):
        return (50, 50)

pygame.init()
pygame.key.set_repeat(300, 30)
screen = pygame.display.set_mode((400,400))
screen.fill((255,255,255))
rect = pygame.Rect((0, 0, 50, 50))
rect.center = screen.get_rect().center
clock = pygame.time.Clock()

pygame.draw.rect(screen, (255,0,0), rect)
pygame.display.flip()

#declaration of some ThorPy elements ...
slider = thorpy.SliderX(100, (12, 35), "My Slider")
button = thorpy.make_button("Quit", func=thorpy.functions.quit_func)
box = thorpy.Box(elements=[slider,button])
#we regroup all elements on a menu, even if we do not launch the menu
menu = thorpy.Menu(box)
#important : set the screen as surface for all elements
for element in menu.get_population():
    element.surface = screen
#use the elements normally...
box.set_topleft((100,100))
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
        elif event.type == pygame.KEYDOWN:
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