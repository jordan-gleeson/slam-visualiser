# Tutorial : how to use ThorPy with a pre-existing code - step 1
import pygame
from pygame.math import Vector2
import numpy as np


class Robot:
    def __init__(self, p_screen, p_world):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load("roomba.png")
        self.image = pygame.transform.smoothscale(self.image, (50, 50))
        self.image_size = self.image.get_size()
        self.og_image = self.image.copy()
        self.rect = self.image.get_rect()
        self.screen = p_screen
        self.world = p_world
        self.x_pos = float(self.screen.get_size()[0] / 2)
        self.y_pos = float(self.screen.get_size()[1] / 2)
        self.rect.center = (self.x_pos, self.y_pos)
        self.velocity = [0, 0, 0]  # (x_vel, y_vel) pixels/tick
        self.max_velocity = 2
        self.acceleration = 0.5
        self.cur_keys = []
        self.direction = 0
        self.angular_velocity = 4
        self.hitbox = pygame.Rect(self.x_pos - (self.image_size[0] / 2),
                                  self.y_pos - (self.image_size[1] / 2),
                                  self.image_size[0],
                                  self.image_size[1])

    def reset(self):
        print("Reseting...")
        self.x_pos = self.screen.get_size()[0] / 2
        self.y_pos = self.screen.get_size()[1] / 2
        self.rect.center = (self.x_pos, self.y_pos)
        self.velocity = [0, 0, 0]
        self.direction = 0
        self.update()

    def update(self):
        self.move_velocity()
        self.rotate()
        self.rect.center = (self.x_pos, self.y_pos)
        self.hitbox.x = self.x_pos - (self.image_size[0] / 2)
        self.hitbox.y = self.y_pos - (self.image_size[1] / 2)
        self.collision_detector()
        pygame.draw.rect(screen, (0, 255, 0), self.hitbox)
        self.screen.blit(self.image, self.rect)

    def rotate(self):
        self.image = pygame.transform.rotate(self.og_image, self.direction)
        self.rect = self.image.get_rect()
        self.rect.center = (self.x_pos, self.y_pos)

    def move_velocity(self):
        deceleration = self.acceleration / 2
        collision_list = self.collision_detector()

        if len(collision_list) == 0:
            self.x_pos += self.velocity[0]
            self.y_pos += self.velocity[1]
            self.rect.center = (self.x_pos, self.y_pos)
        else:
            closest_distance = 100
            closest_iterator = 0
            for i in range(len(collision_list)):
                distance = np.sqrt(np.square(self.world.wall_list[collision_list[i]].center[0] - self.x_pos) + np.square(
                    self.world.wall_list[collision_list[i]].center[1] - self.y_pos))
                if distance < closest_distance:
                    closest_distance = distance
                    closest_iterator = i
            wall = self.world.wall_list[collision_list[closest_iterator]]
            pygame.draw.rect(self.screen, (255, 0, 0), wall)
            if wall.bottom > self.y_pos - self.image_size[1] / 2 and wall.top < self.y_pos - self.image_size[1] / 2:
                self.y_pos = wall.bottom + self.image_size[1] / 2
            elif wall.top < self.y_pos + self.image_size[1] / 2 and wall.bottom > self.y_pos + self.image_size[1] / 2:
                self.y_pos = wall.top - self.image_size[1] / 2
            if wall.right > self.x_pos - self.image_size[0] / 2 and wall.left < self.x_pos - self.image_size[0] / 2:
                self.x_pos = wall.right + self.image_size[0] / 2
            elif wall.left < self.x_pos + self.image_size[0] / 2 and wall.right > self.x_pos + self.image_size[0] / 2:
                self.x_pos = wall.left - self.image_size[0] / 2

        if "UP" not in self.cur_keys:
            if self.velocity[0] > 0:
                self.velocity[0] -= deceleration
            if self.velocity[0] < 0:
                self.velocity[0] += deceleration
            if self.velocity[1] > 0:
                self.velocity[1] -= deceleration
            if self.velocity[1] < 0:
                self.velocity[1] += deceleration
            if self.velocity[0] < deceleration and self.velocity[0] > deceleration * -1:
                self.velocity[0] = 0
            if self.velocity[1] < deceleration and self.velocity[1] > deceleration * -1:
                self.velocity[1] = 0

    def change_velocity(self, keys):
        pressed_keys = self.convert_key(keys)
        if "R" in pressed_keys:
            self.reset()
        if "RIGHT" in pressed_keys:
            self.direction -= 1 * self.angular_velocity
        if "LEFT" in pressed_keys:
            self.direction += 1 * self.angular_velocity

        if self.direction > 180:
            self.direction = -180 + (self.direction - 180)
        elif self.direction < -180:
            self.direction = 180 + (self.direction + 180)

        speed = self.acceleration * 2
        self.velocity[2] = np.sqrt(
            np.square(self.velocity[0]) + np.square(self.velocity[1]))

        x_vec = np.cos(-1 * np.deg2rad(self.direction + 90)) * speed
        y_vec = np.sin(-1 * np.deg2rad(self.direction + 90)) * speed
        if "UP" in pressed_keys:
            self.velocity[0] += self.acceleration * x_vec
            self.velocity[1] += self.acceleration * y_vec
            self.velocity[2] = np.sqrt(
                np.square(self.velocity[0]) + np.square(self.velocity[1]))
            if self.velocity[2] > self.max_velocity:
                divider = self.max_velocity / \
                    np.sqrt(
                        np.square(self.velocity[0]) + np.square(self.velocity[1]))
                self.velocity[0] = divider * self.velocity[0]
                self.velocity[1] = divider * self.velocity[1]

    def convert_key(self, keys):
        _action = False
        _keys_to_check = [[pygame.K_LEFT, "LEFT"],
                          [pygame.K_RIGHT, "RIGHT"],
                          [pygame.K_UP, "UP"],
                          [pygame.K_DOWN, "DOWN"],
                          [pygame.K_r, "R"]]
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

        return self.cur_keys

    def collision_detector(self):
        collision_list = self.hitbox.collidelistall(self.world.wall_list)
        return collision_list


class World():
    def __init__(self, p_screen):
        self.screen = p_screen
        self.size = 5
        self.grid = [[0 for _ in range(self.screen.get_size()[0] // 5)]
                     for __ in range(self.screen.get_size()[1] // 5)]
        self.wall_list = []

        # Drawing map
        for i in range(len(self.grid[0])):
            for j in range(len(self.grid[0])):
                if i == 0 or i == len(self.grid) - 1 or j == 0 or j == len(self.grid[0]) - 1:
                    self.grid[i][j] = 1
                else:
                    self.grid[i][j] = 0

    def draw(self):
        self.wall_list = []
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                if self.grid[i][j]:
                    wall_rect = pygame.Rect(i * self.size,
                                            j * self.size,
                                            self.size,
                                            self.size)
                    self.wall_list.append(wall_rect)
                    pygame.draw.rect(screen,
                                     (0, 0, 0),
                                     wall_rect)


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

world = World(screen)
robot = Robot(screen, world)
robot.update()


playing_game = True
while playing_game:
    clock.tick(45)
    screen.blit(background, (0, 0))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            playing_game = False
            break
    robot.change_velocity(pygame.key.get_pressed())
    world.draw()
    robot.update()
    pygame.display.update()

pygame.quit()
