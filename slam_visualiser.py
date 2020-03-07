import time
import pygame
from pygame.math import Vector2
import numpy as np


class Robot:
    def __init__(self, _screen, _world, _world_screen):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load("roomba.png")
        self.image = pygame.transform.smoothscale(self.image, (50, 50))
        self.image_size = self.image.get_size()
        self.og_image = self.image.copy()
        self.rect = self.image.get_rect()
        self.screen = _screen
        self.world_screen = _world_screen
        self.world = _world
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
                                  self.image_size[0] + 2,
                                  self.image_size[1] + 2)
        self.point_count = 20
        self.dummy_screen = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        self.dummy_world_screen = self.dummy_screen.copy()

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
        self.hitbox.center = (self.x_pos, self.y_pos)
        self.lidar()
        self.screen.blit(self.image, self.rect)

    def rotate(self):
        self.image = pygame.transform.rotate(self.og_image, self.direction)
        self.rect = self.image.get_rect()
        self.rect.center = (self.x_pos, self.y_pos)

    def move_velocity(self):
        deceleration = self.acceleration / 2
        collision_side = self.collision_detector()
        if collision_side == "TOP":
            if self.velocity[1] < 0:
                self.velocity[1] = 0
        elif collision_side == "BOTTOM":
            if self.velocity[1] > 0:
                self.velocity[1] = 0
        elif collision_side == "RIGHT":
            if self.velocity[0] > 0:
                self.velocity[0] = 0
        elif collision_side == "LEFT":
            if self.velocity[0] < 0:
                self.velocity[0] = 0

        self.x_pos += self.velocity[0]
        self.y_pos += self.velocity[1]
        self.rect.center = (self.x_pos, self.y_pos)

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
        # TODO: Solve phasing through inverted corners due to alternating sides
        # TODO: Implement proper circle hitbox (subtractive from current?)
        collision_list = self.hitbox.collidelistall(self.world.wall_list)
        if len(collision_list) > 0:
            closest_distance = 100
            closest_iterator = 0
            for i in range(len(collision_list)):
                distance = np.sqrt(np.square(self.world.wall_list[collision_list[i]].center[0] - self.x_pos) + np.square(
                    self.world.wall_list[collision_list[i]].center[1] - self.y_pos))
                if distance < closest_distance:
                    closest_distance = distance
                    closest_iterator = i
            wall = self.world.wall_list[collision_list[closest_iterator]]
            closest_side_distance = 100
            sides = [self.hitbox.midtop, self.hitbox.midright,
                     self.hitbox.midbottom, self.hitbox.midleft]
            closest_side = -1
            for i in range(len(sides)):
                distance = np.sqrt(np.square(
                    sides[i][0] - wall.center[0]) + np.square(sides[i][1] - wall.center[1]))
                if distance < closest_side_distance:
                    closest_side_distance = distance
                    closest_side = i
            if closest_side == 0:
                return "TOP"
            elif closest_side == 1:
                return "RIGHT"
            if closest_side == 2:
                return "BOTTOM"
            elif closest_side == 3:
                return "LEFT"
        else:
            return "NONE"

    def lidar(self):
        lidar = Vector2()
        lidar.xy = (self.x_pos, self.y_pos)
        lasers = []
        for i in range(self.point_count):
            if i >= 0:
                degree_multiplier = 365 / self.point_count
                cur_angle = i * degree_multiplier
                laser = Vector2()
                laser.from_polar((200, cur_angle))
                self.dummy_screen.fill((0, 0, 0, 0))
                rect = pygame.draw.aaline(
                    self.dummy_screen, (255, 0, 0), lidar, lidar + laser, 1)
                pygame.draw.line(self.screen, (255, 0, 0), lidar, lidar + laser, 1)
                laser_mask = pygame.mask.from_surface(self.dummy_screen)
                
                # print(self.world.wall_list)
                for j in range(len(self.world.wall_list)):
                    self.dummy_world_screen.fill((0, 0, 0, 0))
                    cur_wall = pygame.draw.rect(self.dummy_world_screen, (255, 0, 0), self.world.wall_list[j])
                    world_mask = pygame.mask.from_threshold(self.dummy_world_screen, pygame.Color('red'), (1, 1, 1, 255))
                    
                    # print(rect[0], "-", cur_wall[0], "=", rect[0] - cur_wall[0])
                    # print(rect[1], "-", cur_wall[1], "=", rect[1] - cur_wall[1])
                    overlap = world_mask.overlap(laser_mask, (int((self.x_pos) - cur_wall.x), int((self.y_pos) - cur_wall.y)))
                    # print("Overlap:", overlap)
                    if overlap:
                        pygame.draw.circle(self.screen, (0, 255, 0), overlap, 2)
                # Draw mask REMOVE
                # outline = world_mask.outline()
                # if len(outline) > 0:
                #     pygame.draw.lines(self.screen, (0, 255, 0), 1, outline)
                    # pygame.draw.polygon(self.screen, (200,150,150), outline, 0)
                
                
                
                # heading_direction = laser.normalize()
                # obs_found = False
                # collide_point = (-1, -1)
                # # TODO: Find the closest collide point (SUPER SLOW???)
                # closest_point = (-1, -1)
                # for j in range(len(self.world.wall_list)):
                #     cur_pos = Vector2(self.x_pos, self.y_pos)
                #     for _ in range(int(laser.length())):
                #         cur_pos += heading_direction
                #         # pygame.draw.circle(self.screen, (0, 0, 255), (int(cur_pos.x), int(cur_pos.y)), 1)
                #         if self.world.wall_list[j].collidepoint(cur_pos):
                #             collide_point = (cur_pos.x, cur_pos.y)
                # if not collide_point == (-1, -1):
                #     pygame.draw.circle(self.screen, (0, 0, 255), (int(collide_point[0]), int(collide_point[1])), 5)
        print()
        # laser.from_polar((50, i * degree_multiplier))
        # rect = pygame.draw.aaline(self.screen, (255, 0, 0), lidar, lidar + laser)
        # laser = Vector2()
        # laser.from_polar((50, -self.direction + 90))


class World():
    def __init__(self, p_screen):
        self.screen = p_screen
        self.size = 5
        self.grid = [[0 for _ in range(self.screen.get_size()[0] // 5)]
                     for __ in range(self.screen.get_size()[1] // 5)]
        self.wall_list = []
        self.write_map()
        self.draw()

    def write_map(self):
        # Drawing map
        for i in range(len(self.grid[0])):
            for j in range(len(self.grid[0])):
                # if i == 0 or i == len(self.grid) - 1 or j == 0 or j == len(self.grid[0]) - 1:
                #     self.grid[i][j] = 1
                # else:
                #     self.grid[i][j] = 0
                if i > 40 and i < 50:
                    if j > 25 and j < 30:
                        self.grid[i][j] = 1

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
                    pygame.draw.rect(self.screen,
                                     (0, 0, 0),
                                     wall_rect)


pygame.init()
pygame.key.set_repeat(300, 30)
screen = pygame.display.set_mode((400, 400))
screen.fill((255, 255, 255))
clock = pygame.time.Clock()

background = pygame.Surface(screen.get_size())
background = background.convert()
background.fill((250, 250, 250))

gui = pygame.Surface(screen.get_size())
gui.set_alpha(0)

pygame.display.flip()

world_screen = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
world_screen.fill((255, 255, 255))
world = World(world_screen)
robot = Robot(screen, world, world_screen)
robot.update()


playing_game = True
while playing_game:
    clock.tick(45)
    screen.blit(background, (0, 0))
    screen.blit(world_screen, (0, 0))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            playing_game = False
            break
    robot.change_velocity(pygame.key.get_pressed())
    world.draw()
    robot.update()
    pygame.display.update()

pygame.quit()
