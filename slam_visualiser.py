import pygame
import numpy as np


class Robot(pygame.sprite.Sprite):
    """Sprite for the robot player object.

    Handles the attributes of the robot, including its collision mask. Also handles robot state
    updates including translational and rotational changes.

    Attributes:
        p_screen: The main pygame screen surface.
    """

    def __init__(self, p_screen):
        pygame.sprite.Sprite.__init__(self)
        self.screen = p_screen
        self.image = pygame.image.load("roomba.png")
        self.image = pygame.transform.smoothscale(self.image, (50, 50))
        self.image_size = self.image.get_size()
        self.og_image = self.image.copy()
        self.rect = self.image.get_rect()
        self.x_pos = float(self.screen.get_size()[0] / 2)
        self.y_pos = float(self.screen.get_size()[1] / 2)
        self.rect.center = (self.x_pos, self.y_pos)
        self.hitbox = pygame.Rect(self.x_pos - (self.image_size[0] / 2),
                                  self.y_pos - (self.image_size[1] / 2),
                                  self.image_size[0] + 2,
                                  self.image_size[1] + 2)
        self.mask = pygame.mask.from_surface(self.image)

    def update(self):
        self.rect.center = (self.x_pos, self.y_pos)
        self.hitbox.center = (self.x_pos, self.y_pos)
        self.mask = pygame.mask.from_surface(self.image)

    def rotate(self, direction):
        self.image = pygame.transform.rotate(self.og_image, direction)
        self.rect = self.image.get_rect()
        self.rect.center = (self.x_pos, self.y_pos)


class RobotControl:
    """Controls the robot.

    Handles the robot's translation and rotation based on user input, including collisions,
    acceleration and deceleration. Also contains the lidar sensor calculations.

    Attributes:
        p_screen: The main pygame screen surface.
        p_world: The world map as drawn by the World class.
    """

    def __init__(self, p_screen, p_world):
        self.screen = p_screen
        self.robot = Robot(self.screen)
        self.world = p_world
        self.velocity = [0, 0, 0]  # (+x velocity, +y velocity, velocity magnitude) pixels/tick
        self.max_velocity = 2
        self.acceleration = 0.5
        self.cur_keys = []
        self.direction = 0
        self.angular_velocity = 4
        self.point_count = 10
        self.dummy_screen = pygame.Surface(self.screen.get_size())
        self.collision_list = []
        self.recursion_depth = 0

        # Lidar setup
        self.lasers = pygame.sprite.Group()
        lidar = pygame.math.Vector2()
        lidar.xy = (self.robot.x_pos, self.robot.y_pos)
        self.initial_laser_length = self.screen.get_width() * 2
        for i in range(self.point_count):
            degree_multiplier = 360 / self.point_count
            cur_angle = int(i * degree_multiplier)
            laser = pygame.math.Vector2()
            laser.from_polar((self.initial_laser_length, cur_angle))
            laser_sprite = Laser(self.screen, lidar, laser)
            self.lasers.add(laser_sprite)

    def reset(self):
        print("Reseting...")
        self.robot.x_pos = self.screen.get_size()[0] / 2
        self.robot.y_pos = self.screen.get_size()[1] / 2
        self.robot.rect.center = (self.robot.x_pos, self.robot.y_pos)
        self.velocity = [0, 0, 0]
        self.direction = 0
        self.update()

    def update(self):
        self.move_velocity()
        self.robot.rotate(self.direction)
        self.robot.update()
        self.lidar()
        self.screen.blit(self.robot.image, self.robot.rect)

    def move_velocity(self):
        deceleration = self.acceleration / 2
        collision_side = self.collision_detector()
        self.collision_list.append(collision_side)
        if len(self.collision_list) > 3:
            self.collision_list.pop(0)
        if not collision_side:
            self.collision_list = []
        if "TOP" in self.collision_list:
            if self.velocity[1] < 0:
                self.velocity[1] = 0
        if "BOTTOM" in self.collision_list:
            if self.velocity[1] > 0:
                self.velocity[1] = 0
        if "RIGHT" in self.collision_list:
            if self.velocity[0] > 0:
                self.velocity[0] = 0
        if "LEFT" in self.collision_list:
            if self.velocity[0] < 0:
                self.velocity[0] = 0

        self.robot.x_pos += self.velocity[0]
        self.robot.y_pos += self.velocity[1]
        self.robot.rect.center = (self.robot.x_pos, self.robot.y_pos)

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
        collision_list = pygame.sprite.spritecollide(self.robot,
                                                     self.world.wall_list,
                                                     False,
                                                     pygame.sprite.collide_mask)
        if len(collision_list) > 0:
            closest_distance = self.initial_laser_length
            closest_wall = None
            for wall in collision_list:
                cur_distance = point_distance(self.robot.x_pos,
                                              wall.rect.center[0],
                                              self.robot.y_pos,
                                              wall.rect.center[1])
                if cur_distance < closest_distance:
                    s_closest_wall = closest_wall
                    closest_wall = wall
                    closest_distance = cur_distance
            if self.recursion_depth > 0 and not s_closest_wall is None:
                closest_wall = s_closest_wall
            wall = closest_wall
            sides = [self.robot.hitbox.midtop, self.robot.hitbox.midright,
                     self.robot.hitbox.midbottom, self.robot.hitbox.midleft]
            closest_side = -1
            closest_side_distance = self.initial_laser_length
            for i, side in enumerate(sides):
                distance = point_distance(side[0],
                                          wall.rect.center[0],
                                          side[1],
                                          wall.rect.center[1])
                if distance < closest_side_distance:
                    closest_side_distance = distance
                    closest_side = i
            to_return = None
            if closest_side == 0:
                to_return = "TOP"
            if closest_side == 1:
                to_return = "RIGHT"
            if closest_side == 2:
                to_return = "BOTTOM"
            if closest_side == 3:
                to_return = "LEFT"
            # If the robot is already colliding with a wall, collide the second closest wall
            if len(self.collision_list) > 0:
                if to_return == self.collision_list[len(self.collision_list) - 1]:
                    if self.recursion_depth <= 1:
                        self.recursion_depth += 1
                        return self.collision_detector()
            self.recursion_depth = 0
            return to_return
        return None

    def lidar(self):
        # TODO: Fix flickering on some diagonal lasers
        i = 0
        lidar = pygame.math.Vector2()
        lidar.xy = (self.robot.x_pos, self.robot.y_pos)
        for sprite in self.lasers:
            sprite.origin = lidar
            sprite.update()
            i += 1

        color = (0, 0, 0, 255)
        self.world.wall_list.update(color)
        collide = pygame.sprite.groupcollide(self.lasers,
                                             self.world.wall_list,
                                             False,
                                             False,
                                             pygame.sprite.collide_mask)

        if collide:
            for laser in collide:
                closest_wall = None
                closest_distance = self.initial_laser_length
                for wall in collide[laser]:
                    cur_distance = point_distance(self.robot.x_pos,
                                                  wall.rect.center[0],
                                                  self.robot.y_pos,
                                                  wall.rect.center[1])
                    if cur_distance < closest_distance:
                        closest_wall = wall
                        closest_distance = cur_distance
                current_pos = pygame.math.Vector2()
                current_pos.update(self.robot.x_pos, self.robot.y_pos)
                heading = laser.angle
                direction = heading.normalize()
                closest_point = (self.initial_laser_length,
                                 self.initial_laser_length)
                for _ in range(self.initial_laser_length):
                    current_pos += direction
                    if closest_wall.rect.collidepoint(current_pos):
                        closest_point = (int(current_pos.x),
                                         int(current_pos.y))
                        break
                new_length = point_distance(self.robot.x_pos,
                                            closest_point[0],
                                            self.robot.y_pos,
                                            closest_point[1])
                new_laser = pygame.math.Vector2()
                new_laser.from_polar((new_length, laser.angle.as_polar()[1]))
                pygame.draw.aaline(self.screen,
                                   (255, 0, 0, 255),
                                   lidar,
                                   lidar + new_laser)
                pygame.draw.circle(self.screen,
                                   (0, 0, 255, 255),
                                   (int(closest_point[0]),
                                    int(closest_point[1])),
                                   3)


class Laser(pygame.sprite.Sprite):
    """Sprite for the lidar sensor's laser beams.

    Handles the attributes of each laser. Uses invisible surfaces to calculate positional offsets
    for each laser depending on its given rotation. Also contains the laser's collision mask. It
    also handles the positional updates sent from RobotControl.

    Attributes:
        p_screen: The main pygame screen surface.
        origin: A pygame.math.Vector2() object that is the robot's base position.
        angle: A pygame.math.Vector2() object that contains polar coordinates stating the laser's
            length and direction angle.
    """

    def __init__(self, p_screen, origin, angle):
        pygame.sprite.Sprite.__init__(self)
        dummy_screen = pygame.Surface(
            (p_screen.get_height() * 2, p_screen.get_width() * 2),
            pygame.SRCALPHA)
        dummy_rect = pygame.draw.line(dummy_screen,
                                      (0, 255, 0, 255),
                                      origin + origin,
                                      origin + origin + angle)
        pygame.draw.circle(dummy_screen, (0, 0, 255, 255),
                           (int(origin.x), int(origin.y)), 5)

        self.origin = origin
        self.angle = angle
        int_angle = int(angle.as_polar()[1])
        if int_angle >= 0 and int_angle <= 90:
            self.x_offset = 0
            self.y_offset = 0
        elif int_angle > 90:
            self.x_offset = -dummy_rect.width
            self.y_offset = 0
        elif int_angle < -90:
            self.x_offset = -dummy_rect.width
            self.y_offset = -dummy_rect.height
        elif int_angle < 0 and int_angle >= -90:
            self.x_offset = 0
            self.y_offset = -dummy_rect.height

        self.screen = p_screen
        self.image = pygame.Surface((dummy_rect.width, dummy_rect.height),
                                    pygame.SRCALPHA)
        self.new_start = (self.origin.x + self.x_offset,
                          self.origin.y + self.y_offset)
        self.rect = pygame.draw.aaline(self.image,
                                       (255, 0, 0, 255),
                                       (-self.x_offset, - self.y_offset),
                                       (int(angle.x - self.x_offset), int(angle.y - self.y_offset)))
        self.mask = pygame.mask.from_surface(self.image)

    def update(self):
        self.new_start = (self.origin.x + self.x_offset,
                          self.origin.y + self.y_offset)
        self.rect.topleft = self.new_start
        self.mask = pygame.mask.from_surface(self.image)


class Wall(pygame.sprite.Sprite):
    """Sprite for the lidar sensor's laser beams.

    Handles the attributes of each laser. Uses invisible surfaces to calculate positional offsets
    for each laser depending on its given rotation. Also contains the laser's collision mask.

    Attributes:
        top: The desired pixel for the top of the wall.
        left: The desired pixel for the left of the wall.
        width: The desired width of the wall.
        height: The desired height of the wall.
    """

    def __init__(self, left, top, width, height):
        pygame.sprite.Sprite.__init__(self)
        self.rect = pygame.Rect(left, top, width, height)
        self.color = (0, 0, 0, 255)
        self.image = pygame.Surface((width, height), pygame.SRCALPHA)
        self.image.fill(self.color)
        self.mask = pygame.mask.from_threshold(self.image,
                                               pygame.Color('black'),
                                               (1, 1, 1, 255))

    def update(self, color):
        self.image.fill(color)


class World():
    """Writes and draws the world map.

    Handles the attributes for the world map and draws

    Attributes:
        p_screen: The main pygame screen surface.
    """

    def __init__(self, p_screen):
        self.screen = p_screen
        self.size = 10
        self.grid = [[0 for _ in range(self.screen.get_size()[0] // self.size)]
                     for __ in range(self.screen.get_size()[1] // self.size)]
        self.wall_list = pygame.sprite.Group()
        self.write_map()
        self.create_sprites()

    def write_map(self):
        # Drawing map
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                if i == 0 or i == len(self.grid) - 1 or j == 0 or j == len(self.grid[0]) - 1:
                    self.grid[i][j] = 1
                else:
                    self.grid[i][j] = 0
                if i > 25 and i < 30:
                    if j > 25 and j < 30:
                        self.grid[i][j] = 1

    def create_sprites(self):
        self.wall_list.empty()
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                if self.grid[i][j]:
                    wall_rect = Wall(i * self.size,
                                     j * self.size,
                                     self.size,
                                     self.size)
                    self.wall_list.add(wall_rect)

    def draw(self):
        self.wall_list.draw(self.screen)


def point_distance(x_1, x_2, y_1, y_2):
    return np.sqrt(np.square(x_1 - x_2) + np.square(y_1 - y_2))


pygame.init()
pygame.key.set_repeat(300, 30)
screen = pygame.display.set_mode((400, 400), pygame.SRCALPHA)
screen.fill((255, 255, 255))
clock = pygame.time.Clock()

background = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
background = background.convert()
background.fill((255, 255, 255))

gui = pygame.Surface(screen.get_size())
gui.set_alpha(0)

pygame.display.flip()

world = World(screen)
robot = RobotControl(screen, world)
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
