import pygame
import numpy as np
import time
import operator


class Game(object):
    def __init__(self):
        pygame.init()
        pygame.key.set_repeat(300, 30)
        self.screen = pygame.display.set_mode((1280, 720), pygame.SRCALPHA)
        self.screen.fill((255, 255, 255))
        self.clock = pygame.time.Clock()

        self.background = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        self.background = self.background.convert()
        self.background.fill((255, 255, 255))

        self.gui = pygame.Surface(self.screen.get_size())
        self.gui.set_alpha(0)

        pygame.display.flip()

        self.world = World(self.screen)
        self.robot = RobotControl(self.screen, self.world)
        self.robot.update()

        self.font = pygame.font.Font(None, 30)

        self.main()

    def main(self):
        playing_game = True
        while playing_game:
            self.clock.tick(30)
            self.screen.blit(self.background, (0, 0))
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    playing_game = False
                    break
            self.robot.change_velocity(pygame.key.get_pressed())
            self.world.draw()
            self.robot.update()
            _fps = self.font.render(str(int(self.clock.get_fps())), True, pygame.Color('green'))
            self.screen.blit(_fps, (3, 3))
            pygame.display.update()

        pygame.quit()


class Robot(pygame.sprite.Sprite):
    """Sprite  the robot player object.

    Handles the attributes of the robot, including its collision mask. Also handles robot state
    updates including translational and rotational changes. This class also contains the lidar
    sensor calculations.

    Attributes:
        p_screen: The main pygame screen surface.
    """

    def __init__(self, p_screen, p_world):
        pygame.sprite.Sprite.__init__(self)
        self.screen = p_screen
        self.world = p_world
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
        self.draw_lidar = True

        # Lidar setup
        self.point_cloud = []
        self.sample_rate = 5  # Hz
        self.lidar_state = 0
        self.sample_count = 180

        self.lasers = pygame.sprite.Group()
        lidar = pygame.math.Vector2()
        lidar.xy = (self.x_pos, self.y_pos)
        self.initial_laser_length = int(np.sqrt(np.square(self.screen.get_width()) + np.square(self.screen.get_height())))
        for i in range(self.sample_count):
            degree_multiplier = 360 / self.sample_count
            cur_angle = int(i * degree_multiplier)
            laser = pygame.math.Vector2()
            laser.from_polar((self.initial_laser_length, cur_angle))
            laser_sprite = Laser(self.screen, lidar, laser)
            self.lasers.add(laser_sprite)
        self.lasers_draw = pygame.sprite.Group()

    def update(self):
        """Updates the position of the robot's rect, hitbox and mask."""
        self.rect.center = (self.x_pos, self.y_pos)
        self.hitbox.center = (self.x_pos, self.y_pos)
        self.mask = pygame.mask.from_surface(self.image)
        self.lidar()
        if self.draw_lidar:
            for _point in self.point_cloud:
                pygame.draw.aaline(self.screen,
                                   (255, 0, 0, 255),
                                   (self.x_pos, self.y_pos),
                                   _point)
                pygame.draw.circle(self.screen,
                                   (0, 0, 255, 255),
                                   _point,
                                   3)

    def rotate(self, direction):
        """Rotates the robot around it's centre."""
        self.image = pygame.transform.rotate(self.og_image, direction)
        self.rect = self.image.get_rect()
        self.rect.center = (self.x_pos, self.y_pos)

    def lidar(self):
        """Performs all calculations for laser range finding and handles the drawing of lasers.

        This function uses sprites to determine all of the objects each laser around the robot is
        colliding with, then finds the closest wall. It then finds the closest point on that wall
        to the robot.
        """
        # TODO: Fix flickering on some diagonal lasers
        _iterations_per_frame = int(
            self.sample_count / (30 / self.sample_rate))
        _slice_from = self.lidar_state * _iterations_per_frame
        _slice_to = _slice_from + _iterations_per_frame
        # Update the position of each of the laser sprites in self.lasers
        lidar = pygame.math.Vector2()
        lidar.xy = (self.x_pos, self.y_pos)
        for sprite in self.lasers.sprites()[_slice_from:_slice_to]:
            sprite.origin = lidar
            sprite.update()

        # Check wall collisions in quadrants
        _quad_list = [[[0, 90], operator.ge, operator.ge],
                      [[90, 181], operator.lt, operator.ge],
                      [[-90, 0], operator.ge, operator.lt],
                      [[-181, -90], operator.lt, operator.lt]]
        collision_list = {}
        _pixel_buffer = self.world.size * 2
        for _quad in _quad_list:
            _quad_lasers = pygame.sprite.Group()
            _quad_walls = pygame.sprite.Group()
            for _laser in self.lasers.sprites()[_slice_from:_slice_to]:
                _cur_angle = int(_laser.angle.as_polar()[1])
                if _cur_angle >= _quad[0][0] and _cur_angle < _quad[0][1]:
                    _quad_lasers.add(_laser)
            for _wall in self.world.wall_list:
                _cur_pos = _wall.rect.center
                if _quad[1] == operator.ge:
                    _x_buf = self.x_pos - _pixel_buffer
                else:
                    _x_buf = self.x_pos + _pixel_buffer
                if _quad[2] == operator.ge:
                    _y_buf = self.y_pos - _pixel_buffer
                else:
                    _y_buf = self.y_pos + _pixel_buffer
                if _quad[1](_cur_pos[0], _x_buf):
                    if _quad[2](_cur_pos[1], _y_buf):
                        _quad_walls.add(_wall)
            collision_list.update(pygame.sprite.groupcollide(_quad_lasers,
                                                            _quad_walls,
                                                            False,
                                                            False,
                                                            pygame.sprite.collide_mask))

        if collision_list:
            for laser in collision_list:
                # For each laser, find the closest wall to the robot it is colliding with
                closest_wall = None
                closest_distance = self.initial_laser_length
                for wall in collision_list[laser]:
                    cur_distance = point_distance(self.x_pos,
                                                  wall.rect.center[0],
                                                  self.y_pos,
                                                  wall.rect.center[1])
                    if cur_distance < closest_distance:
                        closest_wall = wall
                        closest_distance = cur_distance

                # Find the closest point on the closest wall to the robot
                current_pos = pygame.math.Vector2()
                current_pos.update(self.x_pos, self.y_pos)
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

                # Write resulting point to the point cloud
                if not closest_point == (self.initial_laser_length, self.initial_laser_length):
                    self.point_cloud.append(closest_point)                
                    if len(self.point_cloud) > self.sample_count:
                        self.point_cloud.pop(0)

        if self.lidar_state == (30 // self.sample_rate) - 1:
            self.lidar_state = 0
        else:
            self.lidar_state += 1


class RobotControl(object):
    """Controls the robot.

    Handles the robot's translation and rotation based on user input, including collisions,
    acceleration and deceleration.

    Attributes:
        p_screen: The main pygame screen surface.
        p_world: The world map as drawn by the World class.
    """

    def __init__(self, p_screen, p_world):
        self.screen = p_screen
        self.robot = Robot(self.screen, p_world)
        self.world = p_world
        # (+x velocity, +y velocity, velocity magnitude) pixels/tick
        self.velocity = [0, 0, 0]
        self.max_velocity = 3
        self.acceleration = 0.5
        self.cur_keys = []
        self.direction = 0
        self.angular_velocity = 6
        self.dummy_screen = pygame.Surface(self.screen.get_size())
        self.collision_list = []
        self.recursion_depth = 0

    def reset(self):
        """Reset the robot's attributes, including position and velocities."""
        self.robot.x_pos = self.screen.get_size()[0] / 2
        self.robot.y_pos = self.screen.get_size()[1] / 2
        self.robot.rect.center = (self.robot.x_pos, self.robot.y_pos)
        self.velocity = [0, 0, 0]
        self.direction = 0
        self.update()

    def update(self):
        """Update all aspects of the robot, including velocities, position and lidar sensor."""
        self.move_velocity()
        self.robot.rotate(self.direction)
        self.robot.update()
        self.screen.blit(self.robot.image, self.robot.rect)

    def move_velocity(self):
        """Controls the robot's position.

        This function takes in the Robot.velocity vector. The collision method returns, what side,
        if any, of the robot is colliding. It then sets the velocity in that direction to zero so
        the robot will maintain it's velocity in the perpendicular axis, but stops moving towards
        the collision. Then update the robot's position. If the robot isn't receiving input to move
        forward, decelerate velocities.
        """
        # Check if a collision has occurred, and zero the velocity axis associated with it.
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

        # Update robot position according to the velocity vector.
        self.robot.x_pos += self.velocity[0]
        self.robot.y_pos += self.velocity[1]
        self.robot.rect.center = (self.robot.x_pos, self.robot.y_pos)

        # Decelerate the velocity vector if no forward input is received.
        deceleration = self.acceleration / 2
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
        """Controls the robot's velocity.

        This function receives input from the user and updates the Robot.angular_velocity and
        Robot.velocity vectors accordingly.

        Attributes:
            keys: An array containing the current state of all keys.
        """
        # Get input and sets the rotation according to the angular velocity.
        pressed_keys = self.convert_key(keys)
        if "R" in pressed_keys:
            self.reset()
        if "RIGHT" in pressed_keys:
            self.direction -= self.angular_velocity
        if "LEFT" in pressed_keys:
            self.direction += self.angular_velocity

        # Bind the direction to remain < 180 and > -180.
        if self.direction > 180:
            self.direction = -180 + (self.direction - 180)
        elif self.direction < -180:
            self.direction = 180 + (self.direction + 180)

        # Calculate the current magnitude of the velocity vector.
        speed = self.acceleration * 2
        self.velocity[2] = np.sqrt(
            np.square(self.velocity[0]) + np.square(self.velocity[1]))

        # Calculate the axis velocity components according to the current direction and desired
        # speed.
        x_vec = np.cos(-1 * np.deg2rad(self.direction + 90)) * speed
        y_vec = np.sin(-1 * np.deg2rad(self.direction + 90)) * speed
        if "UP" in pressed_keys:
            self.velocity[0] += self.acceleration * x_vec
            self.velocity[1] += self.acceleration * y_vec
            self.velocity[2] = np.sqrt(
                np.square(self.velocity[0]) + np.square(self.velocity[1]))
            # Normalise the velocity vectors if the velocity's magnitude is greater than the
            # desired maximum velocity.
            if self.velocity[2] > self.max_velocity:
                divider = self.max_velocity / \
                    np.sqrt(
                        np.square(self.velocity[0]) + np.square(self.velocity[1]))
                self.velocity[0] = divider * self.velocity[0]
                self.velocity[1] = divider * self.velocity[1]

    def convert_key(self, keys):
        """Converts the pressed key information into a string array.

        This function takes the passed array of pygame keys and converts it to a list of the
        currently pressed keys.

        Attributes:
            keys: An array containing the current state of all keys.
        """
        _action = False
        _keys_to_check = [[pygame.K_LEFT, "LEFT"],
                          [pygame.K_RIGHT, "RIGHT"],
                          [pygame.K_UP, "UP"],
                          [pygame.K_DOWN, "DOWN"],
                          [pygame.K_r, "R"]]
        for _key in _keys_to_check:
            if keys[_key[0]]:
                if _key[1] not in self.cur_keys:
                    self.cur_keys.append(_key[1])
                _action = True
            else:
                try:
                    self.cur_keys.remove(_key[1])
                except ValueError:
                    pass
        # When a key is added, remove the first keys so that only the last two remain
        if _action:
            self.cur_keys = self.cur_keys[-2:]
        else:
            self.cur_keys = []
        return self.cur_keys

    def collision_detector(self):
        """Finds if the robot is colliding and the associated side.

        This function uses sprites to determine all of the objects the robot is colliding with,
        then finds the closest wall to determine which side of the robot is colliding. To solve for
        cases where the robot is colliding with two walls simultaneously, the function utilises
        recursion to find the second closest wall.
        """
        collision_list = pygame.sprite.spritecollide(self.robot,
                                                     self.world.wall_list,
                                                     False,
                                                     pygame.sprite.collide_mask)
        if len(collision_list) > 0:
            # Find the closest colliding wall
            closest_distance = self.robot.initial_laser_length
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
            # If performing recursion, find the second closest wall
            if self.recursion_depth > 0 and not s_closest_wall is None:
                closest_wall = s_closest_wall
            wall = closest_wall

            # Find which side of the robot is closest to the closest wall
            sides = [self.robot.hitbox.midtop, self.robot.hitbox.midright,
                     self.robot.hitbox.midbottom, self.robot.hitbox.midleft]
            closest_side = -1
            closest_side_distance = self.robot.initial_laser_length
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

        # Use a "dummy" surface to determine the width and height of the rotated laser rect
        dummy_screen = pygame.Surface(
            (p_screen.get_height() * 2, p_screen.get_width() * 2),
            pygame.SRCALPHA)
        dummy_rect = pygame.draw.line(dummy_screen,
                                      (0, 255, 0, 255),
                                      origin + origin,
                                      origin + origin + angle)

        self.origin = origin
        self.angle = angle
        int_angle = int(angle.as_polar()[1])
        # Find an offset for the laser's draw position depending on its angle
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
        self.mask = pygame.mask.from_surface(self.image, 50)

    def update(self):
        """Update the laser's position."""
        self.new_start = (self.origin.x + self.x_offset,
                          self.origin.y + self.y_offset)
        self.rect.topleft = self.new_start


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
        """Update the wall's colour.

        Used for debugging purposes only at this stage.
        """
        self.image.fill(color)


class World(object):
    """Writes and draws the world map.

    Handles the attributes for the world map and draws

    Attributes:
        p_screen: The main pygame screen surface.
    """

    def __init__(self, p_screen):
        self.screen = p_screen
        self.size = 20
        self.grid = [[0 for _ in range(self.screen.get_size()[0] // self.size)]
                     for __ in range(self.screen.get_size()[1] // self.size)]
        self.wall_list = pygame.sprite.Group()
        self.write_map()
        self.create_sprites()

    def write_map(self):
        """Draws the world map into an array of 1s and 0s."""
        for i, _ in enumerate(self.grid):
            for j, __ in enumerate(self.grid[0]):
                if i == 0 or i == len(self.grid) - 1 or j == 0 or j == len(self.grid[0]) - 1:
                    self.grid[i][j] = 1
                else:
                    self.grid[i][j] = 0
                if i > 20 and i < 30:
                    if j > 20 and j < 30:
                        self.grid[i][j] = 1

    def create_sprites(self):
        """Add sprites in the positions indicated by the self.grid array to a sprite group."""
        self.wall_list.empty()
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                if self.grid[i][j]:
                    wall_rect = Wall(j * self.size,
                                     i * self.size,
                                     self.size,
                                     self.size)
                    self.wall_list.add(wall_rect)

    def draw(self):
        """Draw the ."""
        self.wall_list.draw(self.screen)


def point_distance(x_1, x_2, y_1, y_2):
    """Find the distance between two points on a 2D plane."""
    return np.sqrt(np.square(x_1 - x_2) + np.square(y_1 - y_2))


if __name__ == '__main__':
    Game()