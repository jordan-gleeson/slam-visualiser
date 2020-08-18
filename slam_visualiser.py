import time
import operator
import random
import numpy as np
import pygame
import pygame_gui as pygui
import utils
import gui
import copy


class Game():
    """Main game class.

    Creates the game screen. Contains the main game loop which handles the order of execution of
    robot and SLAM functionality.
    """

    def __init__(self):
        # pygame setup
        pygame.init()
        pygame.key.set_repeat(300, 30)
        self.screen = pygame.display.set_mode((1280, 720), pygame.SRCALPHA)
        self.screen.fill((255, 255, 255))
        self.clock = pygame.time.Clock()

        # Create a white background
        self.background = pygame.Surface(self.screen.get_size(),
                                         pygame.SRCALPHA)
        self.background = self.background.convert()
        self.background.fill((255, 255, 255))
        self.menu_background = copy.copy(self.background)
        self.menu_background.fill((57, 65, 101))

        pygame.display.flip()

        # Setup classes
        self.world = World(self.screen)
        self.robot = RobotControl(self.screen, self.world)
        self.slam = SLAM(self.screen, self.robot)
        self.gui = gui.GUI(self.screen, self.world, self.robot, self.slam)

        self.font = pygame.font.Font(None, 30)
        self.prev_time = time.time()

        self.state = 0
        self.main()

    def main(self):
        """Main game loop."""
        _playing_game = True
        _world_edited = False

        # TEMP
        self.world.world_type = "Landmarks"

        while _playing_game:
            _time_delta = self.clock.tick(30) / 1000.0
            self.screen.blit(self.background, (0, 0))
            for _event in pygame.event.get():
                if _event.type == pygame.QUIT:
                    _playing_game = False
                    break
                if _event.type == pygame.USEREVENT:
                    self.gui.input(_event)
                if _event.type == pygame.MOUSEBUTTONUP:
                    self.gui.last_mouse_pos = None
                    self.gui.we_raise_click = True
                if _event.type == pygame.KEYDOWN:
                    if _event.key == pygame.K_r:
                        self.gui.reset()
                self.gui.manager.process_events(_event)

            # Main Menu
            if self.state == 0:
                # TEMP
                self.gui.main_menu_state = 0
                if self.gui.main_menu_state == 0:
                    self.state += 1
                    self.gui.setup_game(_world_edited)
                    self.init_game()
                elif self.gui.main_menu_state == 2:
                    self.state = 2
                    _world_edited = True
                    self.gui.kill_main_menu()
                    self.gui.world_editor_setup()
                else:
                    self.world.world_type = self.gui.slam_type_drop.selected_option

            # Simulation
            elif self.state == 1:
                self.robot.change_velocity(pygame.key.get_pressed())
                self.world.draw()
                self.slam.update()
                self.robot.update()
                self.slam.odometry(self.robot.velocity)
                self.slam.ekf(time.time() - self.prev_time)
                self.prev_time = time.time()
                if self.robot.robot.new_sample:
                    self.slam.occupancy_grid()
                    self.robot.robot.new_sample = False

            # World Editor
            elif self.state == 2:
                if self.gui.main_menu_state == 1:
                    self.state = 0
                    self.gui.main_menu()
                    self.gui.kill_world_editor()
                self.gui.world_editor(pygame.mouse.get_pressed()[0],
                                      pygame.mouse.get_pos())

            _fps = self.font.render(str(int(self.clock.get_fps())),
                                    True,
                                    pygame.Color('green'))
            _fps = self.font.render(str(pygame.mouse.get_pos()),
                                    True,
                                    pygame.Color('green'))
            self.screen.blit(_fps, (3, 3))
            self.gui.update(_time_delta)
            pygame.display.update()

        pygame.quit()

    def init_game(self):
        self.robot.robot.setup_lasers()
        self.robot.update()


class Robot(pygame.sprite.Sprite):
    """Sprite  the robot player object.

    Handles the attributes of the robot, including its collision mask. Also handles robot state
    updates including translational and rotational changes. This class also contains the lidar
    sensor calculations.

    Attributes:
        _p_screen: The main pygame screen surface.
        _p_world: The world map as drawn by the World class.
    """

    def __init__(self, _p_screen, _p_world):
        pygame.sprite.Sprite.__init__(self)
        self.screen = _p_screen
        self.world = _p_world
        self.image = pygame.image.load("roomba.png")
        self.robot_size = 50
        self.image = pygame.transform.smoothscale(self.image,
                                                  (self.robot_size, self.robot_size))
        self.image_size = self.image.get_size()
        self.og_image = self.image.copy()
        self.rect = self.image.get_rect()
        self.x_pos = float(self.screen.get_size()[0] / 2)
        self.y_pos = float(self.screen.get_size()[1] / 2)
        self.angle = np.radians(90)
        self.rect.center = (self.x_pos, self.y_pos)
        self.hitbox = pygame.Rect(self.x_pos - (self.image_size[0] / 2),
                                  self.y_pos - (self.image_size[1] / 2),
                                  self.image_size[0] + 2,
                                  self.image_size[1] + 2)
        self.mask = pygame.mask.from_surface(self.image)
        self.draw_lidar = True

        # Lidar setup
        self.sample_rate = 5  # Hz
        self.lidar_state = 0
        self.sample_count = 32
        self.angle_ref = []
        self.new_sample = True

        self.initial_laser_length = int(utils.point_distance(self.screen.get_width(), 0,
                                                             self.screen.get_height(), 0))

    def setup_lasers(self):
        """Setup the lasers coming from the robot depending on observation type."""
        if self.world.world_type == "Occupancy Grid":
            self.point_cloud = [[0, 0] for _ in range(self.sample_count)]
            self.lasers = pygame.sprite.Group()
            _lidar = pygame.math.Vector2()
            _lidar.xy = (self.x_pos, self.y_pos)
            for i in range(self.sample_count):
                _degree_multiplier = 360 / self.sample_count
                _cur_angle = int(i * _degree_multiplier)
                self.angle_ref.append(_cur_angle)
                _laser = pygame.math.Vector2()
                _laser.from_polar((self.initial_laser_length, _cur_angle))
                _laser_sprite = OG_Laser(self.screen, _lidar, _laser)
                self.lasers.add(_laser_sprite)
            self.lasers_draw = pygame.sprite.Group()
        elif self.world.world_type == "Landmarks":
            self.point_cloud = [[0, 0]
                                for _ in range(self.world.landmark_count)]
            _landmark_list = self.world.wall_list.sprites()
            self.lasers = []
            for _landmark in _landmark_list:
                _new_laser = LM_Laser(self.screen,
                                      (self.x_pos, self.y_pos),
                                      _landmark.rect.center)
                self.angle_ref.append(_landmark.rect.center)
                self.lasers.append(_new_laser)

    def reset(self):
        """Reset the robots position and sensor data."""
        self.x_pos = float(self.screen.get_size()[0] / 2)
        self.y_pos = float(self.screen.get_size()[1] / 2)
        self.angle = 0
        self.rect.center = (self.x_pos, self.y_pos)
        self.hitbox = pygame.Rect(self.x_pos - (self.image_size[0] / 2),
                                  self.y_pos - (self.image_size[1] / 2),
                                  self.image_size[0] + 2,
                                  self.image_size[1] + 2)
        if self.world.world_type == "Occupancy Grid":
            self.point_cloud = [[0, 0]
                                for _ in range(self.sample_count)]
        elif self.world.world_type == "Landmarks":
            self.point_cloud = [[0, 0]
                                for _ in range(self.world.landmark_count)]

    def update(self):
        """Updates the position of the robot's rect, hitbox and mask."""
        self.rect.center = (self.x_pos, self.y_pos)
        self.hitbox.center = (self.x_pos, self.y_pos)
        self.mask = pygame.mask.from_surface(self.image)
        if self.world.world_type == "Occupancy Grid":
            self.lidar()
        elif self.world.world_type == "Landmarks":
            self.landmark_sensor()
        if self.draw_lidar:
            for _point in self.point_cloud:
                _coords = [int(_point[0] * np.cos(_point[1]) + self.x_pos),
                           int(_point[0] * np.sin(_point[1]) + self.y_pos)]
                pygame.draw.aaline(self.screen,
                                   (255, 0, 0, 255),
                                   (self.x_pos, self.y_pos),
                                   _coords)
                pygame.draw.circle(self.screen,
                                   (0, 0, 255, 255),
                                   _coords, 3)

    def toggle_lidar(self):
        """Toggle whether or not the lidar sensor is visualised."""
        if self.draw_lidar:
            self.draw_lidar = False
        else:
            self.draw_lidar = True

    def rotate(self, _direction):
        """Rotates the robot around it's centre."""
        self.image = pygame.transform.rotate(self.og_image, _direction)
        self.rect = self.image.get_rect()
        self.rect.center = (self.x_pos, self.y_pos)

    def lidar(self):
        """Performs all calculations for laser range finding and handles the drawing of lasers.

        This function uses sprites to determine all of the objects each laser around the robot is
        colliding with, then finds the closest wall. It then finds the closest point on that wall
        to the robot.
        """
        # TODO: Fix flickering on some diagonal lasers
        # TODO: Make lasers that don't find a result return max length instead of previous result
        _iterations_per_frame = int(
            self.sample_count / (30 / self.sample_rate))
        _slice_from = self.lidar_state * _iterations_per_frame
        if self.lidar_state == (30 // self.sample_rate) - 2:
            # Ensure final slice gets the remainder
            _slice_to = self.sample_count
        else:
            _slice_to = _slice_from + _iterations_per_frame
        # Update the position of each of the laser sprites in self.lasers
        _lidar = pygame.math.Vector2()
        _lidar.xy = (self.x_pos, self.y_pos)
        for _sprite in self.lasers.sprites()[_slice_from:_slice_to]:
            _sprite.origin = _lidar
            _sprite.update()

        # Check wall collisions in quadrants
        _quad_list = [[[0, 90], operator.ge, operator.ge],
                      [[90, 181], operator.lt, operator.ge],
                      [[-90, 0], operator.ge, operator.lt],
                      [[-181, -90], operator.lt, operator.lt]]
        _collision_list = {}
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
            _collision_list.update(pygame.sprite.groupcollide(_quad_lasers,
                                                              _quad_walls,
                                                              False,
                                                              False,
                                                              pygame.sprite.collide_mask))

        if _collision_list:
            for _laser in _collision_list:
                # For each laser, find the closest wall to the robot it is colliding with
                _closest_wall = None
                _closest_distance = self.initial_laser_length
                for _wall in _collision_list[_laser]:
                    cur_distance = utils.point_distance(self.x_pos,
                                                        _wall.rect.center[0],
                                                        self.y_pos,
                                                        _wall.rect.center[1])
                    if cur_distance < _closest_distance:
                        _closest_wall = _wall
                        _closest_distance = cur_distance

                # Find the closest point on the closest wall to the robot
                _current_pos = pygame.math.Vector2()
                _current_pos.update(self.x_pos, self.y_pos)
                _heading = _laser.angle
                _direction = _heading.normalize()
                _closest_point = [self.initial_laser_length,
                                  self.initial_laser_length]
                for _ in range(self.initial_laser_length):
                    _current_pos += _direction
                    if _closest_wall.rect.collidepoint(_current_pos):
                        _r = np.sqrt(np.square(self.x_pos - _current_pos.x)
                                     + np.square(self.y_pos - _current_pos.y))
                        _theta = np.arctan2(-(self.y_pos - _current_pos.y), -
                                            (self.x_pos - _current_pos.x))
                        _closest_point = [_r, _theta]
                        break

                # Write resulting point to the point cloud
                if not _closest_point == [self.initial_laser_length, self.initial_laser_length]:
                    _cur_angle = (round(_heading.as_polar()[1]) + 450) % 360
                    try:
                        self.point_cloud[self.angle_ref.index(
                            _cur_angle)] = _closest_point
                    except ValueError:
                        pass

        if self.lidar_state == (30 // self.sample_rate) - 1:
            self.new_sample = True
            self.lidar_state = 0
        else:
            self.lidar_state += 1

    def landmark_sensor(self):
        for _laser in self.lasers:
            _laser.update((self.x_pos, self.y_pos))
            self.point_cloud[self.angle_ref.index(
                _laser.destination)] = _laser.polar


class RobotControl():
    """Controls the robot.

    Handles the robot's translation and rotation based on user input, including collisions,
    acceleration and deceleration.

    Attributes:
        _p_screen: The main pygame screen surface.
        _p_world: The world map as drawn by the World class.
    """

    def __init__(self, _p_screen, _p_world):
        self.screen = _p_screen
        self.robot = Robot(self.screen, _p_world)
        self.world = _p_world
        # (+x velocity, +y velocity, velocity magnitude) pixels/tick
        self.velocity = np.array([0., 0.])
        self.max_velocity = 4.
        self.acceleration = 0.5
        self.cur_keys = []
        self.angular_velocity = np.radians(6)
        self.dummy_screen = pygame.Surface(self.screen.get_size())
        self.collision_list = []
        self.recursion_depth = 0
        self.truth_pos = []
        self.stop = [1, 1, 1]

        self.count = 0  # TODO: Remove
        self.round = 5

    def reset(self):
        """Reset the robot's attributes, including position and velocities."""
        self.robot.x_pos = self.screen.get_size()[0] / 2
        self.robot.y_pos = self.screen.get_size()[1] / 2
        self.robot.rect.center = (self.robot.x_pos, self.robot.y_pos)
        self.velocity = [0, 0, 0]
        self.odo_velocity = self.velocity
        self.robot.angle = 0
        self.truth_pos = []
        self.robot.reset()
        self.update()

    def update(self):
        """Update all aspects of the robot, including velocities, position and lidar sensor."""
        self.move_velocity()
        self.robot.rotate(np.degrees(self.robot.angle))
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

        # Update robot position according to the velocity vector.
        # TODO: Refactor positions to be a numpy array
        # print("CS1", self.robot.angle, np.cos(self.robot.angle), -np.sin(self.robot.angle), self.velocity[1])
        _transformation = np.diag(self.stop) @ (np.array([[np.cos(self.robot.angle), 0],
                                    [-np.sin(self.robot.angle), 0],
                                    [0, 1]]) @ np.vstack(self.velocity[0:2]))
        _positions = (np.eye(3, dtype=float) @ np.vstack([self.robot.x_pos, self.robot.y_pos, self.robot.angle])) + _transformation
        self.robot.x_pos = _positions[0].item()
        self.robot.y_pos = _positions[1].item()
        self.robot.angle = _positions[2].item()

        # Bind the robot.angle to remain < 180 and > -180.
        if self.robot.angle > np.radians(180):
            self.robot.angle = -np.radians(180) + (self.robot.angle - np.radians(180))
        elif self.robot.angle < -np.radians(180):
            self.robot.angle = np.radians(180) + (self.robot.angle + np.radians(180))
        self.robot.angle = round(self.robot.angle, self.round)

        # print("PS1", [b1, b2], [self.robot.x_pos, self.robot.y_pos])
        self.robot.rect.center = (self.robot.x_pos, self.robot.y_pos)
        if len(self.truth_pos) > 1000:
            self.truth_pos.pop(0)
        self.truth_pos.append([self.robot.x_pos, self.robot.y_pos])

    def change_velocity(self, _keys):
        """Controls the robot's velocity.

        This function receives input from the user and updates the Robot.angular_velocity and
        Robot.velocity vectors accordingly.

        Attributes:
            _keys: An array containing the current state of all keys.
        """
        # Get input and sets the rotation according to the angular velocity.
        _pressed_keys = self.convert_key(_keys)
        # self.count += 1
        # _right = [1, 2, 3, 6, 7, 8]
        # _pressed_keys.append("UP")  # TODO: Remove
        # _pressed_keys.append("RIGHT")
        # if self.count in _right:
        #     _pressed_keys.append("RIGHT")
        # if self.count % 4 == 0:
        #     _pressed_keys.append("RIGHT")
        # if self.count == 10:
        #     self.count = 0
        # print(_pressed_keys)
        if "RIGHT" in _pressed_keys:
            # self.robot.angle -= self.angular_velocity
            self.velocity[1] = -1 * self.angular_velocity
        if "LEFT" in _pressed_keys:
            # self.robot.angle += self.angular_velocity
            self.velocity[1] = self.angular_velocity
        if not "RIGHT" in _pressed_keys and not "LEFT" in _pressed_keys:
            self.velocity[1] = 0
        self.robot.angle = round(self.robot.angle, self.round)

        # Calculate the current magnitude of the velocity vector.
        # _speed = self.acceleration * 2

        # Calculate the axis velocity components according to the current direction and desired
        # speed.
        # TODO: Rewrite to transform by a velocity and theta vector
        if "UP" in self.cur_keys:
            self.velocity[0] += self.acceleration
            if self.velocity[0] > 4:
                self.velocity[0] = 4.
        else: # Decelerate the velocity vector if no forward input is received.
            # print("Decelerating", self.velocity)
            _deceleration = self.acceleration / 2
            self.velocity[0] -= _deceleration
            if self.velocity[0] < 0:
                self.velocity[0] = 0

        # Check if a collision has occurred, and zero the velocity axis associated with it.
        _collision_side = self.collision_detector()
        self.collision_list.append(_collision_side)
        if len(self.collision_list) > 3:
            self.collision_list.pop(0)
        if not _collision_side:
            self.collision_list = []
        print(self.robot.angle)
        self.stop = [1, 1, 1]
        if "TOP" in self.collision_list:
            if self.robot.angle > 0:
                self.stop[1] = 0
        if "BOTTOM" in self.collision_list:
            if self.robot.angle < 0:
                self.stop[1] = 0
        if "RIGHT" in self.collision_list:
            if np.radians(-90) < self.robot.angle < np.radians(90):
                self.stop[0] = 0
        if "LEFT" in self.collision_list:
            if self.robot.angle < np.radians(-90) or self.robot.angle > np.radians(90):
                self.stop[0] = 0


    def convert_key(self, _keys):
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
            if _keys[_key[0]]:
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
        _collision_list = pygame.sprite.spritecollide(self.robot,
                                                      self.world.wall_list,
                                                      False,
                                                      pygame.sprite.collide_mask)
        if len(_collision_list) > 0:
            # Find the closest colliding wall
            _closest_distance = self.robot.initial_laser_length
            _closest_wall = None
            for _wall in _collision_list:
                cur_distance = utils.point_distance(self.robot.x_pos,
                                                    _wall.rect.center[0],
                                                    self.robot.y_pos,
                                                    _wall.rect.center[1])
                if cur_distance < _closest_distance:
                    s_closest_wall = _closest_wall
                    _closest_wall = _wall
                    _closest_distance = cur_distance
            # If performing recursion, find the second closest wall
            if self.recursion_depth > 0 and not s_closest_wall is None:
                _closest_wall = s_closest_wall
            _wall = _closest_wall

            # Find which side of the robot is closest to the closest wall
            _sides = [self.robot.hitbox.midtop, self.robot.hitbox.midright,
                      self.robot.hitbox.midbottom, self.robot.hitbox.midleft]
            _closest_side = -1
            _closest_side_distance = self.robot.initial_laser_length
            for _i, _side in enumerate(_sides):
                distance = utils.point_distance(_side[0],
                                                _wall.rect.center[0],
                                                _side[1],
                                                _wall.rect.center[1])
                if distance < _closest_side_distance:
                    _closest_side_distance = distance
                    _closest_side = _i
            _to_return = None
            if _closest_side == 0:
                _to_return = "TOP"
            if _closest_side == 1:
                _to_return = "RIGHT"
            if _closest_side == 2:
                _to_return = "BOTTOM"
            if _closest_side == 3:
                _to_return = "LEFT"

            # If the robot is already colliding with a wall, collide the second closest wall
            if len(self.collision_list) > 0:
                if _to_return == self.collision_list[len(self.collision_list) - 1]:
                    if self.recursion_depth <= 1:
                        self.recursion_depth += 1
                        return self.collision_detector()
            self.recursion_depth = 0
            return _to_return
        return None


class OG_Laser(pygame.sprite.Sprite):
    """Sprite for the lidar sensor's laser beams.

    Handles the attributes of each laser. Uses invisible surfaces to calculate positional offsets
    for each laser depending on its given rotation. Also contains the laser's collision mask. It
    also handles the positional updates sent from RobotControl.

    Attributes:
        _p_screen: The main pygame screen surface.
        _origin: A pygame.math.Vector2() object that is the robot's base position.
        _angle: A pygame.math.Vector2() object that contains polar coordinates stating the laser's
            length and direction _angle.
    """

    def __init__(self, _p_screen, _origin, _angle):
        pygame.sprite.Sprite.__init__(self)

        # Use a "dummy" surface to determine the width and height of the rotated laser rect
        _dummy_screen = pygame.Surface(
            (_p_screen.get_height() * 2, _p_screen.get_width() * 2),
            pygame.SRCALPHA)
        _dummy_rect = pygame.draw.line(_dummy_screen,
                                       (0, 255, 0, 255),
                                       _origin + _origin,
                                       _origin + _origin + _angle)

        self.origin = _origin
        # TODO: Convert to use radians
        self.angle = _angle
        _int_angle = int(_angle.as_polar()[1])
        # Find an offset for the laser's draw position depending on its angle
        if 0 <= _int_angle <= 90:
            self.x_offset = 0
            self.y_offset = 0
        elif _int_angle > 90:
            self.x_offset = -_dummy_rect.width
            self.y_offset = 0
        elif _int_angle < -90:
            self.x_offset = -_dummy_rect.width
            self.y_offset = -_dummy_rect.height
        elif -90 <= _int_angle < 0:
            self.x_offset = 0
            self.y_offset = -_dummy_rect.height

        self.screen = _p_screen
        self.image = pygame.Surface((_dummy_rect.width, _dummy_rect.height),
                                    pygame.SRCALPHA)
        self.new_start = (self.origin.x + self.x_offset,
                          self.origin.y + self.y_offset)
        self.rect = pygame.draw.aaline(self.image,
                                       (255, 0, 0, 255),
                                       (-self.x_offset, - self.y_offset),
                                       (int(_angle.x - self.x_offset),
                                        int(_angle.y - self.y_offset)))
        self.mask = pygame.mask.from_surface(self.image, 50)

    def update(self):
        """Update the laser's position."""
        self.new_start = (self.origin.x + self.x_offset,
                          self.origin.y + self.y_offset)
        self.rect.topleft = self.new_start


class LM_Laser():
    """Laser object containing the attributes of each landmark sensor laser.

    Attributes:
        _p_screen: The main pygame screen surface.
        _origin: A set of coordinates containing the robot's position.
        _destination: A set of coordinates containing the location of the detected landmark.
    """

    def __init__(self, _p_screen, _origin, _destination):
        self.screen = _p_screen
        self.destination = _destination
        self.update(_origin)

    def update(self, _origin):
        """Update the laser's position."""
        self.origin = _origin
        self.angle = self.find_angle(_origin, self.destination)
        self.length = utils.point_distance(_origin[0], self.destination[0],
                                           _origin[1], self.destination[1])
        self.polar = (self.length, self.angle)

    def find_angle(self, _origin, _destination):
        return np.arctan2(_destination[1] - _origin[1],
                          _destination[0] - _origin[0])


class Wall(pygame.sprite.Sprite):
    """Sprite for the lidar sensor's laser beams.

    Handles the attributes of each laser. Uses invisible surfaces to calculate positional offsets
    for each laser depending on its given rotation. Also contains the laser's collision mask.

    Attributes:
        _top: The desired pixel for the top of the wall.
        _left: The desired pixel for the left of the wall.
        _width: The desired width of the wall.
        _height: The desired height of the wall.
    """

    def __init__(self, _left, _top, _width, _height):
        pygame.sprite.Sprite.__init__(self)
        self.rect = pygame.Rect(_left, _top, _width, _height)
        self.color = (0, 0, 0, 255)
        self.image = pygame.Surface((_width, _height), pygame.SRCALPHA)
        self.image.fill(self.color)
        self.mask = pygame.mask.from_threshold(self.image,
                                               pygame.Color('black'),
                                               (1, 1, 1, 255))

    def update(self, _color):
        """Update the wall's colour.

        Used for debugging purposes only at this stage.
        """
        self.image.fill(_color)


class World():
    """Writes and draws the world map.

    Handles the attributes for the world map and draws.

    Attributes:
        _p_screen: The main pygame screen surface.
    """

    def __init__(self, _p_screen):
        self.screen = _p_screen
        self.size = 20
        self.grid = [[0 for _ in range(self.screen.get_size()[0] // self.size)]
                     for __ in range(self.screen.get_size()[1] // self.size)]
        self.wall_list = pygame.sprite.Group()
        self.world_type = "Occupancy Grid"
        self.landmark_count = 10

    def write_map(self, _robot_size):
        """Draws the world map into an array of 1s and 0s."""
        if self.world_type == "Occupancy Grid":
            for i, _ in enumerate(self.grid):
                for j, __ in enumerate(self.grid[0]):
                    if i == 0 or i == len(self.grid) - 1 or j == 0 or j == len(self.grid[0]) - 1:
                        self.grid[i][j] = 1
                    else:
                        self.grid[i][j] = 0
                    if 20 < i < 30:
                        if 20 < j < 30:
                            self.grid[i][j] = 1
        elif self.world_type == "Landmarks":
            _landmark_list = []
            for i in range(self.landmark_count):
                _r_point = [random.randrange(0, len(self.grid)),
                            random.randrange(0, len(self.grid[0]))]
                _hor_cen = self.screen.get_width() / 2
                _vert_cen = self.screen.get_height() / 2
                _return = np.array([_r_point[1] * self.size > _hor_cen - _robot_size / 2,
                                    _r_point[1] * self.size < _hor_cen +
                                    _robot_size / 2,
                                    _r_point[0] * self.size < _vert_cen +
                                    _robot_size / 2,
                                    _r_point[0] * self.size > _vert_cen - _robot_size / 2])
                if not _return.all():
                    _landmark_list.append(_r_point)
            # print(np.array(_landmark_list) * self.size)
            # input()
            for _point in _landmark_list:
                self.grid[_point[0]][_point[1]] = 1

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

    def clear_map(self):
        self.grid = [[0 for _ in range(self.screen.get_size()[0] // self.size)]
                     for __ in range(self.screen.get_size()[1] // self.size)]

    def write_to_map(self, _mode, _x, _y):
        if _mode:
            self.grid[_y][_x] = 1
        else:
            self.grid[_y][_x] = 0

    def draw(self):
        """Draw the world map."""
        self.wall_list.draw(self.screen)


class SLAM():
    """Contains all aspects of the SLAM algorithm (WIP).

    Handles calculations and drawing of the occupancy grid map. Creates fake odometry positioning.

    Attributes:
        _p_screen: The main pygame screen surface.
        _p_robot: The robot object.
    """

    def __init__(self, _p_screen, _p_robot):
        self.screen = _p_screen
        self.robot = _p_robot

        # Occupancy Grid Setup
        self.grid_size = 11
        self.grid = [[0.5 for _ in range(self.screen.get_size()[0] // self.grid_size)]
                     for __ in range(self.screen.get_size()[1] // self.grid_size)]
        self.show_occupancy_grid = False

        # Odometry Setup
        self.odo_velocity = self.robot.velocity
        self.odo_x = self.robot.robot.x_pos
        self.odo_y = self.robot.robot.y_pos
        self.odo_angle = self.robot.robot.angle
        self.odo_error = np.array([0.2, np.radians(1)])
        self.odo_pos = []

        # EKF Setup
        self.ekf_pos = np.vstack(
            [self.robot.robot.x_pos, self.robot.robot.y_pos, self.robot.robot.angle])
        # print("setup", self.ekf_pos)
        self.ekf_cov = np.eye(3)
        self.pose_cov = np.diag([0.1, 0.1, np.deg2rad(10)])
        self.meas_cov = np.diag([1, 1])
        self.lm_pos = np.array([np.vstack([0., 0.]) for i in range(10)])
        self.ekf_first_run = True
        self.ekf_pos_draw = []

    def reset(self):
        """Reset the SLAM state."""
        self.grid = [[0.5 for _ in range(self.screen.get_size()[0] // self.grid_size)]
                     for __ in range(self.screen.get_size()[1] // self.grid_size)]
        self.odo_x = self.robot.robot.x_pos
        self.odo_y = self.robot.robot.y_pos
        self.odo_pos = []

    def update(self):
        """Update SLAM visuals."""
        if self.show_occupancy_grid:
            self.draw_grid()

    def odometry(self, _vel_vector):
        """Adds a random error to the positional data within a percentage tolerance."""
        # try:
        self.odo_velocity = np.vstack([np.random.normal(_vel_vector[0], 
                                                        np.abs(_vel_vector[0]) * self.odo_error[0]),
                                       np.random.normal(_vel_vector[1], 
                                                        np.abs(_vel_vector[1]) * self.odo_error[1])])
        _transformation = np.array([[np.cos(self.odo_angle), 0],
                                    [-np.sin(self.odo_angle), 0],
                                    [0, 1]]) @ np.vstack(self.odo_velocity[0:2])
        _positions = (np.eye(3, dtype=float) @ np.vstack([self.odo_x, self.odo_y, self.odo_angle])) + _transformation
        self.odo_x = _positions[0].item()
        self.odo_y = _positions[1].item()
        self.odo_angle = _positions[2].item()
        if len(self.odo_pos) > 1000:
            self.odo_pos.pop(0)
        self.odo_pos.append([self.odo_x, self.odo_y])
        # print("ODOMETRY", _vel_vector)
        # except ValueError as e:
        #     print(e)

    def occupancy_grid(self):
        """Occupance grid algorithm.

        Loops through all points in the point cloud and lowers the probability of a space in the
        grid being occupied if it is found on a line between the robot and a point, and increases
        the probability if it is found at the end-point of the laser.
        """

        _rate_of_change = 0.05  # The rate at which the probability of a point is changed
        _pc = self.robot.robot.point_cloud
        for _point in _pc:
            try:  # Catch instances where the end-point may be out of the game screen
                _coords = [int(_point[0] * np.cos(_point[1]) + self.odo_x),  # Convert to cartesian
                           int(_point[0] * np.sin(_point[1]) + self.odo_y)]
                # Loop through the points in between the robot and the end-point of a laser
                for _clear in utils.line_between(self.robot.robot.x_pos // self.grid_size,
                                                 self.robot.robot.y_pos // self.grid_size,
                                                 _coords[0] // self.grid_size,
                                                 _coords[1] // self.grid_size)[:-1]:
                    # Decrease occupancy probability
                    self.grid[int(_clear[1])][int(
                        _clear[0])] -= _rate_of_change
                    if self.grid[int(_clear[1])][int(_clear[0])] < 0:
                        self.grid[int(_clear[1])][int(_clear[0])] = 0
                _grid_y = int(_coords[1] // self.grid_size)
                _grid_x = int(_coords[0] // self.grid_size)
                # Increase occupancy probability of the end-point
                self.grid[_grid_y][_grid_x] += _rate_of_change
                if self.grid[_grid_y][_grid_x] > 1:
                    self.grid[_grid_y][_grid_x] = 1
            except IndexError:
                pass

    def toggle_occupancy_grid(self):
        """Toggle whether or not the occupancy grid is visualised."""
        if self.show_occupancy_grid:
            self.show_occupancy_grid = False
        else:
            self.show_occupancy_grid = True

    def draw_grid(self):
        """Draw the occupancy grid as a function of its probability as its alpha."""
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                _alpha = 1 - self.grid[i][j]
                _rect = pygame.Rect(j * self.grid_size,
                                    i * self.grid_size,
                                    self.grid_size,
                                    self.grid_size)
                pygame.draw.rect(self.screen,
                                 (255 * _alpha, 255 * _alpha, 255 * _alpha),
                                 _rect)

    def ekf(self, _time_delta):
        def normalise_angle(_angle):
            return np.array(round(((_angle + np.pi) % (2 * np.pi) - np.pi).item(), self.robot.round))
        # As per http://www.probabilistic-robotics.org/
        _control = self.odo_velocity
        _trans_vel = _control[0]
        _ang_vel = _control[1]
        _prev_theta = self.ekf_pos[2].item()
        _n_ekf_pos = copy.deepcopy(self.ekf_pos)
        if _ang_vel == 0:
            _control_ratio = 1
        else:
            _control_ratio = _trans_vel / _ang_vel
        _time_delta = 1
        _new_angle = round(_prev_theta, self.robot.round)
        c = np.cos(_new_angle)
        s = -np.sin(_new_angle)
        # print("CS2", _new_angle, c, s, _ang_vel)
        # print()
        _predict_move = np.array([[_time_delta * c, 0],
                                  [_time_delta * s, 0],
                                  [0, _time_delta]]) @ np.vstack([_trans_vel, _ang_vel])
        _n_ekf_pos[0:3] = (np.eye(3, dtype=float) @
                           self.ekf_pos[0:3]) + _predict_move
        # print("PS2", [self.ekf_pos[0][0], self.ekf_pos[1][0]], [_n_ekf_pos[0][0], _n_ekf_pos[1][0]])
        # print("Vel dif2", _n_ekf_pos - self.ekf_pos)
        _n_ekf_pos[2][0] = round(_n_ekf_pos[2][0], self.robot.round)
        

        _filter = np.hstack((np.eye(3), np.zeros(
            (3, int(len(self.ekf_pos) - 3)))))  # Fx
        # x = np.vstack([-_control_ratio*np.sin(_prev_theta) + _control_ratio*np.sin(_prev_theta + _ang_vel),
        #                                                    _control_ratio * np.cos(_prev_theta) - _control_ratio * np.cos(_prev_theta + _ang_vel),
        #                                                    _ang_vel])
        # _n_ekf_pos = self.ekf_pos + _filter.T @ x
        
        y = np.array([[0, 0, -_time_delta * _trans_vel * np.sin(_prev_theta)],
                      [0, 0, _time_delta * _trans_vel * np.cos(_prev_theta)],
                      [0, 0, 0]], dtype=float)
        # y = np.array([[0, 0, -_control_ratio * np.cos(_prev_theta) + _control_ratio * np.cos(_prev_theta + _ang_vel)],  # TODO: Check the name is right
        #               [0, 0, -_control_ratio * np.sin(_prev_theta) + _control_ratio * np.sin(_prev_theta + _ang_vel)],  # G
        #               [0, 0, 0]], dtype=float)
        _control_jacobian = np.eye(len(_n_ekf_pos), dtype=float) + _filter.T @ y @ _filter
        # print("nekfcov before", self.ekf_cov)
        _n_ekf_cov = _control_jacobian @ self.ekf_cov @ _control_jacobian.T + \
            _filter.T @ self.pose_cov @ _filter
        # print("nekfcov after", _n_ekf_cov)

        # Correction Step
        for i, _point in enumerate(self.robot.robot.point_cloud):
            _lm_coords = np.array([[int(_point[0] * np.cos(_point[1]) + _n_ekf_pos[0])],
                                   [int(_point[0] * np.sin(_point[1]) + _n_ekf_pos[1])]])
            if self.ekf_first_run:
                # Extand state and covariance matrices

                _n_ekf_cov = np.vstack((np.hstack((_n_ekf_cov, np.zeros((len(_n_ekf_pos), 2)))),
                                        np.hstack((np.zeros((2, len(_n_ekf_pos))), np.eye(2)))))
                _n_ekf_pos = np.vstack((_n_ekf_pos, _lm_coords))

            _lm_dis_coords = _lm_coords - _n_ekf_pos[0:2]
            _lm_combined = (_lm_dis_coords.T @ _lm_dis_coords).item()
            _lm_dis = np.sqrt((_lm_dis_coords.T @ _lm_dis_coords).item())
            _meas_dif = np.vstack([_lm_dis,
                                   normalise_angle(np.arctan2(_lm_dis_coords.T[0][1], _lm_dis_coords.T[0][0]) - _n_ekf_pos[2][0])])

            _lm_filter = np.vstack((np.hstack((np.eye(3), np.zeros((3, int((len(_n_ekf_pos) - 3)))))),
                                    np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * i)),
                                               np.eye(2), np.zeros((2, int((len(_n_ekf_pos) - 3)) - 2 * (i + 1)))))))
            _tay_jac = ((-1 / _lm_combined) * np.array([[-_lm_dis * _lm_dis_coords.T[0][0],  -_lm_dis * _lm_dis_coords.T[0][1], 0, _lm_dis * _lm_dis_coords.T[0][0], _lm_dis * _lm_dis_coords.T[0][1]],
                                                               [_lm_dis_coords.T[0][1], -_lm_dis_coords.T[0][0], -_lm_combined, -_lm_dis_coords.T[0][1], _lm_dis_coords.T[0][0]]])) @ _lm_filter

            _K_gain = (_n_ekf_cov @ _tay_jac.T) @ np.linalg.inv(
                _tay_jac @ _n_ekf_cov @ _tay_jac.T + self.meas_cov[0:2, 0:2])

            if self.ekf_first_run:
                _innov = np.vstack([0, 0])
            else:
                _innov = self.lm_pos[i] - _meas_dif
                _innov[1][0] = normalise_angle(_innov[1][0])
            self.lm_pos[i] = _meas_dif

            _n_ekf_pos = _n_ekf_pos + (_K_gain @ _innov)
            _n_ekf_cov = (np.eye(len(_n_ekf_pos)) -
                          (_K_gain @ _tay_jac)) @ _n_ekf_cov

        _n_ekf_pos[2] = normalise_angle(_n_ekf_pos[2])
        self.ekf_pos = _n_ekf_pos

        self.ekf_cov = _n_ekf_cov

        self.ekf_first_run = False

        # time.sleep(0.1)
        # for i in range(10):
        #     print()
        # print()
        # print("Angle", self.robot.robot.angle, self.ekf_pos[2])
        # print("Velocity", np.sqrt(self.robot.velocity[0]**2 +self.robot.velocity[1]**2), _trans_vel)
        # print("Resulting Pos", [self.robot.robot.x_pos, self.robot.robot.y_pos], [self.ekf_pos[0][0], self.ekf_pos[1][0]])
        # print("Pos difference", self.robot.robot.x_pos - self.ekf_pos[0][0], self.robot.robot.y_pos - self.ekf_pos[1][0])
        # print(self.ekf_pos[0:3])
        self.ekf_pos_draw.append(self.ekf_pos[0:2].T[0].tolist())


if __name__ == '__main__':
    Game()
