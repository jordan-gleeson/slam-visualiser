import time
import operator
import random
import numpy as np
import pygame
import pygame_gui as pygui
import fonts


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

        pygame.display.flip()

        # Setup classes
        self.world = World(self.screen)
        self.robot = RobotControl(self.screen, self.world)
        self.robot.update()
        self.slam = SLAM(self.screen, self.robot)
        self.gui = GUI(self.screen, self.world, self.robot, self.slam)

        self.font = pygame.font.Font(None, 30)

        self.state = 0
        self.main()

    def main(self):
        """Main game loop."""
        _playing_game = True
        while _playing_game:
            _time_delta = self.clock.tick(30) / 1000.0
            self.screen.blit(self.background, (0, 0))
            for _event in pygame.event.get():
                if _event.type == pygame.QUIT:
                    _playing_game = False
                    break
                if _event.type == pygame.USEREVENT:
                    self.gui.input(_event)
                self.gui.manager.process_events(_event)
            if self.state == 0:
                if self.gui.main_menu_state == 0:
                    self.state += 1
                    self.gui.play_game()
            if self.state == 1:
                self.robot.change_velocity(pygame.key.get_pressed())
                self.world.draw()
                self.slam.update()
                self.robot.update()
                self.slam.odometry(self.robot.odo_velocity)
                if self.robot.robot.new_sample:
                    self.slam.occupancy_grid()
                    self.robot.robot.new_sample = False

            _fps = self.font.render(str(int(self.clock.get_fps())),
                                    True,
                                    pygame.Color('green'))
            self.screen.blit(_fps, (3, 3))
            self.gui.update(_time_delta)
            pygame.display.update()

        pygame.quit()


class GUI():
    """Contains all aspects of the GUI.

    Handles setup of GUI elements and the handling of input events.

    Attributes:
        _p_screen: The main pygame screen surface.
        _p_world: The world map object.
        _p_robot: The robot object.
        _p_slam: The slam algorithm object.
    """

    def __init__(self, _p_screen, _p_world, _p_robot, _p_slam):
        self.screen = _p_screen
        self.world = _p_world
        self.robot = _p_robot
        self.slam = _p_slam
        self.manager = pygui.UIManager(self.screen.get_size(), 'theme.json')
        self.manager.set_visual_debug_mode(False)

        self.settings_window = None

        # Main Menu Setup
        self.main_menu_state = True
        self.play_btn = None
        self.world_edit_btn = None
        self.slam_type_drp = None
        self.lidar_chk = None
        self.grid_chk = None
        self.pos_chk = None
        self.title = None
        self.main_menu()

        # Button Setup
        self.toggle_lidar_btn = None
        self.toggle_occupancy_grid_btn = None
        self.toggle_positions_btn = None
        self.done_btn = None
        self.settings_button = None

        # Position Visualisation Setup
        self.truth_pos = self.robot.truth_pos
        self.odo_pos = self.slam.odo_pos
        self.draw_positions = True

    def main_menu(self):
        _button_width = 110
        _button_height = 40
        _vert_padding = 50
        _hor_padding = 20

        _title_width = 290
        _title_rect = pygame.Rect((self.screen.get_width() / 2 - _title_width / 2, _vert_padding),
                                  (_title_width, 40))
        self.title = pygui.elements.UILabel(_title_rect,
                                            "SLAM Visualiser",
                                            self.manager)

        _world_edit_rect = pygame.Rect((self.screen.get_width() / 2 - _button_width / 2, _vert_padding*6),
                                       (_button_width, _button_height))
        self.world_edit_btn = pygui.elements.UIButton(relative_rect=_world_edit_rect,
                                                      text="Edit World",
                                                      manager=self.manager)

        _play_rect = pygame.Rect((self.screen.get_width() / 2 - _button_width / 2, _vert_padding*12),
                                 (_button_width, _button_height))
        self.play_btn = pygui.elements.UIButton(relative_rect=_play_rect,
                                                text="Play",
                                                manager=self.manager)

    def play_game(self):
        _settings_rect = pygame.Rect(
            (self.screen.get_size()[0] - 100, 20), (80, 30))
        self.settings_button = pygui.elements.UIButton(relative_rect=_settings_rect,
                                                       text="Settings",
                                                       manager=self.manager,
                                                       container=self.settings_window)

        self.world_edit_btn.kill()
        self.play_btn.kill()
        self.title.kill()

    def update(self, _time_delta):
        """Draws the GUI."""
        self.position_draw()
        self.manager.update(_time_delta)
        self.manager.draw_ui(self.screen)

    def input(self, _event):
        """Handles pygame_gui input events."""
        if _event.user_type == pygui.UI_BUTTON_PRESSED:
            if _event.ui_element == self.toggle_lidar_btn:
                self.robot.robot.toggle_lidar()
            if _event.ui_element == self.toggle_occupancy_grid_btn:
                self.slam.toggle_occupancy_grid()
            if _event.ui_element == self.settings_button:
                self.settings()
            if _event.ui_element == self.toggle_positions_btn:
                self.toggle_positions()
            if _event.ui_element == self.done_btn:
                self.settings_window.kill()
            if _event.ui_element == self.play_btn:
                self.main_menu_state = False

    def settings(self):
        """Settings window setup."""
        _button_width = 110
        _button_height = 40
        _vert_padding = 15
        _hor_padding = 20
        _window_width = _button_width + (_hor_padding * 4)
        _window_height = (_button_height * 3) + (_vert_padding * 4)

        # TODO: Fix window sizing to use above calculations
        _settings_window_rect = pygame.Rect(((self.screen.get_size()[0] / 2) - (180 / 2),
                                             self.screen.get_size()[1] / 4 - 300 / 2),
                                            (180, 300))
        self.settings_window = pygui.elements.UIWindow(rect=_settings_window_rect,
                                                       manager=self.manager)

        # Button Setup
        _lidar_rect = pygame.Rect((_hor_padding, _vert_padding),
                                  (_button_width, _button_height))
        self.toggle_lidar_btn = pygui.elements.UIButton(relative_rect=_lidar_rect,
                                                        text="Toggle Lidar",
                                                        manager=self.manager,
                                                        container=self.settings_window)
        _occupancy_rect = pygame.Rect((_hor_padding, _vert_padding * 2 + _button_height),
                                      (_button_width, _button_height))
        self.toggle_occupancy_grid_btn = pygui.elements.UIButton(relative_rect=_occupancy_rect,
                                                                 text="Toggle Grid",
                                                                 manager=self.manager,
                                                                 container=self.settings_window)
        _positions_rect = pygame.Rect((_hor_padding, _vert_padding * 3 + _button_height * 2),
                                      (_button_width, _button_height))
        self.toggle_positions_btn = pygui.elements.UIButton(relative_rect=_positions_rect,
                                                            text="Toggle Pos",
                                                            manager=self.manager,
                                                            container=self.settings_window)
        _done_rect = pygame.Rect((_hor_padding, _vert_padding * 4 + _button_height * 3),
                                 (_button_width, _button_height))
        self.done_btn = pygui.elements.UIButton(relative_rect=_done_rect,
                                                text="Done",
                                                manager=self.manager,
                                                container=self.settings_window)

    def position_draw(self):
        """Draw the lines that depict the robot's path historically."""
        if self.draw_positions:
            try:
                pygame.draw.lines(self.screen, (255, 0, 0),
                                  False, self.truth_pos)
                pygame.draw.lines(self.screen, (0, 0, 255),
                                  False, self.odo_pos)
            except ValueError:
                pass

    def toggle_positions(self):
        """Toggle whether or not the robot's historical path is visualised."""
        if self.draw_positions:
            self.draw_positions = False
        else:
            self.draw_positions = True


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
        self.image = pygame.transform.smoothscale(self.image, (50, 50))
        self.image_size = self.image.get_size()
        self.og_image = self.image.copy()
        self.rect = self.image.get_rect()
        self.x_pos = float(self.screen.get_size()[0] / 2)
        self.y_pos = float(self.screen.get_size()[1] / 2)
        self.angle = 0
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
        self.point_cloud = [[0, 0] for _ in range(self.sample_count)]
        self.angle_ref = []
        self.new_sample = True

        self.lasers = pygame.sprite.Group()
        _lidar = pygame.math.Vector2()
        _lidar.xy = (self.x_pos, self.y_pos)
        self.initial_laser_length = int(np.sqrt(
            np.square(self.screen.get_width()) + np.square(self.screen.get_height())))
        for i in range(self.sample_count):
            _degree_multiplier = 360 / self.sample_count
            _cur_angle = int(i * _degree_multiplier)
            self.angle_ref.append(_cur_angle)
            _laser = pygame.math.Vector2()
            _laser.from_polar((self.initial_laser_length, _cur_angle))
            _laser_sprite = Laser(self.screen, _lidar, _laser)
            self.lasers.add(_laser_sprite)
        self.lasers_draw = pygame.sprite.Group()

    def update(self):
        """Updates the position of the robot's rect, hitbox and mask."""
        self.rect.center = (self.x_pos, self.y_pos)
        self.hitbox.center = (self.x_pos, self.y_pos)
        self.mask = pygame.mask.from_surface(self.image)
        self.lidar()
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
                                   _coords,
                                   3)

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
                    cur_distance = point_distance(self.x_pos,
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
        self.velocity = [0, 0, 0]
        self.odo_velocity = self.velocity
        self.max_velocity = 4
        self.acceleration = 0.5
        self.cur_keys = []
        self.angular_velocity = 6
        self.dummy_screen = pygame.Surface(self.screen.get_size())
        self.collision_list = []
        self.recursion_depth = 0
        self.truth_pos = []

    def reset(self):
        """Reset the robot's attributes, including position and velocities."""
        self.robot.x_pos = self.screen.get_size()[0] / 2
        self.robot.y_pos = self.screen.get_size()[1] / 2
        self.robot.rect.center = (self.robot.x_pos, self.robot.y_pos)
        self.velocity = [0, 0, 0]
        self.robot.angle = 0
        self.update()

    def update(self):
        """Update all aspects of the robot, including velocities, position and lidar sensor."""
        self.move_velocity()
        self.robot.rotate(self.robot.angle)
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
        _collision_side = self.collision_detector()
        self.collision_list.append(_collision_side)
        if len(self.collision_list) > 3:
            self.collision_list.pop(0)
        if not _collision_side:
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
        self.odo_velocity = self.velocity
        if len(self.truth_pos) > 1000:
            self.truth_pos.pop(0)
        self.truth_pos.append([self.robot.x_pos, self.robot.y_pos])

        # Decelerate the velocity vector if no forward input is received.
        _deceleration = self.acceleration / 2
        if "UP" not in self.cur_keys:
            if self.velocity[0] > 0:
                self.velocity[0] -= _deceleration
            if self.velocity[0] < 0:
                self.velocity[0] += _deceleration
            if self.velocity[1] > 0:
                self.velocity[1] -= _deceleration
            if self.velocity[1] < 0:
                self.velocity[1] += _deceleration
            if self.velocity[0] < _deceleration and self.velocity[0] > _deceleration * -1:
                self.velocity[0] = 0
            if self.velocity[1] < _deceleration and self.velocity[1] > _deceleration * -1:
                self.velocity[1] = 0

    def change_velocity(self, _keys):
        """Controls the robot's velocity.

        This function receives input from the user and updates the Robot.angular_velocity and
        Robot.velocity vectors accordingly.

        Attributes:
            _keys: An array containing the current state of all keys.
        """
        # Get input and sets the rotation according to the angular velocity.
        _pressed_keys = self.convert_key(_keys)
        if "R" in _pressed_keys:
            self.reset()
        if "RIGHT" in _pressed_keys:
            self.robot.angle -= self.angular_velocity
        if "LEFT" in _pressed_keys:
            self.robot.angle += self.angular_velocity

        # Bind the robot.angle to remain < 180 and > -180.
        if self.robot.angle > 180:
            self.robot.angle = -180 + (self.robot.angle - 180)
        elif self.robot.angle < -180:
            self.robot.angle = 180 + (self.robot.angle + 180)

        # Calculate the current magnitude of the velocity vector.
        _speed = self.acceleration * 2
        self.velocity[2] = np.sqrt(
            np.square(self.velocity[0]) + np.square(self.velocity[1]))

        # Calculate the axis velocity components according to the current direction and desired
        # speed.
        _x_vec = np.cos(-1 * np.deg2rad(self.robot.angle + 90)) * _speed
        _y_vec = np.sin(-1 * np.deg2rad(self.robot.angle + 90)) * _speed
        if "UP" in _pressed_keys:
            self.velocity[0] += self.acceleration * _x_vec
            self.velocity[1] += self.acceleration * _y_vec
            self.velocity[2] = np.sqrt(
                np.square(self.velocity[0]) + np.square(self.velocity[1]))
            # Normalise the velocity vectors if the velocity's magnitude is greater than the
            # desired maximum velocity.
            if self.velocity[2] > self.max_velocity:
                _divider = self.max_velocity / \
                    np.sqrt(
                        np.square(self.velocity[0]) + np.square(self.velocity[1]))
                self.velocity[0] = _divider * self.velocity[0]
                self.velocity[1] = _divider * self.velocity[1]

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
                cur_distance = point_distance(self.robot.x_pos,
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
                distance = point_distance(_side[0],
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


class Laser(pygame.sprite.Sprite):
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
                if 20 < i < 30:
                    if 20 < j < 30:
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
        self.odo_x = self.robot.robot.x_pos
        self.odo_y = self.robot.robot.y_pos
        self.odo_error = 0.2
        self.odo_pos = []

    def update(self):
        """Update SLAM visuals."""
        if self.show_occupancy_grid:
            self.draw_grid()

    def odometry(self, _vel_vector):
        """Adds a random error to the positional data within a percentage tolerance."""
        try:
            self.odo_x += random.uniform(_vel_vector[0] - _vel_vector[0] * self.odo_error,
                                         _vel_vector[0] + _vel_vector[0] * self.odo_error)
            self.odo_y += random.uniform(_vel_vector[1] - _vel_vector[1] * self.odo_error,
                                         _vel_vector[1] + _vel_vector[1] * self.odo_error)
            if len(self.odo_pos) > 1000:
                self.odo_pos.pop(0)
            self.odo_pos.append([self.odo_x, self.odo_y])
        except ValueError:
            pass

    def occupancy_grid(self):
        """Occupance grid algorithm.

        Loops through all points in the point cloud and lowers the probability of a space in the
        grid being occupied if it is found on a line between the robot and a point, and increases
        the probability if it is found at the end-point of the laser.
        """
        def line(_x, _y, _a, _b):
            """Bresenham's line algorithm that returns a list of points."""
            _points_in_line = []
            _dx = abs(_a - _x)
            _dy = abs(_b - _y)
            _nx, _ny = _x, _y
            _sx = -1 if _x > _a else 1
            _sy = -1 if _y > _b else 1
            if _dx > _dy:
                _err = _dx / 2.0
                while _nx != _a:
                    _points_in_line.append((_nx, _ny))
                    _err -= _dy
                    if _err < 0:
                        _ny += _sy
                        _err += _dx
                    _nx += _sx
            else:
                _err = _dy / 2.0
                while _ny != _b:
                    _points_in_line.append((_nx, _ny))
                    _err -= _dx
                    if _err < 0:
                        _nx += _sx
                        _err += _dy
                    _ny += _sy
            _points_in_line.append((_nx, _ny))
            return _points_in_line

        _rate_of_change = 0.05  # The rate at which the probability of a point is changed
        _pc = self.robot.robot.point_cloud
        for _point in _pc:
            try:  # Catch instances where the end-point may be out of the game screen
                _coords = [int(_point[0] * np.cos(_point[1]) + self.odo_x),  # Convert to cartesian
                           int(_point[0] * np.sin(_point[1]) + self.odo_y)]
                # Loop through the points in between the robot and the end-point of a laser
                for _clear in line(self.robot.robot.x_pos // self.grid_size,
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


def point_distance(x_1, x_2, y_1, y_2):
    """Find the distance between two points on a 2D plane."""
    return np.sqrt(np.square(x_1 - x_2) + np.square(y_1 - y_2))


if __name__ == '__main__':
    Game()
