import numpy as np
import pygame
import pygame_gui as pygui
import utils


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
        self.main_menu()

        # Settings Button Setup
        self.toggle_lidar_btn = None
        self.toggle_occupancy_grid_btn = None
        self.toggle_positions_btn = None
        self.done_btn = None
        self.settings_button = None
        self.reset_btn = None
        self.settings_tlt_pnl = None
        self.settings_lbl_pnl = None

        # Position Visualisation Setup
        self.draw_positions = True

        # World Editor Setup
        self.last_mouse_pos = None
        self.we_done_btn = None
        self.we_clear_btn = None
        self.we_mode_btn = None
        self.we_draw_mode = True
        self.we_raise_click = False

    def main_menu(self):
        """Setup the main menu."""
        _button_width = 110
        _button_height = 40
        _vert_out_padding = 60
        _hor_out_padding = 60
        _vert_in_padding = 30
        _hor_in_padding = 30
        _vert_inner_padding = 20
        _hor_inner_padding = 20
        _start_button_height = 80

        _main_menu_pnl_rect = pygame.Rect((0, 0),
                                          (self.screen.get_width(), self.screen.get_height()))
        self.main_menu_pnl = pygui.elements.UIPanel(_main_menu_pnl_rect, 0,
                                                    self.manager,
                                                    object_id="background_panel")

        _title_panel_pos = (_hor_out_padding, _vert_out_padding)
        _title_panel_size = (self.screen.get_width() -
                             _hor_out_padding * 2, 100)
        _title_panel_rect = pygame.Rect(_title_panel_pos, _title_panel_size)
        self.title_pnl = pygui.elements.UIPanel(_title_panel_rect, 0,
                                                manager=self.manager,
                                                object_id="title_panel")

        _title_size = (354, 45)
        _title_pos = (_title_panel_pos[0] + _hor_inner_padding,
                      _title_panel_pos[1] + _title_panel_size[1] / 2 - _title_size[1] / 2 + 5)
        _title_rect = pygame.Rect(_title_pos, _title_size)
        self.title = pygui.elements.UILabel(_title_rect,
                                            "SLAM Visualiser",
                                            self.manager,
                                            object_id="title_label")

        _setup_panel_pos = (_hor_out_padding,
                            _title_panel_pos[1] + _title_panel_size[1] + _vert_in_padding)
        _setup_panel_size = ((self.screen.get_width() - _hor_out_padding * 2) / 3 - _hor_in_padding / 2,
                             self.screen.get_height() - _hor_out_padding * 2 - _hor_in_padding * 2 - _title_panel_size[1] - _start_button_height)
        _setup_panel_rect = pygame.Rect(_setup_panel_pos, _setup_panel_size)
        self.setup_pnl = pygui.elements.UIPanel(_setup_panel_rect, 0,
                                                manager=self.manager,
                                                object_id="menu_panel")

        _setup_label_panel_pos = (_setup_panel_pos[0] + _hor_inner_padding,
                                  _setup_panel_pos[1] + _vert_inner_padding)
        _setup_label_panel_size = (_setup_panel_size[0] - _hor_inner_padding * 2,
                                   70)
        _setup_label_panel_rect = pygame.Rect(_setup_label_panel_pos,
                                              _setup_label_panel_size)
        self.setup_lbl_pnl = pygui.elements.UIPanel(_setup_label_panel_rect, 0,
                                                    manager=self.manager,
                                                    object_id="title_panel")

        _setup_title_size = (98, 35)
        _setup_title_pos = (_setup_label_panel_pos[0] + _hor_inner_padding,
                            _setup_label_panel_pos[1] + _setup_label_panel_size[1] / 2 - _setup_title_size[1] / 2 + 3)
        _setup_title_rect = pygame.Rect(_setup_title_pos, _setup_title_size)
        self.setup_ttl = pygui.elements.UILabel(_setup_title_rect,
                                                "Setup",
                                                self.manager,
                                                object_id="panel_title_label")

        _world_edit_size = (_button_width, _button_height)
        _world_edit_pos = (_setup_label_panel_pos[0],
                           _setup_label_panel_pos[1] + _setup_label_panel_size[1] + _vert_inner_padding)
        _world_edit_rect = pygame.Rect(_world_edit_pos, _world_edit_size)
        self.world_edit_btn = pygui.elements.UIButton(relative_rect=_world_edit_rect,
                                                      text="Edit World",
                                                      manager=self.manager,
                                                      object_id="setup_button")

        _slam_type_size = (_button_width + 60, _button_height)
        _slam_type_pos = (_setup_label_panel_pos[0],
                          _world_edit_pos[1] + _world_edit_size[1] + _vert_inner_padding)
        _slam_type_rect = pygame.Rect(_slam_type_pos, _slam_type_size)
        _slam_list = ["Occupancy Grid", "Landmarks"]
        self.slam_type_drop = pygui.elements.UIDropDownMenu(relative_rect=_slam_type_rect,
                                                            options_list=_slam_list,
                                                            starting_option="Landmarks",
                                                            manager=self.manager,
                                                            object_id="setup_dropdown",
                                                            expansion_height_limit=len(_slam_list)*50)

        _start_button_pos = (_hor_out_padding,
                             _setup_panel_pos[1] + _setup_panel_size[1] + _vert_in_padding)
        _start_button_size = (_setup_panel_size[0], _start_button_height)
        _start_button_rect = pygame.Rect(_start_button_pos, _start_button_size)
        self.start_btn = pygui.elements.UIButton(relative_rect=_start_button_rect,
                                                 text="Start",
                                                 manager=self.manager,
                                                 object_id="start_button")

        _preview_panel_pos = (_setup_panel_pos[0] + _setup_panel_size[0] + _hor_in_padding,
                              _setup_panel_pos[1])
        _preview_panel_size = (_setup_panel_size[0] * 2 + _hor_in_padding / 2,
                               (_start_button_pos[1] + _start_button_height - _setup_panel_pos[1]) / 2 - _hor_in_padding / 2)
        _preview_panel_rect = pygame.Rect(_preview_panel_pos,
                                          _preview_panel_size)
        self.preview_pnl = pygui.elements.UIPanel(_preview_panel_rect, 0,
                                                  manager=self.manager,
                                                  object_id="menu_panel")

        _instructions_panel_pos = (_preview_panel_pos[0],
                                   _preview_panel_pos[1] + _preview_panel_size[1] + _hor_in_padding)
        _instructions_panel_size = _preview_panel_size
        _instructions_panel_rect = pygame.Rect(_instructions_panel_pos,
                                               _instructions_panel_size)
        self.instructions_pnl = pygui.elements.UIPanel(_instructions_panel_rect, 0,
                                                       manager=self.manager,
                                                       object_id="menu_panel")

        _preview_label_panel_pos = (_preview_panel_pos[0] + _hor_inner_padding,
                                    _preview_panel_pos[1] + _vert_inner_padding)
        _preview_label_panel_size = (_preview_panel_size[0] - _hor_inner_padding * 2,
                                     50)
        _preview_label_panel_rect = pygame.Rect(_preview_label_panel_pos,
                                                _preview_label_panel_size)
        self.preview_lbl_pnl = pygui.elements.UIPanel(_preview_label_panel_rect, 0,
                                                      manager=self.manager,
                                                      object_id="title_panel")

        _preview_title_size = (138, 35)
        _preview_title_pos = (_preview_label_panel_pos[0] + _hor_inner_padding,
                              _preview_label_panel_pos[1] + _preview_label_panel_size[1] / 2 - _preview_title_size[1] / 2 + 3)
        _preview_title_rect = pygame.Rect(_preview_title_pos,
                                          _preview_title_size)
        self.preview_ttl = pygui.elements.UILabel(_preview_title_rect,
                                                  "Preview",
                                                  self.manager,
                                                  object_id="panel_title_label")

        _instructions_label_panel_pos = (_instructions_panel_pos[0] + _hor_inner_padding,
                                         _instructions_panel_pos[1] + _vert_inner_padding)
        _instructions_label_panel_size = (_instructions_panel_size[0] - _hor_inner_padding * 2,
                                          50)
        _instructions_label_panel_rect = pygame.Rect(_instructions_label_panel_pos,
                                                     _instructions_label_panel_size)
        self.instructions_lbl_pnl = pygui.elements.UIPanel(_instructions_label_panel_rect, 0,
                                                           manager=self.manager,
                                                           object_id="title_panel")

        _instructions_title_size = (202, 35)
        _instructions_title_pos = (_instructions_label_panel_pos[0] + _hor_inner_padding,
                                   _instructions_label_panel_pos[1] + _instructions_label_panel_size[1] / 2 - _instructions_title_size[1] / 2 + 3)
        _instructions_title_rect = pygame.Rect(_instructions_title_pos,
                                               _instructions_title_size)
        self.instructions_ttl = pygui.elements.UILabel(_instructions_title_rect,
                                                       "Instructions",
                                                       self.manager,
                                                       object_id="panel_title_label")

    def setup_game(self, _world_edited):
        """Add game buttons. Write the world map to sprites."""
        _settings_rect_size = (80, 30)
        _settings_rect = pygame.Rect((self.screen.get_size()[0] - 10 - _settings_rect_size[0], 10),
                                     _settings_rect_size)
        self.settings_button = pygui.elements.UIButton(relative_rect=_settings_rect,
                                                       text="Settings",
                                                       manager=self.manager,
                                                       container=self.settings_window,
                                                       object_id="setup_button")

        self.kill_main_menu()

        if not _world_edited:
            self.world.write_map(self.robot.robot.robot_size)
        self.world.create_sprites()

    def kill_main_menu(self):
        """Removes main menu buttons."""
        try:
            self.main_menu_pnl.kill()
            self.world_edit_btn.kill()
            self.start_btn.kill()
            self.title.kill()
            self.title_pnl.kill()
            self.setup_pnl.kill()
            self.setup_lbl_pnl.kill()
            self.setup_ttl.kill()
            self.preview_pnl.kill()
            self.instructions_pnl.kill()
            self.preview_ttl.kill()
            self.preview_lbl_pnl.kill()
            self.instructions_lbl_pnl.kill()
            self.instructions_ttl.kill()
            self.slam_type_drop.kill()
        except:
            pass

    def kill_world_editor(self):
        """Removes world editor buttons."""
        try:
            self.we_done_btn.kill()
            self.we_clear_btn.kill()
            self.we_mode_btn.kill()
        except:
            pass

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
            if _event.ui_element == self.start_btn:
                self.main_menu_state = 0
            if _event.ui_element == self.world_edit_btn:
                self.main_menu_state = 2
            if _event.ui_element == self.we_done_btn:
                self.main_menu_state = 1
            if _event.ui_element == self.we_clear_btn:
                self.world.clear_map()
            if _event.ui_element == self.we_mode_btn:
                self.world_editor_mode_button()
            if _event.ui_element == self.reset_btn:
                self.reset()

    def settings(self):
        """Settings window setup."""
        _button_width = 110
        _button_height = 40
        _vert_padding = 15
        _hor_padding = 30
        _button_count = 6
        _border = 4 * 1.5

        _setting_window_size = (_button_width + _hor_padding * 2,
                                _button_height * _button_count + _vert_padding * (_button_count + 1))
        _settings_window_pos = (self.screen.get_size()[0] - _setting_window_size[0] - 10,
                                10)
        _settings_window_rect = pygame.Rect(_settings_window_pos,
                                            _setting_window_size)
        self.settings_window = pygui.elements.UIWindow(rect=_settings_window_rect,
                                                       manager=self.manager,
                                                       object_id="settings_window")

        _settings_label_panel_pos = (_hor_padding - _border, _vert_padding)
        _settings_label_panel_size = (_button_width, _button_height)
        _settings_label_panel_rect = pygame.Rect(_settings_label_panel_pos,
                                                 _settings_label_panel_size)
        self.settings_lbl_pnl = pygui.elements.UIPanel(_settings_label_panel_rect, 0,
                                                       manager=self.manager,
                                                       container=self.settings_window,
                                                       object_id="title_panel")

        _settings_title_size = (80, 25)
        _settings_title_pos = (_hor_padding + 10 - _border,
                               _settings_label_panel_pos[1] + (_settings_label_panel_size[1] - _settings_title_size[1]) / 2 + 3)
        _settings_title_rect = pygame.Rect(_settings_title_pos,
                                           _settings_title_size)
        self.settings_tlt_pnl = pygui.elements.UILabel(_settings_title_rect,
                                                       "Settings",
                                                       self.manager,
                                                       container=self.settings_window,
                                                       object_id="settings_title_label")

        # Button Setup
        _lidar_button_pos = (_hor_padding - _border,
                             _settings_label_panel_pos[1] + _button_height + _vert_padding)
        _lidar_button_rect = pygame.Rect(
            _lidar_button_pos, (_button_width, _button_height))
        self.toggle_lidar_btn = pygui.elements.UIButton(relative_rect=_lidar_button_rect,
                                                        text="Toggle Lidar",
                                                        manager=self.manager,
                                                        container=self.settings_window,
                                                        object_id="setup_button")

        _occupancy_button_pos = (_hor_padding - _border,
                                 _lidar_button_pos[1] + _vert_padding + _button_height)
        _occupancy_button_rect = pygame.Rect(_occupancy_button_pos,
                                             (_button_width, _button_height))
        self.toggle_occupancy_grid_btn = pygui.elements.UIButton(relative_rect=_occupancy_button_rect,
                                                                 text="Toggle Grid",
                                                                 manager=self.manager,
                                                                 container=self.settings_window,
                                                                 object_id="setup_button")

        _positions_button_pos = (_hor_padding - _border,
                                 _occupancy_button_pos[1] + _vert_padding + _button_height)
        _positions_button_rect = pygame.Rect(_positions_button_pos,
                                             (_button_width, _button_height))
        self.toggle_positions_btn = pygui.elements.UIButton(relative_rect=_positions_button_rect,
                                                            text="Toggle Pos",
                                                            manager=self.manager,
                                                            container=self.settings_window,
                                                            object_id="setup_button")

        _reset_button_pos = (_hor_padding - _border,
                             _positions_button_pos[1] + _vert_padding + _button_height)
        _reset_button_rect = pygame.Rect(_reset_button_pos,
                                         (_button_width, _button_height))
        self.reset_btn = pygui.elements.UIButton(relative_rect=_reset_button_rect,
                                                 text="Reset",
                                                 manager=self.manager,
                                                 container=self.settings_window,
                                                 object_id="setup_button")

        _done_button_pos = (_hor_padding - _border,
                            _reset_button_pos[1] + _vert_padding + _button_height)
        _done_button_rect = pygame.Rect(_done_button_pos,
                                        (_button_width, _button_height))
        self.done_btn = pygui.elements.UIButton(relative_rect=_done_button_rect,
                                                text="Done",
                                                manager=self.manager,
                                                container=self.settings_window,
                                                object_id="done_button")

    def position_draw(self):
        """Draw the lines that depict the robot's path historically."""
        if self.draw_positions:
            try:
                pygame.draw.lines(self.screen, (255, 0, 0),
                                  False, self.robot.truth_pos)
                pygame.draw.lines(self.screen, (0, 0, 255),
                                  False, self.slam.odo_pos)
                pygame.draw.lines(self.screen, (0, 255, 0),
                                  False, self.slam.ekf_pos_draw)
            except ValueError:
                pass

    def toggle_positions(self):
        """Toggle whether or not the robot's historical path is visualised."""
        if self.draw_positions:
            self.draw_positions = False
        else:
            self.draw_positions = True

    def world_editor_setup(self):
        """Setup the world editor screen."""
        _button_width = 110
        _button_height = 40
        _vert_padding = 20
        _hor_padding = 20

        _done_rect = pygame.Rect((self.screen.get_width() - _button_width - _hor_padding,
                                  self.screen.get_height() - _button_height - _vert_padding),
                                 (_button_width, _button_height))
        self.we_done_btn = pygui.elements.UIButton(relative_rect=_done_rect,
                                                   text="Done",
                                                   manager=self.manager,
                                                   object_id="done_button")

        _clear_rect = pygame.Rect((self.screen.get_width() - _button_width - _hor_padding,
                                   _vert_padding),
                                  (_button_width, _button_height))
        self.we_clear_btn = pygui.elements.UIButton(relative_rect=_clear_rect,
                                                    text="Clear",
                                                    manager=self.manager,
                                                    object_id="setup_button")

        _mode_rect = pygame.Rect((self.screen.get_width() - _button_width - _hor_padding,
                                  _vert_padding * 2 + _button_height),
                                 (_button_width, _button_height))
        self.we_mode_btn = pygui.elements.UIButton(relative_rect=_mode_rect,
                                                   text="Erase",
                                                   manager=self.manager,
                                                   object_id="setup_button")

    def world_editor_mode_button(self):
        """Toggle between draw/erase modes of the world editor."""
        if self.we_draw_mode:
            self.we_mode_btn.set_text("Draw")
            self.we_draw_mode = False
        else:
            self.we_mode_btn.set_text("Erase")
            self.we_draw_mode = True

    def world_editor(self, _mouse_click, _pos):
        """Draw onto the world grid if mouse is down and draw the current world grid."""

        def world_editor_button_hover(_bh_pos):
            """Return true if the position is within any of the world editor buttons."""
            _return = np.array([self.we_clear_btn.hover_point(_bh_pos[0],
                                                              _bh_pos[1]),
                                self.we_done_btn.hover_point(_bh_pos[0],
                                                             _bh_pos[1]),
                                self.we_mode_btn.hover_point(_bh_pos[0],
                                                             _bh_pos[1])])
            return _return.any()

        def world_editor_centre_hover(_ch_pos):
            """Return true if the position is within where the robot will spawn."""
            _hor_cen = self.screen.get_width() / 2
            _vert_cen = self.screen.get_height() / 2
            _robot_size = self.robot.robot.robot_size
            _return = np.array([_ch_pos[0] > _hor_cen - _robot_size,
                                _ch_pos[0] < _hor_cen + _robot_size,
                                _ch_pos[1] < _vert_cen + _robot_size,
                                _ch_pos[1] > _vert_cen - _robot_size])
            return _return.all()

        def pos_to_grid(_pos):
            """Converts game space coordinates to world map grid coordinates."""
            return int(_pos / self.world.size)

        if _mouse_click:
            if self.world.world_type == "Occupancy Grid" or not self.we_draw_mode:
                # If in Occupancy Grid mode, find the distance between the last known mouse
                # position and find the points in a line between them
                if self.last_mouse_pos != None:
                    _last_point_dis = utils.point_distance(self.last_mouse_pos[0], _pos[0],
                                                           _pos[1], self.last_mouse_pos[1])
                else:
                    _last_point_dis = 0
                # If clicking on a button don't draw anything
                if (_last_point_dis < 8 and world_editor_button_hover(_pos)) or _last_point_dis == 0:
                    _line = []
                else:
                    _line = utils.line_between(self.last_mouse_pos[0],
                                               self.last_mouse_pos[1],
                                               _pos[0], _pos[1])
                # Write to the grid map all the points on the line if not in the spawn space
                for _point in _line:
                    if not world_editor_centre_hover(_point):
                        self.world.write_to_map(self.we_draw_mode,
                                                pos_to_grid(_point[0]),
                                                pos_to_grid(_point[1]))
                self.last_mouse_pos = _pos
            elif self.world.world_type == "Landmarks":
                # If in landmark mode, only place one wall per click
                if self.we_raise_click:
                    if not world_editor_centre_hover(_pos):
                        self.world.write_to_map(self.we_draw_mode,
                                                pos_to_grid(_pos[0]),
                                                pos_to_grid(_pos[1]))
                        self.we_raise_click = False

        for i in range(len(self.world.grid)):
            for j in range(len(self.world.grid[0])):
                if self.world.grid[i][j]:
                    pygame.draw.rect(self.screen,
                                     (0, 0, 0),
                                     pygame.Rect((j * self.world.size, i * self.world.size),
                                                 (self.world.size, self.world.size)))

    def reset(self):
        """Reset the game state."""
        self.robot.reset()
        self.slam.reset()
