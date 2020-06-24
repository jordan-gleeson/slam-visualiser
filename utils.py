def line_between(_x, _y, _a, _b):
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