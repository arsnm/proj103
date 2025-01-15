from src.config import GridDimensions
from numpy import pi

case_size = GridDimensions.GRID_CASE
grid_size_x = GridDimensions.GRID_SIZE[0] * case_size
grid_size_y = GridDimensions.GRID_SIZE[1] * case_size


def match_coord_to_case(x, y):
    global case_size, grid_size_x, grid_size_y
    if x < 0 or x > grid_size_x or y < -case_size or y > grid_size_y:
        return None
    y += case_size
    # Calculate the column number (1-based)
    col = int((x - 1) // case_size) + 1

    complete_squares = (grid_size_y - y) // case_size

    letter_index = int(complete_squares)

    letter = chr(65 + letter_index)

    return f"{letter}{col}"


def match_case_to_coord(case: str):
    global case_size, grid_size_x, grid_size_y
    if not case or len(case) < 2:
        return None

    letter = case[0].upper()
    try:
        number = int(case[1:])
    except ValueError:
        return None

    if not "A" <= letter <= "Z":
        return None

    max_rows = int(grid_size_y // case_size)
    max_cols = int(grid_size_x // case_size)

    if number < 1 or number > max_cols:
        return None

    letter_index = ord(letter) - ord("A")

    if letter_index >= max_rows:
        return None

    # Calculate x coordinate (center of the case)
    x = (number - 0.5) * case_size

    # Calculate y coordinate (center of the case)
    # Remember: A is at the top, so we subtract from grid_size_y
    y = grid_size_y - (letter_index + 0.5) * case_size

    return (x, y)


def relative_to_absolute_coord(id, quadrant, rel_x, rel_y):
    global grid_size_x, grid_size_y
    pose = None, None
    if quadrant == "NORTH":
        if id == 1:
            pose = (-rel_x, grid_size_y - rel_y)
        elif id == 2:
            pose = (grid_size_x + rel_x, grid_size_y - rel_y)
    elif quadrant == "WEST":
        if id == 1:
            pose = (rel_y, grid_size_y - rel_x)
        elif id == 4:
            pose = (rel_y, -rel_x)
    elif quadrant == "SOUTH":
        if id == 4:
            pose = (-rel_x, rel_y)
        elif id == 3:
            pose = (grid_size_x - rel_x, rel_y)
    elif quadrant == "EAST":
        if id == 2:
            pose = (grid_size_x - rel_y, -rel_x)
        elif id == 3:
            pose = (grid_size_x - rel_y, grid_size_y + rel_x)
    return pose


def id_to_orientation(id):
    if id == 1:
        return pi / 4
    elif id == 2:
        return pi * 7 / 4
    elif id == 3:
        return pi * 5 / 4
    elif id == 4:
        return pi * 3 / 4


def orientation_to_quadrant(orientation):
    orientation %= 2 * pi
    if pi / 4 <= orientation < pi * 3 / 4:
        return "WEST"
    elif pi * 3 / 4 <= orientation < pi * 5 / 4:
        return "SOUTH"
    elif pi * 5 / 4 <= orientation < pi * 7 / 4:
        return "EAST"
    else:
        return "NORTH"
