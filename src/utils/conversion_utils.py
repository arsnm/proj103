from src.config import ControlConfig
import numpy as np

control_config = ControlConfig()
grid_size_x = control_config.GRID_SIZE_X
grid_size_y = control_config.GRID_SIZE_Y
case_size = control_config.GRID_CASE_SIZE


def match_coord_to_case(position):
    x, y = position.x, position.y
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


def parse_instructions(instructions: str):
    list_instructions = instructions.split(",")
    list_movements = []
    for instruction in list_instructions:
        movement_type = instruction[0]
        movement_data = np.radians(
            -int(instruction[1:])
        )  # - for correct orientation convention
        if movement_type == "r" or movement_type == "a":
            list_movements.append((movement_type, movement_data))
        else:
            print(f"wrong instruction ({instruction}) sent")
    return list_movements
