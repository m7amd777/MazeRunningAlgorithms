"""tremaux_maze_runner controller."""

from controller import Robot
from collections import defaultdict

# Directions
NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3

DIR_VECTORS = {
    NORTH: (0, 1),
    EAST: (1, 0),
    SOUTH: (0, -1),
    WEST: (-1, 0)
}


def normalize_edge(a, b):
    """Return edge in consistent order so (A,B)==(B,A)."""
    return tuple(sorted([a, b]))


def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    prox_sensors = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        sensor = robot.getDevice(sensor_name)
        sensor.enable(timestep)
        prox_sensors.append(sensor)

    # -----------------------------
    # Tremaux state
    # -----------------------------
    current_cell = (0, 0)      # logical maze cell
    heading = NORTH            # robot heading
    edge_marks = defaultdict(int)   # edge -> number of traversals
    junction_stack = []        # optional history, useful for debugging

    # Motion state machine
    state = "DECIDE"
    action = None
    action_steps = 0

    # Tune these experimentally
    WALL_THRESHOLD = 80
    FORWARD_STEPS_PER_CELL = 35   # how many control steps roughly move one cell
    TURN_STEPS_90 = 16            # how many control steps roughly turn 90 deg
    SLOW_FORWARD = 0.7 * max_speed
    TURN_SPEED = 0.6 * max_speed

    def read_openings():
        """
        Reuse your same wall logic style.
        Returns booleans for whether left/front/right are open.
        """
        # Keep same sensor interpretation style as your wall follower
        left_wall = prox_sensors[5].getValue() > WALL_THRESHOLD
        left_corner = prox_sensors[6].getValue() > WALL_THRESHOLD
        front_wall = prox_sensors[7].getValue() > WALL_THRESHOLD

        # Approximate right side using front-right sensors
        right_wall = (
            prox_sensors[0].getValue() > WALL_THRESHOLD or
            prox_sensors[1].getValue() > WALL_THRESHOLD
        )

        left_open = not left_wall and not left_corner
        front_open = not front_wall
        right_open = not right_wall

        return left_open, front_open, right_open

    def relative_to_absolute(direction_name):
        """Convert LEFT/FRONT/RIGHT/BACK relative to robot into absolute heading."""
        nonlocal heading
        if direction_name == "FRONT":
            return heading
        elif direction_name == "LEFT":
            return (heading - 1) % 4
        elif direction_name == "RIGHT":
            return (heading + 1) % 4
        elif direction_name == "BACK":
            return (heading + 2) % 4

    def next_cell_from_direction(abs_dir):
        dx, dy = DIR_VECTORS[abs_dir]
        return (current_cell[0] + dx, current_cell[1] + dy)

    def count_marks_for_move(direction_name):
        abs_dir = relative_to_absolute(direction_name)
        nxt = next_cell_from_direction(abs_dir)
        edge = normalize_edge(current_cell, nxt)
        return edge_marks[edge]

    def mark_move(direction_name):
        """Mark the corridor we are about to traverse."""
        abs_dir = relative_to_absolute(direction_name)
        nxt = next_cell_from_direction(abs_dir)
        edge = normalize_edge(current_cell, nxt)
        edge_marks[edge] += 1
        return nxt

    def choose_tremaux_direction(left_open, front_open, right_open):
        """
        Trémaux:
        1) Prefer unvisited open path (0 marks)
        2) Else choose open path with 1 mark
        3) Avoid 2-mark edges if possible
        4) BACK is allowed for backtracking
        """
        candidates = []

        if left_open:
            candidates.append("LEFT")
        if front_open:
            candidates.append("FRONT")
        if right_open:
            candidates.append("RIGHT")

        # BACK is always possible for backtracking in logical Tremaux
        candidates.append("BACK")

        # Separate by marks
        zero_mark = []
        one_mark = []
        two_plus = []

        for move in candidates:
            marks = count_marks_for_move(move)
            if marks == 0:
                zero_mark.append(move)
            elif marks == 1:
                one_mark.append(move)
            else:
                two_plus.append(move)

        # Preference order among same-mark options:
        # LEFT, FRONT, RIGHT, BACK
        pref_order = {"LEFT": 0, "FRONT": 1, "RIGHT": 2, "BACK": 3}

        if zero_mark:
            zero_mark.sort(key=lambda m: pref_order[m])
            return zero_mark[0]

        if one_mark:
            one_mark.sort(key=lambda m: pref_order[m])
            return one_mark[0]

        two_plus.sort(key=lambda m: pref_order[m])
        return two_plus[0]

    def start_action(chosen_move):
        """Start turn/forward action sequence."""
        nonlocal state, action, action_steps, junction_stack

        junction_stack.append((current_cell, heading, chosen_move))

        if chosen_move == "LEFT":
            action = "TURN_LEFT"
            action_steps = TURN_STEPS_90
            state = "ACT"
        elif chosen_move == "RIGHT":
            action = "TURN_RIGHT"
            action_steps = TURN_STEPS_90
            state = "ACT"
        elif chosen_move == "BACK":
            action = "TURN_BACK"
            action_steps = TURN_STEPS_90 * 2
            state = "ACT"
        else:  # FRONT
            action = "FORWARD"
            action_steps = FORWARD_STEPS_PER_CELL
            state = "ACT"

    def update_heading_after_turn(turn_action):
        nonlocal heading
        if turn_action == "TURN_LEFT":
            heading = (heading - 1) % 4
        elif turn_action == "TURN_RIGHT":
            heading = (heading + 1) % 4
        elif turn_action == "TURN_BACK":
            heading = (heading + 2) % 4

    while robot.step(timestep) != -1:
        # Read sensors
        sensor_vals = [s.getValue() for s in prox_sensors]
        print("Sensors:", sensor_vals)

        if state == "DECIDE":
            left_open, front_open, right_open = read_openings()

            print(f"\nAt cell {current_cell}, heading {heading}")
            print(f"Openings -> Left:{left_open}, Front:{front_open}, Right:{right_open}")

            chosen_move = choose_tremaux_direction(left_open, front_open, right_open)
            print("Chosen move:", chosen_move)

            # Mark the edge NOW, before moving through it
            future_cell = mark_move(chosen_move)
            print("Edge marked. Future cell will be:", future_cell)

            start_action(chosen_move)

        elif state == "ACT":
            left_speed = 0.0
            right_speed = 0.0

            if action == "TURN_LEFT":
                left_speed = -TURN_SPEED
                right_speed = TURN_SPEED

            elif action == "TURN_RIGHT":
                left_speed = TURN_SPEED
                right_speed = -TURN_SPEED

            elif action == "TURN_BACK":
                left_speed = TURN_SPEED
                right_speed = -TURN_SPEED

            elif action == "FORWARD":
                # Small correction using your wall follower idea
                left_wall = prox_sensors[5].getValue() > WALL_THRESHOLD
                left_corner = prox_sensors[6].getValue() > WALL_THRESHOLD
                front_wall = prox_sensors[7].getValue() > WALL_THRESHOLD

                left_speed = SLOW_FORWARD
                right_speed = SLOW_FORWARD

                # Gentle wall-following correction while moving forward
                if front_wall:
                    # emergency stop if too close
                    left_speed = 0.0
                    right_speed = 0.0
                else:
                    if not left_wall:
                        # drift left -> steer left slightly
                        left_speed = 0.5 * max_speed
                        right_speed = 0.75 * max_speed
                    if left_corner:
                        # too close to left -> steer right slightly
                        left_speed = 0.75 * max_speed
                        right_speed = 0.4 * max_speed

            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

            action_steps -= 1

            if action_steps <= 0:
                # Finish action
                if action in ["TURN_LEFT", "TURN_RIGHT", "TURN_BACK"]:
                    update_heading_after_turn(action)

                    # After any turn, move forward one cell
                    action = "FORWARD"
                    action_steps = FORWARD_STEPS_PER_CELL

                elif action == "FORWARD":
                    # Update logical position
                    dx, dy = DIR_VECTORS[heading]
                    current_cell = (current_cell[0] + dx, current_cell[1] + dy)

                    print("Arrived at cell:", current_cell)
                    state = "DECIDE"

        else:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)


if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)