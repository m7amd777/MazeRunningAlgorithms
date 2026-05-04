"""tremaux controller without wall-following behavior."""

from controller import Robot
from collections import defaultdict

# Headings
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
    for i in range(8):
        sensor = robot.getDevice(f'ps{i}')
        sensor.enable(timestep)
        prox_sensors.append(sensor)

    # -----------------------------
    # Tremaux memory
    # -----------------------------
    current_cell = (0, 0)
    heading = NORTH
    edge_marks = defaultdict(int)

    # -----------------------------
    # Motion state
    # -----------------------------
    state = "SENSE"
    action = None
    action_steps = 0
    pending_move = None

    # -----------------------------
    # Tune these
    # -----------------------------
    PAUSE_STEPS         = 30       # pause between each phase so you can observe
    
    
    
    # WALL_THRESHOLD = 78
    WALL_THRESHOLD_FRONT    = 80    # ps0, ps7
    WALL_THRESHOLD_SIDE     = 72
    TURN_STEPS_90 = 32
    TURN_STEPS_180  = 68      # steps for a 180° turn in one go — tune separately!
    FORWARD_STEPS_CELL = 35

    TURN_SPEED = 0.6 * max_speed
    FORWARD_SPEED = 0.7 * max_speed

    def stop():
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

    def get_openings():
        """
        Detect left/front/right openings only.
        No wall-following corrections.
        """
        left_blocked = (
            prox_sensors[5].getValue() > WALL_THRESHOLD_SIDE
            # prox_sensors[6].getValue() > WALL_THRESHOLD 
        )
            # prox_sensors[6].getValue() > WALL_THRESHOLD
        front_blocked = (
            prox_sensors[7].getValue() > WALL_THRESHOLD_FRONT or
            prox_sensors[0].getValue() > WALL_THRESHOLD_FRONT
        ) 
        
        right_blocked = (
            prox_sensors[2].getValue() > WALL_THRESHOLD_SIDE
            # prox_sensors[1].getValue() > WALL_THRESHOLD
        )
                    # prox_sensors[1].getValue() > WALL_THRESHOLD

        left_open = not left_blocked
        front_open = not front_blocked
        right_open = not right_blocked

        return left_open, front_open, right_open

    def relative_to_absolute(move):
        nonlocal heading
        if move == "FRONT":
            return heading
        elif move == "LEFT":
            return (heading - 1) % 4
        elif move == "RIGHT":
            return (heading + 1) % 4
        elif move == "BACK":
            return (heading + 2) % 4

    def get_next_cell(move):
        abs_dir = relative_to_absolute(move)
        dx, dy = DIR_VECTORS[abs_dir]
        return (current_cell[0] + dx, current_cell[1] + dy)

    def get_edge_mark(move):
        nxt = get_next_cell(move)
        edge = normalize_edge(current_cell, nxt)
        return edge_marks[edge]

    def mark_edge(move):
        nxt = get_next_cell(move)
        edge = normalize_edge(current_cell, nxt)
        edge_marks[edge] += 1

    def choose_tremaux(left_open, front_open, right_open):
        """
        Trémaux rules:
        - Prefer unvisited open path
        - Else path visited once
        - Avoid twice-marked if possible
        - BACK is for backtracking
        """
        candidates = []

        if left_open:
            candidates.append("LEFT")
        if front_open:
            candidates.append("FRONT")
        if right_open:
            candidates.append("RIGHT")

        # Backtracking is logically always allowed
        candidates.append("BACK")

        zero_marks = []
        one_mark = []
        two_plus = []

        for move in candidates:
            marks = get_edge_mark(move)
            if marks == 0:
                zero_marks.append(move)
            elif marks == 1:
                one_mark.append(move)
            else:
                two_plus.append(move)

        # Fixed priority among same mark counts
        priority = {"LEFT": 0, "FRONT": 1, "RIGHT": 2, "BACK": 3}

        if zero_marks:
            zero_marks.sort(key=lambda m: priority[m])
            return zero_marks[0]

        if one_mark:
            one_mark.sort(key=lambda m: priority[m])
            return one_mark[0]

        two_plus.sort(key=lambda m: priority[m])
        return two_plus[0]

    def start_move(move):
        nonlocal state, action, action_steps, pending_move
        pending_move = move

        if move == "LEFT":
            action = "TURN_LEFT"
            action_steps = TURN_STEPS_90
        elif move == "RIGHT":
            action = "TURN_RIGHT"
            action_steps = TURN_STEPS_90
        elif move == "BACK":
            action = "TURN_BACK"
            action_steps = TURN_STEPS_180
        else:
            action = "FORWARD"
            action_steps = FORWARD_STEPS_CELL

        state = "MOVE"

    def update_heading(turn_action):
        nonlocal heading
        if turn_action == "TURN_LEFT":
            heading = (heading - 1) % 4
        elif turn_action == "TURN_RIGHT":
            heading = (heading + 1) % 4
        elif turn_action == "TURN_BACK":
            heading = (heading + 2) % 4

    while robot.step(timestep) != -1:

        if state == "SENSE":
            stop()

            left_open, front_open, right_open = get_openings()

            print(f"\nCell: {current_cell}, Heading: {heading}")
            print(f"Openings -> L:{left_open}, F:{front_open}, R:{right_open}")

            chosen_move = choose_tremaux(left_open, front_open, right_open)
            print("Chosen move:", chosen_move)

            # Mark selected corridor before traversal
            mark_edge(chosen_move)
            print("Mark count after marking:", get_edge_mark(chosen_move))

            start_move(chosen_move)

        elif state == "MOVE":
            if action == "TURN_LEFT":
                left_motor.setVelocity(-TURN_SPEED)
                right_motor.setVelocity(TURN_SPEED)

            elif action == "TURN_RIGHT":
                left_motor.setVelocity(TURN_SPEED)
                right_motor.setVelocity(-TURN_SPEED)

            elif action == "TURN_BACK":
                left_motor.setVelocity(TURN_SPEED)
                right_motor.setVelocity(-TURN_SPEED)

            elif action == "FORWARD":
                left_motor.setVelocity(FORWARD_SPEED)
                right_motor.setVelocity(FORWARD_SPEED)

            action_steps -= 1

            if action_steps <= 0:
                stop()

                # If a turn just finished, go forward one cell next
                if action in ["TURN_LEFT", "TURN_RIGHT", "TURN_BACK"]:
                    update_heading(action)
                    action = "FORWARD"
                    action_steps = FORWARD_STEPS_CELL

                elif action == "FORWARD":
                    dx, dy = DIR_VECTORS[heading]
                    current_cell = (current_cell[0] + dx, current_cell[1] + dy)
                    print("Moved to:", current_cell)
                    state = "SENSE"

    stop()


if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)