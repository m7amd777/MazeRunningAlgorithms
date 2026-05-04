"""
Calibration controller for tuning TURN_STEPS_90 and FORWARD_STEPS_CELL.

HOW TO USE:
  1. Set the values in the TUNE block below.
  2. Set TEST_MODE to what you want to test:
       "LEFT"    — turn left 90°, pause, turn right 90° back, repeat
       "RIGHT"   — turn right 90°, pause, turn left 90° back, repeat
       "BACK"    — turn 180°, pause, turn 180° back, repeat
       "FORWARD" — drive forward one cell, pause, drive back, repeat
  3. Run in Webots and watch if the robot lands exactly where it started.
  4. Adjust TURN_STEPS_90 or FORWARD_STEPS_CELL and re-run.

READING THE OUTPUT:
  Each phase is printed with its step count so you can track timing.
  If the robot overshoots → decrease the relevant STEPS value.
  If the robot undershoots → increase the relevant STEPS value.

NOTE on TURN_STEPS_180:
  A 180° turn done in one continuous motion does NOT equal TURN_STEPS_90 * 2.
  Each stop() call lets the robot coast a bit extra. Two separate 90° turns
  have two coast events (overshoots); one continuous 180° has only one (and
  the motor stays at speed longer, so fewer steps cover the same angle).
  Tune TURN_STEPS_180 independently from TURN_STEPS_90.
"""

from controller import Robot

# -------------------------------------------------------
# TUNE THESE
# -------------------------------------------------------
TEST_MODE           = "LEFT"   # "LEFT", "RIGHT", "BACK", "FORWARD"
TURN_STEPS_90       = 32       # steps for a 90° turn (with one coast stop)
TURN_STEPS_180      = 68      # steps for a 180° turn in one go — tune separately!
FORWARD_STEPS_CELL  = 35       # steps to cross one cell
TURN_SPEED          = 0.6 * 6.28
FORWARD_SPEED       = 0.7 * 6.28
PAUSE_STEPS         = 30       # pause between each phase so you can observe
# -------------------------------------------------------


def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())

    left_motor  = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    def stop():
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

    def turn_left():
        left_motor.setVelocity(-TURN_SPEED)
        right_motor.setVelocity(TURN_SPEED)

    def turn_right():
        left_motor.setVelocity(TURN_SPEED)
        right_motor.setVelocity(-TURN_SPEED)

    def drive_forward():
        left_motor.setVelocity(FORWARD_SPEED)
        right_motor.setVelocity(FORWARD_SPEED)

    def drive_backward():
        left_motor.setVelocity(-FORWARD_SPEED)
        right_motor.setVelocity(-FORWARD_SPEED)

    # Build the sequence: [("label", motor_fn, steps), ...]
    # Each test does the action then its inverse so the robot resets itself.
    if TEST_MODE == "LEFT":
        sequence = [
            ("Turn LEFT  (should be 90°)",        turn_left,     TURN_STEPS_90),
            ("PAUSE",                              stop,          PAUSE_STEPS),
            ("Turn RIGHT (returning to start)",    turn_right,    TURN_STEPS_90),
            ("PAUSE",                              stop,          PAUSE_STEPS),
        ]
    elif TEST_MODE == "RIGHT":
        sequence = [
            ("Turn RIGHT (should be 90°)",         turn_right,    TURN_STEPS_90),
            ("PAUSE",                              stop,          PAUSE_STEPS),
            ("Turn LEFT  (returning to start)",    turn_left,     TURN_STEPS_90),
            ("PAUSE",                              stop,          PAUSE_STEPS),
        ]
    elif TEST_MODE == "BACK":
        sequence = [
            ("Turn BACK  (should be 180°)",        turn_right,    TURN_STEPS_180),
            ("PAUSE",                              stop,          PAUSE_STEPS),
            ("Turn BACK  (returning to start)",    turn_right,    TURN_STEPS_180),
            ("PAUSE",                              stop,          PAUSE_STEPS),
        ]
    elif TEST_MODE == "FORWARD":
        sequence = [
            ("Drive FORWARD one cell",             drive_forward, FORWARD_STEPS_CELL),
            ("PAUSE",                              stop,          PAUSE_STEPS),
            ("Drive BACKWARD one cell",            drive_backward, FORWARD_STEPS_CELL),
            ("PAUSE",                              stop,          PAUSE_STEPS),
        ]
    else:
        raise ValueError(f"Unknown TEST_MODE: {TEST_MODE!r}")

    phase_idx   = 0
    phase_steps = 0
    rep         = 1

    label, motor_fn, total_steps = sequence[phase_idx]
    print(f"\n=== Calibration: TEST_MODE={TEST_MODE} ===")
    print(f"    TURN_STEPS_90={TURN_STEPS_90}  TURN_STEPS_180={TURN_STEPS_180}  FORWARD_STEPS_CELL={FORWARD_STEPS_CELL}")
    print(f"\nRep {rep} | Phase: {label}  ({total_steps} steps)")

    while robot.step(timestep) != -1:
        motor_fn()
        phase_steps += 1

        if phase_steps >= total_steps:
            stop()
            phase_steps = 0
            phase_idx   = (phase_idx + 1) % len(sequence)

            if phase_idx == 0:
                rep += 1
                print(f"\nRep {rep} | Phase: {sequence[phase_idx][0]}  ({sequence[phase_idx][2]} steps)")
            else:
                print(f"         | Phase: {sequence[phase_idx][0]}  ({sequence[phase_idx][2]} steps)")

            label, motor_fn, total_steps = sequence[phase_idx]

    stop()


if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
