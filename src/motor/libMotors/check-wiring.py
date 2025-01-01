#!/usr/bin/env python3

import controller
import time


def check_move(c: controller.Controller, right: bool):
    side, other_side, idx, oidx = "right", "left", 1, 0
    if not right:
        side, other_side, idx, oidx = other_side, side, oidx, idx
    c.standby()
    c.get_encoder_ticks()
    print(f"Making {side} motor go forward")
    speed = [0, 0]
    speed[idx] = -50
    c.set_raw_motor_speed(*speed)
    time.sleep(0.5)
    c.standby()
    time.sleep(0.1)
    ticks = list(c.get_encoder_ticks())
    ticks, parasitic = ticks[idx], ticks[oidx]
    if ticks == 0:
        raise ValueError(f"encoder on {side} side does not change")
    if abs(ticks) < 500:
        raise ValueError(
            f"encoder on {side} side does not change significantly ({ticks} ticks)"
        )
    if ticks > 0:
        raise ValueError(
            f"encoder on {side} goes in the reverse direction "
            + f"as expected ({ticks} ticks)"
        )
    if -ticks < abs(parasitic):
        raise ValueError(
            f"encoder on {side} side ({ticks} ticks) changes less than "
            + f"encoder on {other_side} side ({parasitic} ticks)"
        )
    if abs(parasitic) > 5:
        raise ValueError(
            f"encoder on {other_side} side changed unexpectedly ({parasitic} ticks)"
        )


def check_procedure():
    print(
        "Wheels will move forward (left then right), and encoder values will be checked"
    )
    c = controller.Controller()
    c.set_motor_shutdown_timeout(2)
    c.standby()
    time.sleep(0.1)
    check_move(c, False)
    time.sleep(2.0)
    check_move(c, True)


if __name__ == "__main__":
    try:
        check_procedure()
        print(
            "If you saw the wheels move forward as expected (left, then right), "
            + "everything is fine"
        )
    except ValueError as v:
        print(f"ERROR: {v}")
        print("Check that motors and encoders are correctly wired")
    c = controller.Controller()
    c.set_motor_speed(-32767, -32767)
    time.sleep(1)
    c.standby()
