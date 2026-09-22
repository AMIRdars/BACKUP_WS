"""Pure helpers for closed-loop gripper force control."""

from typing import Iterable


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp *value* to the inclusive range [lower, upper]."""
    return max(lower, min(upper, value))


def holding_recovery_position(
        current_position: float,
        commanded_position: float,
        normal_forces: Iterable[float],
        target_force: float,
        minimum_force: float,
        release_force: float,
        position_gain: float,
        maximum_step: float,
        minimum_position: float,
        maximum_position: float,
        allow_release: bool = True) -> float:
    """Keep a grasp while reopening above its safe normal-force ceiling."""
    forces = tuple(max(0.0, float(force)) for force in normal_forces)
    if len(forces) != 2:
        raise ValueError('Exactly two finger forces are required.')
    if release_force <= 0.0:
        raise ValueError('release_force must be positive.')
    # A verified weak contact takes priority: opening because the other finger
    # is high could immediately drop the payload. Once both fingers have the
    # minimum holding load, the high-force release guard is enforced.
    if min(forces) < minimum_force:
        error = max(0.0, target_force - min(forces))
        delta = clamp(position_gain * error, 0.0, maximum_step)
        return clamp(
            current_position + delta, minimum_position, maximum_position)
    if allow_release and max(forces) >= release_force:
        # The gap between the release threshold and the weak-contact floor
        # below prevents a close/open limit cycle during dynamic transport.
        delta = clamp(
            position_gain * (target_force - max(forces)),
            -maximum_step, 0.0)
        return clamp(
            current_position + delta, minimum_position, maximum_position)
    return clamp(commanded_position, minimum_position, maximum_position)


def next_gripper_position(
        current_position: float,
        normal_forces: Iterable[float],
        bilateral_contact: bool,
        target_force: float,
        force_tolerance: float,
        position_gain: float,
        maximum_step: float,
        search_step: float,
        minimum_position: float,
        maximum_position: float) -> float:
    """Return one bounded position update for a two-finger gripper.

    Increasing the AMIR gripper angle closes both fingers.  Before bilateral
    contact, the command advances by ``search_step``.  Once both pads touch,
    the mean of their normal loads is regulated to ``target_force``.  A large
    unilateral force always makes the gripper back off, preventing one pad
    from being crushed while the other pad is still searching for contact.
    """
    forces = tuple(max(0.0, float(force)) for force in normal_forces)
    if len(forces) != 2:
        raise ValueError('Exactly two finger forces are required.')
    if minimum_position > maximum_position:
        raise ValueError('minimum_position must not exceed maximum_position.')

    if max(forces) > target_force + force_tolerance:
        # Protect either finger from excessive load even if the other side is
        # lightly loaded.  A single actuator cannot correct left/right
        # imbalance, so opening is the only safe response.
        delta = clamp(
            position_gain * (target_force - max(forces)),
            -maximum_step, 0.0)
    elif not bilateral_contact:
        delta = search_step
    else:
        error = target_force - sum(forces) / 2.0
        if abs(error) <= force_tolerance:
            delta = 0.0
        else:
            delta = clamp(
                position_gain * error, -maximum_step, maximum_step)

    return clamp(
        current_position + delta, minimum_position, maximum_position)
