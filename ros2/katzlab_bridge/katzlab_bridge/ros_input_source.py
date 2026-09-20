"""An :class:`InputSource` fed by ROS 2 messages instead of a gamepad.

This is the seam the rest of ``katzlab`` never has to know about: ``ControlLoop``
already accepts anything that produces :class:`ControllerFrame` samples, so a
ROS 2 topic/service is just another input device to it, exactly like the
gamepad or the keyboard. No changes to ``katzlab`` itself.

Deliberately out of scope: firing the laser. ``ControllerFrame`` can carry a
``fire`` button, but this class never sets one -- a ROS topic is
remote-triggerable by anything on the graph, including a simulator, and the
laser must stay commandable only from the physical controller. See
``ros2/README.md``.
"""

from __future__ import annotations

import threading
import time

from katzlab.input.base import ControllerFrame, InputSource, InputUnavailable

# How long a one-shot service request (home) reads as "held" afterwards.
# Home requires HOME_HOLD_S (0.6s) of continuous hold in the control loop
# before it triggers, so this has to comfortably outlast that.
HOME_HOLD_WINDOW_S = 1.0

# A cmd_velocity that stops arriving is treated as "stick centred", not as a
# disconnect -- the ROS node itself is still alive and polling, so
# ControlLoop's own watchdog (which looks at ``frame.connected``) would never
# trip. Staleness has to be caught here instead, and it must fail toward
# zero velocity, the same way a dropped gamepad link fails toward centred
# sticks.
CMD_VELOCITY_STALE_S = 0.5


class Ros2InputSource(InputSource):
    """Bridges ROS 2 topics/services into a :class:`ControllerFrame` stream.

    The owning node pushes updates in via ``set_axes``/``request_home``/
    ``request_estop`` from its subscription and service callbacks; ``poll()``
    (called from the control loop's own thread) reads the latest snapshot.
    A lock guards the handful of fields shared between the two threads.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._linear = 0.0
        self._rotation = 0.0
        self._flexion = 0.0
        self._last_cmd_at = 0.0
        self._home_until: float | None = None
        self._estop_pending = False

    # -- InputSource -------------------------------------------------------

    def open(self) -> None:
        self._last_cmd_at = time.monotonic()

    def close(self) -> None:
        pass

    @property
    def name(self) -> str:
        return "ros2"

    def poll(self) -> ControllerFrame:
        now = time.monotonic()
        with self._lock:
            stale = (now - self._last_cmd_at) > CMD_VELOCITY_STALE_S
            axes = {
                "linear": 0.0 if stale else self._linear,
                "rotation": 0.0 if stale else self._rotation,
                "flexion": 0.0 if stale else self._flexion,
            }
            home_held = self._home_until is not None and now < self._home_until
            if self._home_until is not None and now >= self._home_until:
                self._home_until = None

            estop = self._estop_pending
            self._estop_pending = False

        buttons = {"home": home_held, "estop": estop}
        return ControllerFrame(axes=axes, buttons=buttons, connected=True)

    # -- called from the node's ROS callbacks (a different thread) ---------

    def set_axes(self, *, linear: float, rotation: float, flexion: float) -> None:
        """Set normalised -1..1 axis targets, already deadzone/curve shaped
        by the caller (see ``bridge_node._to_axis``)."""
        with self._lock:
            self._linear = max(-1.0, min(1.0, linear))
            self._rotation = max(-1.0, min(1.0, rotation))
            self._flexion = max(-1.0, min(1.0, flexion))
            self._last_cmd_at = time.monotonic()

    def request_home(self) -> None:
        with self._lock:
            self._home_until = time.monotonic() + HOME_HOLD_WINDOW_S

    def request_estop(self) -> None:
        with self._lock:
            self._estop_pending = True


__all__ = ["Ros2InputSource", "InputUnavailable"]
