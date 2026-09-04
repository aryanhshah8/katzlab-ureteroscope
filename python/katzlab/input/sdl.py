"""pygame/SDL initialisation, with a macOS joystick-backend fallback.

macOS's GameController framework claims pads it recognises -- they appear in
System Settings > Game Controllers. SDL prefers that MFi backend, but for many
third-party and clone pads the MFi path exposes *nothing*: the controller reads
"Connected" in System Settings while SDL enumerates zero devices.

Setting SDL_JOYSTICK_MFI=0 forces SDL onto the IOKit HID backend, where those
pads do show up. The fallback is applied only when the default path finds
nothing, so genuine MFi controllers -- which often work better through
GameController -- keep using it.
"""

from __future__ import annotations

import logging
import os

log = logging.getLogger(__name__)


def import_pygame():
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    import pygame

    return pygame


def init_joysticks(force_backend: str = "") -> tuple[object, int, str]:
    """Initialise the joystick subsystem.

    Returns ``(pygame, device_count, backend)`` where backend is "mfi" or
    "iokit", naming which path actually found devices.

    ``force_backend`` pins the choice. A mapping captured on one backend is
    wrong on the other -- different names, different axis and button counts --
    so once a controller has been calibrated, the backend must be pinned or the
    mapping silently points at the wrong physical buttons.
    """
    pygame = import_pygame()

    if force_backend == "iokit":
        os.environ["SDL_JOYSTICK_MFI"] = "0"
        pygame.init()
        pygame.joystick.init()
        return pygame, pygame.joystick.get_count(), "iokit"

    if force_backend == "mfi":
        os.environ.pop("SDL_JOYSTICK_MFI", None)
        pygame.init()
        pygame.joystick.init()
        return pygame, pygame.joystick.get_count(), "mfi"

    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count > 0:
        return pygame, count, "mfi"

    # Nothing on the default path. Re-initialise the subsystem with the MFi
    # backend disabled; SDL reads the hint when the subsystem starts, so the
    # quit/init cycle is what makes the change take effect.
    pygame.joystick.quit()
    os.environ["SDL_JOYSTICK_MFI"] = "0"
    pygame.joystick.init()
    count = pygame.joystick.get_count()

    if count > 0:
        log.info(
            "no controller via the GameController (MFi) backend; found %d on "
            "the IOKit HID backend instead",
            count,
        )
    return pygame, count, "iokit"


NOT_FOUND_HELP = """no game controller detected.

The pad can read "Connected" in System Settings > Game Controllers and still be
invisible here. Things worth trying, in order:

  1. Unplug and replug it (or re-pair over Bluetooth).
  2. If it is a multi-mode pad, try its other mode -- many clones have a
     mode/home button that switches between PS3, XInput and DirectInput.
  3. Grant your terminal Input Monitoring in
     System Settings > Privacy & Security > Input Monitoring.

Run 'python scripts/gamepad_monitor.py' once it is detected to confirm the
axes and buttons actually report."""
