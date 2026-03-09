# control_state.py
from dataclasses import dataclass, field
from threading import Lock

@dataclass
class ControllerState:
    axes: list = field(default_factory=lambda: [0.0] * 6)
    gripper_cmd: float = 0.0
    mode: int = 0
    mode_toggle_requested: bool = False
    home_requested: bool = False
    quit_requested: bool = False

    def __post_init__(self):
        self._lock = Lock()

    def snapshot(self):
        with self._lock:
            return {
                "axes": self.axes.copy(),
                "gripper_cmd": self.gripper_cmd,
                "mode_toggle_requested": self.mode_toggle_requested,
                "home_requested": self.home_requested,
                "quit_requested": self.quit_requested,
            }

    def set_mode(self, value):
            with self._lock:
                self.mode = int(value)

    def set_axes(self, axes):
        with self._lock:
            self.axes = list(axes)

    def set_gripper(self, value):
        with self._lock:
            self.gripper_cmd = float(value)

    def request_mode_toggle(self):
        with self._lock:
            self.mode_toggle_requested = True

    def request_home(self):
        with self._lock:
            self.home_requested = True

    def request_quit(self):
        with self._lock:
            self.quit_requested = True

    def consume_mode_toggle(self):
        with self._lock:
            v = self.mode_toggle_requested
            self.mode_toggle_requested = False
            return v

    def consume_home(self):
        with self._lock:
            v = self.home_requested
            self.home_requested = False
            return v