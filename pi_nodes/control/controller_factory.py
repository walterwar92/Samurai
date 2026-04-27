"""controller_factory — picks the right controller based on config.yaml.

Reads `control.mode` from the config and returns either a stub (off mode)
or a fully-initialised LQRController / MPCController. This is the single
entry-point that motor_node uses, so swapping algorithms is a config edit
plus a restart — no code changes needed.

Modes
-----
  off    → PassthroughController (returns u unchanged)
  lqr    → LQRController
  mpc    → MPCController
  modal  → LQRController with K loaded from `control.matrices.K_modal`

Errors are *not* fatal: if matrices are missing or shapes mismatch, the
factory falls back to passthrough and logs a warning. This keeps the
robot drivable even when a half-finished MATLAB run leaves the config
inconsistent.
"""

from __future__ import annotations

import logging
from typing import Optional, Protocol

import numpy as np

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda key, default=None: default  # type: ignore

log = logging.getLogger(__name__)


class Controller(Protocol):
    """Minimal interface used by motor_node."""
    def step(self, x: np.ndarray, x_ref: Optional[np.ndarray] = None) -> np.ndarray: ...


class PassthroughController:
    """No-op controller: returns the *reference* command verbatim.

    Used when control.mode == 'off'. Keeps the API consistent with
    LQR/MPC controllers so motor_node has a single code path.
    """
    def __init__(self, u_dim: int = 2) -> None:
        self.u_dim = u_dim

    def step(self, x: np.ndarray, x_ref: Optional[np.ndarray] = None) -> np.ndarray:
        # If a reference command exists in x_ref[3:5] (v, omega), pass it.
        if x_ref is not None and len(x_ref) >= 5:
            return np.array([x_ref[3], x_ref[4]])
        return np.zeros(self.u_dim)

    def __repr__(self) -> str:
        return "PassthroughController(off)"


def make_controller(mode: Optional[str] = None) -> Controller:
    """Build a controller from config.yaml.

    Parameters
    ----------
    mode
        Override `control.mode`. If None, read from config.

    Returns
    -------
    Controller
        Object with a `.step(x, x_ref)` method returning a (2,) array.
    """
    if mode is None:
        mode = cfg("control.mode", "off")
    mode = str(mode).lower().strip()

    if mode == "off":
        return PassthroughController()

    try:
        if mode in ("lqr", "modal"):
            from pi_nodes.control.lqr_controller import LQRController

            if mode == "modal":
                K = cfg("control.matrices.K_modal", None)
                if K is None:
                    log.warning(
                        "control.mode=modal but control.matrices.K_modal is missing; "
                        "falling back to LQR K."
                    )
                    return LQRController()
                return LQRController(K=K)
            return LQRController()

        if mode == "mpc":
            from pi_nodes.control.mpc_controller import MPCController
            return MPCController()

    except Exception as exc:                              # pragma: no cover
        log.error("Failed to construct controller mode=%s: %s — using passthrough.",
                  mode, exc)
        return PassthroughController()

    log.warning("Unknown control.mode=%s — using passthrough.", mode)
    return PassthroughController()
