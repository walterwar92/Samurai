"""Tests for config_loader — duplicate-key detection and config.yaml integrity.

Regression context: config.yaml once carried a duplicated `control:` block
(a buggy MATLAB export merged two copies into one mapping). PyYAML's safe_load
silently keeps the LAST duplicate key, so the populated control matrices were
shadowed by a later all-`null` block. On the Raspberry Pi — which deliberately
ships without scipy — StateSpaceModel then fell through to a scipy fallback and
crash-looped.

These tests make that class of corruption loud:
  * config_loader logs every duplicate mapping key, with its line number;
  * the committed config.yaml is free of duplicate keys;
  * control.matrices.A/B are populated, so the Pi never needs scipy.
"""

from __future__ import annotations

import logging
import sys

import yaml

import config_loader


# ── _DuplicateKeyLoader — surfaces duplicate mapping keys ──────────────────
def test_duplicate_key_loader_logs_collision(caplog):
    text = "control:\n  matrices:\n    A: 1\n  matrices:\n    A: null\n"
    with caplog.at_level(logging.ERROR, logger="config_loader"):
        data = yaml.load(text, Loader=config_loader._DuplicateKeyLoader)
    assert "duplicate key" in caplog.text.lower()
    # last-wins is preserved — behaviour unchanged, the collision is just visible
    assert data["control"]["matrices"]["A"] is None


def test_duplicate_key_loader_reports_line_number(caplog):
    text = "a: 1\nb: 2\na: 3\n"          # 'a' is redefined on line 3
    with caplog.at_level(logging.ERROR, logger="config_loader"):
        yaml.load(text, Loader=config_loader._DuplicateKeyLoader)
    assert "line 3" in caplog.text


def test_valid_yaml_loads_without_warnings(caplog):
    text = "a: 1\nb:\n  c: 2\n  d: 3\n"
    with caplog.at_level(logging.ERROR, logger="config_loader"):
        data = yaml.load(text, Loader=config_loader._DuplicateKeyLoader)
    assert "duplicate" not in caplog.text.lower()
    assert data == {"a": 1, "b": {"c": 2, "d": 3}}


def test_load_logs_duplicate_keys_in_config_file(tmp_path, caplog, monkeypatch):
    """config_loader._load() must route config.yaml through the dup-key loader."""
    bad = tmp_path / "config.yaml"
    bad.write_text("mqtt:\n  port: 1883\nmqtt:\n  port: 1884\n", encoding="utf-8")
    monkeypatch.setattr(config_loader, "_CONFIG_PATH", str(bad))
    monkeypatch.setattr(config_loader, "_data", {})
    monkeypatch.setattr(config_loader, "_validation_done", False)
    with caplog.at_level(logging.ERROR, logger="config_loader"):
        config_loader._load()
    assert "duplicate key" in caplog.text.lower()


# ── Committed config.yaml integrity ───────────────────────────────────────
def test_committed_config_has_no_duplicate_keys(caplog):
    with caplog.at_level(logging.ERROR, logger="config_loader"):
        with open(config_loader._CONFIG_PATH, encoding="utf-8") as f:
            yaml.load(f, Loader=config_loader._DuplicateKeyLoader)
    assert "duplicate key" not in caplog.text.lower(), (
        "config.yaml has duplicate keys:\n" + caplog.text
    )


def test_committed_config_control_matrices_populated():
    """control.matrices.A/B must be present so the scipy-free Pi never falls
    back to scipy.linalg.expm in StateSpaceModel._load_or_compute()."""
    with open(config_loader._CONFIG_PATH, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    matrices = data["control"]["matrices"]
    assert matrices["A"] is not None, "control.matrices.A is null — Pi crashes importing scipy"
    assert matrices["B"] is not None, "control.matrices.B is null — Pi crashes importing scipy"


# ── Scipy-free bootstrap (the Raspberry Pi runtime environment) ───────────
def _block_scipy_and_reload_config(monkeypatch):
    """Simulate the Pi: scipy unavailable, config.yaml re-read fresh from disk."""
    for mod in ("scipy", "scipy.linalg", "scipy.optimize"):
        monkeypatch.setitem(sys.modules, mod, None)
    monkeypatch.setattr(config_loader, "_data", {})
    monkeypatch.setattr(config_loader, "_validation_done", False)


def test_state_space_model_bootstraps_without_scipy(monkeypatch):
    """StateSpaceModel() must construct from config.yaml with scipy absent."""
    _block_scipy_and_reload_config(monkeypatch)
    from pi_nodes.control.state_space_model import StateSpaceModel
    model = StateSpaceModel()
    assert model.Ad.shape == (5, 5)
    assert model.Bd.shape == (5, 2)


def test_mpc_controller_bootstraps_without_scipy(monkeypatch):
    """MPCController() must construct from config.yaml with scipy absent."""
    _block_scipy_and_reload_config(monkeypatch)
    from pi_nodes.control.mpc_controller import MPCController
    mpc = MPCController()
    assert mpc.n == 5
    assert mpc.r == 2
