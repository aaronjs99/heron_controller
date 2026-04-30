from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_heron_controller_repo_keeps_controller_source_header_and_config_contracts():
    assert (REPO_ROOT / "heron_controller/src/controller.cpp").exists()
    assert (REPO_ROOT / "heron_controller/include/heron_controller/controller.h").exists()
    assert (REPO_ROOT / "heron_controller/config/heron_controller.yaml").exists()
