"""Verify every workspace sibling package is importable together."""

# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

import importlib

import pytest

# geodude and ada_mj are not here: each has its own environment (setup.sh), apart from this one.
SIBLINGS = [
    "asset_manager",
    "geodude_assets",
    "mj_environment",
    "mj_manipulator",
    "mj_viser",
    "prl_assets",
    "sscbirrt",
    "tsr",
]


@pytest.mark.parametrize("name", SIBLINGS)
def test_sibling_importable(name: str) -> None:
    importlib.import_module(name)
