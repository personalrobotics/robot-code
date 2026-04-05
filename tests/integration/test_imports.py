"""Verify every workspace sibling package is importable together."""

# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

import importlib

import pytest

SIBLINGS = [
    "asset_manager",
    "geodude",
    "geodude_assets",
    "mj_environment",
    "mj_manipulator",
    "mj_viser",
    "prl_assets",
    "pycbirrt",
    "tsr",
]


@pytest.mark.parametrize("name", SIBLINGS)
def test_sibling_importable(name: str) -> None:
    importlib.import_module(name)
