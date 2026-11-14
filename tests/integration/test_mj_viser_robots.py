"""mj_viser renders Geodude's and ADA's sites the way the group convention intends.

Task frames are shown by default; engineering frames (site group 4) appear when an
application enables that group, as a debug preset would (docs/mujoco-groups.md).
"""

# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

import mujoco
import pytest

mj_viser = pytest.importorskip("mj_viser")

pytestmark = pytest.mark.skipif(
    not hasattr(mj_viser.MujocoViewer, "set_visible_groups"),
    reason="the workspace pins mj_viser before 0.2.0 (personalrobotics/robot-code#173)",
)

DEBUG_SITE_GROUPS = {0, 1, 2, 4}


def _robots() -> list[object]:
    import ada_assets
    import geodude_assets

    return [
        pytest.param(
            geodude_assets.MODELS_DIR / "geodude/geodude.xml",
            ["left_ur5e/gripper/grasp_site", "right_ur5e/gripper/grasp_site"],
            ["left_ur5e/ft_sensor_site", "right_ur5e/ft_sensor_site", "left_arm_attachment_site"],
            id="geodude",
        ),
        pytest.param(
            ada_assets.MODELS_DIR / "scene.xml",
            ["ee_site", "articutool/grasp_site", "articutool/fork_tip", "human/mouth"],
            ["tool_attachment_site", "articutool/ft_sensor_site", "camera/color_optical_frame"],
            id="ada",
        ),
    ]


@pytest.mark.parametrize(("path", "task_frames", "engineering_frames"), _robots())
def test_task_frames_shown_and_engineering_frames_one_group_away(
    path: object, task_frames: list[str], engineering_frames: list[str]
) -> None:
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mj_viser.MujocoViewer(model, port=0, show_visibility=False) as viewer:
        viewer.update(data)
        sites = {site["name"]: site for site in viewer.scene_report()["sites"]}
        for name in task_frames + engineering_frames:
            assert sites[name]["status"] == "rendered", f"{name} is {sites[name]['status']}"

        shown = viewer.visible_groups()["sites"]
        assert all(sites[n]["group"] in shown for n in task_frames), "task frames hidden by default"
        assert not any(sites[n]["group"] in shown for n in engineering_frames), "engineering frames shown by default"

        viewer.set_visible_groups(sites=DEBUG_SITE_GROUPS)
        shown = viewer.visible_groups()["sites"]
        assert all(sites[n]["group"] in shown for n in task_frames + engineering_frames)
