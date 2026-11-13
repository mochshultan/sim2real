"""Check policy selection and argument forwarding without starting ROS nodes."""

import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("old", [False, True])
def test_run_policy_selection_and_forwarding(tmp_path, old):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    (scripts / "run.sh").write_text((ROOT / "scripts/run.sh").read_text())
    (scripts / "source_ros.sh").write_text(":\n")
    (tmp_path / "install").mkdir()
    (tmp_path / "install/setup.bash").write_text(":\n")
    ros2 = tmp_path / "ros2"
    ros2.write_text('#!/bin/bash\nprintf "%s\\n" "$@"\n')
    ros2.chmod(0o755)
    args = ["bash", str(scripts / "run.sh")]
    if old:
        args.append("--old")
    args += ["with_hardware:=false", "config_file:=/tmp/config with spaces.yaml"]
    result = subprocess.run(
        args, env={**os.environ, "PATH": f"{tmp_path}:{os.environ['PATH']}"},
        capture_output=True, text=True, check=True,
    )
    forwarded = result.stdout.splitlines()
    assert forwarded[:3] == ["launch", "jaguar_control", "sim2real.launch.py"]
    assert forwarded[-2:] == args[-2:]
    if old:
        assert forwarded[3:6] == [
            f"policy_path:={tmp_path}/models/policy_adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022.pt",
            "policy_reference:=nominal",
            "history_initialization:=zeros",
        ]
    else:
        assert len(forwarded) == 5
