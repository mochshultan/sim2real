"""Keep the native VHIP runtime artifact paired with its training export."""

import hashlib
import json
from pathlib import Path

import torch

BUNDLE = Path(__file__).resolve().parents[1] / "models/history/dreamwaq_vhip"


def test_vhip_native_history_contract_and_checkpoint_bundle():
    manifest = json.loads(next(BUNDLE.glob("deployment_*.json")).read_text())
    assert manifest["checkpoint_iteration"] == 5000
    for name, expected in manifest["file_sha256"].items():
        assert hashlib.sha256((BUNDLE / name).read_bytes()).hexdigest() == expected
    native = torch.jit.load(str(BUNDLE / manifest["policy_file"])).eval()
    raw = torch.jit.load(str(BUNDLE / "policy.pt")).eval()
    assert native.joint_order == "pion_native_FL_FR_BL_BR"
    assert native.policy.mean.numel() == 64
    history = torch.arange(225, dtype=torch.float32).reshape(1, 5, 45) / 1000
    # The training contract packs each observation term's five frames together.
    packed = torch.tensor(
        [
            [
                history[0, frame, index].item()
                for lower, upper in ((0, 3), (3, 6), (6, 9), (9, 21), (21, 33), (33, 45))
                for frame in range(5)
                for index in range(lower, upper)
            ]
        ]
    )
    with torch.inference_mode():
        expected = raw(history[:, -1], packed).clamp(-10, 10)
        torch.testing.assert_close(native(history), expected, rtol=0, atol=0)
