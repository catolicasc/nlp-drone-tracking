import argparse
import os


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out",
        required=True,
        help="Output path for TorchScript file (e.g. /tmp/dummy_policy.ts.pt)",
    )
    args = parser.parse_args()

    import torch

    class DummyPolicy(torch.nn.Module):
        def __init__(self):
            super().__init__()

        def forward(self, x):
            # x: float32 tensor with shape [6]
            # Return: float32 tensor with shape [3] => [vx, vy, yaw_rate]
            # Keep it simple and deterministic.
            z = torch.zeros((3,), dtype=torch.float32)
            return z

    model = DummyPolicy().eval()

    example = torch.zeros((6,), dtype=torch.float32)
    ts = torch.jit.trace(model, example)
    ts = torch.jit.freeze(ts)

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    ts.save(out_path)

    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
