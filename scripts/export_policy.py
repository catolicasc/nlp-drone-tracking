import argparse
import os


def _build_policy(torch):
    class Policy(torch.nn.Module):
        def __init__(self):
            super().__init__()

        def forward(self, x):
            cx_norm = x[0]
            det_score = x[1]
            have_det = x[2]

            max_v = 0.6
            max_yaw_rate = 0.8

            yaw_rate = torch.clamp(-0.6 * cx_norm, -max_yaw_rate, max_yaw_rate)

            score_gate = torch.clamp(det_score, 0.0, 1.0)
            vx = torch.zeros((), dtype=x.dtype, device=x.device)
            vy = torch.zeros((), dtype=x.dtype, device=x.device)

            vy_search = torch.tensor(0.4, dtype=x.dtype, device=x.device)
            yaw_search = torch.tensor(0.25, dtype=x.dtype, device=x.device)

            vx = torch.where(have_det > 0.5, vx, torch.zeros_like(vx))
            vy = torch.where(have_det > 0.5, vy, vy_search)
            yaw_rate = torch.where(have_det > 0.5, yaw_rate * (0.2 + 0.8 * score_gate), yaw_search)

            v = torch.sqrt(vx * vx + vy * vy)
            scale = torch.where(v > max_v, max_v / (v + 1e-9), torch.ones_like(v))
            vx = vx * scale
            vy = vy * scale

            return torch.stack([vx, vy, yaw_rate]).to(torch.float32)

    return Policy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="scripts/policy.ts")
    args = parser.parse_args()

    import torch

    policy = _build_policy(torch)
    policy.eval()

    scripted = torch.jit.script(policy)

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    scripted.save(out_path)

    x = torch.zeros((6,), dtype=torch.float32)
    y = scripted(x)
    y = y.detach().cpu().numpy().reshape(-1)

    print(f"Saved TorchScript policy to: {out_path}")
    print(f"Sanity output y=[vx, vy, yaw_rate]: {y.tolist()}")


if __name__ == "__main__":
    main()
