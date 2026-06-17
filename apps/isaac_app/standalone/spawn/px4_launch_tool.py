"""
Launcher PX4 compatível com versões recentes do PX4-Autopilot.

O Pegasus original usa ROMFS/px4fmu_common/ + cwd temporário, o que deixa de
popular etc/init.d/rc.vehicle_setup. O SITL oficial usa build/px4_sitl_default/etc.
"""

from __future__ import annotations

import os
import subprocess
import tempfile


def _px4_lock_paths(vehicle_id: int) -> list[str]:
    vid = int(vehicle_id)
    return [f"/tmp/px4_lock-{vid}", f"/tmp/px4-sock-{vid}"]


def cleanup_px4_lock_files(vehicle_id: int = 0) -> None:
    """Remove locks/sockets PX4 órfãos (ex.: criados como root em /tmp)."""
    blocked: list[str] = []
    for path in _px4_lock_paths(vehicle_id):
        if not os.path.exists(path):
            continue
        try:
            os.remove(path)
            print(f"[px4] removido artefato antigo: {path}")
        except PermissionError:
            blocked.append(path)
        except OSError as exc:
            print(f"[px4] aviso ao remover {path}: {exc}")

    if blocked:
        joined = " ".join(blocked)
        raise PermissionError(
            "Arquivos PX4 em /tmp sem permissão de escrita "
            f"({joined}). Rode: sudo rm -f {joined}"
        )


class PX4LaunchTool:
    def __init__(self, px4_dir: str, vehicle_id: int = 0, px4_model: str = "gazebo-classic_iris"):
        self.px4_process: subprocess.Popen | None = None
        self.vehicle_id = int(vehicle_id)
        self.px4_dir = px4_dir
        self.root_fs = tempfile.TemporaryDirectory()

        self.px4_bin = os.path.join(px4_dir, "build/px4_sitl_default/bin/px4")
        self.etc_root = os.path.join(px4_dir, "build/px4_sitl_default/etc")

        if not os.path.isfile(self.px4_bin):
            raise FileNotFoundError(f"PX4 SITL não encontrado: {self.px4_bin}")
        if not os.path.isdir(self.etc_root):
            raise FileNotFoundError(
                f"PX4 etc root não encontrado: {self.etc_root} "
                "(rode 'make px4_sitl_default' no PX4-Autopilot)"
            )

        self.environment = os.environ.copy()
        self.environment["PX4_SIM_MODEL"] = px4_model

    def _prepare_mavlink_gcs_broadcast(self) -> str:
        """
        PX4 SITL escuta MAVLink GCS em UDP 18570 e envia para 14550.
        Sem broadcast (-p), o QGroundControl (que escuta 14550) não recebe dados.
        """
        src = os.path.join(self.etc_root, "init.d-posix", "px4-rc.mavlink")
        overlay_dir = os.path.join(self.root_fs.name, "init.d-posix")
        os.makedirs(overlay_dir, exist_ok=True)
        dst = os.path.join(overlay_dir, "px4-rc.mavlink")

        with open(src, encoding="utf-8") as f:
            lines = f.read().splitlines()

        patched: list[str] = []
        for line in lines:
            if (
                "mavlink start" in line
                and "udp_gcs_port_local" in line
                and "-p" not in line
            ):
                line = line.replace(
                    "-r 4000000 -f",
                    "-r 4000000 -f -p -o $((14550+px4_instance))",
                    1,
                )
            patched.append(line)

        with open(dst, "w", encoding="utf-8") as f:
            f.write("\n".join(patched) + "\n")

        # rcS faz `. px4-rc.mavlink` via PATH — overlay tem prioridade.
        path_prefix = overlay_dir
        self.environment["PATH"] = (
            f"{path_prefix}:{self.environment.get('PATH', '')}"
        )
        print("[px4] MAVLink GCS: broadcast habilitado para QGroundControl (porta 14550)")
        return dst

    def launch_px4(self) -> None:
        cleanup_px4_lock_files(self.vehicle_id)
        self._prepare_mavlink_gcs_broadcast()
        self.px4_process = subprocess.Popen(
            [
                self.px4_bin,
                self.etc_root,
                "-s",
                "etc/init.d-posix/rcS",
                "-i",
                str(self.vehicle_id),
                "-w",
                self.root_fs.name,
                "-d",
            ],
            cwd=self.root_fs.name,
            shell=False,
            env=self.environment,
        )

    def kill_px4(self) -> None:
        if self.px4_process is not None:
            self.px4_process.kill()
            self.px4_process = None

    def __del__(self) -> None:
        if self.px4_process is not None:
            self.kill_px4()
        self.root_fs.cleanup()
