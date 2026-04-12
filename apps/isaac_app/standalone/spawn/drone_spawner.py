"""
Spawn de multirotor via Pegasus + PX4. Toda a dependência de Pegasus fica aqui.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS


@dataclass
class PegasusQuadrotorSpec:
    """Parâmetros do Iris (ou outro USD em ROBOTS) com backend PX4 MAVLink."""

    stage_prefix: str = "/World/quadrotor"
    usd_file_key: str = "Iris"
    vehicle_id: int = 0
    init_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    init_orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    connection_type: str = "tcpin"
    connection_ip: str = "localhost"
    connection_baseport: int = 4560
    enable_lockstep: bool = True
    num_rotors: int = 4
    px4_autolaunch: bool = True
    airframe: str | None = None
    """Se None, usa env PX4_AIRFRAME ou PegasusInterface().px4_default_airframe ou gazebo-classic_iris."""

    mavlink_extra: dict = field(default_factory=dict)
    """Campos extra passados ao PX4MavlinkBackendConfig (opcional)."""


def resolve_airframe(explicit: str | None) -> str:
    if explicit:
        return explicit
    pg = PegasusInterface()
    return os.environ.get("PX4_AIRFRAME") or pg.px4_default_airframe or "gazebo-classic_iris"


def spawn_pegasus_quadrotor(world, px4_path: str, spec: PegasusQuadrotorSpec | None = None) -> None:
    if spec is None:
        spec = PegasusQuadrotorSpec()

    pg = PegasusInterface()
    pg._world = world
    pg.set_px4_path(px4_path)

    airframe = resolve_airframe(spec.airframe)
    print(f"PX4 airframe (PX4_SIM_MODEL): {airframe}")

    cfg_dict = {
        "vehicle_id": spec.vehicle_id,
        "connection_type": spec.connection_type,
        "connection_ip": spec.connection_ip,
        "connection_baseport": spec.connection_baseport,
        "enable_lockstep": spec.enable_lockstep,
        "num_rotors": spec.num_rotors,
        "px4_autolaunch": spec.px4_autolaunch,
        "px4_dir": px4_path,
        "px4_vehicle_model": airframe,
        **spec.mavlink_extra,
    }
    mavlink_config = PX4MavlinkBackendConfig(cfg_dict)

    config_multirotor = MultirotorConfig()
    config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]

    usd_file = ROBOTS[spec.usd_file_key]
    pos = spec.init_pos
    rot = spec.init_orientation_xyzw

    Multirotor(
        spec.stage_prefix,
        usd_file,
        spec.vehicle_id,
        [float(pos[0]), float(pos[1]), float(pos[2])],
        [float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])],
        config=config_multirotor,
    )
