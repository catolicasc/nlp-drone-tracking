"""
Spawn de personagens via USD do pacote Isaac (Nucleus / assets root).
Sem dependência de Pegasus ou do resto da app.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import omni.usd
import isaacsim.core.experimental.utils.stage as _stage_utils
from pxr import Gf, UsdGeom


def _is_prim_path_valid(prim_path: str) -> bool:
    """True se o prim existe no stage (substitui omni.isaac.core.utils.prims.is_prim_path_valid)."""
    if not prim_path:
        return False
    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    return prim is not None and prim.IsValid()


def _add_reference_to_stage(usd_path: str, prim_path: str) -> None:
    """Referencia um USD no stage (substitui omni.isaac.core.utils.stage.add_reference_to_stage)."""
    _stage_utils.add_reference_to_stage(usd_path=usd_path, path=prim_path)


@dataclass(frozen=True)
class PersonSpawnSpec:
    prim_path: str
    """Caminho USD do prim (ex.: /World/People/person_1)."""

    asset_rel_path: str
    """Caminho relativo após <assets_root>/Isaac/ (ex.: People/Characters/.../file.usd)."""

    position: tuple[float, float, float]


def spawn_person(assets_root: str, spec: PersonSpawnSpec) -> bool:
    """Referencia o USD e aplica translate no prim. Retorna False se assets_root for inválido."""
    if not assets_root:
        return False

    usd_path = f"{assets_root}/Isaac/{spec.asset_rel_path}"
    if not _is_prim_path_valid(spec.prim_path):
        _add_reference_to_stage(usd_path=usd_path, prim_path=spec.prim_path)

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(spec.prim_path)
    xform = UsdGeom.Xformable(prim)

    translate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break

    if translate_op is None:
        translate_op = xform.AddTranslateOp()

    p = spec.position
    translate_op.Set(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
    return True


def spawn_people(assets_root: str, specs: Sequence[PersonSpawnSpec]) -> int:
    """Spawna várias pessoas; retorna quantas tiveram sucesso."""
    n = 0
    for spec in specs:
        if spawn_person(assets_root, spec):
            n += 1
    return n
