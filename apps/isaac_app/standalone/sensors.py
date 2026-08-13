from __future__ import annotations

import omni.usd


def _get_current_stage():
    """Stage USD atual (substitui omni.isaac.core.utils.stage.get_current_stage no Isaac 6.0)."""
    return omni.usd.get_context().get_stage()


def _is_prim_path_valid(prim_path: str) -> bool:
    """True se o prim existe no stage (substitui omni.isaac.core.utils.prims.is_prim_path_valid)."""
    if not prim_path:
        return False
    prim = _get_current_stage().GetPrimAtPath(prim_path)
    return prim is not None and prim.IsValid()


def _create_camera_prim(camera_prim_path: str, frequency: int, width: int, height: int):
    """Cria o prim de câmera via pxr (fallback p/ omni.isaac.sensor.Camera removido no 6.0).

    A resolução do render product é definida pelos inputs do IsaacCreateRenderProduct
    (ver setup_sensors); os attrs camera:* ficam registrados para compatibilidade.
    """
    from pxr import Gf, Sdf, UsdGeom

    stage = _get_current_stage()
    prim = stage.GetPrimAtPath(camera_prim_path)
    if not prim or not prim.IsValid():
        prim = UsdGeom.Camera.Define(stage, camera_prim_path).GetPrim()

    prim.CreateAttribute("camera:frequency", Sdf.ValueTypeNames.Int).Set(int(frequency))
    prim.CreateAttribute("camera:resolution", Sdf.ValueTypeNames.Int2).Set(
        Gf.Vec2i(int(width), int(height))
    )
    return prim


def _ensure_camera_visual_marker(camera_prim_path: str) -> None:
    try:
        from pxr import Gf, UsdGeom
    except Exception:
        return

    stage = _get_current_stage()
    marker_path = f"{camera_prim_path}/visual_marker"

    cube = UsdGeom.Cube.Define(stage, marker_path)
    cube.CreateSizeAttr(0.06)

    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

    gprim = UsdGeom.Gprim(cube.GetPrim())
    gprim.CreateDisplayColorAttr().Set([(1.0, 0.0, 0.0)])
    gprim.CreateDisplayOpacityAttr().Set([1.0])


def _quat_xyzw_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[float, float, float, float]:
    import math

    r = math.radians(float(roll_deg))
    p = math.radians(float(pitch_deg))
    y = math.radians(float(yaw_deg))

    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (float(qx), float(qy), float(qz), float(qw))


def _quat_xyzw_from_rotation_matrix(m: tuple[tuple[float, float, float], ...]) -> tuple[float, float, float, float]:
    import math

    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return (float(qx), float(qy), float(qz), float(qw))


def _quat_xyzw_camera_forward_down(pitch_down_deg: float) -> tuple[float, float, float, float]:
    import math

    pitch = math.radians(float(pitch_down_deg))
    c = math.cos(pitch)
    s = math.sin(pitch)

    # USD cameras look along local -Z. The drone frame uses +X as forward.
    x_right = (0.0, -1.0, 0.0)
    y_up = (s, 0.0, c)
    z_back = (-c, 0.0, s)
    return _quat_xyzw_from_rotation_matrix(
        (
            (x_right[0], y_up[0], z_back[0]),
            (x_right[1], y_up[1], z_back[1]),
            (x_right[2], y_up[2], z_back[2]),
        )
    )


def _set_local_xform(prim_path: str, position_xyz: tuple[float, float, float], orientation_xyzw: tuple[float, float, float, float]) -> None:
    try:
        from pxr import Gf, UsdGeom
    except Exception:
        return

    stage = _get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return

    x, y, z = position_xyz
    qx, qy, qz, qw = orientation_xyzw

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(float(x), float(y), float(z)))
    # O prim de câmera criado pelo Isaac já tem xformOp:orient em dupla precisão (quatd);
    # reutilizar o op com a mesma precisão evita o erro de typeName mismatch.
    xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(float(qw), Gf.Vec3d(float(qx), float(qy), float(qz)))
    )


def _configure_camera_prim(camera_prim_path: str, cam_cfg: dict) -> None:
    try:
        from pxr import Gf, UsdGeom
    except Exception:
        return

    stage = _get_current_stage()
    prim = stage.GetPrimAtPath(camera_prim_path)
    if not prim or not prim.IsValid():
        return

    camera = UsdGeom.Camera(prim)
    focal_length = float(cam_cfg.get("focal_length_mm", 18.0))
    horizontal_aperture = float(cam_cfg.get("horizontal_aperture_mm", 20.955))
    vertical_aperture = float(cam_cfg.get("vertical_aperture_mm", 15.2908))
    clipping_range = cam_cfg.get("clipping_range_m", (0.05, 1000.0))
    focus_distance = float(cam_cfg.get("focus_distance_m", 10.0))

    camera.GetFocalLengthAttr().Set(focal_length)
    camera.GetHorizontalApertureAttr().Set(horizontal_aperture)
    camera.GetVerticalApertureAttr().Set(vertical_aperture)
    camera.GetFocusDistanceAttr().Set(focus_distance)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(float(clipping_range[0]), float(clipping_range[1])))
    print(
        "[setup_sensors] Camera optics: "
        f"focal_length_mm={focal_length}, horizontal_aperture_mm={horizontal_aperture}, "
        f"vertical_aperture_mm={vertical_aperture}, clipping_range_m={clipping_range}"
    )


def _print_world_translation(prim_path: str, label: str) -> None:
    try:
        from pxr import UsdGeom
    except Exception:
        return

    stage = _get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return

    try:
        cache = UsdGeom.XformCache()
        mat = cache.GetLocalToWorldTransform(prim)
        t = mat.ExtractTranslation()
        print(f"[setup_sensors] {label} world translation: ({float(t[0]):.3f}, {float(t[1]):.3f}, {float(t[2]):.3f})")
    except Exception:
        return


def setup_sensors(config: dict, drone_prim_path: str = "/World/quadrotor"):
    sensors_cfg = (config or {}).get("sensors", {})
    cam_cfg = sensors_cfg.get("camera", {})
    if not cam_cfg.get("enabled", False):
        return None

    width = int(cam_cfg.get("width", 1280))
    height = int(cam_cfg.get("height", 720))
    frequency = int(cam_cfg.get("frequency", 30))

    pos_cfg = cam_cfg.get("position", (0.0, 0.0, 0.0))
    cam_orientation: tuple[float, float, float, float]
    orientation_mode = str(cam_cfg.get("orientation_mode", "forward_down")).strip().lower()
    if orientation_mode == "forward_down":
        cam_orientation = _quat_xyzw_camera_forward_down(float(cam_cfg.get("pitch_down_deg", 25.0)))
    elif "orientation_xyzw" in cam_cfg:
        ori_cfg = cam_cfg.get("orientation_xyzw", (0.0, 0.0, 0.0, 1.0))
        cam_orientation = (float(ori_cfg[0]), float(ori_cfg[1]), float(ori_cfg[2]), float(ori_cfg[3]))
    elif "orientation_rpy_deg" in cam_cfg:
        rpy = cam_cfg.get("orientation_rpy_deg", (0.0, 0.0, 0.0))
        cam_orientation = _quat_xyzw_from_rpy_deg(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    else:
        cam_orientation = (0.0, 0.0, 0.0, 1.0)

    try:
        cam_position = (float(pos_cfg[0]), float(pos_cfg[1]), float(pos_cfg[2]))
    except Exception:
        cam_position = (0.0, 0.0, 0.0)

    mount_parent = str(drone_prim_path)
    mount_prim = str(cam_cfg.get("mount_prim", "")).strip()
    try:
        configured_mount_parent: str | None = None
        if mount_prim:
            if mount_prim.startswith("/"):
                configured_mount_parent = mount_prim
            else:
                configured_mount_parent = f"{drone_prim_path}/{mount_prim}"

        if configured_mount_parent and _is_prim_path_valid(configured_mount_parent):
            mount_parent = configured_mount_parent
            print(f"[setup_sensors] Configured camera mount parent: {mount_parent}")
        else:
            if configured_mount_parent:
                print(
                    "[setup_sensors] WARNING: camera mount prim does not exist; "
                    f"falling back to auto-mount. configured={configured_mount_parent}"
                )

            candidates = [
                f"{drone_prim_path}/base_link",
                f"{drone_prim_path}/body",
                f"{drone_prim_path}/chassis",
                f"{drone_prim_path}/fmu",
                f"{drone_prim_path}/base_link_frd",
            ]
            found = []
            for p in candidates:
                if _is_prim_path_valid(p):
                    found.append(p)

            if found:
                mount_parent = found[0]
                print(f"[setup_sensors] Auto camera mount parent: {mount_parent} (candidates found: {found})")
            else:
                print(
                    "[setup_sensors] Auto camera mount parent fallback: "
                    f"{mount_parent} (no known moving child prims found under {drone_prim_path})"
                )
    except Exception:
        pass

    camera_prim_path = f"{mount_parent}/camera"
    print(f"[setup_sensors] Camera prim path: {camera_prim_path}")

    # Isaac Sim 6.0: omni.isaac.sensor.Camera foi removido. Usa isaacsim.sensors.camera
    # (deprecado mas presente, mesma stack do MonocularCamera do Pegasus) com fallback pxr.
    cam = None
    try:
        from isaacsim.sensors.camera.camera import Camera as _IsaacCamera

        cam = _IsaacCamera(
            prim_path=camera_prim_path,
            position=(0.0, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            frequency=frequency,
            resolution=(width, height),
        )
        cam.initialize()
    except Exception as e:  # noqa: BLE001
        print(f"[setup_sensors] Camera do isaacsim.sensors.camera indisponível ({e}); criando prim via pxr.")
        _create_camera_prim(camera_prim_path, frequency, width, height)

    _set_local_xform(camera_prim_path, cam_position, cam_orientation)
    _configure_camera_prim(camera_prim_path, cam_cfg)
    print(
        "[setup_sensors] Camera mount config: "
        f"position={cam_position}, orientation_mode={orientation_mode}, orientation_xyzw={cam_orientation}"
    )
    _print_world_translation(camera_prim_path, "Camera")

    if bool(cam_cfg.get("visualize", True)):
        _ensure_camera_visual_marker(camera_prim_path)

    try:
        import omni.graph.core as og

        from omni.usd import get_stage_next_free_path

        graph_path = get_stage_next_free_path(
            _get_current_stage(), cam_cfg.get("graph_path", "/Graph/ROS_Camera"), ""
        )
        node_namespace = cam_cfg.get("namespace", "drone/camera")
        image_topic = cam_cfg.get("topic", "image_raw")
        publish_depth = bool(cam_cfg.get("publish_depth", False))
        depth_topic = cam_cfg.get("depth_topic", "depth")
        depth_type = cam_cfg.get("depth_type", "depth")
        info_topic = cam_cfg.get("info_topic", "camera_info")
        frame_id = cam_cfg.get("frame_id", "drone_camera")
        publish_realsense_compat = bool(cam_cfg.get("publish_realsense_compat", False))
        rs_namespace = cam_cfg.get("realsense_namespace", "camera")
        rs_color_frame_id = cam_cfg.get("realsense_color_frame_id", "camera_color_optical_frame")
        rs_depth_frame_id = cam_cfg.get("realsense_depth_frame_id", "camera_depth_optical_frame")
        rs_color_image_topic = cam_cfg.get("realsense_color_topic", "color/image_raw")
        rs_color_info_topic = cam_cfg.get("realsense_color_info_topic", "color/camera_info")
        rs_depth_image_topic = cam_cfg.get("realsense_depth_topic", "depth/image_rect_raw")
        rs_depth_info_topic = cam_cfg.get("realsense_depth_info_topic", "depth/camera_info")

        keys = og.Controller.Keys

        create_nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("RgbPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ]
        set_values = [
            ("RenderProduct.inputs:cameraPrim", camera_prim_path),
            ("RenderProduct.inputs:width", width),
            ("RenderProduct.inputs:height", height),
            ("CameraInfoPublish.inputs:topicName", info_topic),
            ("CameraInfoPublish.inputs:frameId", frame_id),
            ("CameraInfoPublish.inputs:nodeNamespace", node_namespace),
            ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
            ("RgbPublish.inputs:topicName", image_topic),
            ("RgbPublish.inputs:type", "rgb"),
            ("RgbPublish.inputs:frameId", frame_id),
            ("RgbPublish.inputs:nodeNamespace", node_namespace),
            ("RgbPublish.inputs:resetSimulationTimeOnStop", True),
        ]
        connect = [
            ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
            ("RenderProduct.outputs:execOut", "RgbPublish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "RgbPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "RgbPublish.inputs:context"),
        ]

        if publish_depth:
            create_nodes.append(("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"))
            set_values.extend(
                [
                    ("DepthPublish.inputs:topicName", depth_topic),
                    ("DepthPublish.inputs:type", depth_type),
                    ("DepthPublish.inputs:frameId", frame_id),
                    ("DepthPublish.inputs:nodeNamespace", node_namespace),
                    ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
                ]
            )
            connect.extend(
                [
                    ("RenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
                    ("Context.outputs:context", "DepthPublish.inputs:context"),
                ]
            )

        if publish_realsense_compat:
            create_nodes.extend(
                [
                    ("RsColorInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("RsColorPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ]
            )
            set_values.extend(
                [
                    ("RsColorInfoPublish.inputs:topicName", rs_color_info_topic),
                    ("RsColorInfoPublish.inputs:frameId", rs_color_frame_id),
                    ("RsColorInfoPublish.inputs:nodeNamespace", rs_namespace),
                    ("RsColorInfoPublish.inputs:resetSimulationTimeOnStop", True),
                    ("RsColorPublish.inputs:topicName", rs_color_image_topic),
                    ("RsColorPublish.inputs:type", "rgb"),
                    ("RsColorPublish.inputs:frameId", rs_color_frame_id),
                    ("RsColorPublish.inputs:nodeNamespace", rs_namespace),
                    ("RsColorPublish.inputs:resetSimulationTimeOnStop", True),
                ]
            )
            connect.extend(
                [
                    ("RenderProduct.outputs:execOut", "RsColorInfoPublish.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "RsColorInfoPublish.inputs:renderProductPath"),
                    ("Context.outputs:context", "RsColorInfoPublish.inputs:context"),
                    ("RenderProduct.outputs:execOut", "RsColorPublish.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "RsColorPublish.inputs:renderProductPath"),
                    ("Context.outputs:context", "RsColorPublish.inputs:context"),
                ]
            )

            if publish_depth:
                create_nodes.extend(
                    [
                        ("RsDepthInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                        ("RsDepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ]
                )
                set_values.extend(
                    [
                        ("RsDepthInfoPublish.inputs:topicName", rs_depth_info_topic),
                        ("RsDepthInfoPublish.inputs:frameId", rs_depth_frame_id),
                        ("RsDepthInfoPublish.inputs:nodeNamespace", rs_namespace),
                        ("RsDepthInfoPublish.inputs:resetSimulationTimeOnStop", True),
                        ("RsDepthPublish.inputs:topicName", rs_depth_image_topic),
                        ("RsDepthPublish.inputs:type", depth_type),
                        ("RsDepthPublish.inputs:frameId", rs_depth_frame_id),
                        ("RsDepthPublish.inputs:nodeNamespace", rs_namespace),
                        ("RsDepthPublish.inputs:resetSimulationTimeOnStop", True),
                    ]
                )
                connect.extend(
                    [
                        ("RenderProduct.outputs:execOut", "RsDepthInfoPublish.inputs:execIn"),
                        ("RenderProduct.outputs:renderProductPath", "RsDepthInfoPublish.inputs:renderProductPath"),
                        ("Context.outputs:context", "RsDepthInfoPublish.inputs:context"),
                        ("RenderProduct.outputs:execOut", "RsDepthPublish.inputs:execIn"),
                        ("RenderProduct.outputs:renderProductPath", "RsDepthPublish.inputs:renderProductPath"),
                        ("Context.outputs:context", "RsDepthPublish.inputs:context"),
                    ]
                )

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: create_nodes,
                keys.SET_VALUES: set_values,
                keys.CONNECT: connect,
            },
        )
    except Exception as e:  # noqa: BLE001
        print(
            "ROS2 bridge para câmera não está disponível (isaacsim.ros2.bridge). "
            f"A câmera foi criada no stage, mas não será publicada no ROS2. Erro: {e}"
        )

    return cam
