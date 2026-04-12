from __future__ import annotations


def _ensure_camera_visual_marker(camera_prim_path: str) -> None:
    try:
        from omni.isaac.core.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom
    except Exception:
        return

    stage = get_current_stage()
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


def _set_local_xform(prim_path: str, position_xyz: tuple[float, float, float], orientation_xyzw: tuple[float, float, float, float]) -> None:
    try:
        from omni.isaac.core.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom
    except Exception:
        return

    stage = get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return

    x, y, z = position_xyz
    qx, qy, qz, qw = orientation_xyzw

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(float(x), float(y), float(z)))
    xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(float(qw), Gf.Vec3d(float(qx), float(qy), float(qz)))
    )


def _print_world_translation(prim_path: str, label: str) -> None:
    try:
        from omni.isaac.core.utils.stage import get_current_stage
        from pxr import UsdGeom
    except Exception:
        return

    stage = get_current_stage()
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

    pos_cfg = cam_cfg.get("position", (0.0, 0.0, 0.0))
    cam_orientation: tuple[float, float, float, float]
    if "orientation_xyzw" in cam_cfg:
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
        from omni.isaac.core.utils.prims import is_prim_path_valid

        configured_mount_parent: str | None = None
        if mount_prim:
            if mount_prim.startswith("/"):
                configured_mount_parent = mount_prim
            else:
                configured_mount_parent = f"{drone_prim_path}/{mount_prim}"

        if configured_mount_parent and is_prim_path_valid(configured_mount_parent):
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
                if is_prim_path_valid(p):
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

    try:
        from omni.isaac.sensor import Camera
    except Exception as e:  # noqa: BLE001
        print(f"Não foi possível importar Camera do Isaac: {e}")
        return None

    cam = Camera(
        prim_path=camera_prim_path,
        position=(0.0, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        frequency=30,
        resolution=(width, height),
    )
    cam.initialize()

    _set_local_xform(camera_prim_path, cam_position, cam_orientation)
    _print_world_translation(camera_prim_path, "Camera")

    if bool(cam_cfg.get("visualize", True)):
        _ensure_camera_visual_marker(camera_prim_path)

    try:
        import omni.graph.core as og

        from isaacsim.core.utils.stage import get_next_free_path

        graph_path = get_next_free_path(cam_cfg.get("graph_path", "/Graph/ROS_Camera"), "")
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
