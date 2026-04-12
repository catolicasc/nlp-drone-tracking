from __future__ import annotations


def setup_sensors(config: dict, drone_prim_path: str = "/World/quadrotor"):
    sensors_cfg = (config or {}).get("sensors", {})
    cam_cfg = sensors_cfg.get("camera", {})
    if not cam_cfg.get("enabled", False):
        return None

    width = int(cam_cfg.get("width", 1280))
    height = int(cam_cfg.get("height", 720))

    camera_prim_path = f"{drone_prim_path}/camera"

    try:
        from omni.isaac.sensor import Camera
    except Exception as e:  # noqa: BLE001
        print(f"Não foi possível importar Camera do Isaac: {e}")
        return None

    cam = Camera(
        prim_path=camera_prim_path,
        position=(0.20, 0.0, 0.10),
        orientation=(0.0, 0.0, 0.0, 1.0),
        frequency=30,
        resolution=(width, height),
    )
    cam.initialize()

    try:
        import omni.graph.core as og

        from isaacsim.core.utils.stage import get_next_free_path

        graph_path = get_next_free_path(cam_cfg.get("graph_path", "/Graph/ROS_Camera"), "")
        node_namespace = cam_cfg.get("namespace", "drone/camera")
        image_topic = cam_cfg.get("topic", "image_raw")
        info_topic = cam_cfg.get("info_topic", "camera_info")
        frame_id = cam_cfg.get("frame_id", "drone_camera")

        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("RgbPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                keys.SET_VALUES: [
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
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
                    ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
                    ("RenderProduct.outputs:execOut", "RgbPublish.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "RgbPublish.inputs:renderProductPath"),
                    ("Context.outputs:context", "RgbPublish.inputs:context"),
                ],
            },
        )
    except Exception as e:  # noqa: BLE001
        print(
            "ROS2 bridge para câmera não está disponível (isaacsim.ros2.bridge). "
            f"A câmera foi criada no stage, mas não será publicada no ROS2. Erro: {e}"
        )

    return cam
