# V2 Camera Setup

A câmera V2 é configurada em `config/sim.yaml` dentro de `sensors.camera`.

Perfil atual:

```yaml
sensors:
  camera:
    enabled: true
    namespace: drone/camera
    topic: image_raw
    info_topic: camera_info
    frame_id: drone_camera
    width: 960
    height: 540
    frequency: 15
    position: [0.22, 0.0, 0.06]
    orientation_mode: forward_down
    pitch_down_deg: 25.0
    publish_depth: false
```

`orientation_mode: forward_down` faz a câmera olhar para frente do drone, usando o eixo `+X` como frente, com inclinação para baixo definida por `pitch_down_deg`.

## Teste Rápido

Depois de abrir Isaac, pressionar Play e iniciar ROS:

```bash
./scripts/v2_status.sh doctor
ros2 topic hz /drone/camera/image_raw
ros2 topic echo --once /drone/camera/camera_info
ros2 run rqt_image_view rqt_image_view
```

No `rqt_image_view`, selecione:

```text
/drone/camera/image_raw
```

## Ajustes Comuns

- Para ver mais chão perto do drone, aumente `pitch_down_deg` para `35.0`.
- Para ver mais horizonte/pessoas à frente, reduza `pitch_down_deg` para `10.0` ou `15.0`.
- Para melhorar desempenho da VLM, use `width: 640` e `height: 360`.
- Para mais detalhe visual, use `width: 1280` e `height: 720`.

Reinicie o Isaac Sim depois de mudar `config/sim.yaml`, porque a câmera é criada no startup da simulação.
