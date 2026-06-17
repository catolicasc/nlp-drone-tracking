# QGroundControl + Oracle Vision (PX4 SITL)

## Como o MAVLink funciona neste projeto

| Papel | Porta UDP | Quem escuta |
|-------|-----------|-------------|
| PX4 GCS (MAVLink) | **18570** | PX4 |
| Destino GCS (QGC) | **14550** | QGroundControl |
| MAVROS (opcional) | fala com **18570** | cliente |
| Simulador HIL (Pegasus) | **4560** TCP | Isaac/Pegasus |

O PX4 **não** fala na 14550 diretamente — ele escuta na **18570** e **envia** telemetria para a **14550** (onde o QGC escuta por padrão).

Sem **broadcast MAVLink** (`-p` no `mavlink start`), o QGC fica em "Disconnected" mesmo com a sim rodando.

O launcher do projeto (`px4_launch_tool.py`) já habilita esse broadcast automaticamente.

---

## Passo a passo

### 1) Subir a simulação

```bash
./scripts/iniciar_drone.sh
```

Aguarde carregar e confirme **PLAY** na timeline do Isaac.

### 2) Abrir o QGroundControl

Baixe em [qgroundcontrol.com](https://qgroundcontrol.com/) (AppImage no Linux).

### 3) Conectar (modo fácil — recomendado)

1. **Não crie** link UDP escutando na porta **18570** (conflita com o PX4).
2. Em **Application Settings → Comm Links**, remova links customizados na 18570 se existirem.
3. Deixe o link padrão **UDP / porta 14550** (auto-connect ligado).
4. Reinicie o QGC ou clique **Connect**.

A barra superior deve ficar **verde** com telemetria.

### 4) Voar

1. Aba **Fly**
2. **Slide to Arm** ou botão **Takeoff**
3. Joystick (PS5): **Settings → Joystick → Enable** e calibre

---

## Se ainda não conectar

### Opção B — link manual para o PX4

Em **Comm Links → Add → UDP**:

| Campo | Valor |
|-------|--------|
| **Port** (porta local do QGC) | `15871` (ou outra livre) |
| **Server Address** | `127.0.0.1` |
| **Server Port** | `18570` |

**Não** use "Listening" na 18570.

### Reiniciar a sim após atualizar o projeto

Se o PX4 já estava rodando **antes** do patch de broadcast:

```bash
# pare a sim (Ctrl+C no iniciar_drone.sh) e suba de novo
./scripts/iniciar_drone.sh
```

### Verificar portas no terminal

```bash
./scripts/check_mavlink_ports.sh
```

Esperado com sim + QGC abertos:

- `px4` em UDP **18570**
- `QGroundControl` em UDP **14550**

### MAVROS ao mesmo tempo

MAVROS e QGC podem coexistir. O MAVROS usa `udp://@127.0.0.1:18570`.

Se usar só QGC, **não** precisa do `run_mavros_px4.sh`.

---

## Erros comuns

| Sintoma | Causa | Solução |
|---------|--------|---------|
| Disconnected | Link errado na 18570 (listen) | Use padrão 14550 ou link manual acima |
| Disconnected | Sim não rodando / pausada | `iniciar_drone.sh` + PLAY |
| Não arma | Sem GCS na preflight | Conectar QGC primeiro |
| Conflito | MAVROS + link errado | Feche links duplicados na 18570 |
