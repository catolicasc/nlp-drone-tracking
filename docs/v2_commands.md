# V2 Commands

A V2 recebe linguagem natural em `/v2/task`. O VLM decide quais tools chamar, então os comandos abaixo são exemplos de intenção, não uma gramática fixa.

## Voo básico

```bash
./scripts/v2_send_task.sh "decole 1 metro"
./scripts/v2_send_task.sh "avance 1 metro e mantenha a altitude"
./scripts/v2_send_task.sh "gire 90 graus para a direita"
./scripts/v2_send_task.sh "fique parado por 5 segundos"
./scripts/v2_send_task.sh "pouse"
```

## Missões compostas

```bash
./scripts/v2_send_task.sh "decole 1 metro, avance 1 metro, olhe a câmera e descreva o que vê"
./scripts/v2_send_task.sh "decole 80 centímetros, vá meio metro para a esquerda e pouse"
./scripts/v2_send_task.sh "suba para 1.5 metro, gire lentamente para observar a cena e finalize"
```

## Percepção pela câmera

```bash
./scripts/v2_send_task.sh "olhe pela câmera e diga se existe uma pessoa visível"
./scripts/v2_send_task.sh "decole 1 metro e use a câmera para descrever o ambiente"
```

`inspect_camera` usa o último frame de `/drone/camera/image_raw` e envia a imagem ao mesmo endpoint VLM configurado.

## Segurança

- Movimentos exigem MAVROS conectado.
- `takeoff` limita altitude pelos parâmetros ROS `min_alt_m` e `max_alt_m`.
- `goto_position` e `goto_relative` limitam distância pelo parâmetro ROS `max_radius_m`.
- `land`, `pouse`, `stop` e similares podem interromper uma missão em andamento.
- Não há planner heurístico: sem VLM configurado, nenhum comando é executado.

