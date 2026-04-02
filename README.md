# Oracle Vision

Projeto versionável com:
- Isaac Sim
- Pegasus Simulator
- PX4
- ROS 2 (humble)

## Estrutura

- `apps/isaac_app/standalone`: scripts principais do Isaac Sim
- `apps/pegasus_app`: lógica ligada ao Pegasus
- `ros2_ws`: workspace ROS 2
- `config`: arquivos de configuração
- `scripts`: scripts utilitários
- `assets`: USD, mapas, URDFs
- `tests`: testes simples

## Como usar

### 1. Criar o projeto
Esse projeto já foi gerado com o script de scaffold.

### 2. Ajustar o .env
Copie o exemplo:

```bash
cp .env.example .env
```

Edite os caminhos conforme sua máquina.

### 3. Rodar ROS 2
```bash
./scripts/run_ros.sh
```

### 4. Rodar Isaac Sim
```bash
./scripts/run_isaac.sh
```

## Observações

- O Isaac Sim não é versionado no Git.
- O projeto referencia instalações já existentes na máquina.
- Os cenários devem ser reproduzíveis por código.
