# Experimentos UAV V2 — Postgres + coletor

Infraestrutura para coletar os eventos de `/v2/status` das missões num Postgres
(docker-compose), **sem tocar no agente** — zero impacto na performance da
aplicação. O agente continua publicando `/v2/status`; um nó coletor separado
assina, enfileira e grava em batch.

```text
v2_vlm_agent (não muda) ── publica /v2/status ──▶ v2_experiment_recorder
                                                       │ fila + thread
                                                       ▼
                                              Postgres (docker)
                                                       │
                                              metrics.py (análise)
```

## 1. Subir o banco

```bash
cd experiments
docker compose up -d
docker compose exec -T postgres psql -U v2 -d v2_experiments -f /schema.sql
```

Credenciais default: `v2 / v2` (banco `v2_experiments`). Sobrescreva via `.env`
se precisar. Dados persistem no volume `pgdata`.

## 2. Instalar dependência do coletor

O coletor usa `psycopg` (import lazy — o agente principal não quebra se faltar):

```bash
pip install "psycopg[binary]"
```

## 3. Build + rodar o coletor

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --packages-select v2_vlm_drone && source install/setup.bash
ros2 run v2_vlm_drone v2_experiment_recorder
```

Parâmetros ROS (opcionais, com defaults):

| Parâmetro | Default | Descrição |
| --- | --- | --- |
| `status_topic` | `/v2/status` | tópico de eventos do agente |
| `db_url` | `postgresql://v2:v2@localhost:5432/v2_experiments` | conexão Postgres |
| `experiment_id` | timestamp do início | rótulo da sessão (agrupa as missões) |
| `flush_interval_s` | `2` | grava em batch a cada N segundos |
| `batch_size` | `100` | tamanho máximo do batch |

Exemplo com `experiment_id` explícito:

```bash
ros2 run v2_vlm_drone v2_experiment_recorder --ros-args -p experiment_id:=cenario_A_run1
```

Se o banco cair, o coletor loga e tenta de novo na próxima rodada — a missão
não é afetada.

## 4. Analisar os resultados

### Resumo rápido (SQL, sem dependências)

```bash
python3 experiments/metrics.py --db postgresql://v2:v2@localhost:5432/v2_experiments
```

Imprime por `experiment_id`: taxa de conclusão, nº de passos, falhas por tipo e
tempo de missão, em tabela Markdown pronta para o artigo.

### Análise rica com pandas (gráficos)

Para exploração interativa e gráficos (taxa de sucesso, distribuição de passos)
que entram na seção Experiments do artigo:

```bash
pip install pandas matplotlib seaborn "psycopg[binary]" jupyter
jupyter notebook experiments/analyze.ipynb
```

O notebook lê direto do Postgres via `pandas.read_sql()`, agrupa os eventos em
missões e gera:

- `experiments/outputs/taxa_sucesso.png` — taxa de sucesso por experimento
- `experiments/outputs/passos_missao.png` — histograma + boxplot de passos
- tabela resumo (com `to_latex()` pronto para o artigo)

Os PNGs em `outputs/` alimentam as figuras da seção Experiments; a tabela
resumo alimenta o `% TODO(evidência)` do abstract.

## 5. Consultas SQL úteis

```sql
-- Taxa de conclusão por experimento
SELECT experiment_id,
       count(*) FILTER (WHERE event_type='finished' AND payload->>'success'='true') AS ok,
       count(*) FILTER (WHERE event_type='finished') AS total
FROM experiment_events GROUP BY experiment_id;

-- Eventos de uma missão (ex.: a mais recente do experimento X)
SELECT ts, event_type, payload FROM experiment_events
WHERE experiment_id = 'X' ORDER BY ts;
```
