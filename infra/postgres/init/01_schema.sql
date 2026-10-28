-- Schema criado automaticamente na primeira subida do container
-- (docker-entrypoint-initdb.d roda só quando o volume está vazio).
--
-- Tabela consumida pelo nó v2_experiment_recorder (ROS 2).
-- Cada evento publicado em /v2/status vira uma linha aqui.

CREATE TABLE IF NOT EXISTS experiment_events (
    id          BIGSERIAL    PRIMARY KEY,
    experiment_id TEXT       NOT NULL,
    event_type    TEXT       NOT NULL,
    payload       JSONB      NOT NULL,
    ts            TIMESTAMPTZ NOT NULL DEFAULT now()
);

-- Índices para os filtros mais comuns no pandas pós-missão:
--   - agrupar por experimento
--   - filtrar por tipo de evento (vlm_step, tool_result, finished...)
--   - ordenar cronologicamente dentro de um experimento
CREATE INDEX IF NOT EXISTS idx_events_experiment
    ON experiment_events (experiment_id, ts);
CREATE INDEX IF NOT EXISTS idx_events_type
    ON experiment_events (experiment_id, event_type);

-- View de conveniência: últimos eventos de cada experimento
CREATE OR REPLACE VIEW v_latest_events AS
SELECT DISTINCT ON (experiment_id) experiment_id, event_type, payload, ts
FROM experiment_events
ORDER BY experiment_id, ts DESC;

COMMENT ON TABLE  experiment_events IS 'Stream de eventos do agente VLM gravado pelo v2_experiment_recorder.';
COMMENT ON COLUMN experiment_events.payload IS 'JSON completo do evento (task, tool, arguments, result, timestamp do agente...).';
