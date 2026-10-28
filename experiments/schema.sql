-- Schema dos experimentos do UAV V2.
-- Tabela de eventos brutos coletados de /v2/status pelo v2_experiment_recorder.
-- Análises agregadas são feitas por experiments/metrics.py (não aqui).

CREATE TABLE IF NOT EXISTS experiment_events (
    id            BIGSERIAL PRIMARY KEY,
    experiment_id TEXT NOT NULL,
    ts            TIMESTAMPTZ NOT NULL DEFAULT now(),
    event_type    TEXT NOT NULL,
    payload       JSONB NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_events_experiment_ts ON experiment_events (experiment_id, ts);
CREATE INDEX IF NOT EXISTS idx_events_type          ON experiment_events (event_type);

-- Exemplo de consulta: taxa de conclusão por experimento.
-- SELECT experiment_id,
--        count(*) FILTER (WHERE event_type = 'finished' AND payload->>'success' = 'true') AS ok,
--        count(*) FILTER (WHERE event_type = 'finished') AS total
-- FROM experiment_events
-- GROUP BY experiment_id;
