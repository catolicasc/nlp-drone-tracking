"""ROS 2 node that records /v2/status events into Postgres.

Assina /v2/status (String JSON) publicado pelo v2_vlm_agent, enfileira os
eventos em memória e uma thread grava em batch no Postgres. O agente não é
alterado — este nó é um observador separado, então não impacta a performance
da aplicação. Se o banco estiver fora do ar, o coletor loga e tenta de novo.

Uso:
    ros2 run v2_vlm_drone v2_experiment_recorder
    ros2 run v2_vlm_drone v2_experiment_recorder \
        --ros-args -p experiment_id:=cenario_A_run1
"""

from __future__ import annotations

import json
import queue
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Import lazy: só o coletor precisa de psycopg. Se faltar, o agente principal
# (v2_vlm_agent) continua funcionando normalmente.
try:
    import psycopg
except ImportError:  # pragma: no cover - ambiente sem a dependência
    psycopg = None


def _db_connect(db_url: str) -> Any:
    if psycopg is None:
        raise RuntimeError(
            "psycopg não está instalado. Rode: pip install 'psycopg[binary]'"
        )
    return psycopg.connect(db_url)


class ExperimentRecorder(Node):
    """Assina /v2/status e grava eventos em Postgres em batch."""

    def __init__(self) -> None:
        super().__init__("v2_experiment_recorder")

        self.declare_parameter("status_topic", "/v2/status")
        self.declare_parameter(
            "db_url", "postgresql://v2:v2@localhost:5432/v2_experiments"
        )
        self.declare_parameter("experiment_id", "")
        self.declare_parameter("flush_interval_s", 2.0)
        self.declare_parameter("batch_size", 100)

        self._experiment_id = (
            self.get_parameter("experiment_id").value
            or time.strftime("exp_%Y%m%d_%H%M%S")
        )
        self._db_url = self.get_parameter("db_url").value
        self._flush_interval = float(self.get_parameter("flush_interval_s").value)
        self._batch_size = int(self.get_parameter("batch_size").value)

        self._queue: queue.Queue[tuple[str, dict[str, Any]]] = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._flush_loop, name="recorder_flush", daemon=True
        )

        self.create_subscription(
            String,
            self.get_parameter("status_topic").value,
            self._on_status,
            10,
        )

        self.get_logger().info(
            f"Coletor de experimentos pronto | experiment_id={self._experiment_id} "
            f"| topic={self.get_parameter('status_topic').value}"
        )
        self._thread.start()

    # ---- recepção ----------------------------------------------------------

    def _on_status(self, msg: String) -> None:
        """Enfileira o evento; nunca bloqueia aqui (o agente segue livre)."""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"JSON inválido em /v2/status: {msg.data[:120]!r}")
            return
        event_type = str(payload.get("event", "unknown"))
        self._queue.put((event_type, payload))

    # ---- escrita em batch --------------------------------------------------

    def _flush_loop(self) -> None:
        """Thread daemon: drena a fila e grava em batch no Postgres."""
        while not self._stop.is_set():
            try:
                self._flush_once()
            except Exception as exc:  # noqa: BLE001 - banco fora do ar etc.
                self.get_logger().error(f"Falha ao gravar no banco: {exc}")
                # Não derruba a thread; tenta de novo no próximo ciclo.
            self._stop.wait(self._flush_interval)

    def _flush_once(self) -> None:
        if self._queue.empty():
            return
        batch: list[tuple[str, dict[str, Any]]] = []
        while len(batch) < self._batch_size and not self._queue.empty():
            batch.append(self._queue.get_nowait())

        conn = None
        try:
            conn = _db_connect(self._db_url)
            with conn.cursor() as cur:
                cur.executemany(
                    """
                    INSERT INTO experiment_events (experiment_id, event_type, payload)
                    VALUES (%s, %s, %s::jsonb)
                    """,
                    [
                        (self._experiment_id, evt, json.dumps(pld, ensure_ascii=False))
                        for evt, pld in batch
                    ],
                )
            conn.commit()
            self.get_logger().debug(f"Gravados {len(batch)} eventos")
        finally:
            if conn is not None:
                conn.close()

    # ---- shutdown ----------------------------------------------------------

    def shutdown(self) -> None:
        # Drena o que sobrou antes de parar.
        self._stop.set()
        if not self._queue.empty():
            try:
                self._flush_once()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Falha no flush final: {exc}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ExperimentRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
