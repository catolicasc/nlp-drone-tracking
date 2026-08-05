#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${PROJECT_ROOT}/.python_deps/yolo-venv"

if ! python3 -m venv --help >/dev/null 2>&1; then
  echo "Erro: python3-venv não está disponível."
  echo "Instale com: sudo apt install python3-venv python3-full"
  exit 1
fi

echo "==> Instalando YOLO em venv isolado"
echo "    Destino: ${VENV}"
echo "    (não usa pip do sistema — evita PEP 668 e conflito com colcon)"

python3 -m venv "$VENV"
"${VENV}/bin/pip" install 'numpy<1.28' ultralytics

echo ""
echo "OK. Dependências YOLO prontas."
echo "Teste:"
echo "  ${VENV}/bin/python -c \"from ultralytics import YOLO; print('ultralytics OK')\""
