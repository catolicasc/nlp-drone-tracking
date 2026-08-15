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

# Ubuntu sem python3-venv completo: 'python3 -m venv' falha no ensurepip
# ("apt install python3.12-venv"). Fallback sem root: venv --without-pip +
# bootstrap do pip via get-pip.py.
if python3 -m venv "$VENV" 2>/tmp/yolo_venv_err.log; then
  PIP="${VENV}/bin/pip"
else
  echo "    (venv sem ensurepip — bootstrapando pip via get-pip.py;"
  echo "     alternativa definitiva: sudo apt install python3.12-venv)"
  rm -rf "$VENV"
  python3 -m venv --without-pip "$VENV"
  curl -sSL https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
  "${VENV}/bin/python" /tmp/get-pip.py -q
  PIP="${VENV}/bin/pip"
fi

"${PIP}" install 'numpy<1.28' ultralytics

echo ""
echo "OK. Dependências YOLO prontas."
echo "Teste:"
echo "  ${VENV}/bin/python -c \"from ultralytics import YOLO; print('ultralytics OK')\""
