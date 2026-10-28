# shellcheck shell=bash
# Adiciona dependências YOLO ao PYTHONPATH (venv isolado ou --target legado).
# Requer PROJECT_ROOT definido antes do source.

_yolo_python_site() {
  local pyver
  pyver="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  echo "${PROJECT_ROOT}/.python_deps/yolo-venv/lib/python${pyver}/site-packages"
}

if [ -n "${PROJECT_ROOT:-}" ]; then
  YOLO_SITE="$(_yolo_python_site)"
  if [ -d "$YOLO_SITE" ]; then
    export PYTHONPATH="${YOLO_SITE}${PYTHONPATH:+:$PYTHONPATH}"
  elif [ -d "${PROJECT_ROOT}/.python_deps/yolo" ]; then
    export PYTHONPATH="${PROJECT_ROOT}/.python_deps/yolo${PYTHONPATH:+:$PYTHONPATH}"
  fi
fi
