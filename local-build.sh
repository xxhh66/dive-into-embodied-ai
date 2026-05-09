#!/usr/bin/env sh

set -eu

REQUESTED_PORT="${1:-3001}"
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

is_port_in_use() {
  port="$1"

  if command -v lsof >/dev/null 2>&1; then
    lsof -nP -iTCP:"$port" -sTCP:LISTEN >/dev/null 2>&1
    return $?
  fi

  if command -v nc >/dev/null 2>&1; then
    nc -z 127.0.0.1 "$port" >/dev/null 2>&1 || nc -z ::1 "$port" >/dev/null 2>&1
    return $?
  fi

  return 1
}

find_available_port() {
  port="$1"

  while is_port_in_use "$port"; do
    port=$((port + 1))
    if [ "$port" -gt 65535 ]; then
      echo "No available port found starting from ${REQUESTED_PORT}." >&2
      exit 1
    fi
  done

  printf '%s\n' "$port"
}

cd "$SCRIPT_DIR"

# 本地预览不需要真实职位数据,默认走 fixture,避免 prebuild 因缺 JOBS_API_TOKEN 报错
export JOBS_DATA_MODE="${JOBS_DATA_MODE:-fixture}"

echo "==> Building site... (JOBS_DATA_MODE=${JOBS_DATA_MODE})"
npm run build

PORT=$(find_available_port "$REQUESTED_PORT")

if [ "$PORT" != "$REQUESTED_PORT" ]; then
  echo "==> Port ${REQUESTED_PORT} is in use; using ${PORT} instead"
fi

echo "==> Serving build output on http://localhost:${PORT}"
exec ./node_modules/.bin/docusaurus serve --dir build --port "${PORT}"
