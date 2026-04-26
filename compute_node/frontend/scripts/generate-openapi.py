"""
Сгенерировать openapi.json из FastAPI app в compute_node/dashboard/.

Запуск (из compute_node/frontend/):
    python scripts/generate-openapi.py

Или через npm:
    npm run generate:openapi

Файл сохраняется в openapi.json в текущей директории (compute_node/frontend/).
Дальше openapi-typescript-codegen генерирует TypeScript клиент:
    npm run generate:api
"""
from __future__ import annotations

import json
import os
import sys

# Добавляем корень репо в sys.path чтобы импортить compute_node.dashboard
HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, REPO_ROOT)


def main(out_path: str = 'openapi.json') -> int:
    from compute_node.dashboard.app import create_app
    from compute_node.dashboard.state import DashboardState

    state = DashboardState()
    app = create_app(state, mqtt=None, ros2=None, enable_socketio=False)
    spec = app.openapi()

    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(spec, f, indent=2, ensure_ascii=False)

    paths = sorted(spec.get('paths', {}).keys())
    print(f'OpenAPI written: {out_path}')
    print(f'  paths: {len(paths)}')
    print(f'  components.schemas: {len(spec.get("components", {}).get("schemas", {}))}')
    return 0


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else 'openapi.json'
    sys.exit(main(out))
