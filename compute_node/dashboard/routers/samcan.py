"""
Router: HTTP-прокси /api/samcan/* → samcan_bridge на :5005.

samcan_bridge.py — отдельный хост-процесс который держит USB-Serial к
второму роботу (Arduino Uno). dashboard прокидывает запросы фронта
без логики (просто перебрасывает body, query, headers, status).

На Linux Docker (--net=host) → localhost:5005.
На Windows/macOS Docker → host.docker.internal:5005 (сетит .sh скрипт
через переменную SAMCAN_BRIDGE_URL).
"""
from __future__ import annotations

import logging
import os

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse, Response

log = logging.getLogger(__name__)

SAMCAN_BRIDGE_URL = os.environ.get('SAMCAN_BRIDGE_URL', 'http://localhost:5005')

router = APIRouter()


@router.api_route(
    '/{path:path}',
    methods=['GET', 'POST', 'PUT', 'DELETE', 'PATCH'],
    tags=['samcan'],
)
async def samcan_proxy(path: str, request: Request) -> Response:
    """Catch-all proxy: пересылаем body, query params, headers; возвращаем как есть."""
    try:
        import httpx
    except ImportError:
        return JSONResponse({'error': 'httpx not installed'}, status_code=500)

    target = f'{SAMCAN_BRIDGE_URL}/api/samcan/{path}'
    body = await request.body()
    headers = {
        k: v for k, v in request.headers.items()
        if k.lower() not in ('host', 'content-length', 'connection')
    }

    try:
        async with httpx.AsyncClient(timeout=3.0) as client:
            r = await client.request(
                method=request.method,
                url=target,
                params=dict(request.query_params),
                content=body,
                headers=headers,
            )
        return Response(
            content=r.content,
            status_code=r.status_code,
            media_type=r.headers.get('content-type', 'application/json'),
        )
    except Exception as exc:
        log.warning('samcan_bridge unreachable: %s', exc)
        return JSONResponse(
            {'error': 'samcan_bridge unreachable', 'detail': str(exc)},
            status_code=503,
        )
