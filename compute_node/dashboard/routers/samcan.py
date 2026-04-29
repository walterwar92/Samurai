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

import asyncio
import logging
import os

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse, Response, StreamingResponse

try:
    import httpx
    _HAS_HTTPX = True
except ImportError:
    httpx = None  # type: ignore[assignment]
    _HAS_HTTPX = False

log = logging.getLogger(__name__)

SAMCAN_BRIDGE_URL = os.environ.get('SAMCAN_BRIDGE_URL', 'http://localhost:5005')

# Tuned for a localhost / docker-host bridge: connect should be near-instant,
# read may legitimately take longer when the Arduino is mid-command.
# Pool reuse avoids the per-request TLS handshake / socket setup that the old
# `async with httpx.AsyncClient()` per-request pattern incurred.
_TIMEOUT = httpx.Timeout(connect=1.0, read=3.0, write=3.0, pool=3.0) if _HAS_HTTPX else None
_LIMITS = httpx.Limits(max_connections=10, max_keepalive_connections=5) if _HAS_HTTPX else None
_RETRYABLE_METHODS = {'GET', 'HEAD', 'PUT', 'DELETE', 'OPTIONS'}
_BACKOFF_S = (0.1, 0.3, 0.6)        # 3 retries → ≤1s extra latency before failing
_MAX_ATTEMPTS = len(_BACKOFF_S) + 1

# Module-level shared client. FastAPI lifespan cleanup is in app.py if needed,
# but a leaked AsyncClient on shutdown is harmless (process exit closes sockets).
_shared_client: httpx.AsyncClient | None = None


def _get_client() -> httpx.AsyncClient:
    global _shared_client
    if _shared_client is None:
        _shared_client = httpx.AsyncClient(timeout=_TIMEOUT, limits=_LIMITS)
    return _shared_client


router = APIRouter()


async def _proxy(path: str, request: Request) -> Response:
    """Общая логика проксирования (используется всеми HTTP методами).

    Retries:
      Idempotent methods retry on connect errors AND read timeouts AND 5xx.
      POST/PATCH retry ONLY on connect-time errors (the request never reached
      the bridge). Retrying after a read timeout could double-execute a
      command that the Arduino already started processing.
    """
    if not _HAS_HTTPX:
        return JSONResponse({'error': 'httpx not installed'}, status_code=500)

    target = f'{SAMCAN_BRIDGE_URL}/api/samcan/{path}'
    body = await request.body()
    headers = {
        k: v for k, v in request.headers.items()
        if k.lower() not in ('host', 'content-length', 'connection')
    }
    method = request.method.upper()
    is_idempotent = method in _RETRYABLE_METHODS

    client = _get_client()
    last_exc: Exception | None = None
    last_response: httpx.Response | None = None

    for attempt in range(_MAX_ATTEMPTS):
        try:
            r = await client.request(
                method=method,
                url=target,
                params=dict(request.query_params),
                content=body,
                headers=headers,
            )
        except (httpx.ConnectError, httpx.ConnectTimeout) as exc:
            # Request never reached the bridge — always safe to retry, even POST.
            last_exc = exc
            if attempt < _MAX_ATTEMPTS - 1:
                await asyncio.sleep(_BACKOFF_S[attempt])
                continue
            break
        except httpx.ReadTimeout as exc:
            last_exc = exc
            # Bridge accepted the request but didn't respond in time. For a
            # POST that may already have actuated the Arduino, retrying
            # could duplicate the command — only retry idempotent verbs.
            if is_idempotent and attempt < _MAX_ATTEMPTS - 1:
                await asyncio.sleep(_BACKOFF_S[attempt])
                continue
            break
        except Exception as exc:
            # Unknown httpx error — fail fast, surface to caller.
            log.warning('samcan_bridge proxy error [%s]: %s', method, exc)
            return JSONResponse(
                {'error': 'samcan_bridge unreachable', 'detail': str(exc)},
                status_code=503,
            )

        # Got a response. Retry on 5xx for idempotent verbs only.
        if 500 <= r.status_code < 600 and is_idempotent and attempt < _MAX_ATTEMPTS - 1:
            last_response = r
            await asyncio.sleep(_BACKOFF_S[attempt])
            continue

        return Response(
            content=r.content,
            status_code=r.status_code,
            media_type=r.headers.get('content-type', 'application/json'),
        )

    # All attempts exhausted.
    if last_response is not None:
        # Last attempt was a 5xx — return it so caller sees the actual status.
        return Response(
            content=last_response.content,
            status_code=last_response.status_code,
            media_type=last_response.headers.get('content-type', 'application/json'),
        )
    log.warning('samcan_bridge unreachable after %d attempts: %s',
                _MAX_ATTEMPTS, last_exc)
    return JSONResponse(
        {'error': 'samcan_bridge unreachable',
         'detail': str(last_exc) if last_exc else 'unknown',
         'attempts': _MAX_ATTEMPTS},
        status_code=503,
    )


# Streaming proxy specifically for SSE (#29). httpx's regular client
# buffers the whole response, which would defeat the point of an
# event stream. We use stream() and forward chunks as they arrive.
@router.get('/stream', tags=['samcan'], operation_id='samcan_stream')
async def samcan_stream(request: Request) -> Response:
    if not _HAS_HTTPX:
        return JSONResponse({'error': 'httpx not installed'}, status_code=500)
    target = f'{SAMCAN_BRIDGE_URL}/api/samcan/stream'

    async def gen():
        # Use a fresh client for streaming so the keepalive isn't shared
        # with short-request consumers.
        timeout = httpx.Timeout(connect=2.0, read=None, write=2.0, pool=2.0)
        try:
            async with httpx.AsyncClient(timeout=timeout) as sclient:
                async with sclient.stream('GET', target) as r:
                    async for chunk in r.aiter_bytes():
                        if await request.is_disconnected():
                            break
                        yield chunk
        except Exception as exc:
            log.warning('samcan SSE proxy error: %s', exc)
            yield f'event: error\ndata: {{"error":"{exc}"}}\n\n'.encode()

    return StreamingResponse(
        gen(),
        media_type='text/event-stream',
        headers={
            'Cache-Control': 'no-cache',
            'Connection': 'keep-alive',
            'X-Accel-Buffering': 'no',
        },
    )


# Отдельные routes per-method чтобы у каждого был уникальный operation_id —
# openapi-typescript-codegen иначе ломается на duplicate IDs.
@router.get('/{path:path}', tags=['samcan'], operation_id='samcan_get')
async def samcan_get(path: str, request: Request) -> Response:
    return await _proxy(path, request)


@router.post('/{path:path}', tags=['samcan'], operation_id='samcan_post')
async def samcan_post(path: str, request: Request) -> Response:
    return await _proxy(path, request)


@router.put('/{path:path}', tags=['samcan'], operation_id='samcan_put')
async def samcan_put(path: str, request: Request) -> Response:
    return await _proxy(path, request)


@router.delete('/{path:path}', tags=['samcan'], operation_id='samcan_delete')
async def samcan_delete(path: str, request: Request) -> Response:
    return await _proxy(path, request)


@router.patch('/{path:path}', tags=['samcan'], operation_id='samcan_patch')
async def samcan_patch(path: str, request: Request) -> Response:
    return await _proxy(path, request)
