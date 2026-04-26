"""
Routers: H.264 endpoint discovery + WebSocket прокси /ws/h264.

С #9 (2026-04) JPEG MQTT поток заменён на H.264 TCP сервер на Pi.
camera_node на Pi публикует discovery JSON в samurai/{id}/camera/endpoint
(retained). Здесь два endpoint'а:

  GET       /api/camera/endpoint  → JSON (host, port, codec, w, h, fps)
  WebSocket /ws/h264              → проксирует TCP-поток с Pi в WebSocket

Каждый WS клиент → отдельный TCP коннект к Pi (Pi-side TCPStreamServer
multiplex'ит сам и шлёт init_buffer late-joiner'ам).

Старые endpoints /video_feed, /api/camera/frame и /ws/camera УДАЛЕНЫ —
браузер декодирует H.264 через WebCodecs API + canvas (см. CameraFeed.tsx).
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect

from ..schemas.common import OkResponse
from ._deps import StateDep

log = logging.getLogger(__name__)


class CameraEndpointResponse(OkResponse):
    """H.264 discovery: host/port/codec параметры для VideoDecoder в браузере."""
    host: str
    port: int = 8554
    codec: str = 'h264'
    format: str = 'annex-b'
    width: int = 0
    height: int = 0
    fps: int = 25
    extra: dict[str, Any] = {}


router = APIRouter()


@router.get('/endpoint', response_model=CameraEndpointResponse, tags=['camera'])
async def get_camera_endpoint(state: StateDep) -> CameraEndpointResponse:
    """Куда подключаться за H.264 потоком (host:port + codec params).

    503 если Pi-side camera_node не подключён.
    """
    with state.lock:
        ep = state.camera.h264_endpoint
    if not ep:
        raise HTTPException(503, 'Camera offline')
    known = {'host', 'port', 'codec', 'format', 'width', 'height', 'fps'}
    extra = {k: v for k, v in ep.items() if k not in known}
    return CameraEndpointResponse(
        host=ep.get('host', ''),
        port=int(ep.get('port', 8554)),
        codec=ep.get('codec', 'h264'),
        format=ep.get('format', 'annex-b'),
        width=int(ep.get('width', 0)),
        height=int(ep.get('height', 0)),
        fps=int(ep.get('fps', 25)),
        extra=extra,
    )


# WebSocket /ws/h264 — отдельный sub-router (без префикса /api).
ws_router = APIRouter()


@ws_router.websocket('/ws/h264')
async def ws_h264(ws: WebSocket, state: StateDep):
    """Прокси H.264 TCP→WS потока (Pi → браузер).

    Каждый клиент открывает свой TCP коннект к Pi camera_node.
    Pi-side TCPStreamServer выдаёт init_buffer late-joiner'ам.
    """
    await ws.accept()

    with state.lock:
        ep = state.camera.h264_endpoint
    if not ep:
        await ws.close(code=1011, reason='Camera offline (no endpoint)')
        return

    host = ep.get('host')
    port = int(ep.get('port', 8554))
    if not host:
        await ws.close(code=1011, reason='Bad endpoint (no host)')
        return

    # Сначала шлём metadata (codec/dim/fps) — фронт нужно для VideoDecoder init
    try:
        await ws.send_json({
            'type': 'endpoint',
            'codec': ep.get('codec', 'h264'),
            'format': ep.get('format', 'annex-b'),
            'width': ep.get('width'),
            'height': ep.get('height'),
            'fps': ep.get('fps'),
        })
    except Exception:
        return

    try:
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(host, port), timeout=5.0)
    except (OSError, asyncio.TimeoutError) as e:
        await ws.close(code=1011, reason=f'TCP connect failed: {e}')
        return

    try:
        while True:
            data = await reader.read(64 * 1024)
            if not data:
                break
            await ws.send_bytes(data)
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        log.warning('WS h264 proxy error: %s', exc)
    finally:
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass
