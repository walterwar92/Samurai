"""
Базовые схемы: успех/ошибка, общие типы.
"""
from __future__ import annotations

from typing import Any, Optional

from pydantic import BaseModel, Field


class OkResponse(BaseModel):
    """Стандартный успешный ответ. Дополнительные поля — у наследников."""
    ok: bool = True


class ErrorResponse(BaseModel):
    """Стандартная ошибка. Старый dashboard_node возвращал {ok: False, error: str}."""
    ok: bool = False
    error: str
    code: Optional[str] = Field(
        default=None,
        description='Машинно-читаемый код ошибки (опционально, для i18n на фронте).'
    )
    detail: Optional[Any] = Field(
        default=None,
        description='Дополнительный контекст ошибки (stacktrace, conflict info и т.д.).'
    )


class CommandAck(OkResponse):
    """Подтверждение команды (без возвращаемых данных)."""
    pass
