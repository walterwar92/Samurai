"""
Pydantic schemas for dashboard API request/response models.

Каждый модуль — один домен. Схемы используются в:
  - FastAPI route handlers (валидация входа, типизированный выход)
  - Авто-генерации OpenAPI на /docs
  - Авто-генерации TypeScript клиента (openapi-typescript-codegen)

Conventions:
  - Все response модели наследуются от common.OkResponse или ErrorResponse
  - snake_case для полей (совместимо с Python), camelCase автоген конвертит для TS
  - Используем Optional[T] вместо T | None для совместимости со старым Python
"""

from .common import OkResponse, ErrorResponse

__all__ = ['OkResponse', 'ErrorResponse']
