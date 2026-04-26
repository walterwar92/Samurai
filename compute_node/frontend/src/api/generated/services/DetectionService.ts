/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BallsResponse } from '../models/BallsResponse';
import type { ClosestDetectionResponse } from '../models/ClosestDetectionResponse';
import type { CommandAck } from '../models/CommandAck';
import type { DetectionResponse } from '../models/DetectionResponse';
import type { DetectionStatusResponse } from '../models/DetectionStatusResponse';
import type { DetectionToggleCommand } from '../models/DetectionToggleCommand';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class DetectionService {
    /**
     * Get Detection
     * Все объекты последнего YOLO кадра.
     * @returns DetectionResponse Successful Response
     * @throws ApiError
     */
    public static getDetectionApiV1DetectionGet(): CancelablePromise<DetectionResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/detection',
        });
    }
    /**
     * Get Closest
     * Ближайший объект, опционально с фильтром по цвету.
     *
     * Сортировка: по distance (если задана), иначе по площади bbox (больше=ближе).
     * @returns ClosestDetectionResponse Successful Response
     * @throws ApiError
     */
    public static getClosestApiV1DetectionClosestGet({
        color,
    }: {
        /**
         * Фильтр по цвету (red, blue, ...)
         */
        color?: (string | null),
    }): CancelablePromise<ClosestDetectionResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/detection/closest',
            query: {
                'color': color,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Detection Status
     * @returns DetectionStatusResponse Successful Response
     * @throws ApiError
     */
    public static detectionStatusApiV1DetectionStatusGet(): CancelablePromise<DetectionStatusResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/detection/status',
        });
    }
    /**
     * Toggle Detection
     * Включить/выключить YOLO. Публикуем в MQTT и (если ROS2 доступен) в /yolo/enable.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static toggleDetectionApiV1DetectionTogglePost({
        requestBody,
    }: {
        requestBody: DetectionToggleCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/detection/toggle',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Balls
     * Tracked balls (постоянный ID между кадрами).
     *
     * Симулятор/трекер заполняет state.detection.balls. На реальном роботе
     * обычно пусто (трекинг не реализован) — возвращаем то, что есть.
     * @returns BallsResponse Successful Response
     * @throws ApiError
     */
    public static getBallsApiV1BallsGet(): CancelablePromise<BallsResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/balls',
        });
    }
}
