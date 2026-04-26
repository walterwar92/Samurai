/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { CommandAck } from '../models/CommandAck';
import type { HardwareActiveResponse } from '../models/HardwareActiveResponse';
import type { HardwareApplyCommand } from '../models/HardwareApplyCommand';
import type { HardwarePresetListResponse } from '../models/HardwarePresetListResponse';
import type { HardwarePresetResponse } from '../models/HardwarePresetResponse';
import type { HardwarePresetSaveCommand } from '../models/HardwarePresetSaveCommand';
import type { LogResponse } from '../models/LogResponse';
import type { MqttStatusResponse } from '../models/MqttStatusResponse';
import type { MultiRobotCallCommand } from '../models/MultiRobotCallCommand';
import type { MultiRobotListResponseSystem } from '../models/MultiRobotListResponseSystem';
import type { StatusSnapshotResponse } from '../models/StatusSnapshotResponse';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class SystemService {
    /**
     * Get Status
     * Атомарный snapshot всего state — для polling-fallback на фронте.
     * @returns StatusSnapshotResponse Successful Response
     * @throws ApiError
     */
    public static getStatusApiV1StatusGet(): CancelablePromise<StatusSnapshotResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/status',
        });
    }
    /**
     * Get Log
     * Последние event/voice записи (max 200).
     * @returns LogResponse Successful Response
     * @throws ApiError
     */
    public static getLogApiV1LogGet({
        limit = 50,
    }: {
        limit?: number,
    }): CancelablePromise<LogResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/log',
            query: {
                'limit': limit,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Mqtt Status
     * @returns MqttStatusResponse Successful Response
     * @throws ApiError
     */
    public static getMqttStatusApiV1MqttStatusGet(): CancelablePromise<MqttStatusResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/mqtt/status',
        });
    }
    /**
     * Multi Robot List
     * Stub: возвращает только текущего робота. Multi-prefix MQTT TODO.
     * @returns MultiRobotListResponseSystem Successful Response
     * @throws ApiError
     */
    public static multiRobotListApiV1MultiRobotListGet(): CancelablePromise<MultiRobotListResponseSystem> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/multi_robot/list',
        });
    }
    /**
     * Multi Robot Call
     * Послать другому роботу call (для координации по mesh).
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static multiRobotCallApiV1MultiRobotCallPost({
        requestBody,
    }: {
        requestBody: MultiRobotCallCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/multi_robot/call',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Hardware Presets List
     * @returns HardwarePresetListResponse Successful Response
     * @throws ApiError
     */
    public static hardwarePresetsListApiV1HardwarePresetsGet(): CancelablePromise<HardwarePresetListResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/hardware/presets',
        });
    }
    /**
     * Hardware Preset Save
     * @returns HardwarePresetResponse Successful Response
     * @throws ApiError
     */
    public static hardwarePresetSaveApiV1HardwarePresetsPost({
        requestBody,
    }: {
        requestBody: HardwarePresetSaveCommand,
    }): CancelablePromise<HardwarePresetResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/hardware/presets',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Hardware Preset Get
     * @returns HardwarePresetResponse Successful Response
     * @throws ApiError
     */
    public static hardwarePresetGetApiV1HardwarePresetsNameGet({
        name,
    }: {
        name: string,
    }): CancelablePromise<HardwarePresetResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/hardware/presets/{name}',
            path: {
                'name': name,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Hardware Preset Delete
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static hardwarePresetDeleteApiV1HardwarePresetsNameDelete({
        name,
    }: {
        name: string,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'DELETE',
            url: '/api/v1/hardware/presets/{name}',
            path: {
                'name': name,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Hardware Preset Apply
     * Активировать пресет: запись active + push в MQTT для пере-конфигурации Pi.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static hardwarePresetApplyApiV1HardwareApplyPost({
        requestBody,
    }: {
        requestBody: HardwareApplyCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/hardware/apply',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Hardware Active
     * @returns HardwareActiveResponse Successful Response
     * @throws ApiError
     */
    public static hardwareActiveApiV1HardwareActiveGet(): CancelablePromise<HardwareActiveResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/hardware/active',
        });
    }
}
