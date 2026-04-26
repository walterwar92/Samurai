/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { ActuatorsResponse } from '../models/ActuatorsResponse';
import type { ArmJointCommand } from '../models/ArmJointCommand';
import type { ArmResponse } from '../models/ArmResponse';
import type { ClawCommand } from '../models/ClawCommand';
import type { ClawResponse } from '../models/ClawResponse';
import type { CommandAck } from '../models/CommandAck';
import type { HeadCommand } from '../models/HeadCommand';
import type { HeadResponse } from '../models/HeadResponse';
import type { LedCommand } from '../models/LedCommand';
import type { PresetListResponse } from '../models/PresetListResponse';
import type { PresetLoadCommand } from '../models/PresetLoadCommand';
import type { PresetSaveCommand } from '../models/PresetSaveCommand';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class ActuatorsService {
    /**
     * Get Actuators
     * @returns ActuatorsResponse Successful Response
     * @throws ApiError
     */
    public static getActuatorsApiV1ActuatorsGet(): CancelablePromise<ActuatorsResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/actuators',
        });
    }
    /**
     * Get Claw
     * @returns ClawResponse Successful Response
     * @throws ApiError
     */
    public static getClawApiV1ActuatorsClawGet(): CancelablePromise<ClawResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/actuators/claw',
        });
    }
    /**
     * Set Claw
     * Клешня = arm joint 4 (1-indexed). open=0°, close=180°.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static setClawApiV1ActuatorsClawPost({
        requestBody,
    }: {
        requestBody: ClawCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/actuators/claw',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Head
     * @returns HeadResponse Successful Response
     * @throws ApiError
     */
    public static getHeadApiV1ActuatorsHeadGet(): CancelablePromise<HeadResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/actuators/head',
        });
    }
    /**
     * Set Head
     * Один POST → одна команда. Если задано несколько полей — порядок:
     * angle → center → locked → frozen (последняя команда побеждает).
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static setHeadApiV1ActuatorsHeadPost({
        requestBody,
    }: {
        requestBody: HeadCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/actuators/head',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * List Head Presets
     * Cached список presets из state. Параллельно публикуем list_presets чтобы
     * head_node освежил кэш через head/presets retained.
     * @returns PresetListResponse Successful Response
     * @throws ApiError
     */
    public static listHeadPresetsApiV1ActuatorsHeadPresetsGet(): CancelablePromise<PresetListResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/actuators/head/presets',
        });
    }
    /**
     * Save Head Preset
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static saveHeadPresetApiV1ActuatorsHeadPresetSavePost({
        requestBody,
    }: {
        requestBody: PresetSaveCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/actuators/head/preset/save',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Load Head Preset
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static loadHeadPresetApiV1ActuatorsHeadPresetLoadPost({
        requestBody,
    }: {
        requestBody: PresetLoadCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/actuators/head/preset/load',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Delete Head Preset
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static deleteHeadPresetApiV1ActuatorsHeadPresetNameDelete({
        name,
    }: {
        name: string,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'DELETE',
            url: '/api/v1/actuators/head/preset/{name}',
            path: {
                'name': name,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Arm
     * @returns ArmResponse Successful Response
     * @throws ApiError
     */
    public static getArmApiV1ActuatorsArmGet(): CancelablePromise<ArmResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/actuators/arm',
        });
    }
    /**
     * Set Arm
     * Поддерживаемые комбинации (порядок проверки):
     * home=true                 → {"command": "home"}
     * preset='X'                → {"command": "load_preset", "name": "X"}
     * freeze=true/false         → {"command": "freeze"|"unfreeze", joint?: N}
     * joints=[..,..,..,..]      → {"joints": [...]}
     * jN установлен             → {"joint": N, "angle": jN}
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static setArmApiV1ActuatorsArmPost({
        requestBody,
    }: {
        requestBody: ArmJointCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/actuators/arm',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * List Arm Presets
     * @returns PresetListResponse Successful Response
     * @throws ApiError
     */
    public static listArmPresetsApiV1ActuatorsArmPresetsGet(): CancelablePromise<PresetListResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/actuators/arm/presets',
        });
    }
    /**
     * Save Arm Preset
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static saveArmPresetApiV1ActuatorsArmPresetSavePost({
        requestBody,
    }: {
        requestBody: PresetSaveCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/actuators/arm/preset/save',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Load Arm Preset
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static loadArmPresetApiV1ActuatorsArmPresetLoadPost({
        requestBody,
    }: {
        requestBody: PresetLoadCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/actuators/arm/preset/load',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Delete Arm Preset
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static deleteArmPresetApiV1ActuatorsArmPresetNameDelete({
        name,
    }: {
        name: string,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'DELETE',
            url: '/api/v1/actuators/arm/preset/{name}',
            path: {
                'name': name,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Led Command
     * LED панель WS2812B. Pi-side led_node ждёт {mode, color?, brightness?}.
     *
     * Маппим animation→mode для backward-compat с led_node.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static ledCommandApiV1LedCommandPost({
        requestBody,
    }: {
        requestBody: LedCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/led/command',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
