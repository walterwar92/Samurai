/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { CommandAck } from '../models/CommandAck';
import type { PoseResponse } from '../models/PoseResponse';
import type { SpeedProfileCommand } from '../models/SpeedProfileCommand';
import type { SpeedProfileResponse } from '../models/SpeedProfileResponse';
import type { VelocityCommand } from '../models/VelocityCommand';
import type { VelocityResponse } from '../models/VelocityResponse';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class RobotService {
    /**
     * Get Pose
     * Текущая поза робота (мировые координаты, м/радианы).
     * @returns PoseResponse Successful Response
     * @throws ApiError
     */
    public static getPoseApiV1RobotPoseGet(): CancelablePromise<PoseResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/robot/pose',
        });
    }
    /**
     * Get Velocity
     * Скорость робота: estimated (одометрия) и commanded (последний cmd_vel).
     * @returns VelocityResponse Successful Response
     * @throws ApiError
     */
    public static getVelocityApiV1RobotVelocityGet(): CancelablePromise<VelocityResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/robot/velocity',
        });
    }
    /**
     * Set Velocity
     * Послать cmd_vel/manual напрямую на Pi (приоритет выше autonomous).
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static setVelocityApiV1RobotVelocityPost({
        requestBody,
    }: {
        requestBody: VelocityCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/robot/velocity',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Stop
     * Немедленный стоп — нулевая скорость напрямую на Pi.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static stopApiV1RobotStopPost(): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/robot/stop',
        });
    }
    /**
     * Reset Position
     * Сбросить одометрию: текущая поза становится (0,0,0) — новый home.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static resetPositionApiV1RobotResetPositionPost(): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/robot/reset_position',
        });
    }
    /**
     * Get Speed Profile
     * @returns SpeedProfileResponse Successful Response
     * @throws ApiError
     */
    public static getSpeedProfileApiV1SpeedProfileGet(): CancelablePromise<SpeedProfileResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/speed_profile',
        });
    }
    /**
     * Set Speed Profile
     * Переключить профиль скорости (slow|normal|fast).
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static setSpeedProfileApiV1SpeedProfilePost({
        requestBody,
    }: {
        requestBody: SpeedProfileCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/speed_profile',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Emergency Stop
     * Alias для POST /robot/stop, путь /api/emergency_stop.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static emergencyStopApiV1EmergencyStopPost(): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/emergency_stop',
        });
    }
}
