/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { CalibrationCoefficientsResponse } from '../models/CalibrationCoefficientsResponse';
import type { CalibrationCommand } from '../models/CalibrationCommand';
import type { CalibrationProfileDeleteCommand } from '../models/CalibrationProfileDeleteCommand';
import type { CalibrationProfileListResponse } from '../models/CalibrationProfileListResponse';
import type { CalibrationProfileLoadCommand } from '../models/CalibrationProfileLoadCommand';
import type { CalibrationProfileSaveCommand } from '../models/CalibrationProfileSaveCommand';
import type { CalibrationSetCommand } from '../models/CalibrationSetCommand';
import type { CommandAck } from '../models/CommandAck';
import type { ExplorerCommand } from '../models/ExplorerCommand';
import type { FollowMeCommand } from '../models/FollowMeCommand';
import type { MissionCommand } from '../models/MissionCommand';
import type { MissionListResponse } from '../models/MissionListResponse';
import type { PathListResponse } from '../models/PathListResponse';
import type { PathPlannerGoalCommand } from '../models/PathPlannerGoalCommand';
import type { PathPlannerPathResponse } from '../models/PathPlannerPathResponse';
import type { PathPlannerStatusResponse } from '../models/PathPlannerStatusResponse';
import type { PathRecorderCommand } from '../models/PathRecorderCommand';
import type { PathRecorderPathResponse } from '../models/PathRecorderPathResponse';
import type { PatrolCommand } from '../models/PatrolCommand';
import type { PatrolWaypointsCommand } from '../models/PatrolWaypointsCommand';
import type { PrecisionDriveCommand } from '../models/PrecisionDriveCommand';
import type { StatusResponse } from '../models/StatusResponse';
import type { ToggleCommand } from '../models/ToggleCommand';
import type { TTSSpeakCommand } from '../models/TTSSpeakCommand';
import type { TTSToggleCommand } from '../models/TTSToggleCommand';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class ControlService {
    /**
     * Patrol Command
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static patrolCommandApiV1PatrolCommandPost({
        requestBody,
    }: {
        requestBody: PatrolCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/patrol/command',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Patrol Waypoints
     * Установить список waypoints для patrol-маршрута.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static patrolWaypointsApiV1PatrolWaypointsPost({
        requestBody,
    }: {
        requestBody: PatrolWaypointsCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/patrol/waypoints',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Follow Me
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static followMeApiV1FollowMePost({
        requestBody,
    }: {
        requestBody: FollowMeCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/follow_me',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Path Recorder Command
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static pathRecorderCommandApiV1PathRecorderCommandPost({
        requestBody,
    }: {
        requestBody: PathRecorderCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/path_recorder/command',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Path Recorder Status
     * @returns StatusResponse Successful Response
     * @throws ApiError
     */
    public static pathRecorderStatusApiV1PathRecorderStatusGet(): CancelablePromise<StatusResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/path_recorder/status',
        });
    }
    /**
     * Path Recorder Path
     * @returns PathRecorderPathResponse Successful Response
     * @throws ApiError
     */
    public static pathRecorderPathApiV1PathRecorderPathGet(): CancelablePromise<PathRecorderPathResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/path_recorder/path',
        });
    }
    /**
     * Path Recorder List
     * @returns PathListResponse Successful Response
     * @throws ApiError
     */
    public static pathRecorderListApiV1PathRecorderListGet(): CancelablePromise<PathListResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/path_recorder/list',
        });
    }
    /**
     * Precision Drive Command
     * Сценарии точного драйвинга (cross/square/line/zigzag/goto).
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static precisionDriveCommandApiV1PrecisionDriveCommandPost({
        requestBody,
    }: {
        requestBody: PrecisionDriveCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/precision_drive/command',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Precision Drive Status
     * @returns StatusResponse Successful Response
     * @throws ApiError
     */
    public static precisionDriveStatusApiV1PrecisionDriveStatusGet(): CancelablePromise<StatusResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/precision_drive/status',
        });
    }
    /**
     * Calibration Command
     * Admin: start/stop/reset калибровки.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static calibrationCommandApiV1CalibrationCommandPost({
        requestBody,
    }: {
        requestBody: CalibrationCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/calibration/command',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Calibration Set
     * Установить scale_fwd/bwd/motor_trim напрямую.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static calibrationSetApiV1CalibrationSetPost({
        requestBody,
    }: {
        requestBody: CalibrationSetCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/calibration/set',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Calibration Profile Load
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static calibrationProfileLoadApiV1CalibrationProfileLoadPost({
        requestBody,
    }: {
        requestBody: CalibrationProfileLoadCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/calibration/profile/load',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Calibration Profile Save
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static calibrationProfileSaveApiV1CalibrationProfileSavePost({
        requestBody,
    }: {
        requestBody: CalibrationProfileSaveCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/calibration/profile/save',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Calibration Profile Delete
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static calibrationProfileDeleteApiV1CalibrationProfileDeletePost({
        requestBody,
    }: {
        requestBody: CalibrationProfileDeleteCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/calibration/profile/delete',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Calibration Profile List
     * Запросить актуальный список (Pi пушит через calibration/profile/all)
     * и одновременно вернуть текущий кэш из state.
     * @returns CalibrationProfileListResponse Successful Response
     * @throws ApiError
     */
    public static calibrationProfileListApiV1CalibrationProfileListGet(): CancelablePromise<CalibrationProfileListResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/calibration/profile/list',
        });
    }
    /**
     * Calibration Coefficients
     * @returns CalibrationCoefficientsResponse Successful Response
     * @throws ApiError
     */
    public static calibrationCoefficientsApiV1CalibrationCoefficientsGet(): CancelablePromise<CalibrationCoefficientsResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/calibration/coefficients',
        });
    }
    /**
     * Path Planner Goto
     * Запросить планирование A* до точки (x, y) в мировых координатах.
     *
     * Path planner живёт на ноутбуке (compute_node/path_planner) и публикует
     * результат в samurai/{robot_id}/path_planner/path. Этот endpoint
     * отправляет goal — нода-планировщик асинхронно посчитает путь.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static pathPlannerGotoApiV1PathPlannerGotoPost({
        requestBody,
    }: {
        requestBody: PathPlannerGoalCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/path_planner/goto',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Path Planner Path
     * @returns PathPlannerPathResponse Successful Response
     * @throws ApiError
     */
    public static pathPlannerPathApiV1PathPlannerPathGet(): CancelablePromise<PathPlannerPathResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/path_planner/path',
        });
    }
    /**
     * Path Planner Status
     * @returns PathPlannerStatusResponse Successful Response
     * @throws ApiError
     */
    public static pathPlannerStatusApiV1PathPlannerStatusGet(): CancelablePromise<PathPlannerStatusResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/path_planner/status',
        });
    }
    /**
     * Mission Command
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static missionCommandApiV1MissionCommandPost({
        requestBody,
    }: {
        requestBody: MissionCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/mission/command',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Mission List
     * @returns MissionListResponse Successful Response
     * @throws ApiError
     */
    public static missionListApiV1MissionListGet(): CancelablePromise<MissionListResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/mission/list',
        });
    }
    /**
     * Explorer Command
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static explorerCommandApiV1ExplorerCommandPost({
        requestBody,
    }: {
        requestBody: ExplorerCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/explorer/command',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Tts Toggle
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static ttsToggleApiV1TtsTogglePost({
        requestBody,
    }: {
        requestBody: TTSToggleCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/tts/toggle',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Tts Speak
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static ttsSpeakApiV1TtsSpeakPost({
        requestBody,
    }: {
        requestBody: TTSSpeakCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/tts/speak',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Obstacle Avoidance Toggle
     * Включить/выключить obstacle avoidance во время path replay.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static obstacleAvoidanceToggleApiV1ObstacleAvoidanceTogglePost({
        requestBody,
    }: {
        requestBody: ToggleCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/obstacle_avoidance/toggle',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Collision Guard Toggle
     * Включить/выключить collision guard для manual control.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static collisionGuardToggleApiV1CollisionGuardTogglePost({
        requestBody,
    }: {
        requestBody: ToggleCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/collision_guard/toggle',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
