/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { CommandAck } from '../models/CommandAck';
import type { MapInfoResponse } from '../models/MapInfoResponse';
import type { MapListResponse } from '../models/MapListResponse';
import type { MapLoadCommand } from '../models/MapLoadCommand';
import type { MapSaveCommand } from '../models/MapSaveCommand';
import type { PlannedPathResponse } from '../models/PlannedPathResponse';
import type { SlamMapResponse } from '../models/SlamMapResponse';
import type { ZoneCreateCommand } from '../models/ZoneCreateCommand';
import type { ZoneCreatedResponse } from '../models/ZoneCreatedResponse';
import type { ZonesResponse } from '../models/ZonesResponse';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class MapsService {
    /**
     * Get Map Info
     * @returns MapInfoResponse Successful Response
     * @throws ApiError
     */
    public static getMapInfoApiV1MapInfoGet(): CancelablePromise<MapInfoResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/map/info',
        });
    }
    /**
     * List Maps
     * Список сохранённых SLAM карт из ~/maps*.yaml.
     * @returns MapListResponse Successful Response
     * @throws ApiError
     */
    public static listMapsApiV1MapListGet(): CancelablePromise<MapListResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/map/list',
        });
    }
    /**
     * Get Map Image
     * PNG карты. Приоритет: state.map.png (ROS2 SLAM Toolbox);
     * fallback: рендер из Pi-side SLAM (slam_map_node).
     * @returns any Successful Response
     * @throws ApiError
     */
    public static getMapImageApiV1MapImageGet(): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/map/image',
        });
    }
    /**
     * Save Map
     * Сохранить текущую SLAM карту (ROS2 /map_manager/save). Требует ROS2 stack.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static saveMapApiV1MapSavePost({
        requestBody,
    }: {
        requestBody: MapSaveCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/map/save',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Load Map
     * Загрузить сохранённую карту в SLAM (ROS2 /map_manager/load).
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static loadMapApiV1MapLoadPost({
        requestBody,
    }: {
        requestBody: MapLoadCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/map/load',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Slam Map
     * Pi-side ultrasonic SLAM data (obstacles, trail, robot, objects, info).
     * @returns SlamMapResponse Successful Response
     * @throws ApiError
     */
    public static getSlamMapApiV1SlamMapGet(): CancelablePromise<SlamMapResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/slam_map',
        });
    }
    /**
     * Get Zones
     * @returns ZonesResponse Successful Response
     * @throws ApiError
     */
    public static getZonesApiV1ZonesGet(): CancelablePromise<ZonesResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/zones',
        });
    }
    /**
     * Create Zone
     * Создать запретную зону (нормализуется чтобы x1<x2, y1<y2).
     * @returns ZoneCreatedResponse Successful Response
     * @throws ApiError
     */
    public static createZoneApiV1ZonesPost({
        requestBody,
    }: {
        requestBody: ZoneCreateCommand,
    }): CancelablePromise<ZoneCreatedResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/zones',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Delete Zone
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static deleteZoneApiV1ZonesZoneIdDelete({
        zoneId,
    }: {
        zoneId: number,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'DELETE',
            url: '/api/v1/zones/{zone_id}',
            path: {
                'zone_id': zoneId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Clear Zones
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static clearZonesApiV1ZonesClearPost(): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/zones/clear',
        });
    }
    /**
     * Get Planned Path
     * Текущий планируемый путь (от path_planner или path_recorder).
     *
     * На текущей реализации path_recorder_path = [[x, y], ...] из Pi.
     * @returns PlannedPathResponse Successful Response
     * @throws ApiError
     */
    public static getPlannedPathApiV1PlannedPathGet(): CancelablePromise<PlannedPathResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/planned_path',
        });
    }
}
