/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { CameraEndpointResponse } from '../models/CameraEndpointResponse';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class CameraService {
    /**
     * Get Camera Endpoint
     * Куда подключаться за H.264 потоком (host:port + codec params).
     *
     * 503 если Pi-side camera_node не подключён.
     * @returns CameraEndpointResponse Successful Response
     * @throws ApiError
     */
    public static getCameraEndpointApiV1CameraEndpointGet(): CancelablePromise<CameraEndpointResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/camera/endpoint',
        });
    }
}
