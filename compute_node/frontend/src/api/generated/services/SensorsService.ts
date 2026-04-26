/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BatteryResponse } from '../models/BatteryResponse';
import type { ImuResponse } from '../models/ImuResponse';
import type { SensorsResponse } from '../models/SensorsResponse';
import type { TemperatureResponse } from '../models/TemperatureResponse';
import type { UltrasonicResponse } from '../models/UltrasonicResponse';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class SensorsService {
    /**
     * Get Sensors
     * Bundle: ультразвук + IMU. Для batched-запросов с дашборда.
     * @returns SensorsResponse Successful Response
     * @throws ApiError
     */
    public static getSensorsApiV1SensorsGet(): CancelablePromise<SensorsResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/sensors',
        });
    }
    /**
     * Get Ultrasonic
     * @returns UltrasonicResponse Successful Response
     * @throws ApiError
     */
    public static getUltrasonicApiV1SensorsUltrasonicGet(): CancelablePromise<UltrasonicResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/sensors/ultrasonic',
        });
    }
    /**
     * Get Imu
     * @returns ImuResponse Successful Response
     * @throws ApiError
     */
    public static getImuApiV1SensorsImuGet(): CancelablePromise<ImuResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/sensors/imu',
        });
    }
    /**
     * Get Battery
     * Voltage, percent, status (ok/low/critical).
     * @returns BatteryResponse Successful Response
     * @throws ApiError
     */
    public static getBatteryApiV1BatteryGet(): CancelablePromise<BatteryResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/battery',
        });
    }
    /**
     * Get Temperature
     * CPU температура Pi.
     * @returns TemperatureResponse Successful Response
     * @throws ApiError
     */
    public static getTemperatureApiV1TemperatureGet(): CancelablePromise<TemperatureResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/temperature',
        });
    }
}
