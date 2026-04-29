/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { ImuYpr } from './ImuYpr';
import type { Vec3 } from './Vec3';
export type ImuResponse = {
    yaw?: number;
    pitch?: number;
    roll?: number;
    gyro?: Vec3;
    accel?: Vec3;
    /**
     * Углы из EKF фильтра (если включен imu.ekf.enabled в config)
     */
    ekf?: (ImuYpr | null);
    ok?: boolean;
};

