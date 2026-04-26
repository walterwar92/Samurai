/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * HC-SR04 показание дистанции.
 */
export type UltrasonicData = {
    /**
     * Дистанция в метрах. -1 = sensor error
     */
    range_m?: number;
    /**
     * Возраст показания (сек) — если > 1.0, считать stale
     */
    age_s?: (number | null);
};

