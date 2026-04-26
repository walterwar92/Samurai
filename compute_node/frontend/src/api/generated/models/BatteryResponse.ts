/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
export type BatteryResponse = {
    /**
     * Напряжение в вольтах. -1 = нет данных
     */
    voltage?: number;
    /**
     * 0..100, -1 = нет данных
     */
    percent?: number;
    /**
     * Опциональный статус: ok / low / critical
     */
    status?: (string | null);
    ok?: boolean;
};

