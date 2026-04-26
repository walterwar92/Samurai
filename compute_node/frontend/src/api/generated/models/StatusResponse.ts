/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * Универсальная обёртка для GET .../status — отдаёт сырой dict состояния.
 *
 * Используется когда формат статуса от Pi-нод (calibration, mission,
 * explorer, path_recorder, ...) часто меняется и строгая типизация
 * мешает развитию.
 */
export type StatusResponse = {
    ok?: boolean;
    status?: Record<string, any>;
};

