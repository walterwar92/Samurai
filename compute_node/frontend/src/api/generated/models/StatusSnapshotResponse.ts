/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * GET /api/status — атомарный snapshot всего state.
 *
 * Структура динамическая (см. DashboardState.snapshot()), фронт берёт
 * нужные поля. Не типизируем строго — снапшот часто меняется.
 */
export type StatusSnapshotResponse = {
    ok?: boolean;
    sim_time?: number;
    pose?: Record<string, any>;
    velocity?: Record<string, any>;
    robot_status?: Record<string, any>;
    speed_profile?: string;
    sensors?: Record<string, any>;
    battery?: Record<string, any>;
    temperature?: Record<string, any>;
    watchdog?: Record<string, any>;
    actuators?: Record<string, any>;
    detection?: Record<string, any>;
    map?: Record<string, any>;
    control?: Record<string, any>;
    camera?: Record<string, any>;
    system?: Record<string, any>;
};

