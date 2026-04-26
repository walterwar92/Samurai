/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * POST /api/robot/velocity body.
 */
export type VelocityCommand = {
    /**
     * Линейная скорость м/с (X в локальной СК)
     */
    linear?: number;
    /**
     * Угловая скорость рад/с
     */
    angular?: number;
};

