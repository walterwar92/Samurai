/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * POST /api/actuators/claw.
 */
export type ClawCommand = {
    state?: ('open' | 'close' | null);
    /**
     * Прямой угол (если state не задан)
     */
    angle?: (number | null);
};

