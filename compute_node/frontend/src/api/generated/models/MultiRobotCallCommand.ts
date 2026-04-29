/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * POST /api/multi_robot/call — вызвать второго робота.
 */
export type MultiRobotCallCommand = {
    target_id: string;
    colour?: (string | null);
    action?: 'grab' | 'burn';
};

