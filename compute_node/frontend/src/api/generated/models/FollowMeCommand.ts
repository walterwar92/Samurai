/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * POST /api/follow_me.
 */
export type FollowMeCommand = {
    command: 'start' | 'stop';
    /**
     * Метры
     */
    target_distance?: (number | null);
};

