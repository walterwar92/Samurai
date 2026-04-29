/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * POST /api/fsm/transition — принудительный переход (admin).
 */
export type FsmTransitionCommand = {
    state: 'IDLE' | 'SEARCHING' | 'TARGETING' | 'APPROACHING' | 'GRABBING' | 'CALLING' | 'RETURNING' | 'PATROLLING' | 'FOLLOWING' | 'PATH_REPLAY';
};

