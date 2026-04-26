/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { ArmState } from './ArmState';
import type { ClawState } from './ClawState';
import type { HeadState } from './HeadState';
export type ActuatorsResponse = {
    claw?: ClawState;
    head?: (HeadState | null);
    arm?: (ArmState | null);
    ok?: boolean;
};

