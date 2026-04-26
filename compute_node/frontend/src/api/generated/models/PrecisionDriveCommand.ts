/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * POST /api/precision_drive/command.
 */
export type PrecisionDriveCommand = {
    scenario: 'cross' | 'square' | 'line' | 'zigzag' | 'goto';
    target_x?: (number | null);
    target_y?: (number | null);
    target_theta?: (number | null);
    distance_cm?: (number | null);
};

