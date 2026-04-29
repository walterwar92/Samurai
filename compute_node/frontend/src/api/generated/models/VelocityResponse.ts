/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { VelocityDetail } from './VelocityDetail';
/**
 * GET /api/robot/velocity → estimated + commanded блоки.
 */
export type VelocityResponse = {
    ok?: boolean;
    estimated?: VelocityDetail;
    commanded?: VelocityDetail;
};

