/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * Состояние 4-DOF руки. j1..j4 = углы суставов в градусах.
 */
export type ArmState = {
    j1?: number;
    j2?: number;
    j3?: number;
    j4?: number;
    frozen?: Array<boolean>;
    locked?: boolean;
};

