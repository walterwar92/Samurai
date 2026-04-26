/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { ForbiddenZone } from './ForbiddenZone';
/**
 * Ответ POST /zones — созданная зона + ok-флаг.
 */
export type ZoneCreatedResponse = {
    ok?: boolean;
    zone: ForbiddenZone;
};

