/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * Статус последнего запроса к планировщику.
 */
export type PathPlannerStatusResponse = {
    ok?: boolean;
    /**
     * idle|success|failed|error
     */
    state?: string;
    message?: (string | null);
    planning_ms?: (number | null);
};

