/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * POST /api/path_recorder/command.
 */
export type PathRecorderCommand = {
    command: 'record' | 'stop' | 'replay' | 'pause' | 'resume';
    /**
     * Имя пути для load/save (только safe chars)
     */
    name?: (string | null);
};

