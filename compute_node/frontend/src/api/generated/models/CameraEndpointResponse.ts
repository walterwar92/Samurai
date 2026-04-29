/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * H.264 discovery: host/port/codec параметры для VideoDecoder в браузере.
 */
export type CameraEndpointResponse = {
    ok?: boolean;
    host: string;
    port?: number;
    codec?: string;
    format?: string;
    width?: number;
    height?: number;
    fps?: number;
    extra?: Record<string, any>;
};

