/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { MapInfo } from './MapInfo';
import type { SlamObject } from './SlamObject';
export type SlamMapResponse = {
    /**
     * [[wx, wy], ...] координаты occupied ячеек
     */
    obstacles?: Array<Array<number>>;
    /**
     * [[x, y], ...] последние ~500 точек одометрии
     */
    trail?: Array<Array<number>>;
    /**
     * {x, y, theta}
     */
    robot?: Record<string, any>;
    detected_objects?: Array<SlamObject>;
    info?: (MapInfo | null);
    /**
     * {occupied, free, unknown, detected_objects}
     */
    stats?: Record<string, any>;
    ts?: number;
    ok?: boolean;
};

