/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * Один объект из детектора (compute_node/detector.py).
 */
export type Detection = {
    /**
     * YOLO class name (sports ball, person, ...)
     */
    class?: string;
    colour?: 'red' | 'orange' | 'yellow' | 'green' | 'blue' | 'white' | 'black' | 'unknown';
    'x'?: number;
    'y'?: number;
    'w'?: number;
    'h'?: number;
    conf?: number;
    /**
     * Метры. -1 = bbox слишком мал
     */
    distance?: number;
    dist_method?: 'mono' | 'ultra' | 'blend' | 'none';
    /**
     * Мировая X координата (м), если поза робота известна
     */
    world_x?: (number | null);
    world_y?: (number | null);
};

