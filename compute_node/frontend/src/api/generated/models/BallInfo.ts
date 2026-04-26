/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * Tracked ball на арене (постоянный ID между кадрами).
 */
export type BallInfo = {
    id: number;
    colour?: 'red' | 'orange' | 'yellow' | 'green' | 'blue' | 'white' | 'black' | 'unknown';
    'x': number;
    'y': number;
    grabbed?: boolean;
};

