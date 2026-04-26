/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
export type DetectionStatusResponse = {
    ok?: boolean;
    enabled: boolean;
    /**
     * yolo | hsv (active backend в detector.py)
     */
    backend?: (string | null);
    fps?: (number | null);
};

