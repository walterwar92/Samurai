/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * POST /api/actuators/head — заполняем только нужное поле, остальные null.
 *
 * Семантика на Pi (head_node):
 * angle  → выставить угол серво (0..180)
 * center → "command": "center" (вернуть в home)
 * locked → True: "lock" / False: "unlock"
 * frozen → True: "freeze" / False: "unfreeze"
 */
export type HeadCommand = {
    angle?: (number | null);
    center?: boolean;
    locked?: (boolean | null);
    frozen?: (boolean | null);
};

