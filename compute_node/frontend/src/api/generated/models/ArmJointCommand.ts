/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * POST /api/actuators/arm — установка суставов или admin-команды.
 *
 * j1..j4 → одиночные углы (любая комбинация). joints → весь массив сразу.
 * home/freeze/unfreeze/preset — admin команды на Pi (arm_node).
 */
export type ArmJointCommand = {
    j1?: (number | null);
    j2?: (number | null);
    j3?: (number | null);
    j4?: (number | null);
    /**
     * Все 4 угла одним массивом (альтернатива j1..j4)
     */
    joints?: (Array<number> | null);
    home?: boolean;
    /**
     * True → "command": "freeze". False → "unfreeze".
     */
    freeze?: (boolean | null);
    /**
     * Индекс сустава для freeze/unfreeze (1..4)
     */
    joint_index?: (number | null);
    /**
     * Имя пресета — load_preset
     */
    preset?: (string | null);
};

