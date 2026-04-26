/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * POST /api/hardware/presets — сохранить пресет (имя обязательно).
 */
export type HardwarePresetSaveCommand = {
    name: string;
    /**
     * Произвольная конфигурация для сохранения
     */
    config?: Record<string, any>;
};

