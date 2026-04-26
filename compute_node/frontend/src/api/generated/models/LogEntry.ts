/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * Одна запись event/voice лога.
 */
export type LogEntry = {
    ts?: (number | null);
    /**
     * HH:MM:SS
     */
    time?: (string | null);
    /**
     * voice|api_command|system
     */
    type?: (string | null);
    source?: (string | null);
    level?: (string | null);
    text?: string;
};

