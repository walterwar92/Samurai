/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * POST /api/explorer/command.
 */
export type ExplorerCommand = {
    command: 'start' | 'stop';
    strategy?: ('frontier' | 'spiral' | 'zigzag' | null);
};

