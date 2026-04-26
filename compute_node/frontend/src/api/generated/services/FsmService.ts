/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { CommandAck } from '../models/CommandAck';
import type { FsmCommand } from '../models/FsmCommand';
import type { FsmStateResponse } from '../models/FsmStateResponse';
import type { FsmTransitionCommand } from '../models/FsmTransitionCommand';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class FsmService {
    /**
     * Get Fsm
     * Текущее состояние FSM на Pi (state, target_colour, target_action).
     * @returns FsmStateResponse Successful Response
     * @throws ApiError
     */
    public static getFsmApiV1FsmGet(): CancelablePromise<FsmStateResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/fsm',
        });
    }
    /**
     * Send Fsm Command
     * Послать текстовую команду на Pi (через voice_command топик).
     *
     * Pi-side voice_listener распарсит как обычное голосовое выражение.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static sendFsmCommandApiV1FsmCommandPost({
        requestBody,
    }: {
        requestBody: FsmCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/fsm/command',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Force Fsm Transition
     * Admin override: принудительно переключить FSM в указанное состояние.
     * @returns CommandAck Successful Response
     * @throws ApiError
     */
    public static forceFsmTransitionApiV1FsmTransitionPost({
        requestBody,
    }: {
        requestBody: FsmTransitionCommand,
    }): CancelablePromise<CommandAck> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/fsm/transition',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
