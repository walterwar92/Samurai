/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class SamcanService {
    /**
     * Samcan Get
     * @returns any Successful Response
     * @throws ApiError
     */
    public static samcanGet({
        path,
    }: {
        path: string,
    }): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/samcan/{path}',
            path: {
                'path': path,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Samcan Post
     * @returns any Successful Response
     * @throws ApiError
     */
    public static samcanPost({
        path,
    }: {
        path: string,
    }): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/samcan/{path}',
            path: {
                'path': path,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Samcan Put
     * @returns any Successful Response
     * @throws ApiError
     */
    public static samcanPut({
        path,
    }: {
        path: string,
    }): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'PUT',
            url: '/api/v1/samcan/{path}',
            path: {
                'path': path,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Samcan Delete
     * @returns any Successful Response
     * @throws ApiError
     */
    public static samcanDelete({
        path,
    }: {
        path: string,
    }): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'DELETE',
            url: '/api/v1/samcan/{path}',
            path: {
                'path': path,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Samcan Patch
     * @returns any Successful Response
     * @throws ApiError
     */
    public static samcanPatch({
        path,
    }: {
        path: string,
    }): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'PATCH',
            url: '/api/v1/samcan/{path}',
            path: {
                'path': path,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
