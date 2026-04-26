/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class DefaultService {
    /**
     * Serve Root
     * @returns any Successful Response
     * @throws ApiError
     */
    public static serveRootGet(): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/',
        });
    }
    /**
     *  Serve Spa
     * index.html ссылается на хешированные assets — не кешируем HTML.
     * @returns any Successful Response
     * @throws ApiError
     */
    public static serveSpaDashboardGet(): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/dashboard',
        });
    }
    /**
     *  Serve Spa
     * index.html ссылается на хешированные assets — не кешируем HTML.
     * @returns any Successful Response
     * @throws ApiError
     */
    public static serveSpaAdminGet(): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/admin',
        });
    }
    /**
     *  Serve Spa
     * index.html ссылается на хешированные assets — не кешируем HTML.
     * @returns any Successful Response
     * @throws ApiError
     */
    public static serveSpa3DGet(): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/3d',
        });
    }
}
