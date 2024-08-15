import axios, { AxiosRequestConfig } from 'axios';
import { assert } from 'chai';
import { SERVICE_URL } from '../config';
import { logger } from '../utils';

export class Api {
  public response: any;

  public error: any;

  private axiosInstance = axios.create({
    baseURL: SERVICE_URL,
  });

  get = async (
    endpoint: string,
    config: AxiosRequestConfig,
    failOnError = true,
  ): Promise<object> => {
    this.logRequest({}, config);
    try {
      this.response = await this.axiosInstance.get(endpoint, config);
      this.logResponse();
      return this.response.data;
    } catch (error) {
      this.logError(error, failOnError);
      return null as any;
    }
  };

  post = async (
    endpoint: string,
    requestObject: object,
    config: AxiosRequestConfig,
    failOnError = true,
  ): Promise<object> => {
    this.logRequest(requestObject, config);
    try {
      this.response = await this.axiosInstance.post(
        endpoint,
        requestObject,
        config,
      );
      this.logResponse();
      return this.response.data;
    } catch (error) {
      this.logError(error, failOnError);
      return null as any;
    }
  };

  put = async (
    endpoint: string,
    requestObject: object,
    config: AxiosRequestConfig,
    failOnError = true,
  ): Promise<object> => {
    this.logRequest(requestObject, config);
    try {
      this.response = await this.axiosInstance.put(
        endpoint,
        requestObject,
        config,
      );
      this.logResponse();
      return this.response.data;
    } catch (error) {
      this.logError(error, failOnError);
      return null as any;
    }
  };

  delete = async (
    endpoint: string,
    config: AxiosRequestConfig,
    failOnError = true,
  ): Promise<object> => {
    this.logRequest({}, config);
    try {
      this.response = await this.axiosInstance.delete(endpoint, config);
      this.logResponse();
      return this.response;
    } catch (error) {
      this.logError(error, failOnError);
      return null as any;
    }
  };

  get responseData(): object {
    return this.response.data;
  }

  logResponse = (): void => {
    logger.info(this.response.config);
    logger.info(`status code: ${this.response.status}`);
    logger.info(this.response.data);
  };

  /* eslint-disable class-methods-use-this */
  logRequest = (request: any, config: any): void => {
    logger.info(request);
    logger.info(config);
  };

  logError = (error: any, failOnError = true): void => {
    this.error = error;
    const errorObjStr = JSON.stringify(error);
    const errorData = error.response.data;

    if (failOnError) {
      logger.error(errorObjStr);
      logger.error(errorData);
      assert.fail(JSON.stringify(errorData));
    } else {
      logger.info(errorObjStr);
      logger.info(errorData);
    }
  };
}

export const api = new Api();
