import { ResponseErrorObject } from '../table-objects/table-objects';

export default class ResponseError {
  public detail: string | undefined;

  public title: string | undefined;

  public status: string | undefined;

  /**
   * Error response must include all the following properties.
   *
   * @param errObj ResponseErrorObject
   */
  constructor(errObj: ResponseErrorObject) {
    this.detail = errObj.detail;
    this.title = errObj.title;
    this.status = errObj.status;
  }
}
