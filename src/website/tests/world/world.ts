import { setWorldConstructor } from '@cucumber/cucumber';
import { api } from '../shared/classes/api';

export default class World {
  public config: object = {
    headers: {},
    params: {},
  };

  constructor() {
    api.response = undefined;
    api.error = undefined;
  }
}

setWorldConstructor(World);
