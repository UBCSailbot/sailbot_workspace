import { fork } from 'redux-saga/effects';

export default class BaseSaga {
  /**
   * Checks if a function is a worker saga. For this to be true, the function should end with the name 'Watcher'.
   *
   * Saga functions are separated into two classes: workers and watchers.
   * - Watchers ensure that an action is dispatched to the redux store, if it matches the action they are told to handle.
   * - Workers are assigned to a specific watcher and perform the specific action they are told to do.
   *
   * In our use case, we only want to execute watcher sagas since these functions are responsible for knowing when a specific
   * worker saga should be dispatched.
   *
   * @param {*} fn - name of the function
   * @returns true or false
   */
  static isWatcher(fn) {
    return fn.endsWith('Watcher');
  }

  /**
   * Returns all saga functions to execute.
   *
   * @returns a list of saga functions.
   */
  forkSagas() {
    const sagaFunctions = [];
    const functions = Object.getOwnPropertyNames(Object.getPrototypeOf(this));
    functions.forEach((func) => {
      if (func !== 'constructor') {
        sagaFunctions.push(fork([this, this[func]]));
      }
    });
    return sagaFunctions;
  }
}
