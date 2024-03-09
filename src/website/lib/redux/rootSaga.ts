import { ForkEffect, all } from 'redux-saga/effects';
import AISShipsSagas from '@/stores/AISShips/AISShipsSagas';
import GPSSagas from '@/stores/GPS/GPSSagas';
import LocalPathSagas from '@/stores/LocalPath/LocalPathSagas';
import GlobalPathSagas from '@/stores/GlobalPath/GlobalPathSagas';
import BatteriesSagas from '@/stores/Batteries/BatteriesSagas';
import WindSensorsSagas from '@/stores/WindSensors/WindSensorsSagas';
import GenericSensorsSagas from '@/stores/GenericSensors/GenericSensorsSagas';

export function* rootSaga() {
  const rootSagaMap = {
    gps: new GPSSagas().forkSagas(),
    aisShips: new AISShipsSagas().forkSagas(),
    localPath: new LocalPathSagas().forkSagas(),
    globalPath: new GlobalPathSagas().forkSagas(),
    batteries: new BatteriesSagas().forkSagas(),
    windSensors: new WindSensorsSagas().forkSagas(),
    genericSensors: new GenericSensorsSagas().forkSagas(),
  };

  yield all(combineSagas(rootSagaMap));
}

function combineSagas(sagaMap: { [s: string]: ForkEffect[] }) {
  return Object.values(sagaMap).reduce((acc, arr) => acc.concat(arr), []);
}
