/* Instruments */
import { combineReducers } from 'redux';
import GPSReducer from '@/stores/GPS/GPSReducers';
import GlobalPathReducer from '@/stores/GlobalPath/GlobalPathReducers';
import AISShipsReducer from '@/stores/AISShips/AISShipsReducers';
import LocalPathReducer from '@/stores/LocalPath/LocalPathReducers';
import BatteriesReducer from '@/stores/Batteries/BatteriesReducers';
import WindSensorsReducer from '@/stores/WindSensors/WindSensorsReducers';
import GenericSensorsReducer from '@/stores/GenericSensors/GenericSensorsReducers';
import DataFilterReducer from '@/stores/DataFilter/DataFilterReducers';

export function rootReducer() {
  const reducerMap = {
    gps: new GPSReducer().reducer,
    aisShips: new AISShipsReducer().reducer,
    localPath: new LocalPathReducer().reducer,
    globalPath: new GlobalPathReducer().reducer,
    batteries: new BatteriesReducer().reducer,
    windSensors: new WindSensorsReducer().reducer,
    genericSensors: new GenericSensorsReducer().reducer,
    dataFilter: new DataFilterReducer().reducer,
  };

  return combineReducers(reducerMap);
}
