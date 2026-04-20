/* Instruments */
import { combineReducers } from 'redux';
import themeReducer from './theme/themeSlice';
import GPSReducer from '@/stores/GPS/GPSReducers';
import GlobalPathReducer from '@/stores/GlobalPath/GlobalPathReducers';
import AISShipsReducer from '@/stores/AISShips/AISShipsReducers';
import LocalPathReducer from '@/stores/LocalPath/LocalPathReducers';
import BatteriesReducer from '@/stores/Batteries/BatteriesReducers';
import WindSensorsReducer from '@/stores/WindSensors/WindSensorsReducers';
import GenericSensorsReducer from '@/stores/GenericSensors/GenericSensorsReducers';
import GraphsReducer from '@/stores/Graphs/GraphsReducers';
import DataFilterReducer from '@/stores/DataFilter/DataFilterReducers';

export function rootReducer() {
  const reducerMap = {
    theme: themeReducer,
    gps: new GPSReducer().reducer,
    aisShips: new AISShipsReducer().reducer,
    localPath: new LocalPathReducer().reducer,
    globalPath: new GlobalPathReducer().reducer,
    batteries: new BatteriesReducer().reducer,
    windSensors: new WindSensorsReducer().reducer,
    genericSensors: new GenericSensorsReducer().reducer,
    graphs: new GraphsReducer().reducer,
    dataFilter: new DataFilterReducer().reducer,
  };

  return combineReducers(reducerMap);
}
