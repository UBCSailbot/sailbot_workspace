export type TempSensor = {
  temperature: number;
};

export type TempSensors = {
  tempSensors: TempSensor[];
  timestamp: string;
};

export type TempSensorsState = {
  data: TempSensors[];
  error?: any;
};
