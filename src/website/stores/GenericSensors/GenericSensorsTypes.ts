export type GenericSensor = {
  id: number;
  data: bigint;
};

export type GenericSensors = {
  genericSensors: GenericSensor[];
  timestamp: string;
};

export type GenericSensorsState = {
  data: GenericSensors;
  error?: any;
};
