export type GenericSensor = {
  id: number;
  data: bigint;
};

export type GenericSensors = {
  genericSensors: GenericSensor[];
};

export type GenericSensorsState = {
  data: GenericSensors;
  error?: any;
};
