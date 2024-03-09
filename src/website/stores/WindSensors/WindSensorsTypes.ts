export type WindSensor = {
  speed: number;
  direction: number;
};

export type WindSensors = {
  windSensors: WindSensor[];
};

export type WindSensorsState = {
  data: WindSensors;
  error?: any;
};
