export type WindSensor = {
  speed: number;
  direction: number;
};

export type WindSensors = {
  windSensors: WindSensor[];
  timestamp: string;
};

export type WindSensorsState = {
  data: WindSensors[];
  error?: any;
};
