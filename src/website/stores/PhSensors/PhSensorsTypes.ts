export type PhSensor = {
  ph: number;
};

export type PhSensors = {
  phSensors: PhSensor[];
  timestamp: string;
};

export type PhSensorsState = {
  data: PhSensors[];
  error?: any;
};
