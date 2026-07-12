export type SalinitySensor = {
  salinity: number;
};

export type SalinitySensors = {
  salinitySensors: SalinitySensor[];
  timestamp: string;
};

export type SalinitySensorsState = {
  data: SalinitySensors[];
  error?: any;
};
