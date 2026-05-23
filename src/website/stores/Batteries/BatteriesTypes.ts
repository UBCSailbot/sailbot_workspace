export type Battery = {
  voltage: number;
  current: number;
};

export type Batteries = {
  batteries: Battery[];
  timestamp: number;
};

export type BatteriesState = {
  data: Batteries[];
  error?: any;
};
