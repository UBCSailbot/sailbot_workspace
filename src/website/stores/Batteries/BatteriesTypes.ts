export type Battery = {
  voltage: number;
  current: number;
};

export type Batteries = {
  batteries: Battery[];
  timestamp: string;
};

export type BatteriesState = {
  data: Batteries[];
  error?: any;
};
