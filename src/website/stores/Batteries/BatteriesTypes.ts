export type Battery = {
  voltage: number;
  current: number;
};

export type Batteries = {
  batteries: Battery[];
};

export type BatteriesState = {
  data: Batteries;
  error?: any;
};
