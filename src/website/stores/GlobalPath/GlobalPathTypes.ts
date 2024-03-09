export type WayPoint = {
  latitude: number;
  longitude: number;
};

export type GlobalPath = {
  waypoints: WayPoint[];
};

export type GlobalPathState = {
  data: GlobalPath;
  error?: any;
};
