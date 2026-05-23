export type WayPoint = {
  latitude: number;
  longitude: number;
};

export type GlobalPath = {
  waypoints: WayPoint[];
  timestamp: number;
};

export type GlobalPathState = {
  data: GlobalPath;
  error?: any;
};
