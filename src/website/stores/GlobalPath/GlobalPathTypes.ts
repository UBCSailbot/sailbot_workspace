export type WayPoint = {
  latitude: number;
  longitude: number;
};

export type GlobalPath = {
  waypoints: WayPoint[];
  timestamp: string;
};

export type GlobalPathState = {
  data: GlobalPath;
  error?: any;
};
