export type WayPoint = {
  latitude: number;
  longitude: number;
};

export type LocalPath = {
  waypoints: WayPoint[];
  timestamp: number;
};

export type LocalPathState = {
  data: LocalPath;
  error?: any;
};
