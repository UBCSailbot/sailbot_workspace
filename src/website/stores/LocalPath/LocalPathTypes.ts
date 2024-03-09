export type WayPoint = {
  latitude: number;
  longitude: number;
};

export type LocalPath = {
  waypoints: WayPoint[];
};

export type LocalPathState = {
  data: LocalPath;
  error?: any;
};
