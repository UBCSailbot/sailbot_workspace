export type WayPoint = {
  latitude: number;
  longitude: number;
};

export type LocalPath = {
  waypoints: WayPoint[];
  timestamp: string;
};

export type LocalPathState = {
  data: LocalPath;
  error?: any;
};
