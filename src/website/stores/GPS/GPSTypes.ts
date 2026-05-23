export interface GPS {
  latitude: number;
  longitude: number;
  speed: number;
  heading: number;
  timestamp: number;
}
export interface GPSState {
  data: GPS[];
  error?: any;
}
