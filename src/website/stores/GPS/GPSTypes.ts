export interface GPS {
  timestamp: any;
  latitude: number;
  longitude: number;
  speed: number;
  heading: number;
}
export interface GPSState {
  data: GPS[];
  error?: any;
}
