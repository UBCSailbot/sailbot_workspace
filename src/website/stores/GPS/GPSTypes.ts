export interface GPS {
  latitude: number;
  longitude: number;
  speed: number;
  heading: number;
}
export interface GPSState {
  data: GPS[];
  error?: any;
}
