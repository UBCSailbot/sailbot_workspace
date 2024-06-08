export interface GPS {
  latitude: number;
  longitude: number;
  speed: number;
  heading: number;
  timestamp: string;
}
export interface GPSState {
  data: GPS[];
  error?: any;
}
