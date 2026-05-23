export type AISShip = {
  id: number;
  latitude: number;
  longitude: number;
  cog: number;
  rot: number;
  sog: number;
  width: number;
  length: number;
};

export type AISShips = {
  ships: AISShip[];
  timestamp: number;
};

export type AISShipsState = {
  data: AISShips;
  error?: any;
};
