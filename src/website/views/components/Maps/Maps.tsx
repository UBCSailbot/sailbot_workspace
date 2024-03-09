import React from 'react';
/* Leaflet related imports */
import {
  MapContainer,
  Marker,
  Popup,
  TileLayer,
  Polyline,
  LayersControl,
  LayerGroup,
  Polygon,
} from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import 'leaflet-defaulticon-compatibility';
import 'leaflet-defaulticon-compatibility/dist/leaflet-defaulticon-compatibility.css';
import L from 'leaflet';
import 'leaflet-geometryutil';
import { GPS } from '@/stores/GPS/GPSTypes';
import { AISShip } from '@/stores/AISShips/AISShipsTypes';

export interface IMapsProps {
  gpsLocation: GPS | undefined;
  gpsPath: L.LatLngExpression[];
  globalPath: L.LatLngExpression[];
  localPath: L.LatLngExpression[];
  aisShips: AISShip[];
}

export interface IMapsState {
  map: L.Map | null;
}

/**
 * Converts an object's properties into digestible text.
 * For example, given an object {a: 1, b: 2}, the function will output the text:
 * """
 *  a: 1
 *  b: 1
 * """
 *
 * @param obj - the object
 * @returns an array of the pretty printed objects
 */
export const printObjectInfo = (obj: any): any[] => {
  let ele: any[] = [];
  Object.keys(obj).forEach((key, i) => {
    ele.push(`${key}: ${obj[key]}`, <br key={i} />);
  });
  return ele;
};

/**
 * Converts an object with latitude and longitude fields into an array.
 * This is conversion is necessary for Leaflet.
 *
 * @param obj - object with fields latitude and longitude
 * @returns an array containing [latitude, longitude]
 */
export const convertToLatLng = (obj: any): L.LatLngExpression => {
  return L.latLng(obj.latitude, obj.longitude);
};

export default class Maps extends React.Component<IMapsProps, IMapsState> {
  readonly state: IMapsState = {
    map: null,
  };

  /**
   * Sets the map reference in the component's state.
   *
   * @param map - The Leaflet map instance to be stored in the component's state.
   *              This instance is used for various map operations within the component.
   */
  setMapRef = (map: L.Map) => {
    this.setState((state) => ({ ...state, map: map }));
  };

  /**
   * Rotates a point around a specified axis by a given angle.
   *
   * @param point - The point (latitude and longitude) to be rotated. Expects an L.LatLngExpression.
   * @param angle - The angle of rotation in degrees. Positive values will rotate
   *                the point in a clockwise direction, while negative values rotate
   *                counterclockwise.
   * @param axis  - The axis point (latitude and longitude) around which the rotation
   *                will occur. Expects an L.LatLngExpression.
   * @returns     - The rotated point as an L.LatLngExpression. If the map is not
   *                initialized, the original point is returned without modification.
   */
  rotatePoint = (
    point: L.LatLngExpression,
    angle: number,
    axis: L.LatLngExpression,
  ) => {
    const map = this.state.map;
    if (map == null) {
      return point;
    }
    return L.GeometryUtil.rotatePoint(map, point, angle, axis);
  };

  /**
   * Renders ships on a map as rectangles. Each ship is represented by a rectangle
   * calculated based on its latitude, longitude, width, length, and course over ground (cog).
   * The method calculates the coordinates for the corners of each rectangle, rotates them
   * according to the ship's cog, and then renders a Polygon on the map for each ship.
   *
   * @returns - An array of React Polygon components. Each Polygon represents a ship and
   *            is positioned on the map based on the ship's data. The Polygons are styled
   *            with red borders and contain a Popup that shows the ship's information.
   *            If the aisShips prop is empty, no Polygons are rendered.
   */
  renderShips = () => {
    const EARTH_RADIUS_METERS = 6378000;
    const PI = Math.PI;

    return this.props.aisShips.map((ship, index) => {
      const { latitude, longitude, width, length, cog } = ship;

      // Assuming that length and width are in meters
      const dy = length / 2;
      const dx = width / 2;

      // Calculate the top left and bottom right coordinates of the rectangle
      const newLatitudeNorth =
        latitude + (dy / EARTH_RADIUS_METERS) * (180 / PI);
      const newLongitudeWest =
        longitude -
        ((dx / EARTH_RADIUS_METERS) * (180 / PI)) /
          Math.cos((latitude * PI) / 180);
      const newLatitudeSouth =
        latitude - (dy / EARTH_RADIUS_METERS) * (180 / PI);
      const newLongitudeEast =
        longitude +
        ((dx / EARTH_RADIUS_METERS) * (180 / PI)) /
          Math.cos((latitude * PI) / 180);

      const topLeftRotated = this.rotatePoint(
        L.latLng(newLatitudeNorth, newLongitudeWest),
        cog,
        L.latLng(latitude, longitude),
      );
      const topRightRotated = this.rotatePoint(
        L.latLng(newLatitudeNorth, newLongitudeEast),
        cog,
        L.latLng(latitude, longitude),
      );
      const bottomLeftRotated = this.rotatePoint(
        L.latLng(newLatitudeSouth, newLongitudeWest),
        cog,
        L.latLng(latitude, longitude),
      );
      const bottomRightRotated = this.rotatePoint(
        L.latLng(newLatitudeSouth, newLongitudeEast),
        cog,
        L.latLng(latitude, longitude),
      );

      const bounds: L.LatLngExpression[] = [
        topLeftRotated,
        topRightRotated,
        bottomRightRotated,
        bottomLeftRotated,
      ];

      // Rectangle options
      const redOptions = { color: 'red' };

      return (
        <Polygon key={index} positions={bounds} pathOptions={redOptions}>
          <Popup>{printObjectInfo(ship)}</Popup>
        </Polygon>
      );
    });
  };

  render() {
    return (
      <MapContainer
        center={convertToLatLng(this.props.gpsLocation)}
        zoom={13}
        scrollWheelZoom={true}
        style={{ height: 'calc(100vh - 115px)', width: '100wh' }}
        ref={this.setMapRef}
      >
        <TileLayer
          attribution='&copy; OpenStreetMap contributors'
          url='https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png'
        />
        <LayersControl position='bottomleft'>
          <LayersControl.Overlay name='AIS Ships' checked>
            <LayerGroup>{this.renderShips()}</LayerGroup>
          </LayersControl.Overlay>
          <LayersControl.Overlay name='Local Path' checked>
            <Polyline
              pathOptions={{ color: 'red' }}
              positions={this.props.localPath}
            />
          </LayersControl.Overlay>
          <LayersControl.Overlay name='Global Path' checked>
            <Polyline
              pathOptions={{ color: 'black', opacity: 0.25 }}
              positions={this.props.globalPath}
            />
          </LayersControl.Overlay>
        </LayersControl>
        <Marker position={convertToLatLng(this.props.gpsLocation)}>
          <Popup>{printObjectInfo(this.props.gpsLocation)}</Popup>
        </Marker>
        <Polyline
          pathOptions={{ color: 'black' }}
          positions={this.props.gpsPath}
        />
      </MapContainer>
    );
  }
}
