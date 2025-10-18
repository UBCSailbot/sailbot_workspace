import React, { useRef, useCallback } from 'react';
/* Leaflet related imports */
import {
  MapContainer,
  Marker,
  Popup,
  TileLayer,
  Polyline,
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
import styles from './maps.module.css';

export interface IMapsProps {
  gpsLocation: GPS | undefined;
  gpsPath: L.LatLngExpression[];
  globalPath: L.LatLngExpression[];
  showGlobalPath: boolean;
  localPath: L.LatLngExpression[];
  showLocalPath: boolean;
  aisShips: AISShip[];
  showAIShips: boolean;
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

const Maps: React.FC<IMapsProps> = ({
  gpsLocation,
  gpsPath,
  globalPath,
  showGlobalPath,
  localPath,
  showLocalPath,
  aisShips,
  showAIShips,
}) => {
  const mapRef = useRef<L.Map | null>(null);

  /**
   * Sets the map reference.
   *
   * @param mapInstance - The Leaflet map instance to be stored in the ref.
   *              This instance is used for various map operations within the component.
   */
  const setMapRef = useCallback((mapInstance: L.Map) => {
    if (mapInstance) {
      mapRef.current = mapInstance;
      setTimeout(() => {
        mapInstance.invalidateSize();
      }, 100);
    }
  }, []);

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
  const rotatePoint = useCallback(
    (point: L.LatLngExpression, angle: number, axis: L.LatLngExpression) => {
      if (!mapRef.current) {
        return point;
      }
      return L.GeometryUtil.rotatePoint(mapRef.current, point, angle, axis);
    },
    [mapRef],
  );

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
  const renderShips = useCallback(() => {
    const EARTH_RADIUS_METERS = 6378000;
    const PI = Math.PI;

    return aisShips.map((ship, index) => {
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

      const topLeftRotated = rotatePoint(
        L.latLng(newLatitudeNorth, newLongitudeWest),
        cog,
        L.latLng(latitude, longitude),
      );
      const topRightRotated = rotatePoint(
        L.latLng(newLatitudeNorth, newLongitudeEast),
        cog,
        L.latLng(latitude, longitude),
      );
      const bottomLeftRotated = rotatePoint(
        L.latLng(newLatitudeSouth, newLongitudeWest),
        cog,
        L.latLng(latitude, longitude),
      );
      const bottomRightRotated = rotatePoint(
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
  }, [aisShips, rotatePoint]);

  return (
    <MapContainer
      center={convertToLatLng(gpsLocation)}
      zoom={13}
      scrollWheelZoom={true}
      className={styles.maps}
      ref={setMapRef}
    >
      {/* this for future light mode */}
      {/* <TileLayer
        attribution='&copy; OpenStreetMap contributors &copy; CARTO'
        url='https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png'
      /> */}
      <TileLayer
        attribution='&copy; OpenStreetMap contributors &copy; CARTO'
        url='https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png'
      />

      {showAIShips && <LayerGroup>{renderShips()}</LayerGroup>}
      {showLocalPath && (
        <Polyline pathOptions={{ color: 'red' }} positions={localPath} />
      )}
      {showGlobalPath && (
        <Polyline
          pathOptions={{ color: 'black', opacity: 0.25 }}
          positions={globalPath}
        />
      )}
      <Marker position={convertToLatLng(gpsLocation)} icon={createCustomIcon()}>
        <Popup>{printObjectInfo(gpsLocation)}</Popup>
      </Marker>
      <Polyline pathOptions={{ color: 'black' }} positions={gpsPath} />
    </MapContainer>
  );
};

export default Maps;

const createCustomIcon = () => {
  return L.divIcon({
    className: 'custom-marker',
    html: `
      <div style="
        width: 20px;
        height: 20px;
        background: #2563eb;
        border: 3px solid white;
        border-radius: 50%;
        box-shadow: 0 2px 8px rgba(0,0,0,0.3);
        position: relative;
      ">
        <div style="
          position: absolute;
          top: -8px;
          left: 50%;
          transform: translateX(-50%);
          width: 0;
          height: 0;
          border-left: 6px solid transparent;
          border-right: 6px solid transparent;
          border-bottom: 8px solid #2563eb;
        "></div>
      </div>
    `,
    iconSize: [20, 20],
    iconAnchor: [10, 10],
  });
};
