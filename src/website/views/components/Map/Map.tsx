import { useState } from 'react';
import dynamic from 'next/dynamic';
import styles from './map.module.css';
import { connect } from 'react-redux';
import { GPSState } from '@/stores/GPS/GPSTypes';
import { GlobalPathState } from '@/stores/GlobalPath/GlobalPathTypes';
import { LocalPathState } from '@/stores/LocalPath/LocalPathTypes';
import { AISShipsState } from '@/stores/AISShips/AISShipsTypes';

// Dynamically import Maps component with SSR disabled
const Maps = dynamic(() => import('../Maps/Maps'), {
  ssr: false,
  loading: () => <div>Loading map...</div>,
});

// Create a simple conversion function that doesn't depend on Leaflet
const convertToLatLng = (obj: any): [number, number] => {
  return [obj.latitude, obj.longitude];
};

// Fake GPS data around Vancouver, BC (sailbot location)
const fakeGPSData = [
  {
    latitude: 49.2827,
    longitude: -123.1207,
    speed: 12.5,
    heading: 45,
    timestamp: '2024-01-01T10:00:00Z',
  },
  {
    latitude: 49.283,
    longitude: -123.12,
    speed: 11.8,
    heading: 50,
    timestamp: '2024-01-01T10:01:00Z',
  },
  {
    latitude: 49.2835,
    longitude: -123.1195,
    speed: 13.2,
    heading: 48,
    timestamp: '2024-01-01T10:02:00Z',
  },
  {
    latitude: 49.284,
    longitude: -123.119,
    speed: 12.9,
    heading: 52,
    timestamp: '2024-01-01T10:03:00Z',
  },
  {
    latitude: 49.2845,
    longitude: -123.1185,
    speed: 14.1,
    heading: 47,
    timestamp: '2024-01-01T10:04:00Z',
  },
];

// Current GPS location (latest point)
const currentGPSLocation = fakeGPSData[fakeGPSData.length - 1];

// Fake global path (planned route)
const fakeGlobalPath = [
  { latitude: 49.2827, longitude: -123.1207 },
  { latitude: 49.285, longitude: -123.118 },
  { latitude: 49.287, longitude: -123.115 },
  { latitude: 49.289, longitude: -123.112 },
  { latitude: 49.291, longitude: -123.109 },
  { latitude: 49.293, longitude: -123.106 },
];

// Fake local path (immediate navigation)
const fakeLocalPath = [
  { latitude: 49.2845, longitude: -123.1185 },
  { latitude: 49.2847, longitude: -123.1182 },
  { latitude: 49.2849, longitude: -123.1179 },
  { latitude: 49.2851, longitude: -123.1176 },
];

// Fake AIS ships (other vessels)
const fakeAISShips = [
  {
    id: 1,
    rot: 0,
    sog: 0,
    latitude: 49.286,
    longitude: -123.117,
    width: 20,
    length: 80,
    cog: 120,
    name: 'Cargo Ship Alpha',
    speed: 8.5,
    mmsi: '123456789',
  },
  {
    id: 2,
    rot: 0,
    sog: 0,
    latitude: 49.282,
    longitude: -123.122,
    width: 15,
    length: 60,
    cog: 270,
    name: 'Ferry Beta',
    speed: 15.2,
    mmsi: '987654321',
  },
  {
    id: 3,
    rot: 0,
    sog: 0,
    latitude: 49.288,
    longitude: -123.114,
    width: 8,
    length: 25,
    cog: 45,
    name: 'Sailboat Gamma',
    speed: 6.8,
    mmsi: '456789123',
  },
];

const Map = ({
  gps,
  globalPath,
  localPath,
  aisShips,
}: {
  gps: GPSState;
  globalPath: GlobalPathState;
  localPath: LocalPathState;
  aisShips: AISShipsState;
}) => {
  const [showAIShips, setShowAIShips] = useState(true);
  const [showGlobalPath, setShowGlobalPath] = useState(true);
  const [showLocalPath, setShowLocalPath] = useState(true);

  const gpsData = gps.data;
  const globalPathData = globalPath.data;
  const localPathData = localPath.data;
  const aisShipsData = aisShips.data;

  return (
    <div className={styles.map}>
      <Maps
        gpsLocation={gpsData[gpsData.length - 1]}
        gpsPath={gpsData.map((gpsPoint) => convertToLatLng(gpsPoint))}
        globalPath={globalPathData.waypoints.map((waypoint) =>
          convertToLatLng(waypoint),
        )}
        showGlobalPath={showGlobalPath}
        localPath={localPathData.waypoints.map((waypoint) =>
          convertToLatLng(waypoint),
        )}
        showLocalPath={showLocalPath}
        aisShips={aisShipsData.ships}
        showAIShips={showAIShips}
      />
      <div className={styles.toolbar}>
        <div className={styles.filter}>
          <label>
            <input
              type='checkbox'
              defaultChecked
              onChange={(e) => setShowAIShips(e.target.checked)}
            />
            AIS Ships
          </label>
        </div>
        <div className={styles.filter}>
          <label>
            <input
              type='checkbox'
              defaultChecked
              onChange={(e) => setShowGlobalPath(e.target.checked)}
            />
            Global Path
          </label>
        </div>
        <div className={styles.filter}>
          <label>
            <input
              type='checkbox'
              defaultChecked
              onChange={(e) => setShowLocalPath(e.target.checked)}
            />
            Local Path
          </label>
        </div>
      </div>
    </div>
  );
};

const mapStateToProps = (state: any) => ({
  gps: state.gps,
  globalPath: state.globalPath,
  localPath: state.localPath,
  aisShips: state.aisShips,
});

export default connect(mapStateToProps, null)(Map);
