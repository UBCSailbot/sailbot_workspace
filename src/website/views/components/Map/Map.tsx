'use client';

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
  loading: () => <div className={styles.loading}>Loading map...</div>,
});

// Create a simple conversion function that doesn't depend on Leaflet
const convertToLatLng = (obj: any): [number, number] => {
  return [obj.latitude, obj.longitude];
};

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

  const filters = [
    {
      label: 'AIS Ships',
      key: 'aisShips',
      setState: setShowAIShips,
    },
    {
      label: 'Global Path',
      key: 'globalPath',
      setState: setShowGlobalPath,
    },
    {
      label: 'Local Path',
      key: 'localPath',
      setState: setShowLocalPath,
    },
  ];

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
        {filters.map((filter) => (
          <div className={styles.filter} key={filter.key}>
            <label>
              <input
                type='checkbox'
                defaultChecked
                onChange={(e) => filter.setState(e.target.checked)}
              />
              {filter.label}
            </label>
          </div>
        ))}
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
