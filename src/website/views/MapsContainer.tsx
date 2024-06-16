import React from 'react';
import { connect } from 'react-redux';
import { GPS, GPSState } from '@/stores/GPS/GPSTypes';
import { GlobalPathState } from '@/stores/GlobalPath/GlobalPathTypes';
import { AISShipsState } from '@/stores/AISShips/AISShipsTypes';
import { WayPoint, LocalPathState } from '@/stores/LocalPath/LocalPathTypes';
import { DataFilterState } from '@/stores/DataFilter/DataFilterTypes';
import Maps, { convertToLatLng } from './components/Maps/Maps';
import SingleValueLine from './components/SingleValueLine/SingleValueLine';
import styles from './components/SingleValueLine/singlevalueline.module.css';
import BoatCompass from './components/BoatCompass/BoatCompass';

export interface MapsContainerProps {
  gps: GPSState;
  globalPath: GlobalPathState;
  aisShips: AISShipsState;
  localPath: LocalPathState;
  dataFilter: DataFilterState;
}

class MapsContainer extends React.PureComponent<MapsContainerProps> {
  render() {
    const { gps } = this.props;

    const gpsData = gps.data.filter(
      (data: GPS) =>
        this._validTimestamp(this._parseISOString(data.timestamp)) == true,
    );

    const totalTripDistance = this._computeTotalTripDistance(gpsData);

    return (
      <div className={styles.parent}>
        <div className={styles.topRight}>
          <SingleValueLine
            title='Distance'
            data={totalTripDistance}
            unit='km'
          />
        </div>
        <div className={styles.bottomRight}>
          <BoatCompass angle={gpsData.at(-1)?.heading} />
        </div>
        <Maps
          gpsLocation={this._validGPSLocation(gpsData)}
          gpsPath={gpsData.map((gpsPoint: GPS) => convertToLatLng(gpsPoint))}
          globalPath={this.props.globalPath.data.waypoints.map(
            (waypoint: WayPoint) => convertToLatLng(waypoint),
          )}
          aisShips={this._validAISShips()}
          localPath={this._validLocalPath()}
        />
      </div>
    );
  }

  _parseISOString(s: string) {
    return Math.floor(Date.parse(s) / 1000); // Converts to seconds
  }

  _validTimestamp(timestampISO: number) {
    if (
      this.props.dataFilter.timestamps.startDate == null &&
      this.props.dataFilter.timestamps.endDate == null
    ) {
      return true;
    }

    if (
      timestampISO >=
        // @ts-ignore
        this._parseISOString(this.props.dataFilter.timestamps.startDate) &&
      timestampISO <=
        // @ts-ignore
        this._parseISOString(this.props.dataFilter.timestamps.endDate)
    ) {
      return true;
    }
  }

  _haversineDistance(lat1: number, long1: number, lat2: number, long2: number) {
    function toRadians(angle: number): number {
      return (angle * Math.PI) / 180;
    }

    const EARTH_RADIUS = 6571; // in km

    let delta_lat = lat2 - lat1;
    let delta_lat_rad = toRadians(delta_lat);
    let delta_long = long2 - long1;
    let delta_long_rad = toRadians(delta_long);

    let a =
      Math.sin(delta_lat_rad / 2) * Math.sin(delta_lat_rad / 2) +
      Math.cos(toRadians(lat1)) *
        Math.cos(toRadians(lat2)) *
        Math.sin(delta_long_rad / 2) *
        Math.sin(delta_long_rad / 2);
    let c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    let distance = EARTH_RADIUS * c;

    return distance;
  }

  _computeTotalTripDistance(gpsData: GPS[]) {
    let totalDistance = 0;

    for (let i = 1; i < gpsData.length; i++) {
      totalDistance += this._haversineDistance(
        gpsData[i - 1].latitude,
        gpsData[i - 1].longitude,
        gpsData[i].latitude,
        gpsData[i].longitude,
      );
    }

    return Number(totalDistance.toFixed(2));
  }

  _validGPSLocation(gpsdata: GPS[]) {
    if (typeof gpsdata.at(-1) === 'undefined') {
      return {
        latitude: 999,
        longitude: 999,
        speed: 999,
        heading: 999,
        timestamp: new Date().toISOString(),
      };
    }
    return gpsdata.at(-1);
  }

  _validLocalPath() {
    if (
      this._validTimestamp(
        this._parseISOString(this.props.localPath.data.timestamp),
      ) == true
    ) {
      return this.props.localPath.data.waypoints.map((waypoint: WayPoint) =>
        convertToLatLng(waypoint),
      );
    } else {
      return [];
    }
  }

  _validAISShips() {
    if (
      this._validTimestamp(
        this._parseISOString(this.props.aisShips.data.timestamp),
      ) == true
    ) {
      return this.props.aisShips.data.ships;
    } else {
      return [];
    }
  }
}

const mapStateToProps = (state: any) => ({
  gps: state.gps,
  globalPath: state.globalPath,
  aisShips: state.aisShips,
  localPath: state.localPath,
  dataFilter: state.dataFilter,
});
const mapDispatchToProps = {};

export default connect(mapStateToProps, mapDispatchToProps)(MapsContainer);
