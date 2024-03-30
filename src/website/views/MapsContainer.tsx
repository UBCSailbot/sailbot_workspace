import React from 'react';
import { connect } from 'react-redux';
import { GPS, GPSState } from '@/stores/GPS/GPSTypes';
import { GlobalPathState } from '@/stores/GlobalPath/GlobalPathTypes';
import { AISShipsState } from '@/stores/AISShips/AISShipsTypes';
import { WayPoint, LocalPathState } from '@/stores/LocalPath/LocalPathTypes';
import { DataFilterState } from '@/stores/DataFilter/DataFilterTypes';
import Maps, { convertToLatLng } from './components/Maps/Maps';
import SingleValueLine from './components/SingleValueLine/SingleValueLine';
import styles from './components/SingleValueLine/singlevalueline.module.css'
import BoatCompass from './components/BoatCompass/BoatCompass';

export interface MapsContainerProps {
  gps: GPSState;
  globalPath: GlobalPathState;
  aisShips: AISShipsState;
  localPath: LocalPathState;
  timestamp: DataFilterState;
}

class MapsContainer extends React.PureComponent<MapsContainerProps> {
  render() {
    const { gps, localPath } = this.props;

    const gpsData = [
      gps.data.map((data) => data.latitude),
      gps.data.map((data) => data.longitude),
      gps.data.map((data) => data.speed),
      gps.data.map((data) => data.heading),
      gps.data.map((data) => data.timestamp),
      gps.data.map((data) => this._parseISOString(data.timestamp)),
    ];

    const validGPSData = [[], [], [], [], []]
    for (let i = 0; i < gpsData[0].length; i++) {
      if (this._validTimestamp(gpsData[5][i]) == true) {
        validGPSData[0].push((gpsData[0][i])),
          validGPSData[1].push((gpsData[1][i])),
          validGPSData[2].push((gpsData[2][i])),
          validGPSData[3].push((gpsData[3][i])),
          validGPSData[4].push((gpsData[4][i]))
      }
    }

    const validGPSObject = []
    for (let i = 0; i < gpsData[0].length; i++) {
      if (this._validTimestamp(gpsData[5][i]) == true) {
        validGPSObject.push({
          latitude:   gpsData[0][i],
          longitude:  gpsData[1][i],
          speed:      gpsData[2][i],
          heading:    gpsData[3][i],
          timestamp:  gpsData[4][i]
        })
      }
    }

    const totalTripDistance = this._computeTotalTripDistance(validGPSData[0], validGPSData[1])

    return (
      <div className={styles.parent}>
        <div className={styles.topRight}>
          <SingleValueLine title="Distance" data={totalTripDistance} unit="km" />
        </div>
        <div className={styles.bottomRight}>
          <BoatCompass angle={validGPSData[3].at(-1)} />
        </div>
        <Maps
          gpsLocation={this._validGPSLocation(validGPSData)}
          gpsPath={validGPSObject.map((validGPSPoint: GPS) => convertToLatLng(validGPSPoint))}
          globalPath={this.props.globalPath.data.waypoints.map(
            (waypoint: WayPoint) => convertToLatLng(waypoint),
          )}
          aisShips={this.props.aisShips.data.ships}
          localPath={this._validLocalPath}
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

    if (timestampISO >=
      this._parseISOString(this.props.dataFilter.timestamps.startDate) &&
      timestampISO <=
      this._parseISOString(this.props.dataFilter.timestamps.endDate)) {
      return true;
    }
  }

  _haversineDistance(lat1: number, long1: number, lat2: number, long2: number) {

    function toRadians(angle: number): number {
      return angle * Math.PI / 180
    }

    const EARTH_RADIUS = 6571 // in km

    let delta_lat = lat2 - lat1
    let delta_lat_rad = toRadians(delta_lat)
    let delta_long = long2 - long1
    let delta_long_rad = toRadians(delta_long)

    let a = (Math.sin(delta_lat_rad / 2) * Math.sin(delta_lat_rad / 2))
      + (Math.cos(toRadians(lat1)) * Math.cos(toRadians(lat2))
        * Math.sin(delta_long_rad / 2)
        * Math.sin(delta_long_rad / 2))
    let c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    let distance = EARTH_RADIUS * c;

    return distance
  }

  _computeTotalTripDistance(latitude: number[], longitude: number[]) {
    if (latitude.length != longitude.length) {
      return -1;
    }

    let totalDistance = 0;

    for (let i = 1; i < latitude.length; i++) {
      totalDistance += this._haversineDistance(latitude[i - 1], longitude[i - 1], latitude[i], longitude[i]);
    }

    return Number(totalDistance.toFixed(2));
  }

  _validGPSLocation(gpsdata: [number[], number[], number[], number[], number[]]) {
    if (typeof gpsdata[0].at(-1) == 'undefined') {
      return {
        latitude: 49.37614179786771,
        longitude: -123.27376619978901,
        speed: 0,
        heading: 0,
      }
    }
    else {
      return {
        latitude: gpsdata[0].at(-1),
        longitude: gpsdata[1].at(-1),
        speed: gpsdata[2].at(-1),
        heading: gpsdata[3].at(-1),
        timestamp: gpsdata[4].at(-1)
      }
    }
  }

  _validLocalPath(){
    if (this._validTimestamp(this._parseISOString(this.props.localPath.data.timestamp)) == true) {
      return this.props.localPath.data.waypoints.map((waypoint: WayPoint) => convertToLatLng(waypoint));
    }
    else {
      return null;
    }
  }
}

const mapStateToProps = (state: any) => ({
  gps: state.gps,
  globalPath: state.globalPath,
  aisShips: state.aisShips,
  localPath: state.localPath,
  dataFilter: state.dataFilter
});
const mapDispatchToProps = {};

export default connect(mapStateToProps, mapDispatchToProps)(MapsContainer);
