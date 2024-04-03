import React from 'react';
import { connect } from 'react-redux';
import { GPS, GPSState } from '@/stores/GPS/GPSTypes';
import { GlobalPathState } from '@/stores/GlobalPath/GlobalPathTypes';
import { AISShipsState } from '@/stores/AISShips/AISShipsTypes';
import { WayPoint, LocalPathState } from '@/stores/LocalPath/LocalPathTypes';
import Maps, { convertToLatLng } from './components/Maps/Maps';
import SingleValueLine from './components/SingleValueLine/SingleValueLine';
import styles from './components/SingleValueLine/singlevalueline.module.css';
import BoatCompass from './components/BoatCompass/BoatCompass';

export interface MapsContainerProps {
  gps: GPSState;
  globalPath: GlobalPathState;
  aisShips: AISShipsState;
  localPath: LocalPathState;
}

class MapsContainer extends React.PureComponent<MapsContainerProps> {
  render() {
    const { gps } = this.props;

    const gpsDistanceData = [
      gps.data.map((data) => data.latitude),
      gps.data.map((data) => data.longitude),
    ];

    const totalTripDistance = this._computeTotalTripDistance(
      gpsDistanceData[0],
      gpsDistanceData[1],
    );

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
          <BoatCompass angle={this.props.gps.data.at(-1)?.heading} />
        </div>
        <Maps
          gpsLocation={this.props.gps.data.at(-1)}
          gpsPath={this.props.gps.data.map((gpsPoint: GPS) =>
            convertToLatLng(gpsPoint),
          )}
          globalPath={this.props.globalPath.data.waypoints.map(
            (waypoint: WayPoint) => convertToLatLng(waypoint),
          )}
          aisShips={this.props.aisShips.data.ships}
          localPath={this.props.localPath.data.waypoints.map(
            (waypoint: WayPoint) => convertToLatLng(waypoint),
          )}
        />
      </div>
    );
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

  _computeTotalTripDistance(latitude: number[], longitude: number[]) {
    if (latitude.length != longitude.length) {
      return -1;
    }

    let totalDistance = 0;

    for (let i = 1; i < latitude.length; i++) {
      totalDistance += this._haversineDistance(
        latitude[i - 1],
        longitude[i - 1],
        latitude[i],
        longitude[i],
      );
    }

    return Number(totalDistance.toFixed(2));
  }
}

const mapStateToProps = (state: any) => ({
  gps: state.gps,
  globalPath: state.globalPath,
  aisShips: state.aisShips,
  localPath: state.localPath,
});
const mapDispatchToProps = {};

export default connect(mapStateToProps, mapDispatchToProps)(MapsContainer);
