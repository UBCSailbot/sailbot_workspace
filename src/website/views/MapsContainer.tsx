import React from 'react';
import { connect } from 'react-redux';
import { GPS, GPSState } from '@/stores/GPS/GPSTypes';
import { GlobalPathState } from '@/stores/GlobalPath/GlobalPathTypes';
import { AISShipsState } from '@/stores/AISShips/AISShipsTypes';
import { WayPoint, LocalPathState } from '@/stores/LocalPath/LocalPathTypes';
import Maps, { convertToLatLng } from './components/Maps/Maps';

export interface MapsContainerProps {
  gps: GPSState;
  globalPath: GlobalPathState;
  aisShips: AISShipsState;
  localPath: LocalPathState;
}

class MapsContainer extends React.PureComponent<MapsContainerProps> {
  render() {
    return (
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
    );
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
