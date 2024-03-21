import React from 'react';
import { connect } from 'react-redux';
import UPlotLineChartComponent from './components/LineChart/UPlotLineChart';
import { GPSState } from '@/stores/GPS/GPSTypes';
import { BatteriesState } from '@/stores/Batteries/BatteriesTypes';
import { WindSensorsState } from '@/stores/WindSensors/WindSensorsTypes';
import { TimestampState } from '@/stores/Timestamp/TimestampTypes';
import UPlotMultiLineChartComponent from './components/LineChart/UPlotMultiLineChart';
import SingleValueChart from './components/SingleValueChart/SingleValueChart';
import { Grid } from '@mui/material';

export interface DashboardContainerProps {
  gps: GPSState;
  batteries: BatteriesState;
  windSensors: WindSensorsState;
}

class DashboardContainer extends React.PureComponent<DashboardContainerProps> {
  render() {
    const { gps, batteries, windSensors, timestampFilter } = this.props;

    // if( typeof timestampFilter !== 'undefined'){
    //   console.log(timestampFilter[0])
    // }

    // if( timestampFilter.isArray() == true ){
    //   console.log("hello");
    // }

    console.log(timestampFilter.timestamps[0]);

    const gpsChartData = [
      gps.data.map((data) => this._parseISOString(data.timestamp)),
      gps.data.map((data) => data.speed),
    ];

    const gpsDistanceData = [
      gps.data.map((data) => data.latitude),
      gps.data.map((data) => data.longitude)
    ];

    const batteriesVoltageData = [
      batteries.data.map((data) => this._parseISOString(data.timestamp)),
      batteries.data.map((data) => data.batteries[0].voltage),
      batteries.data.map((data) => data.batteries[1].voltage),
    ];

    const batteriesCurrentData = [
      batteries.data.map((data) => this._parseISOString(data.timestamp)),
      batteries.data.map((data) => data.batteries[0].current),
      batteries.data.map((data) => data.batteries[1].current),
    ];

    const windSensorsSpeedData = [
      windSensors.data.map((data) => this._parseISOString(data.timestamp)),
      windSensors.data.map((data) => data.windSensors[0].speed),
      windSensors.data.map((data) => data.windSensors[1].speed),
    ];

    // console.log(gpsChartData[1])

    const totalTripDistance = this._computeTotalTripDistance(gpsDistanceData[0], gpsDistanceData[1])

    return (
      <div>
        {/* <Grid
          container spacing={0}
          direction={"row"}
          justifyContent={"center"}
          alignItems={"center"}
          >
          <Grid container justifyContent="center" item xs={3}>
            <SingleValueChart title="Distance" data={totalTripDistance} unit="km" />
          </Grid>
          <Grid container justifyContent="center" item xs={3}>
            <SingleValueChart title="Heading" data={this.props.gps.data.at(-1)?.heading} unit=""/>
          </Grid>
          <Grid container justifyContent="center" item xs={3}>
            <SingleValueChart title="Latitude" data={this.props.gps.data.at(-1)?.latitude.toFixed(2)} unit="°"/>
          </Grid>
          <Grid container justifyContent="center" item xs={3}>
            <SingleValueChart title="Longitude" data={this.props.gps.data.at(-1)?.longitude.toFixed(2)} unit="°"/>
          </Grid>
        </Grid> */}
        <UPlotLineChartComponent
          data={gpsChartData}
          label='Boat Speed'
          unit='km/hr'
        />
        <UPlotMultiLineChartComponent
          data={batteriesVoltageData}
          labelOne='Battery 1 Voltage'
          labelTwo='Battery 2 Voltage'
          unit='V'
        />
        <UPlotMultiLineChartComponent
          data={batteriesCurrentData}
          labelOne='Battery 1 Current'
          labelTwo='Battery 2 Current'
          unit='A'
        />
        <UPlotMultiLineChartComponent
          data={windSensorsSpeedData}
          labelOne='Wind Sensor 1 Speed'
          labelTwo='Wind Sensor 2 Speed'
          unit='m/s'
        />
      </div>
    );
  }

  _parseISOString(s: string) {
    return Math.floor(Date.parse(s) / 1000); // Converts to seconds
  }

  _haversineDistance(lat1: number, long1: number, lat2: number, long2: number) {

    function toRadians(angle: number): number{
      return angle * Math.PI / 180
    }

    const EARTH_RADIUS = 6571 // in km

    let delta_lat = lat2-lat1
    let delta_lat_rad = toRadians(delta_lat)
    let delta_long = long2-long1
    let delta_long_rad = toRadians(delta_long)

    let a = (Math.sin(delta_lat_rad/2) * Math.sin(delta_lat_rad/2))
    + (Math.cos(toRadians(lat1)) * Math.cos(toRadians(lat2))
    * Math.sin(delta_long_rad/2)
    * Math.sin(delta_long_rad/2))
    let c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

    let distance = EARTH_RADIUS * c;

    return distance
  }

  _computeTotalTripDistance(latitude: number[], longitude: number[]) {
    if(latitude.length != longitude.length){
      return -1;
    }

    let totalDistance = 0;

    for(let i = 1; i < latitude.length; i++){
        totalDistance += this._haversineDistance(latitude[i-1], longitude[i-1], latitude[i], longitude[i]);
    }

    return Number(totalDistance.toFixed(2));
  }
}

const mapStateToProps = (state: any) => ({
  gps: state.gps,
  batteries: state.batteries,
  windSensors: state.windSensors,
  timestampFilter: state.timestampFilter
});

const mapDispatchToProps = {};

export default connect(mapStateToProps, mapDispatchToProps)(DashboardContainer);
