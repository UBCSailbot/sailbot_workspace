import React from 'react';
import { connect } from 'react-redux';
import UPlotLineChartComponent from './components/LineChart/UPlotLineChart';
import { GPS } from '@/stores/GPS/GPSTypes';
import { Batteries } from '@/stores/Batteries/BatteriesTypes';
import { WindSensors } from '@/stores/WindSensors/WindSensorsTypes';
import UPlotMultiLineChartComponent from './components/LineChart/UPlotMultiLineChart';

export interface DashboardContainerProps {
  gps: GPS[];
  batteries: Batteries[];
  windSensors: WindSensors[];
}

class DashboardContainer extends React.PureComponent<DashboardContainerProps> {
  render() {
    const { gps, batteries, windSensors } = this.props;

    const gpsChartData = [
      gps.data.map((data) => this._parseISOString(data.timestamp)),
      gps.data.map((data) => data.speed),
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

    return (
      <div>
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
}

const mapStateToProps = (state: any) => ({
  gps: state.gps,
  batteries: state.batteries,
  windSensors: state.windSensors,
});

const mapDispatchToProps = {};

export default connect(mapStateToProps, mapDispatchToProps)(DashboardContainer);
