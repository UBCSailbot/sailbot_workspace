import dynamic from 'next/dynamic';
import styles from './stats.module.css';
import HistoricDataDropdown from './HistoricDataDropdown';
import RearrangeGraphDropdown from './RearrangeGraphDropdown';
import { connect } from 'react-redux';
import { GPS, GPSState } from '@/stores/GPS/GPSTypes';
import { Batteries, BatteriesState } from '@/stores/Batteries/BatteriesTypes';
import {
  WindSensors,
  WindSensorsState,
} from '@/stores/WindSensors/WindSensorsTypes';
import { DataFilterState } from '@/stores/DataFilter/DataFilterTypes';
import {
  downloadGPSData,
  downloadBatteriesData,
  downloadWindSensorsData,
  downloadDataFromJSON,
} from '@/utils/DownloadData';

// LineChart needs to be rendered client side
const LineChart = dynamic(() => import('../LineChart/LineChart'), {
  ssr: false,
});

const parseISOString = (s: string) => {
  return Math.floor(Date.parse(s) / 1000); // Converts to seconds
};

const isValidTimestamp = (
  timestampISO: number,
  startDate: string | null,
  endDate: string | null,
) => {
  if (startDate == null && endDate == null) {
    return true;
  }

  if (startDate == null || endDate == null) {
    return false;
  }

  if (
    timestampISO >= parseISOString(startDate) &&
    timestampISO <= parseISOString(endDate)
  ) {
    return true;
  }

  return false;
};

interface StatsProps {
  gps: GPSState;
  batteries: BatteriesState;
  windSensors: WindSensorsState;
  graphsOrder: string[];
  dataFilter: DataFilterState;
}

const Stats = ({
  gps,
  batteries,
  windSensors,
  graphsOrder,
  dataFilter,
}: StatsProps) => {
  const startDate = dataFilter.timestamps.startDate;
  const endDate = dataFilter.timestamps.endDate;

  const gpsChartData = [
    gps.data
      .map((data: GPS) => parseISOString(data.timestamp))
      .filter(
        (time: number) => isValidTimestamp(time, startDate, endDate) == true,
      ),
    gps.data
      .filter(
        (data: GPS) =>
          isValidTimestamp(
            parseISOString(data.timestamp),
            startDate,
            endDate,
          ) == true,
      )
      .map((data: GPS) => data.speed),
  ];

  const batteriesVoltageData = [
    batteries.data
      .map((data: Batteries) => parseISOString(data.timestamp))
      .filter(
        (time: number) => isValidTimestamp(time, startDate, endDate) == true,
      ),
    batteries.data
      .filter(
        (data: Batteries) =>
          isValidTimestamp(
            parseISOString(data.timestamp),
            startDate,
            endDate,
          ) == true,
      )
      .map((data: Batteries) => data.batteries[0].voltage),
    batteries.data
      .filter(
        (data: Batteries) =>
          isValidTimestamp(
            parseISOString(data.timestamp),
            startDate,
            endDate,
          ) == true,
      )
      .map((data: Batteries) => data.batteries[1].voltage),
  ];

  const batteriesCurrentData = [
    batteries.data
      .map((data: Batteries) => parseISOString(data.timestamp))
      .filter(
        (time: number) => isValidTimestamp(time, startDate, endDate) == true,
      ),
    batteries.data
      .filter(
        (data: Batteries) =>
          isValidTimestamp(
            parseISOString(data.timestamp),
            startDate,
            endDate,
          ) == true,
      )
      .map((data: Batteries) => data.batteries[0].current),
    batteries.data
      .filter(
        (data: Batteries) =>
          isValidTimestamp(
            parseISOString(data.timestamp),
            startDate,
            endDate,
          ) == true,
      )
      .map((data: Batteries) => data.batteries[1].current),
  ];

  const windSensorsSpeedData = [
    windSensors.data
      .map((data: WindSensors) => parseISOString(data.timestamp))
      .filter(
        (time: number) => isValidTimestamp(time, startDate, endDate) == true,
      ),
    windSensors.data
      .filter(
        (data: WindSensors) =>
          isValidTimestamp(
            parseISOString(data.timestamp),
            startDate,
            endDate,
          ) == true,
      )
      .map((data: WindSensors) => data.windSensors[0].speed),
    windSensors.data
      .filter(
        (data: WindSensors) =>
          isValidTimestamp(
            parseISOString(data.timestamp),
            startDate,
            endDate,
          ) == true,
      )
      .map((data: WindSensors) => data.windSensors[1].speed),
  ];

  return (
    <div className={styles.stats}>
      <div className={styles.heading}>
        <div className={styles.title}>POLARIS IS CURRENTLY:</div>
        <div className={styles.summary}>
          12.5 KNOTS | 278.3° | 37.7749° N, 122.4194° W
        </div>
        <div className={styles.toolbar}>
          <HistoricDataDropdown />
          <RearrangeGraphDropdown />
        </div>
      </div>
      <div className={styles.lineCharts}>
        <LineChart
          data={gpsChartData}
          title='GPS'
          seriesData={[{ label: 'Time' }, { label: 'Speed' }]}
        />
        <LineChart
          data={batteriesVoltageData}
          title='Batteries Voltage'
          seriesData={[
            { label: 'Time' },
            { label: 'Battery 1 Voltage' },
            { label: 'Battery 2 Voltage' },
          ]}
        />
        <LineChart
          data={batteriesCurrentData}
          title='Batteries Current'
          seriesData={[
            { label: 'Time' },
            { label: 'Battery 1 Current' },
            { label: 'Battery 2 Current' },
          ]}
        />
        <LineChart
          data={windSensorsSpeedData}
          title='Wind Sensors'
          seriesData={[
            { label: 'Time' },
            { label: 'Wind Sensor 1 Speed' },
            { label: 'Wind Sensor 2 Speed' },
          ]}
        />
      </div>
    </div>
  );
};

const mapStateToProps = (state: any) => ({
  gps: state.gps,
  batteries: state.batteries,
  windSensors: state.windSensors,
  graphsOrder: state.graphs.order,
  dataFilter: state.dataFilter,
});

const mapDispatchToProps = {};

export default connect(mapStateToProps, mapDispatchToProps)(Stats);
