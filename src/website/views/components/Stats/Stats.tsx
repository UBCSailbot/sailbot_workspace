'use client';

import { useLayoutEffect, useState } from 'react';
import LineChart from '../LineChart/LineChart';
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

const speedGraph = (speedChartData: any) => {
  return (
    <LineChart
      key='speedChartData'
      data={speedChartData}
      title='Speed'
      seriesData={[{ label: 'Time' }, { label: 'Speed', unit: 'km/hr', stroke: 'blue'}]}
    />
  );
};

const BatteriesVoltageGraph = (batteriesVoltageData: any) => {
  return (
    <LineChart
      key='batteriesVoltageData'
      data={batteriesVoltageData}
      title='Batteries Voltage'
      seriesData={[
        { label: 'Time' },
        { label: 'Battery 1 Voltage', unit: 'V', stroke: 'blue' },
        { label: 'Battery 2 Voltage', unit: 'V', stroke: 'red' },
      ]}
    />
  );
};

const BatteriesCurrentGraph = (batteriesCurrentData: any) => {
  return (
    <LineChart
      key='batteriesCurrentData'
      data={batteriesCurrentData}
      title='Batteries Current'
      seriesData={[
        { label: 'Time' },
        { label: 'Battery 1 Current', unit: 'A', stroke: 'blue' },
        { label: 'Battery 2 Current', unit: 'A', stroke: 'red' },
      ]}
    />
  );
};

const WindSensorsSpeedGraph = (windSensorsSpeedData: any) => {
  return (
    <LineChart
      key='windSensorsSpeedData'
      data={windSensorsSpeedData}
      title='Wind Sensors'
      seriesData={[
        { label: 'Time' },
        { label: 'Wind Sensor 1 Speed', unit: 'm/s', stroke: 'blue' },
        { label: 'Wind Sensor 2 Speed', unit: 'm/s', stroke: 'red' },
      ]}
    />
  );
};

const getStatsSummary = (filteredGpsData: GPS[]) => {
  if (!filteredGpsData.length) {
    return 'NO DATA';
  }

  const lastValidGpsData = filteredGpsData[filteredGpsData.length - 1];
  return `${lastValidGpsData.speed} KM/HR | ${lastValidGpsData.heading}° | ${lastValidGpsData.latitude}° N, ${lastValidGpsData.longitude}° W`;
};

interface StatsProps {
  gps: GPSState;
  batteries: BatteriesState;
  windSensors: WindSensorsState;
  graphsOrder: string[];
  dataFilter: DataFilterState;
}

// make this cleaner later
const Stats = ({
  gps,
  batteries,
  windSensors,
  graphsOrder,
  dataFilter,
}: StatsProps) => {
  const [summary, setSummary] = useState<string>('LOADING...');

  const startDate = dataFilter.timestamps.startDate;
  const endDate = dataFilter.timestamps.endDate;

  useLayoutEffect(() => {
    const filteredGpsData = gps.data.filter(
      (data: GPS) =>
        isValidTimestamp(parseISOString(data.timestamp), startDate, endDate) ===
        true,
    );

    setSummary(getStatsSummary(filteredGpsData));
  }, [gps.data, startDate, endDate]);

  const speedChartData = [
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

  const graphsMap = {
    GPS: speedGraph(speedChartData),
    BatteriesVoltage: BatteriesVoltageGraph(batteriesVoltageData),
    BatteriesCurrent: BatteriesCurrentGraph(batteriesCurrentData),
    WindSensors: WindSensorsSpeedGraph(windSensorsSpeedData),
  };

  const graphs = graphsOrder.map(
    (graph: string) => graphsMap[graph as keyof typeof graphsMap],
  );

  return (
    <div className={styles.stats}>
      <div className={styles.heading}>
        <div className={styles.title}>POLARIS IS CURRENTLY:</div>
        <div className={styles.summary}>{summary}</div>
        <div className={styles.toolbar}>
          <HistoricDataDropdown />
          <RearrangeGraphDropdown />
        </div>
      </div>
      <div className={styles.lineCharts}>{graphs}</div>
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
