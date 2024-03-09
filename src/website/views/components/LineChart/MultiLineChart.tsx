import React from 'react';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
} from 'recharts';

export interface MultiLineChartProps {
  data: any[];
  xAxisKey: string;
  yAxisKey1: string;
  yAxisKey2: string;
}

export interface MultiLineChartState {}

export default class MultiLineChartComponent extends React.Component<
  MultiLineChartProps,
  MultiLineChartState
> {
  render() {
    const { data, xAxisKey, yAxisKey1, yAxisKey2 } = this.props;
    return (
      <ResponsiveContainer width='100%' height={250}>
        <LineChart
          data={data}
          margin={{ top: 5, right: 30, left: 20, bottom: 5 }}
        >
          <CartesianGrid strokeDasharray='3 3' />
          <XAxis dataKey={xAxisKey} />
          <YAxis />
          <Tooltip />
          <Legend />
          <Line type='monotone' dataKey={yAxisKey1} stroke='#8884d8' />
          <Line type='monotone' dataKey={yAxisKey2} stroke='#82ca9d' />
        </LineChart>
      </ResponsiveContainer>
    );
  }
}
