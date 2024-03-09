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

export interface ILineChartProps {
  data: any[];
  xAxisKey: string;
  yAxisKey: string;
}

export interface ILineChartState {}

export default class LineChartComponent extends React.Component<
  ILineChartProps,
  ILineChartState
> {
  render() {
    const { data, xAxisKey, yAxisKey } = this.props;
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
          <Line type='monotone' dataKey={yAxisKey} stroke='#8884d8' />
        </LineChart>
      </ResponsiveContainer>
    );
  }
}
