import React, { useState } from 'react';
import { connect } from 'react-redux';
import styles from './timestampfilter.module.css';
import { GPSState } from '@/stores/GPS/GPSTypes';
import { DataFilterState } from '@/stores/DataFilter/DataFilterTypes';
import { fmtDate, tzDate } from '../LineChart/UPlotLineChart';
import TimestampFilter from './TimestampFilter';

interface TimestampFilterBtnProps {
  gps: GPSState;
  dataFilter: DataFilterState;
}

const TimestampBtn = ({ gps, dataFilter }: TimestampFilterBtnProps) => {
  const [showMenu, setShowMenu] = useState(false);

  const handleClick = () => {
    setShowMenu(!showMenu);
  };

  const parseISOString = (s: string) => {
    return Math.floor(Date.parse(s) / 1000); // Converts to seconds
  };

  return (
    <div className='menuContainer'>
      <div
        suppressHydrationWarning={true}
        className={showMenu ? styles.iconButtonOpen : styles.iconButton}
        onClick={handleClick}
      >
        {`${
          dataFilter.timestamps.startDate
            ? fmtDate(tzDate(parseISOString(dataFilter.timestamps.startDate)))
            : fmtDate(tzDate(parseISOString(gps.data[0].timestamp)))
        } to ${
          dataFilter.timestamps.endDate
            ? fmtDate(tzDate(parseISOString(dataFilter.timestamps.endDate)))
            : // @ts-ignore
              fmtDate(tzDate(parseISOString(gps.data.at(-1).timestamp)))
        }`}
      </div>
      <div
        className={showMenu ? styles.dropdownActive : styles.dropdownInactive}
      >
        <TimestampFilter />
      </div>
    </div>
  );
};

const mapStateToProps = (state: any) => ({
  gps: state.gps,
  dataFilter: state.dataFilter,
});

const mapDispatchToProps = {};

export default connect(mapStateToProps, mapDispatchToProps)(TimestampBtn);
