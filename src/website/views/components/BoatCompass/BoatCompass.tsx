import React from 'react';
import Image from 'next/image';
import { Grid, Typography } from '@mui/material';
import styles from './boatcompass.module.css';

interface BoatCompassProps {
  angle: number | undefined;
}

class BoatCompass extends React.Component<BoatCompassProps> {
  render() {
    const { angle } = this.props;

    const validAngle = typeof angle !== 'undefined' ? angle : null;

    return (
      <Grid
        container
        direction='row'
        justifyContent='center'
        alignItems='center'
        width={80}
        height={80}
      >
        <Grid className={styles.top}>
          <Typography align='center' variant='subtitle2'>
            {`${this._rotateBoat(validAngle).toFixed()}°`}
          </Typography>
        </Grid>
        <Image
          src='/BoatIconFinal.png'
          width={60}
          height={60}
          alt='Boat Icon'
          style={{ transform: this._rotateBoatString(validAngle) }}
          className={styles.middle}
        />
        <Image
          src='/NSEWCompassBackdrop.png'
          width={90}
          height={90}
          alt='Compass Background'
          className={styles.bottom}
        />
      </Grid>
    );
  }

  _rotateBoat(boatAngle: number | null): number {
    if (boatAngle == null) {
      return 0;
    } else {
      return boatAngle < 0 ? boatAngle + 360 : boatAngle;
    }
  }

  _rotateBoatString(boatAngle: number | null): string {
    if (boatAngle == null) {
      return `rotate(0deg)`;
    } else {
      return `rotate(${this._rotateBoat(boatAngle)}deg)`;
    }
  }
}

export default BoatCompass;
