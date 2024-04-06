import React from 'react';
import { Grid, Paper, Typography } from '@mui/material';

interface SingleValueLineProps {
  title: string;
  data: number | string | undefined;
  unit: string;
}

class SingleValueLine extends React.Component<SingleValueLineProps> {
  render() {
    const { title, data, unit } = this.props;

    return (
      <Paper elevation={0}>
        <Grid
          container
          direction='row'
          justifyContent='space-around'
          alignItems='flex-end'
        >
          <Grid item m={1}>
            <Typography align='center' variant='subtitle2'>
              {data ? `${title}: ${data} ${unit}` : `-- ${unit}`}
            </Typography>
          </Grid>
        </Grid>
      </Paper>
    );
  }
}

export default SingleValueLine;
