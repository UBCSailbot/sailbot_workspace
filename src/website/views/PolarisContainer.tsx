import React from 'react';
import { connect } from 'react-redux';
import CustomAccordion from './components/DropDown/DropDown';
import Grid from '@mui/material/Grid';
import { styled } from '@mui/material/styles';

const Container = styled('div')(({ theme }) => ({
  flexGrow: 1,
  padding: theme.spacing(2),
  backgroundColor: '#ecf0f1',
}));

const TopHeader = styled('div')(({ theme }) => ({
  backgroundColor: '#2c3e50',
  color: 'white',
  padding: theme.spacing(13),
  display: 'flex',
  justifyContent: 'flex-start',
}));

const Title = styled('span')(({ theme }) => ({
  marginLeft: '700px',
  fontWeight: 'bold',
  fontSize: '3.5em',
}));

const DataSetHeader = styled('div')(({ theme }) => ({
    marginTop: theme.spacing(2.5),
    marginBottom: theme.spacing(0),
    display: 'flex',
    justifyContent: 'center',
    alignItems: 'center',
    fontSize: '2em',
    fontWeight: 'bold',
    color: 'white',
    backgroundColor: '#2c3e50',
    width: '100%',
    padding: `${theme.spacing(50)}px 0`,
  }));

class PolarisContainer extends React.Component {
    render() {
        const voyageInfo = {
            image: "./Polaris.png",
            header: "Polaris",
            desc: "Long description here...",
        };

        return (
            <Container>
                <Grid container spacing={3} justifyContent="center"> {/* Center the grid items */}
                    <Grid item xs={12}>
                        <TopHeader>
                            <Title>{voyageInfo.header}</Title>
                        </TopHeader>
                    </Grid>

                    <Grid item xs={12} style={{ transform: 'translate(400px, -200px)' }}>
                        <img
                            src={voyageInfo.image}
                            alt={voyageInfo.header}
                            style={{
                                width: '22.5%',
                                objectFit: 'cover'
                            }}
                        />
                    </Grid>

                    <Grid item xs={12}>
                        <p style={{ textAlign: 'center' }}>{voyageInfo.desc}</p>
                    </Grid>
                    <Grid item xs={7} style={{ padding: '0px', margin: '0px' }}>
                        <DataSetHeader>Downloadable Data Sets</DataSetHeader>
                    </Grid>
                    <Grid item xs={7} style={{ padding: '0px', margin: '0px' }}>
                        <CustomAccordion title="Download Data Set 1" style={{ marginBottom: '0px', marginTop: '0px' }}>
                            Insert Data sets here
                        </CustomAccordion>
                    </Grid>
                    <Grid item xs={7} style={{ padding: '0px', margin: '0px' }}>
                        <CustomAccordion title="Download Data Set 2" style={{ marginBottom: '0px', marginTop: '0px' }}>
                            Insert Data sets here
                        </CustomAccordion>
                    </Grid>
                    <Grid item xs={7} style={{ padding: '0px', margin: '0px' }}>
                        <CustomAccordion title="Download Data Set 3" style={{ marginBottom: '0px', marginTop: '0px' }}>
                            Insert Data sets here
                        </CustomAccordion>
                    </Grid>
                    <Grid item xs={7} style={{ padding: '0px', margin: '0px' }}>
                        <CustomAccordion title="Download Data Set 4" style={{ marginBottom: '0px', marginTop: '0px' }}>
                            Insert Data sets here
                        </CustomAccordion>
                    </Grid>
                </Grid>
            </Container>
        );
    }
}

export default connect()(PolarisContainer);
