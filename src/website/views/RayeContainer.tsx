import React from 'react';
import { connect } from 'react-redux';
import CustomAccordion from './components/DropDown/DropDown';
import { styled } from '@mui/material/styles';
import {
  downloadGPSData,
  downloadAISShipsData,
  downloadGlobalPathData,
  downloadLocalPathData,
  downloadBatteriesData,
  downloadWindSensorsData,
  downloadGenericSensorsData
} from './components/DownloadData/DownloadData';

const textAndTitleColor = '#092e4e';

const Container = styled('div')(({ theme }) => ({
  flexGrow: 1,
  padding: theme.spacing(0),
  display: 'flex',
  flexDirection: 'column',
  alignItems: 'center', 
  fontFamily: 'Switzer, sans-serif',
  color: textAndTitleColor,
}));

const Description = styled('p')(({ theme }) => ({
  textAlign: 'left',
  maxWidth: '800px',
  margin: '20px 20px',
}));

const DataSetsContainer = styled('div')(({ theme }) => ({
  width: '800px', 
  display: 'flex',
  flexDirection: 'column',
}));

const DataSetHeader = styled('div')(({ theme }) => ({
  display: 'flex',
  justifyContent: 'center',
  alignItems: 'center',
  fontSize: '1.5em',
  fontWeight: 'bold',
  color: 'white',
  backgroundColor: '#00263e',
  padding: '20px 0',
}));

const AccordionWrapper = styled('div')(({ theme }) => ({
  marginBottom: '0px', 
  '&:last-child': {
    marginBottom: '0',
  },
}));

const SpatialCoverageContainer = styled('div')(({ theme }) => ({
  display: 'flex',
  flexDirection: 'row',
  flexWrap: 'wrap',
  justifyContent: 'flex-start',
  alignItems: 'flex-start',
  width: '800px',
  margin: '20px',
  borderBottom: '1px solid black',
  borderTop: '1px solid black',
  paddingBottom: '20px',
  paddingTop: '45px', 
  color: textAndTitleColor,
}));

const SpatialCoverageTitle = styled('div')(({ theme }) => ({
  width: '100%',
  fontWeight: 'bold',
  marginBottom: theme.spacing(4),
}));

const CoverageItem = styled('div')(({ theme }) => ({
  marginRight: '20px',
  marginBottom: theme.spacing(-1.5),
  display: 'flex',
  flexDirection: 'column',
  gap: theme.spacing(3),
}));

const CoverageContent = styled('div')(({ theme }) => ({
  marginBottom: theme.spacing(4), 
}));

function stringToParagraphs(text) {
  return text.split('\n').map((item, key) => (
    <span key={key}>{item}<br/><br/></span>
  ));
}
class RayeContainer extends React.Component {
  render() {
    
    const voyageInfo = {
      desc: "Raye is an 18 foot fully autonomous sailboat set to sail from Victoria, BC, to Maui, Hawaii, this summer! The boat was designed and built entirely by our Sailbot team, with over 200+ UBC students having worked on Raye in its 6 years of development. With lots of on land and in water testing proving that our initial designs were successful, we are excited to see the team’s further development over the next few months before the launch this summer (2022). Being an autonomous sailboat, Raye required a heavy emphasis on a broad range of engineering fields, with the main three being Mechanical, Electrical, and Software engineering. Below is more information on the tasks of each sub-team and how their contribution has been essential to the success of the project.\n Raye’s hull was designed off a Volvo 60 with several considerations for the internal electrical components and external sensors. The hull and deck are composed of a corecell foam core sandwiched between 6 layers of carbon fibre. The deck is filled with solar panels, hatches, sensors, rigging, and much more. With an 8 foot keel and a 20 foot mast, Raye’s sail area gives it optimal speed with great directional accuracy. Since Raye is a sloop, it has a single mast with a headsail (jib) and a mainsail (main) attached to a boom. Each individual sail is controlled by its own winch that is specifically designed for autonomous sailing where there is no crew to retie ropes. The dual rudder system allows Raye to have rudder control even when it heels/tilts to the sides.\n Our electrical system is designed to emphasize robustness and modularity, allowing us to swap a whole “battery box” or cable assembly out of Raye with at most a couple of screws! Raye has six 18VDC, 66Ah lithium-ion battery assemblies along with many other related power devices. Our central computing is a mix of ARM-based computing and x86 based computing, which provides a well-supported platform for software development. Both CAN Bus and wireless communications are connected to this assembly, centralizing internal telemetry, control, and data reporting. This optimizes power consumption while allowing easy communication between the sensors and the main computer. Enhancing this communication is a UCCM, which is a commonly iterated PCB design that can be considered as a “smart, programmable gateway” between any device to the CAN bus.\n Raye’s software can be separated into three categories: pathfinding, navigation, and controller. The pathfinding team is responsible for Raye’s global and local pathfinding capabilities. Global pathfinding periodically creates sailing paths from the current position to destination with minimized length and desirable wind speeds throughout. Local pathfinding navigates along the global path while avoiding upwind/downwind sailing and minimizing turning and path length. In short, global pathfinding finds optimized checkpoints for the boat along the path and local pathfinding tells the boat how to arrive at the checkpoint. The navigation team makes sure that data gets from point A to point B, and through everything in between. For the programming languages, this team uses C++ for efficient low-level communication on the boat itself, but uses Python to process data on the mainland because resource efficiency is not as critical. The controller team is responsible for translating pathfinding commands and sensor data to commands for the rudder and sail. After being given navigation instructions, the controller translates those into commands on how to steer Raye’s rudder and sails to reach the destination. The control team also monitors sensor data from Raye such as power levels and makes adjustments to keep it sailing smoothly.",
    };

    const paragraphs = stringToParagraphs(voyageInfo.desc);
    
    const customContents = [
      { title: "GPS", data: ["GPS", "12 hours", "JSON"], action: downloadGPSData },
      { title: "AIS Ships", data: ["AIS Ships", "12 hours", "JSON"], action: downloadAISShipsData },
      { title: "Global Path", data: ["Global Path", "12 hours", "JSON"], action: downloadGlobalPathData },
      { title: "Local Path", data: ["Local Path", "12 hours", "JSON"], action: downloadLocalPathData },
      { title: "Batteries", data: ["Batteries", "12 hours", "JSON"], action: downloadBatteriesData },
      { title: "Wind Sensors", data: ["Wind Sensors", "12 hours", "JSON", "Download"], action: downloadWindSensorsData },
      { title: "Generic Sensors", data: ["Generic Sensors", "12 hours", "JSON", "Download"], action: downloadGenericSensorsData }
    ];

    return (
      <Container>
        <SpatialCoverageContainer>
          <SpatialCoverageTitle>Spatial Coverage</SpatialCoverageTitle>
          <CoverageItem>
            <div>Latitudes</div>
            <CoverageContent>Hello1</CoverageContent>
          </CoverageItem>
          <CoverageItem>
            <div>Longitudes</div>
            <CoverageContent>Hello2</CoverageContent>
          </CoverageItem>
          <CoverageItem>
            <div>Date Range</div>
            <CoverageContent>Hello3</CoverageContent>
          </CoverageItem>
          <CoverageItem>
            <div>Geography</div>
            <CoverageContent>Hello4</CoverageContent>
          </CoverageItem>
        </SpatialCoverageContainer>
        <Description>{paragraphs}</Description>
        <DataSetsContainer>
          <DataSetHeader>Access Data Sets</DataSetHeader>
          {customContents.map((content) => (
            <AccordionWrapper key={content.title}>
              <CustomAccordion
                title={content.title}
                content={content.data}
                downloadAction={content.action}
              />
            </AccordionWrapper>
          ))}
        </DataSetsContainer>
      </Container>
    );
  }
}


export default connect()(RayeContainer);

