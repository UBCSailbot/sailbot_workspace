import React from 'react';
import { connect } from 'react-redux';
import CustomAccordion from './components/DropDown/DropDown';
import { styled } from '@mui/material/styles';

const Container = styled('div')(({ theme }) => ({
  flexGrow: 1,
  padding: theme.spacing(0),
  display: 'flex',
  flexDirection: 'column',
  alignItems: 'center', 
}));

const TopHeader = styled('div')(({ theme }) => ({
  padding: '20px', 
  display: 'flex',
  justifyContent: 'center',
  alignItems: 'center',
  background: 'linear-gradient(to right, #26619c, #3498db)',
  color: 'white',
  width: '100%',
}));

const Title = styled('span')(() => ({
  fontFamily: 'Verdana, Geneva, sans-serif',
  fontWeight: 'bold',
  fontSize: '48px',
  color: 'white',
  letterSpacing: '2px',
}));

const Logo = styled('img')(() => ({
  width: '300px',
  marginRight: '40px', 
  border: '2px solid black', 
}));

const Description = styled('p')(() => ({
  textAlign: 'center',
  maxWidth: '800px',
  margin: '180px 20px 180px', 
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
  backgroundColor: '#2c3e50',
  padding: '20px 0',
}));

const AccordionWrapper = styled('div')(() => ({
  marginBottom: '0px', 
  '&:last-child': {
    marginBottom: '0',
  },
}));

function stringToParagraphs(text) {
  return text.split('\n').map((item, key) => {
    return <span key={key}>{item}<br/><br/></span>;
  });
}

class RayeContainer extends React.Component {
  render() {
    const voyageInfo = {
      image: "./Polaris.png",
      header: "Raye",
      desc: "Raye is an 18 foot fully autonomous sailboat set to sail from Victoria, BC, to Maui, Hawaii, this summer! The boat was designed and built entirely by our Sailbot team, with over 200+ UBC students having worked on Raye in its 6 years of development. With lots of on land and in water testing proving that our initial designs were successful, we are excited to see the team’s further development over the next few months before the launch this summer (2022). Being an autonomous sailboat, Raye required a heavy emphasis on a broad range of engineering fields, with the main three being Mechanical, Electrical, and Software engineering. Below is more information on the tasks of each sub-team and how their contribution has been essential to the success of the project.\n Raye’s hull was designed off a Volvo 60 with several considerations for the internal electrical components and external sensors. The hull and deck are composed of a corecell foam core sandwiched between 6 layers of carbon fibre. The deck is filled with solar panels, hatches, sensors, rigging, and much more. With an 8 foot keel and a 20 foot mast, Raye’s sail area gives it optimal speed with great directional accuracy. Since Raye is a sloop, it has a single mast with a headsail (jib) and a mainsail (main) attached to a boom. Each individual sail is controlled by its own winch that is specifically designed for autonomous sailing where there is no crew to retie ropes. The dual rudder system allows Raye to have rudder control even when it heels/tilts to the sides.\n Our electrical system is designed to emphasize robustness and modularity, allowing us to swap a whole “battery box” or cable assembly out of Raye with at most a couple of screws! Raye has six 18VDC, 66Ah lithium-ion battery assemblies along with many other related power devices. Our central computing is a mix of ARM-based computing and x86 based computing, which provides a well-supported platform for software development. Both CAN Bus and wireless communications are connected to this assembly, centralizing internal telemetry, control, and data reporting. This optimizes power consumption while allowing easy communication between the sensors and the main computer. Enhancing this communication is a UCCM, which is a commonly iterated PCB design that can be considered as a “smart, programmable gateway” between any device to the CAN bus.\n Raye’s software can be separated into three categories: pathfinding, navigation, and controller. The pathfinding team is responsible for Raye’s global and local pathfinding capabilities. Global pathfinding periodically creates sailing paths from the current position to destination with minimized length and desirable wind speeds throughout. Local pathfinding navigates along the global path while avoiding upwind/downwind sailing and minimizing turning and path length. In short, global pathfinding finds optimized checkpoints for the boat along the path and local pathfinding tells the boat how to arrive at the checkpoint. The navigation team makes sure that data gets from point A to point B, and through everything in between. For the programming languages, this team uses C++ for efficient low-level communication on the boat itself, but uses Python to process data on the mainland because resource efficiency is not as critical. The controller team is responsible for translating pathfinding commands and sensor data to commands for the rudder and sail. After being given navigation instructions, the controller translates those into commands on how to steer Raye’s rudder and sails to reach the destination. The control team also monitors sensor data from Raye such as power levels and makes adjustments to keep it sailing smoothly.",
    };
    
    const paragraphs = stringToParagraphs(voyageInfo.desc);
   
    return (
      <Container>
        <TopHeader>
          <Logo src={voyageInfo.image} alt="Logo" />
          <Title>{voyageInfo.header}</Title>
        </TopHeader>
        <Description>{paragraphs}</Description>
        <DataSetsContainer>
          <DataSetHeader>Downloadable Data Sets</DataSetHeader>
          {[1, 2, 3, 4].map((index) => (
            <AccordionWrapper key={index}>
              <CustomAccordion title={`Download Data Set ${index}`}>
                  Insert Data sets here
              </CustomAccordion>
            </AccordionWrapper>
          ))}
        </DataSetsContainer>
      </Container>
    );
  }
}

export default connect()(RayeContainer);

