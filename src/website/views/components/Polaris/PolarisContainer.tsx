import React from 'react';
import { connect } from 'react-redux';
import styles from '@/views/components/Polaris/PolarisContainer.module.css';
import {
  downloadAISShipsData,
  downloadBatteriesData,
  downloadGPSData,
  downloadGenericSensorsData,
  downloadGlobalPathData,
  downloadLocalPathData,
  downloadWindSensorsData,
} from '@/utils/DownloadData.js';
import Dataset from '@/views/components/Dataset/Dataset';

function stringToParagraphs(text: string) {
  return text.split('\n').map((item, key) => (
    <span key={key}>
      {item}
      <br />
      <br />
    </span>
  ));
}



const PolarisContainer = () => {
  const voyageInfo = {
    desc: 'Raye is an 18 foot fully autonomous sailboat set to sail from Victoria, BC, to Maui, Hawaii, this summer! The boat was designed and built entirely by our Sailbot team, with over 200+ UBC students having worked on Raye in its 6 years of development. With lots of on land and in water testing proving that our initial designs were successful, we are excited to see the team’s further development over the next few months before the launch this summer (2022). Being an autonomous sailboat, Raye required a heavy emphasis on a broad range of engineering fields, with the main three being Mechanical, Electrical, and Software engineering. Below is more information on the tasks of each sub-team and how their contribution has been essential to the success of the project.\n Raye’s hull was designed off a Volvo 60 with several considerations for the internal electrical components and external sensors. The hull and deck are composed of a corecell foam core sandwiched between 6 layers of carbon fibre. The deck is filled with solar panels, hatches, sensors, rigging, and much more. With an 8 foot keel and a 20 foot mast, Raye’s sail area gives it optimal speed with great directional accuracy. Since Raye is a sloop, it has a single mast with a headsail (jib) and a mainsail (main) attached to a boom. Each individual sail is controlled by its own winch that is specifically designed for autonomous sailing where there is no crew to retie ropes. The dual rudder system allows Raye to have rudder control even when it heels/tilts to the sides.\n Our electrical system is designed to emphasize robustness and modularity, allowing us to swap a whole “battery box” or cable assembly out of Raye with at most a couple of screws! Raye has six 18VDC, 66Ah lithium-ion battery assemblies along with many other related power devices. Our central computing is a mix of ARM-based computing and x86 based computing, which provides a well-supported platform for software development. Both CAN Bus and wireless communications are connected to this assembly, centralizing internal telemetry, control, and data reporting. This optimizes power consumption while allowing easy communication between the sensors and the main computer. Enhancing this communication is a UCCM, which is a commonly iterated PCB design that can be considered as a “smart, programmable gateway” between any device to the CAN bus.\n Raye’s software can be separated into three categories: pathfinding, navigation, and controller. The pathfinding team is responsible for Raye’s global and local pathfinding capabilities. Global pathfinding periodically creates sailing paths from the current position to destination with minimized length and desirable wind speeds throughout. Local pathfinding navigates along the global path while avoiding upwind/downwind sailing and minimizing turning and path length. In short, global pathfinding finds optimized checkpoints for the boat along the path and local pathfinding tells the boat how to arrive at the checkpoint. The navigation team makes sure that data gets from point A to point B, and through everything in between. For the programming languages, this team uses C++ for efficient low-level communication on the boat itself, but uses Python to process data on the mainland because resource efficiency is not as critical. The controller team is responsible for translating pathfinding commands and sensor data to commands for the rudder and sail. After being given navigation instructions, the controller translates those into commands on how to steer Raye’s rudder and sails to reach the destination. The control team also monitors sensor data from Raye such as power levels and makes adjustments to keep it sailing smoothly.',
  };

  const paragraphs = stringToParagraphs(voyageInfo.desc);

  const customContents = [
    {
      title: 'GPS',
      data: ['GPS', '12 hours'],
      action: downloadGPSData,
    },
    {
      title: 'AIS Ships',
      data: ['AIS Ships', '12 hours'],
      action: downloadAISShipsData,
    },
    {
      title: 'Global Path',
      data: ['Global Path', '12 hours'],
      action: downloadGlobalPathData,
    },
    {
      title: 'Local Path',
      data: ['Local Path', '12 hours'],
      action: downloadLocalPathData,
    },
    {
      title: 'Batteries',
      data: ['Batteries', '12 hours'],
      action: downloadBatteriesData,
    },
    {
      title: 'Wind Sensors',
      data: ['Wind Sensors', '12 hours'],
      action: downloadWindSensorsData,
    },
    {
      title: 'Generic Sensors',
      data: ['Generic Sensors', '12 hours'],
      action: downloadGenericSensorsData,
    },
  ];

  return (
    <div className={styles.container}>
      <div className={styles.spatialCoverageContainer}>
        <div className={styles.spatialCoverageTitle}>Polaris</div>
        <div className={styles.coverageItem}>
          <div>Latitudes</div>
          <div className={styles.coverageContent}>Hello1</div>
        </div>
        <div className={styles.coverageItem}>
          <div>Longitudes</div>
          <div className={styles.coverageContent}>Hello2</div>
        </div>
        <div className={styles.coverageItem}>
          <div>Date Range</div>
          <div className={styles.coverageContent}>Hello3</div>
        </div>
        <div className={styles.coverageItem}>
          <div>Geography</div>
          <div className={styles.coverageContent}>Hello4</div>
        </div>
      </div>
      <p className={styles.description}>{paragraphs}</p>
      <div className={styles.dataSetsContainer}>
        <div className={styles.dataSetHeader}>Access Data Sets</div>
        {customContents.map((content) => (
          <div className={styles.accordionWrapper} key={content.title}>
            <Dataset
              title={content.title}
              content={content.data}
              downloadAction={content.action}
            />
          </div>
        ))}
      </div>
    </div>
  );
}

export default connect()(PolarisContainer);
