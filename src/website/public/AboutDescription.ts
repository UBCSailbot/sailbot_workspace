const DESCRIPTION = [
  {
    title: 'OUR BOAT',
    content:
      'POLARIS is a 3-metre fully autonomous sailboat built to explore the Pacific Ocean while collecting valuable oceanic and atmospheric data. Designed for multi-day missions, POLARIS brings together engineering and climate research to better understand the changing health of our oceans. From navigating on its own to measuring conditions above and below the water, the boat is built to go where collecting data is difficult. Our goal is to turn that data into something useful, accessible, and capable of supporting ocean conservation.',
    imageSrc: '/images/polaris-boat.jpg',
    imageAlt: 'image of the Polaris boat',
  },
  {
    title: 'MECHANICAL TEAM',
    content:
      'POLARIS’s hull was designed off a Volvo 60 with several considerations for the internal electrical components and external sensors. The hull and deck are composed of a corecell foam core sandwiched between 6 layers of carbon fibre. The deck is filled with solar panels, hatches, sensors, rigging, and much more. With an 8 foot keel and a 20 foot mast, POLARIS’s sail area gives it optimal speed with great directional accuracy. Since POLARIS is a sloop, it has a single mast with a headsail (jib) and a mainsail (main) attached to a boom. ',
    imageSrc: '/images/mech-team.jpg',
    imageAlt: 'Candid image of the mechanical team',
  },
  {
    title: 'ELECTRICAL TEAM',
    content:
      'Our electrical system is designed to emphasize robustness and modularity, allowing us to swap a whole “battery box” or cable assembly out of POLARIS with at most a couple of screws! POLARIS has six 18VDC, 66Ah lithium-ion battery assemblies along with many other related power devices. Our central computing is a mix of ARM-based computing and x86 based computing, which provides a well-supported platform for software development. Both CAN Bus and wireless communications are connected to this assembly.',
    imageSrc: '/images/elec-team.jpg',
    imageAlt: 'Candid image of the electrical team',
  },
  {
    title: 'SOFTWARE TEAM',
    content:
      'POLARIS’s software can be separated into three categories: pathfinding, navigation, and controller. The pathfinding team is responsible for POLARIS’s global and local pathfinding capabilities. Global pathfinding periodically creates sailing paths from the current position to destination with minimized length and desirable wind speeds throughout. Local pathfinding navigates along the global path while avoiding upwind/downwind sailing and minimizing turning and path length. ',
    imageSrc: '/images/soft-team.jpg',
    imageAlt: 'Candid image of the software team',
  },
];

export default DESCRIPTION;
