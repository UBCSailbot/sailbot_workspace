const DESCRIPTION = [
  {
    title: 'OUR BOAT',
    content: 'Raye is an 18 foot fully autonomous sailboat set to sail from Victoria, BC, to Maui, Hawaii, this summer! The boat was designed and built entirely by our Sailbot team, with over 200+ UBC students having worked on Raye in its 6 years of development. With lots of on land and in water testing proving that our initial designs were successful, we are excited to see the team’s further development over the next few months before the launch this summer (2022). ',
    imageSrc: '/images/polaris-boat.jpg',
    imageAlt: 'image of the Polaris boat'
  
  },
  {
    title: 'MECHANICAL TEAM',
    content: 'Raye’s hull was designed off a Volvo 60 with several considerations for the internal electrical components and external sensors. The hull and deck are composed of a corecell foam core sandwiched between 6 layers of carbon fibre. The deck is filled with solar panels, hatches, sensors, rigging, and much more. With an 8 foot keel and a 20 foot mast, Raye’s sail area gives it optimal speed with great directional accuracy. Since Raye is a sloop, it has a single mast with a headsail (jib) and a mainsail (main) attached to a boom. ',
    imageSrc: '/images/mech-team.jpg',
    imageAlt: 'Candid image of the mechanical team'
  },
  {
    title: 'ELECTRICAL TEAM',
    content: 'Our electrical system is designed to emphasize robustness and modularity, allowing us to swap a whole “battery box” or cable assembly out of Raye with at most a couple of screws! Raye has six 18VDC, 66Ah lithium-ion battery assemblies along with many other related power devices. Our central computing is a mix of ARM-based computing and x86 based computing, which provides a well-supported platform for software development. Both CAN Bus and wireless communications are connected to this assembly.',
    imageSrc: '/images/elec-team.jpg',
    imageAlt: 'Candid image of the electrical team'
  },
  {
    title: 'SOFTWARE TEAM',
    content: 'Raye’s software can be separated into three categories: pathfinding, navigation, and controller. The pathfinding team is responsible for Raye’s global and local pathfinding capabilities. Global pathfinding periodically creates sailing paths from the current position to destination with minimized length and desirable wind speeds throughout. Local pathfinding navigates along the global path while avoiding upwind/downwind sailing and minimizing turning and path length. ',
    imageSrc: '/images/soft-team.jpg',
    imageAlt: 'Candid image of the software team'
  },
]

export default DESCRIPTION;
