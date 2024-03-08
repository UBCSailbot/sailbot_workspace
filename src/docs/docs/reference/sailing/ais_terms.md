# AIS Terms

This section explains the most unfamiliar fields that we receive from the AIS.

## MMSI a.k.a ID

A 9-digit, unique identification number for the ship.

## COG: Course over Ground

The direction the boat is travelling, relative to the sea floor. This is the direction of the rate of change
of the [Track Made Good](miscellaneous.md/#track-made-good).

This is measured with the navigational angle convention, where 0° is towards the North, and angles increase in the
clockwise direction. If we make the slight simplification of neglecting the effect of the wind, then

- If the boatspeed is positive and there is no current, the boat's Course over Ground will be the same as the Heading.
- If the boatspeed is zero and there is positive current, the boat's Course over Ground will be the same direction as the
current is flowing.

## SOG: Speed over Ground

The speed the boat is travelling at, relative to the sea floor. This is the magnitude of the rate of change
of the [Track Made Good](miscellaneous.md/#track-made-good).

$\begin{align*}
\text{SoG} &= \left|\frac{d}{dt} \overrightarrow{(\text{Track Made Good})} \right|\\
\end{align*}$

If we make the slight simplification of neglecting the effect of the wind, then

- If the boatspeed is positive and there is no current, the boat's Speed over Ground will be the same as the speed of water
hitting your hand, if you were sitting on the boat and put your hand in the water.
- If the boatspeed is zero and there is positive current, the boat's Speed over Ground will be the same speed as the
current.

## RoT: Rate of Turn

The angular velocity of the boat (how fast it's turning), measured in degrees per minute.
