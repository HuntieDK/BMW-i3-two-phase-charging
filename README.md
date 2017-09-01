# BMW i3 60Ah two-phase charging

This repository contains schematics, PCB layout and Arduino IDE compatible
code for extending the BMW i3 60Ah BEV/REX charging to use two phases
for charging through the type 2 receptable in the car.

## Background

The BMW i3 60Ah is designed to charge with 230V/32A (7.4kW) from a single
phase supply. However often a three-phase 230V/16A supply is available
which can supply 11kW, but this only lets the i3 charge with 230V/16A (3.7kW),
as the i3 can only use one of the phases.

Internally in the car, the 32A current is split between two charging units,
the EME and the KLE. These actually pull up to 16A each, and can, with
modification, be made to charge from two individual phases. In this way,
the car can charge from two phases, 230V at 16A each, in total 7.4kW.

## Implementation

* The wiring in the car must be changed. A new three-phase socket must be
  installed and new wiring must be pulled from the replaced socket to the
  KLE and EME units

* The car must be persuaded to pull what it believes is 32A in order to
  pull 16A on each phase. That is accomplished by the electronics and
  firmware in this repository. It must be inserted in the proximity and
  control pilot signals between the car's charging socket and the charging
  controller, LIM. The main function of the circuit is to signal "32A" over
  the control pilot line whenever the connected EVSE signals "16A" or more.

## Warning

This modification of the BMW i3 60Ah IS NOT SIMPLE! It requires electrical,
mecanical and electronical skills to complete the modification succesfully
in a safe way. The information here is provided "as is" without warranty of
any kind. See attached license.

ANY WORK YOU PERFORM BASED ON THIS DESCRIPTION IS YOUR OWN RESPONSIBILITY!
