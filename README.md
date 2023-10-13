# Virtaboksi

Virtaboksi is a multi-purpose 3-channel common-earth power switch,
rating 12–48 VDC, 20 A. It has multiple operating modes:

* **Home/away switch** supporting three individual groups and is optimized
  for solar energy off-grid applications.

* **Water pump controller** with pressure sensor input and control for
  combined submersible and pressure pump system. Additionally, a
  throw-over switching capability to drop loads while pumping.

* Generic **Modbus RTU server** (slave) with 3 solid-state "coils", 3 digital
  outputs, and 4 digital inputs.

![3D model](docs/3d.avif)

In the picture above the Powerpole connector colors are incorrect. Red
is used for inputs and black for outputs. In a real product, the three
groups have different colors: red, yellow and blue.

## Design principles

* **Flexible voltage** options; 10–34 V input on first group, 10–60 V on
  second and third group.
* **High current** support. With passive cooling, 20 A loads can be
  controlled. MOSFETs are capable of switching over 100 A continuous
  current in case where cooling and PCB traces are reinforced.
* **Small** size: Thanks to transistor control, the size is way smaller than for
  three contactors of similar rating.
* **Low losses**: Internal resistance <10mΩ.
* **Rugged**: All digital I/O ports are short-circuit, reverse voltage and
  surge protected.

## Operating modes

### Home/away mode

Status: 💚 **Functional**

This unit provides *priority* output on OUT1 when the switch is in
either *home* or *away* position. *Home* controls OUT2 and OUT3 groups
and can be used to control two separate voltages as long as they have
a common ground.

An example use case is a solar off-grid system with two 12V batteries
in series providing 24 volts, plus 12 volts generated from 24V using a
step-down DC converter. DC converter is not included in the package.

In that case this switch can be used to turn the loads partially while
being away, such as keeping refrigerator running but turning off
lights while not present. When using a DC converter, its input is connected
OUT1 its output (12V) to IN3.

Maximum voltage in IN1 is 34 VDC. The limitation comes from the
internal power supply. In case you want to use higher voltages in IN1,
cut the trace F1 on the bottom of this board and power the system
directly from a regulated 3.3 VDC supply via the terminals.

IN2 and IN3 are rated for 60 volts maximum, making it possible to
control 48V nominal voltage loads. Minimum voltage for all groups is
about 10 volts. Under 10 volts, MOSFET control becomes unreliable.

#### How to connect

In home/away mode, when I1 is shorted to the ground, the oprating mode
is *away*. When I2 is shorted, the mode is *home*. If both connectors
are open, operating mode is *off*.

Optionally, LED anodes can be connected directly to O1 and O2. They
represent home and away operating modes, in that order. There is a
current limiter controllable by R1 to limit the maximum current of
O1...O3 to about 15 mA by default, so series resistors are not needed.

If your solar controller has a low battery condition relay, it can be
connected to I3. When the input is high, the system is forced to off mode,
ignoring the state of home/away switch.

I4 is not used in this mode.

### Water pump controller mode

Status: 🔬 **Hardware OK**

Hardware is capable, but no firmware support yet.

### Modbus RTU server mode

Status: 🔬 **Hardware OK**

RS-485 hardware is tested and some firmware tests performed, but no
Modbus implementation exists yet.

## License

The firmware is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

The hardware is provided as source-available: The schematics and PCB
layout are provided for reference only. The files can be freely
distributed without limitations but no derivative works are allowed
without a permission from the copyright holders.
