# Virtaboksi

12/24V power switch with home/away switch for use with a solar charge
controller.

![3D model](docs/3d.avif)

In the picture above the Powerpole connector colors are incorrect. Red
is used for 24 volts and yellow for 12 volts, and black for the ground.

## Principle of operation

This unit provides *priority* outputs for 24 V and 12 V
which are on when the switch is in either *home* or *away*
position. *Home* outputs for 24 V and 12 V are on when the switch is
in *home* position.

Battery / charge controller load output is connected to this unit and
it delivers the power to an external 24 volt to 12 volt DC/DC
controller. DC/DC converter is disconnected if the when the switch
position is *off* or if J2 (power good) pins are open.

## How to connect

Connect J2 to NO and COM connectors of your solar charge controller
power good relay pins. If the charge controller can disconnect the
load, you may just short the pins of J2.

Connect J1 to home/away switch. Three position switch is good since it
allows positions off, away, and home. Pin 1 is away, 2 is home and 3
is common. I recommend a shorting rotary switch, since it doesn't
disconnect the load when you change between home and away positions.

Connect J6 pin 1 to DC/DC converter input pin (24V) and pin 2 to
its output pin (12V).

Power connector block J3-J5 from left to right:

1. Ground
2. Battery input 24V
3. Priority output 24V
4. Home output 24V
5. Priority output 12V
6. Home output 12V

## License

The firmware is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

The hardware is provided as source-available: The schematics and PCB
layout are provided for reference only. The files can be freely
distributed without limitations but no derivative works are allowed
without the permission of copyright holders.
