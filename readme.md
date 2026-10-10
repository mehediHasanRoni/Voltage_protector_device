Bangladeshi power grid system is not reliable at all. According to specs, it should provide 220 volts, but it provides 280 volts or sometimes more than that. Also, don’t forget the surge current. However, to save our devices from over voltage, we must use some voltage protection device.

There are many ways to detect over voltage and shut down the device. The easiest method is to use a voltage and current measurement using a microcontroller.

In my device, I used a resistor-based voltage drop approach to measure the voltage and a current sensor called ACS712 to measure the current through the appliance.

An AVR microcontroller such as ATmega328P is more than enough for this kind of task. Also, a relay module can be used to cut out the voltage when it is dangerous to pass.

The later part of the project documents explains this with Proteus simulation.

I also added the complete schematic design, which was developed as a complete module.