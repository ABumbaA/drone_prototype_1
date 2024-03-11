# drone_prototype_1
- The goal of this initial prototype is to lay a strong foundation for future enhancements
- This includes not only the drone, but also designing the RF controller.
- Since I am a Mechatronics student (lol), I will be designing and implementing all parts of the drone spanning from mech to software
- The following will be done outside of software:
  1. Design drone frame in Solidworks and 3D print using Bambu Printer. Assemble using fastners and inserts.
  2. Do the same for the RF Controller frame.
  3. Design a PCB for drone (Arduino MEGA shield), and PCB for Controller (Arduino Pro Mini). Solder and check for any shorts, failures, voltage spikes, etc.
  4. Assemble drone with hardware and mechanical. Test that they work in unison.

- After the hardware and mechanical aspects are complete, the software is the biggest hurdle. There will need to be two builds, one for the
- drone and one for the controller. Aspects to be aware of:
  1. SPI Interacing for Radio communication between drone and controller. Also, integratting dual way interfacing
  2. I2C Interfacing on Drone for Gyroscope and temperature/pressure sensor.
  3. Integrating Gyroscope adjustment code.
  4. Creating software for peripherals and safety features.
  5. Developing software for GPS module.
