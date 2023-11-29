# Thread die gear motor

https://github.com/aavogt/thread_die_turner/raw/master/thread_die.webm

A gear motor turns a thread die. When I push lightly it rotates slowly. This allows the threads to engage. Then I push harder, which brings the motor up to full speed and it reverses after a fixed time specified in `main/main.c`.

Materials:

  - LuatOS esp32c3
  - HX711 and 5kg load cell
  - YP8/AMS1117 5V regulator board
    - 2 pole dip switch or some other way to disconnect the dev board if you use USB power
  - L298N board
  - 32GP31ZY 24V 1544RPM gear motor; 5840-31ZY has more torque but the bearing is looser
  - 24V >2A power supply
    - matching DC Female connector
  - M10 thread die and a 3D printed ![die holder](http://aavogt.github.io/die_holder/)

Start with a LuatOS esp32c3 dev board. Connect the HX711 SCK and DT to GPIO10 and 6 respectively. Connect GPIO 0 and 19 through optocouplers to the IN1 and IN2 of the L298N board. The dev board can be powered from the DC if the DIP switches are ON.

## software

[install esp-idf into ~/esp/esp-idf using these instructions](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html). Connect the usb cable, then run

        cd ~/esp
        git clone https://github.com/aavogt/thread_die_turner.git
        cd thread_die_turner
        . ~/esp/esp-idf/export.sh
        idf.py build flash monitor
