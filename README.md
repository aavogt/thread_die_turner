# Thread die gear motor

See the [wiki](https://github.com/aavogt/thread_die_turner/wiki/Hardware) about hardware.

## build/flash

[install esp-idf into ~/esp/esp-idf using these instructions](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html). Connect the usb cable to the esp32c3 dev board, then run

        cd ~/esp
        git clone https://github.com/aavogt/thread_die_turner.git
        cd thread_die_turner
        . ~/esp/esp-idf/export.sh
        idf.py build flash monitor

