# Pico Midi Synth



Install `gcc-arm-none-eabi`: https://askubuntu.com/a/1243405 (https://askubuntu.com/questions/1243252/how-to-install-arm-none-eabi-gdb-on-ubuntu-20-04-lts-focal-fossa)


### Openocd

Install raspberrypi version of `openocd` according to Appenix A (p. 58) in the official [getting started guide](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf). Copy `udev` rules `sudo cp contrib/60-openocd.rules /etc/udev/rules.d/` and reload rules `sudo udevadm control --reload`.

Launch `openocd` with `src/openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -s tcl` (from wherever you installed it, eg. `~/pico/openocd` if you follow the original instructions).



## Useful resources 
- [Getting Started with Rust on a Raspberry Pi Pico (Part 3)](https://reltech.substack.com/p/getting-started-with-rust-on-a-raspberry-a88?s=r)
- [hp-rs/rp2040-project-template]https://github.com/rp-rs/rp2040-project-template)