## Building and Flashing

Run these two commands while in the project folder to build and flash the code to the board:

```bash
cargo build --target thumbv7em-none-eabihf
cargo flash --chip STM32F407VG --release
