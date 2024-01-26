Start OpenOCD: `openocd -f openocd.cfg`

Load program via telnet: `program target/thumbv7em-none-eabihf/debug/hcu verify reset`

Load gdb-multiarch: `gdb-multiarch -x openocd.gdb target/thumbv7em-none-eabihf/debug/hcu`
