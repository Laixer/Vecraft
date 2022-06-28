target extended-remote :3333

# monitor tpiu config internal /tmp/itm.fifo uart off 2000000
# monitor itm port 0 on

# Print demangled symbols
set print asm-demangle on

# Detect unhandled exceptions, hard faults and panics
break DefaultHandler
break HardFault
break rust_begin_unwind
# break main

# Enable semihosting
# monitor arm semihosting enable

#load

# start the process but immediately halt the processor
#stepi
continue
