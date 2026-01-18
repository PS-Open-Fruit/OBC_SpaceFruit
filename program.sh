ELF=build/hardware-test.elf && \
make && \
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "program $ELF verify reset exit" && \
arm-none-eabi-size $ELF | \
awk '
NR==2 {
  flash = $1 + $2;
  ram   = $2 + $3;
  printf "\nFlash used: %d bytes\nRAM used:   %d bytes\n", flash, ram;
}'
