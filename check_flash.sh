ELF=build/hardware-test.elf && \
make && \
arm-none-eabi-size $ELF | \
awk '
NR==2 {
  flash = $1 + $2;
  ram   = $2 + $3;
  printf "\nFlash used: %d bytes\nRAM used:   %d bytes\n", flash, ram;
}'
