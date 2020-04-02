avrdude -c atmelice_isp -p t84 -U lfuse:w:0xE2:m -U hfuse:w:0xDE:m -U efuse:w:0xFE:m
avrdude -c atmelice_isp -p t84 -e -U flash:w:programmer.hex

