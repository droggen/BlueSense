rem avrdude -p atmega1284p -c avrispmkii -P usb -B .4 -D -V -U flash:w:mainbl.hex:i 
avrdude -p atmega1284p -c avrispmkii -P usb -B .4 -D -V -U flash:w:booter.hex:i 
