
Sketch for Arduino Nano 3.0 (CH340 - China) and board STM32F103C8T6 (Blue Pill)

rtl8367c.ino: VLAN switch setup based on Realtek RTL8367C(S) chips.
Used TP-Link TL-SG1005D rev.7 (RTL8367S) after remove EEPROM chip from board switch!

Copyright (C) 2017 McMCC <mcmcc_at_mail_ru>

For ARDUINO AVR:

Remove EEPROM 24cXX on switch board!!!!
Pin SDA - D6 (convertor 5v-to-3.3v pin SDA on EEPROM switch(pin 5))
Pin SCK - D5 (convertor 5v-to-3.3v pin SCK on EEPROM switch(pin 6))
Arduino Vin - get from power connector +12V
+5V - get from Arduino 5V
+3.3V - get from EEPROM pin 8 on switch board

For ARDUINO STM32:

No need Logic Level Convertor, use pin-to-pin conection.
Remove EEPROM 24cXX on switch board!!!!
Cut on the board STM32 the track from power USB!!!!
+3.3V get from EEPROM pin 8 on switch board.
GND get from EEPROM pin 1-4 on switch board.

Pin SDA - PB10 (to EEPROM pin 5 on switch board)
Pin SCK - PB11 (to EEPROM pin 6 on switch board)
