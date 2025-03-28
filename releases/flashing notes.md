
# Flashing Guide
** How to manually flash your R-IoT board from the command line: **
- Turn your module into bootloader mode using reset + flash switches (or short PIX / GPIO #0)
- Identify your module's COM port in the device manager or in /dev
- Replace the COM port in the command line below


## Flash only necessary program memory locations (boot, bootloader, app)
esptool.exe --chip esp32s3 --port COM8 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode keep --flash_freq keep --flash_size keep 0x0 Riot-v3.18.ino.bootloader.bin 0x8000 Riot-v3.18.ino.partitions.bin 0xe000 boot_app0.bin 0x10000 Riot-v3.18.ino.bin 

## Flash the whole chip (8MB, takes longer)
esptool.exe --chip esp32s3 --port COM8 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode keep --flash_freq keep --flash_size keep 0x0 Riot-v3.18.ino.merged.bin

** How to flash from the Arduino IDE **
Compile the code, then go in the tools menu and select the COM port of your board
Click the upload icon to flash the board

** How to update the fw using the release .bin file: **
copy the firmware (about 1MB) to the USB flashdrive exposed by the R-IoT
rename the file to update.bin
reset the board or power cycle it - the onboard pixel will flash red then white before blinking at the end of the update which 
will take a few seconds.


** After flashing, reset the module and copy the files provided in /data to the root of the exposed USB flashdrive of the R-IoT **
Those files contain the default configuration file, help file and webserver HTML / JS / CSS files for web configuration and OTA updates
Reset the module again to apply changes. Edit the configuration file to match your WIFI ssid and credentials

