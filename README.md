# R-IoT v3


# This is the main repo of the IRCAM R-IoT wirelesss (WIFI) to Open Sound Control IMU module

## UPDATE 25/03/2025 : works great with toolchain 3.1.1 - code size 55% (with OTA) + RAM 24%

The R-IoT v3 is ESP32-S3 based and compiles witht the ESPRESSIF arduino toolchain, currently v3.1.1
To compile with the Arduino IDE, ensure the following toolchain repository is added to the Arduino IDE preferences :
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json


### Compilation target : 
**ESP32-S3 dev module** - USB Mode : USB-OTG / CDC on boot : enabled / DFU on boot : disabled
Upload Mode : USB-OTG(TinyUSB) - CPU frequency : 160 MHz (or 80 MHz) / Flash Mode : QIO 80 MHz
Flash Size : 8 MB / Partition : 8MB with FAT (2MB app / 3.7MB FFAT) - PSRAM : disabled


The toolchain now features a default 8MB flash size partition with OTA (3MB) and a 1.5MB FATFS exposed over USB and combined with OTA updates, which avoids dealing with the toolchain modification.
However, the FW initially targetted to use 3.7MB FATFS and 2MB app, so the /tools/partitions/default_8MB.csv can be modified with the provided partition file that extends the FAT partition to 3.7MB.

To be able to use the 8MB-FATFS partition, a new item must be added in the boards.txt definition.
Search for :
**esp32s3.name=ESP32S3 Dev Module**

###then in the partition scheme menu definition, add:
esp32s3.menu.PartitionScheme.8MB_ota_ffat=8M with OTA and FATFS (2MB APP/3.7MB FATFS)
esp32s3.menu.PartitionScheme.8MB_ota_ffat.build.partitions=default_ffat_8MB
esp32s3.menu.PartitionScheme.8MB_ota_ffat.upload.maximum_size=2097152

The toolchain must be updated with an additional partition table to use the 8MB flash size along with a 3.7 MB FATFS exposed over USB and combined with OTA updates

## Library dependencies
The firmware uses several libraries that have to be separately installed either manually or using the Arduino library manager. 
We tried to limit those by using either local versions of the libraries or custom code & implementation like the low-level drivers of the IMU sensors.

- U8g2lib.h

## Once flashed...
Connect the R-IoT module to computer by USB
It should mount a USB flashdrive of 3.7 MB. Format (FAT) if necessary
Copy all data from the src/data folder in the root dir of the flashdrive
Edit config.txt to your liking. Mandatory settings are the WiFi SSID (network name) and password to connect to


