from machine import Pin, SPI
import os
import sdcard_c as sdcard

spi = SPI(0, baudrate=12000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
sd = sdcard.SDCard(spi, Pin(17), baudrate=12000000)

mounted = False

try:
	os.mount(sd, "/sd")
	mounted = True
	print(os.listdir("/sd"))
finally:
	if mounted:
		try:
			os.umount("/sd")
		except OSError:
			pass
	sd.deinit()
	if hasattr(spi, "deinit"):
		try:
			spi.deinit()
		except OSError:
			pass