from machine import Pin, SPI
import gc
import time

import sdcard_c as sdcard


SD_SCK_PIN = 18
SD_MOSI_PIN = 19
SD_MISO_PIN = 16
SD_CS_PIN = 17
SD_SPI_ID = 0

TEST_BAUDRATES = (8000000, 12000000, 16000000, 20000000, 25000000, 32000000, 40000000)
TRIALS = 3
BLOCKS_PER_READ = 8
TOTAL_BLOCKS = 256


def make_spi(baudrate):
    return SPI(
        SD_SPI_ID,
        baudrate=baudrate,
        sck=Pin(SD_SCK_PIN),
        mosi=Pin(SD_MOSI_PIN),
        miso=Pin(SD_MISO_PIN),
    )


def bench_rate(baudrate):
    ok = 0
    read_kib_s = []
    init_ms = []

    for _ in range(TRIALS):
        spi = make_spi(baudrate)
        started = time.ticks_ms()
        try:
            sd = sdcard.SDCard(spi, Pin(SD_CS_PIN), baudrate=baudrate)
        except Exception as exc:
            print("%8d Hz init failed: %s" % (baudrate, exc))
            continue

        init_ms.append(time.ticks_diff(time.ticks_ms(), started))

        total_blocks = min(TOTAL_BLOCKS, sd.ioctl(4, 0) - BLOCKS_PER_READ)
        if total_blocks < BLOCKS_PER_READ:
            print("%8d Hz card too small for test" % baudrate)
            return

        buf = bytearray(BLOCKS_PER_READ * 512)
        gc.collect()

        started = time.ticks_ms()
        block = 0
        while block < total_blocks:
            sd.readblocks(block, buf)
            block += BLOCKS_PER_READ
        elapsed = time.ticks_diff(time.ticks_ms(), started)
        kib_per_s = ((total_blocks * 512) * 1000) // (max(elapsed, 1) * 1024)

        read_kib_s.append(kib_per_s)
        ok += 1

    if not ok:
        return

    avg_init = sum(init_ms) // len(init_ms)
    avg_read = sum(read_kib_s) // len(read_kib_s)
    best_read = max(read_kib_s)
    print(
        "%8d Hz ok=%d/%d init=%d ms avg=%d KiB/s best=%d KiB/s"
        % (baudrate, ok, TRIALS, avg_init, avg_read, best_read)
    )


def main():
    print("SD card SPI benchmark")
    for baudrate in TEST_BAUDRATES:
        bench_rate(baudrate)


main()