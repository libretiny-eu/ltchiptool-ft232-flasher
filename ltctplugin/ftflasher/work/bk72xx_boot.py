#  Copyright (c) Kuba Szczodrzyński 2023-12-7.

from logging import debug, info
from pathlib import Path
from time import sleep
from typing import Callable

from ltchiptool.gui.work.base import BaseThread
from ltchiptool.util.streams import ClickProgressCallback
from pyftdi.gpio import GpioAsyncController, GpioSyncController
from pyftdi.spi import SpiController, SpiGpioPort, SpiPort
from pyftdibb.spi import BitBangSpiController
from spiflash.serialflash import SerialFlashManager, SerialFlashUnknownJedec

from ltctplugin.ftflasher.gpio import CS0
from ltctplugin.ftflasher.types import FtdiMode
from ltctplugin.ftflasher.work.spi_flash_device import SpiFlashDevice


class Bk72xxBootThread(BaseThread):
    callback: ClickProgressCallback
    spi: SpiController

    def __init__(
        self,
        device: str,
        mode: FtdiMode,
        frequency: int,
        gpio: dict[str, int],
        on_chip_info_full: Callable[[list[tuple[str, str]]], None],
        **_,
    ):
        super().__init__()
        self.device = device
        self.mode = mode
        self.frequency = frequency
        self.gpio = gpio
        self.on_chip_info_full = on_chip_info_full

    def run_impl(self):
        self.callback = ClickProgressCallback()
        with self.callback:
            cen = self.gpio.pop("cen")
            cen_low = 0x00
            cen_high = cen_mask = cen_out = 1 << cen

            match self.mode:
                case FtdiMode.SYNC:
                    self.spi = BitBangSpiController(GpioSyncController(), **self.gpio)
                    cs = 0  # only one CS pin configured
                case FtdiMode.ASYNC:
                    self.spi = BitBangSpiController(GpioAsyncController(), **self.gpio)
                    cs = 0  # only one CS pin configured
                case FtdiMode.MPSSE:
                    self.spi = SpiController()
                    cs = self.gpio["cs"] - CS0
                case _:
                    return

            self.spi.configure(url=self.device, frequency=self.frequency)
            port = self.spi.get_port(cs=cs)
            gpio = SpiGpioPort(self.spi)
            gpio.set_direction(pins=cen_mask, direction=cen_out)

            SpiFlashDevice.initialize(
                Path(__file__).parent.with_name("res").joinpath("spi_flash_chips.json")
            )

            self.callback.on_message("Checking flash ID...")
            flash_id = SerialFlashManager.read_jedec_id(port)
            if flash_id == b"\xff\xff\xff":
                flash_id = b"\x00\x00\x00"
            if any(flash_id):
                self._finalize(port, flash_id)
                return

            while self.should_run():
                self.callback.on_message("Rebooting chip...")
                gpio.write(cen_low)
                sleep(0.1)
                gpio.write(cen_high)

                self.callback.on_message("Entering download mode...")
                tx = b"\xd2" * 64
                rx = port.exchange(out=tx, readlen=len(tx), duplex=True)
                debug(f"SPI RX: {rx.hex(' ')}")
                if rx.count(0xD2) == 0:
                    continue

                for i in range(5):
                    self.callback.on_message(f"Checking flash ID ({i + 1})...")
                    sleep(0.2)
                    flash_id = SerialFlashManager.read_jedec_id(port)
                    if flash_id == b"\xff\xff\xff":
                        flash_id = b"\x00\x00\x00"
                    debug(f"Flash ID {i + 1}: {flash_id.hex(' ')}")
                    if any(flash_id):
                        self._finalize(port, flash_id)
                        return

    def _finalize(self, port: SpiPort, flash_id: bytes) -> None:
        info(f"Chip connected, flash ID: {flash_id.hex(' ')}")
        try:
            # noinspection PyProtectedMember
            flash = SerialFlashManager._get_flash(port, flash_id)
        except SerialFlashUnknownJedec:
            flash = None

        chip_info = [
            ("Flash chip JEDEC ID", flash_id.hex(" ").upper()),
            ("Flash chip name", flash and str(flash) or "Unknown"),
        ]
        self.on_chip_info_full(chip_info)

        self.spi.close()

    def stop(self):
        super().stop()
        # try to break flashing & cleanup
        # noinspection PyProtectedMember
        self.spi._ftdi._readbuffer = None
        self.spi._ftdi._writebuffer_chunksize = None
