#  Copyright (c) Kuba Szczodrzyński 2023-8-31.

from logging import info, warning
from pathlib import Path

import wx.adv
import wx.xrc
from ltchiptool.gui.mixin.devices import DevicesBase
from ltchiptool.gui.mixin.file_dump import FileDumpBase
from ltchiptool.gui.utils import int_or_zero, on_event
from pyftdi.ftdi import Ftdi
from pyftdi.usbtools import UsbTools

from .gpio import CS0, MISO, MOSI, SCK, GpioChooserPanel
from .types import FtdiMode, ProtocolType, SpiOperation


# noinspection PyPep8Naming
class FlasherPanel(FileDumpBase, DevicesBase):
    prev_file: Path | None = None
    auto_file: Path | None = None
    prev_state: tuple[bool, bool, bool] = None
    devices: list[tuple[str, str, bool]]
    last_device: str | None = None

    def __init__(self, parent: wx.Window, frame):
        super().__init__(parent, frame)
        self.LoadXRCFile("ftflasher.xrc")
        self.LoadXRC("FlasherPanel")
        self.AddToNotebook("FT232 Flasher")

        self.devices = []

        self.Device = self.BindComboBox("combo_device")
        self.Rescan = self.BindButton("button_rescan", self.OnRescanClick)

        self.FileText = self.FindStaticText("text_file")
        self.File = self.BindTextCtrl("input_file")
        self.Browse = self.BindButton("button_browse", self.OnBrowseClick)
        self.Start = self.BindCommandButton("button_start", self.OnStartClick)
        self.Cancel = self.BindCommandButton("button_cancel", self.OnCancelClick)

        self.Frequency = {
            self.BindRadioButton("radio_freq_1"): {
                FtdiMode.SYNC: 100_000,
                FtdiMode.ASYNC: 57600,
                FtdiMode.MPSSE: 500_000,
            },
            self.BindRadioButton("radio_freq_2"): {
                FtdiMode.SYNC: 500_000,
                FtdiMode.ASYNC: 115200,
                FtdiMode.MPSSE: 1_000_000,
            },
            self.BindRadioButton("radio_freq_3"): {
                FtdiMode.SYNC: 1_000_000,
                FtdiMode.ASYNC: 230400,
                FtdiMode.MPSSE: 1_500_000,
            },
            self.BindRadioButton("radio_freq_4"): {
                FtdiMode.SYNC: 1_500_000,
                FtdiMode.ASYNC: 460800,
                FtdiMode.MPSSE: 2_000_000,
            },
            self.BindRadioButton("radio_freq_5"): {
                FtdiMode.SYNC: 2_000_000,
                FtdiMode.ASYNC: 921600,
                FtdiMode.MPSSE: 6_000_000,
            },
        }

        self.Offset = self.BindTextCtrl("input_offset")
        self.SkipText = self.FindStaticText("text_skip")
        self.Skip = self.BindTextCtrl("input_skip")
        self.LengthText = self.FindStaticText("text_length")
        self.Length = self.BindTextCtrl("input_length")

        self.File.Bind(wx.EVT_KILL_FOCUS, self.OnBlur)
        self.Cancel.SetNote("")

        self.Modes = {
            FtdiMode.SYNC: self.BindRadioButton("radio_mode_sync"),
            FtdiMode.ASYNC: self.BindRadioButton("radio_mode_async"),
            FtdiMode.MPSSE: self.BindRadioButton("radio_mode_mpsse"),
        }
        self.Notebook: wx.Notebook = self.BindWindow("notebook")

        self.SpiPage: wx.NotebookPage = self.BindWindow("page_spi")
        self.SpiGpio = GpioChooserPanel(
            parent=self.SpiPage,
            frame=frame,
            names=["SCK", "MOSI", "MISO", "CS"],
            labels=["SCK / F_SCK", "MOSI / F_SI", "MISO / F_SO", "CS / F_CS"],
            default=[SCK, MOSI, MISO, CS0],
        )
        self.SpiPage.GetSizer().Insert(0, self.SpiGpio, flag=wx.EXPAND)
        self.SpiOperations = {
            SpiOperation.READ_ID: self.BindRadioButton("radio_spi_read_id"),
            SpiOperation.READ: self.BindRadioButton("radio_spi_read"),
            SpiOperation.WRITE: self.BindRadioButton("radio_spi_write"),
            SpiOperation.ERASE: self.BindRadioButton("radio_spi_erase"),
        }

        self.Protocols = {
            ProtocolType.SPI: self.SpiPage,
        }

        self.EnableFileDrop()

        get_string = UsbTools.get_string

        def get_string_safe(*args, **kwargs):
            try:
                return get_string(*args, **kwargs)
            except NotImplementedError:
                return ""

        UsbTools.get_string = get_string_safe

    def GetSettings(self) -> dict:
        return dict(
            device=self.device or self.last_device,
            mode=self.mode.value,
            frequency=self.frequency,
            protocol=self.protocol.value,
            offset=self.offset,
            skip=self.skip,
            length=self.length,
            spi=dict(
                gpio=self.spi_gpio,
                operation=self.spi_operation.value,
            ),
            **self.GetFileSettings(),
        )

    def SetSettings(
        self,
        device: str = None,
        mode: str = None,
        frequency: int = None,
        protocol: str = None,
        spi: dict = None,
        offset: int = None,
        skip: int = None,
        length: int | None = None,
        **kwargs,
    ) -> None:
        if device:
            self.device = device
        if mode:
            self.mode = FtdiMode(mode)
        if frequency:
            self.frequency = frequency
        if protocol:
            self.protocol = ProtocolType(protocol)
        self.offset = offset
        self.skip = skip
        self.length = length
        if spi:
            if "gpio" in spi:
                self.spi_gpio = spi["gpio"]
            if "operation" in spi:
                self.spi_operation = SpiOperation(spi["operation"])
        self.SetFileSettings(**kwargs)

    def OnActivate(self) -> None:
        self.StartDeviceWatcher()

    def OnDeactivate(self) -> None:
        self.StopDeviceWatcher()

    def OnUpdate(self, target: wx.Window = None) -> None:
        mode = self.mode
        reading = self.is_reading
        writing = self.is_writing
        erasing = self.is_erasing
        new_state = (reading, writing, erasing)

        for button, speeds in self.Frequency.items():
            speed = speeds[mode]
            button.SetLabel(f"{speed:,}".replace(",", " "))

        if self.prev_state != new_state:
            if reading:
                # generate a new filename for reading, to prevent
                # accidentally overwriting firmware files
                self.generate_read_filename()
            elif writing:
                # restore filename previously used for writing
                self.restore_write_filename()

        self.Offset.Enable(reading or writing or erasing)
        self.SkipText.Enable(writing)
        self.Skip.Enable(writing)
        self.Length.Enable(reading or writing or erasing)
        self.File.Enable(reading or writing)
        self.Browse.Enable(reading or writing)

        errors = []
        warnings = []

        if writing:
            self.FileText.SetLabel("Input file")
            self.LengthText.SetLabel("Writing length")
            if not self.file:
                errors.append("Choose an input file")
            elif not self.file.is_file():
                errors.append("File does not exist")
            else:
                size = self.file.stat().st_size
                if self.skip >= size:
                    errors.append(
                        f"Skip offset (0x{self.skip:X}) "
                        f"not within input file bounds "
                        f"(0x{size:X})"
                    )
                elif self.skip + (self.length or 0) > size:
                    errors.append(
                        f"Writing length (0x{self.skip:X} + 0x{self.length:X}) "
                        f"not within input file bounds "
                        f"(0x{size:X})"
                    )
        elif reading:
            self.FileText.SetLabel("Output file")
            self.LengthText.SetLabel("Reading length")
            if not self.file:
                errors.append("Choose an output file")
            self.skip = 0
        elif erasing:
            self.LengthText.SetLabel("Erasing length")
            self.skip = 0

        if not self.device:
            errors.append("Choose a correct device")
        elif not self.device_supported:
            errors.append("Unsupported device driver\nPlease install libusbK driver")

        if mode != FtdiMode.MPSSE:
            self.SpiGpio.EnablePins()
        else:
            self.SpiGpio.SetChoice(SCK=SCK, MISO=MISO, MOSI=MOSI)
            self.SpiGpio.EnablePins("CS")

        if errors:
            self.Start.SetNote(errors[0])
        elif warnings:
            self.Start.SetNote(warnings[0])
        else:
            self.Start.SetNote("")

        self.prev_state = new_state
        self.Start.Enable(not errors)
        self.Cancel.Disable()

    def OnDevicesUpdated(self) -> None:
        UsbTools.flush_cache()
        devices = []
        for desc, if_count in Ftdi.list_devices():
            vid, pid, bus, address, sn, index, description = desc
            if vid not in Ftdi.PRODUCT_IDS:
                continue
            device_name = "Unknown"
            for dev_name, dev_pid in Ftdi.PRODUCT_IDS[vid].items():
                if pid == dev_pid:
                    device_name = dev_name.upper()
            url, _ = UsbTools.build_dev_strings(
                scheme="ftdi",
                vdict=Ftdi.VENDOR_IDS,
                pdict=Ftdi.PRODUCT_IDS,
                devdescs=[(desc, if_count)],
            )[0]
            description = f"{device_name} ({vid:04X}/{pid:04X}, bus {bus}:{address})"
            if sn:
                description += f" - S/N: {sn}"
                supported = True
            else:
                description += " - UNSUPPORTED DRIVER TYPE"
                supported = False
            devices.append((url, description, supported))

        user_device = self.device or self.last_device
        auto_device = None

        for url, description, supported in set(devices) - set(self.devices):
            info(f"Found new device: {description}")
            if not supported:
                warning("Device driver NOT supported for FTDI flashing!")
                warning("Please install a libusbK driver (use Zadig on Windows)")
            if supported:
                auto_device = url
        for _, description, _ in set(self.devices) - set(devices):
            info(f"Device unplugged: {description}")

        self.Device.Enable(bool(devices))
        if devices:
            self.Device.Set([device[1] for device in devices])
            self.devices = devices
            self.device = user_device or auto_device
        else:
            self.Device.Set(["No FTDI devices found"])
            self.Device.SetSelection(0)
            self.devices = []

    @on_event
    def OnRescanClick(self) -> None:
        self.OnDevicesUpdated()

    @property
    def filename_stem(self) -> str:
        return self.protocol and self.protocol.value or "dump"

    @property
    def device(self) -> str | None:
        if not self.devices:
            return None
        if self.Device.GetSelection() == wx.NOT_FOUND:
            return None
        return self.devices[self.Device.GetSelection()][0]

    @device.setter
    def device(self, value: str | None) -> None:
        if value is None:
            self.Device.SetSelection(wx.NOT_FOUND)
        else:
            for url, description, _ in self.devices:
                if value == url:
                    self.Device.SetValue(description)
                    self.last_device = None
                    self.DoUpdate(self.Device)
                    return
            self.last_device = value
            self.DoUpdate(self.Device)

    @property
    def device_supported(self) -> bool:
        if not self.devices:
            return False
        if self.Device.GetSelection() == wx.NOT_FOUND:
            return False
        return self.devices[self.Device.GetSelection()][2]

    @property
    def mode(self) -> FtdiMode:
        for mode, button in self.Modes.items():
            if button.GetValue():
                return mode

    @mode.setter
    def mode(self, value: FtdiMode) -> None:
        self.Modes[value].SetValue(True)

    @property
    def frequency(self) -> int:
        for button, speeds in self.Frequency.items():
            if button.GetValue():
                return speeds[self.mode]

    @frequency.setter
    def frequency(self, value: int) -> None:
        for button, speeds in self.Frequency.items():
            if value in speeds.values():
                button.SetValue(True)
                return

    @property
    def protocol(self) -> ProtocolType:
        current_page = self.Notebook.GetCurrentPage()
        for protocol, page in self.Protocols.items():
            if page == current_page:
                return protocol

    @protocol.setter
    def protocol(self, value: ProtocolType) -> None:
        page = self.Protocols[value]
        self.Notebook.SetSelection(self.Notebook.FindPage(page))

    @property
    def is_reading(self) -> bool:
        match self.protocol:
            case ProtocolType.SPI:
                return self.spi_operation == SpiOperation.READ
        return False

    @property
    def is_writing(self) -> bool:
        match self.protocol:
            case ProtocolType.SPI:
                return self.spi_operation == SpiOperation.WRITE
        return False

    @property
    def is_erasing(self) -> bool:
        match self.protocol:
            case ProtocolType.SPI:
                return self.spi_operation == SpiOperation.ERASE
        return False

    def set_writing(self) -> None:
        match self.protocol:
            case ProtocolType.SPI:
                self.spi_operation = SpiOperation.WRITE

    @property
    def offset(self) -> int:
        text: str = self.Offset.GetValue().strip() or "0"
        value = int_or_zero(text)
        return value

    @offset.setter
    def offset(self, value: int) -> None:
        value = value or 0
        self.Offset.SetValue(f"0x{value:X}")

    @property
    def skip(self) -> int:
        text: str = self.Skip.GetValue().strip() or "0"
        value = int_or_zero(text)
        return value

    @skip.setter
    def skip(self, value: int) -> None:
        value = value or 0
        self.Skip.SetValue(f"0x{value:X}")

    @property
    def length(self) -> int | None:
        text: str = self.Length.GetValue().strip()
        if not text:
            return None
        value = int_or_zero(text)
        return value

    @length.setter
    def length(self, value: int | None):
        if value:
            self.Length.SetValue(f"0x{value:X}")
        else:
            self.Length.SetValue("")

    @property
    def spi_gpio(self) -> dict[str, int]:
        return self.SpiGpio.GetChoice()

    @spi_gpio.setter
    def spi_gpio(self, value: dict[str, int]) -> None:
        self.SpiGpio.SetChoice(**value)

    @property
    def spi_operation(self) -> SpiOperation:
        for operation, button in self.SpiOperations.items():
            if button.GetValue():
                return operation

    @spi_operation.setter
    def spi_operation(self, value: SpiOperation) -> None:
        self.SpiOperations[value].SetValue(True)

    @on_event
    def OnStartClick(self):
        pass

    @on_event
    def OnCancelClick(self):
        pass
