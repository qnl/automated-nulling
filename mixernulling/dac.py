from qcodes import InstrumentChannel, VisaInstrument, ChannelList
from qcodes.instrument.parameter import Parameter
from qcodes.utils.validators import Numbers

import time

class LTC2704Channel(InstrumentChannel):
    """
    """

    def __init__(self, parent: VisaInstrument, name: str, channel: int):
        super().__init__(parent, name)

        self.channel = channel

        self.span = Parameter(
            'span',
            get_cmd=f"{name}:SPAN?",
            set_cmd=f"{name}:SPAN {{:d}}",
            label="Span",
            instrument=self
        )

        self.code = Parameter(
            'code',
            get_cmd=f"{name}:CODE?",
            set_cmd=f"{name}:CODE {{:d}}",
            get_parser=int,
            set_parser=int,
            label="Code",
            instrument=self
        )

class LTC2704(VisaInstrument):
    """
    This is a QCoDes driver for the Holzworth HS900x series Multi-channel RF synthesizers.
    Developed in-house at QNL. Does not work.
    """

    def __init__(self, name, address, **kwargs):
        """

        :param name:
        :param address:
        :param kwargs:
        """
        NUM_CHANNELS = 24

        super().__init__(name, address, terminator='\n', **kwargs)

        channels = ChannelList(
            self,
            'DACChannels',
            LTC2704Channel,
        )

        for cid in range(NUM_CHANNELS):
            ch = LTC2704Channel(self, f"CH{cid}", cid)
            channels.append(ch)
            self.add_submodule(f'CH{cid}', ch)
        
        channels.lock()
        self.add_submodule('channels', channels)

        time.sleep(0.1)
        self.connect_message()

    def get_idn(self):
        IDN = self.ask_raw('*IDN?')
        
        try:
            vendor, model, firmware = [s.strip() for s in IDN.split(',')]
        except ValueError:
            vendor = model = firmware = None

        info = dict(vendor=vendor, model=model, firmware=firmware)

        return info

