from migen import *
from litex.soc.interconnect.csr import *
from gateware.LimeDFB.general.ZDAParser import ZDAParser

class GNSSTop(Module, AutoCSR):
    def __init__(self, soc):
        self.zda_parser = zda_parser = ZDAParser(soc)
        
        # GNSS module control
        self.gnsscfg_en = CSRStorage(1, reset=1, description="GNSS module control (0-disable, 1-enable)")

        # Current time registers
        self.time_min_sec = CSRStatus(size=16, description="Time in minutes and seconds, current", fields=[
            CSRField("sec", size=6, offset=0, description="Current time, seconds"),
            CSRField("min", size=6, offset=6, description="Current time, minutes")
        ])
        self.time_mon_day_hrs = CSRStatus(size=16, description="Time in months, days and hours, current", fields=[
            CSRField("hrs", size=5, offset=0, description="Current time, hours"),
            CSRField("day", size=5, offset=5, description="Current start time, days"),
            CSRField("mon", size=4, offset=10, description="Current start time, months"),
        ])
        self.time_yrs = CSRStatus(size=16, description="Time in years, current", fields=[
            CSRField("yrs", size=12, offset=0, description="Current time, years")
        ])

        self.comb += [
            self.time_min_sec.fields.sec.eq(zda_parser.time_seconds),
            self.time_min_sec.fields.min.eq(zda_parser.time_minutes),
            self.time_mon_day_hrs.fields.hrs.eq(zda_parser.time_hours),
            self.time_mon_day_hrs.fields.day.eq(zda_parser.time_day),
            self.time_mon_day_hrs.fields.mon.eq(zda_parser.time_month),
            self.time_yrs.fields.yrs.eq(zda_parser.time_year),
        ]
