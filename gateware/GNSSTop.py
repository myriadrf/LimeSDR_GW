from litex.gen import LiteXModule
from migen import *
from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream
from gateware.LimeDFB.general.ZDAParser import ZDAParser
from gateware.LimeDFB.general.RMCParser import RMCParser
from gateware.LimeDFB.general.GSAParser import GSAParser

def to_bcd(signal):
    bcd = Signal(8)
    cases = {}
    for i in range(1 << len(signal)):
        val = i % 100
        cases[i] = bcd.eq((val // 10) << 4 | (val % 10))
    return bcd, cases

class GNSSTop(LiteXModule, AutoCSR):
    def __init__(self, soc, with_zda=True, with_rmc=True, with_gsa=True):
        self.sink = sink = stream.Endpoint([("data", 8)])
        self.pps = pps = Signal()
        
        if with_zda:
            self.zda_parser = ZDAParser(soc)
            self.comb += [
                self.zda_parser.sink.data.eq(sink.data),
                self.zda_parser.sink.valid.eq(sink.valid),
                self.zda_parser.pps.eq(pps)
            ]
            
        if with_rmc:
            self.rmc_parser = RMCParser(soc, parse_time=not with_zda)
            self.comb += [
                self.rmc_parser.sink.data.eq(sink.data),
                self.rmc_parser.sink.valid.eq(sink.valid)
            ]
            
        if with_gsa:
            self.gsa_parser = GSAParser(soc)
            self.comb += [
                self.gsa_parser.sink.data.eq(sink.data),
                self.gsa_parser.sink.valid.eq(sink.valid)
            ]

        # GNSS module control
        # self.gnsscfg_en = CSRStorage(1, reset=1, description="GNSS module control (0-disable, 1-enable)")

        # Current time registers (Legacy)
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

        # New GNSSCFG CSRs
        if with_zda or with_rmc:
            self.gnss_utc_sss0 = CSRStatus(16, description="GNSS UTC sub-seconds")
            self.gnss_utc_mm_ss1 = CSRStatus(16, description="GNSS UTC minutes and seconds")
            self.gnss_utc_hh = CSRStatus(16, description="GNSS UTC hours")
            self.gnss_date_mm_yy = CSRStatus(16, description="GNSS Date month and year")
            self.gnss_date_dd = CSRStatus(16, description="GNSS Date day")

        if with_rmc:
            self.gnss_status = CSRStatus(16, description="GNSS Status (bit 0: RMC status, 1: Lat N/S, 2: Long E/W)")
            self.gnss_lat = CSRStatus(32, description="GNSS Latitude (BCD digits)")
            self.gnss_long = CSRStatus(32, description="GNSS Longitude (BCD digits)")
            self.gnss_long_ext = CSRStatus(16, description="GNSS Longitude digit Y4")
            self.gnss_speed = CSRStatus(24, description="GNSS Speed (BCD digits)")
            self.gnss_course = CSRStatus(24, description="GNSS Course (BCD digits)")
            
            self.comb += [
                self.gnss_status.status.eq(Cat(self.rmc_parser.status, self.rmc_parser.lat_n_s, self.rmc_parser.long_e_w)),
                self.gnss_lat.status.eq(self.rmc_parser.lat),
                self.gnss_long.status.eq(self.rmc_parser.long),
                self.gnss_long_ext.status.eq(self.rmc_parser.long_ext),
                self.gnss_speed.status.eq(self.rmc_parser.speed),
                self.gnss_course.status.eq(self.rmc_parser.course)
            ]

        if with_gsa:
            self.gnss_fix = CSRStatus(16, description="GNSS Fix Status (4 bits per constellation: GL, GB, GP, GA)")
            self.comb += self.gnss_fix.status.eq(self.gsa_parser.fix_status)

        if with_zda:
            self.comb += [
                self.time_min_sec.fields.sec.eq(self.zda_parser.time_seconds),
                self.time_min_sec.fields.min.eq(self.zda_parser.time_minutes),
                self.time_mon_day_hrs.fields.hrs.eq(self.zda_parser.time_hours),
                self.time_mon_day_hrs.fields.day.eq(self.zda_parser.time_day),
                self.time_mon_day_hrs.fields.mon.eq(self.zda_parser.time_month),
                self.time_yrs.fields.yrs.eq(self.zda_parser.time_year),
            ]
            
            bcd_ss, ss_cases = to_bcd(self.zda_parser.time_seconds)
            bcd_mm, mm_cases = to_bcd(self.zda_parser.time_minutes)
            bcd_hh, hh_cases = to_bcd(self.zda_parser.time_hours)
            bcd_mon, mon_cases = to_bcd(self.zda_parser.time_month)
            bcd_day, day_cases = to_bcd(self.zda_parser.time_day)
            bcd_yy, yy_cases = to_bcd(self.zda_parser.time_year)

            self.comb += [
                Case(self.zda_parser.time_minutes, mm_cases),
                Case(self.zda_parser.time_seconds, ss_cases),
                Case(self.zda_parser.time_hours, hh_cases),
                Case(self.zda_parser.time_month, mon_cases),
                Case(self.zda_parser.time_day, day_cases),
                Case(self.zda_parser.time_year, yy_cases),

                self.gnss_utc_mm_ss1.status.eq(Cat(bcd_ss, bcd_mm)),
                self.gnss_utc_hh.status.eq(bcd_hh),
                self.gnss_date_mm_yy.status.eq(Cat(bcd_yy, bcd_mon)),
                self.gnss_date_dd.status.eq(bcd_day),
            ]
        elif with_rmc:
            self.comb += [
                self.gnss_utc_sss0.status.eq(self.rmc_parser.time_sss),
                self.gnss_utc_mm_ss1.status.eq(Cat(self.rmc_parser.time_ss, self.rmc_parser.time_mm)),
                self.gnss_utc_hh.status.eq(self.rmc_parser.time_hh),
                self.gnss_date_mm_yy.status.eq(Cat(self.rmc_parser.date_yy, self.rmc_parser.date_mm)),
                self.gnss_date_dd.status.eq(self.rmc_parser.date_dd),
            ]
