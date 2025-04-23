from migen import *
from litex.soc.interconnect.csr import CSRStorage, AutoCSR, CSRField, CSRStatus
from litex.soc.interconnect import stream


class ZDAParser(Module, AutoCSR):
    def __init__(self, soc):
        soc.add_constant("ZDAParser_present")
        soc.add_constant("TimeSource_present")
        """
        Monitors UARTPHY output for ZDA messages and extracts:
          - UTC Time: hh, mm, ss
          - Date: day, month, year
        Expected message format:
          $GPZDA,hhmmss.ss,dd,mm,yyyy,xx,xx*CS
        """
        # Constant header to match: "$GPZDA,"
        # HEADER = b"$GPZDA,"
        # Omit talker ID to have easy compatibility with all GNSS's'
        HEADER = b"ZDA,"
        HEADER_LEN = len(HEADER)
        HEADER = Array(list(HEADER))


        # Connect UART source stream.
        self.sink = stream.Endpoint([("data", 8)])
        self.pps = Signal()
        pps_reg = Signal()
        pps_reg_1 = Signal()
        pps_reg_2 = Signal()


        #     # CSR registers for storing parsed fields.
        #     self.time_hours = CSRStatus(5, description="Extracted UTC Hours")
        # self.time_minutes = CSRStatus(6, description="Extracted UTC Minutes")
        # self.time_seconds = CSRStatus(6, description="Extracted UTC Seconds")
        # self.time_day = CSRStatus(5, description="Extracted UTC Day")
        # self.time_month = CSRStatus(4, description="Extracted UTC Month")
        # self.time_year = CSRStatus(12, description="Extracted UTC Year")
        # self.time_valid = CSRStatus(1, description="Indicates if the UTC time is valid", reset=0)
        self.time_hours = Signal(5)  # Extracted UTC Hours
        self.time_minutes = Signal(6)  # Extracted UTC Minutes
        self.time_seconds = Signal(6)  # Extracted UTC Seconds
        self.time_day = Signal(5)  # Extracted UTC Day
        self.time_month = Signal(4)  # Extracted UTC Month
        self.time_year = Signal(12)  # Extracted UTC Year
        self.time_valid = Signal()  # Indicates if the UTC time is valid
        self.time_reset_valid = Signal()

        # Internal signals for header matching and parsing.
        self.header_index = header_index = Signal(max=HEADER_LEN + 1, reset=0)
        time_digit_cnt = Signal(4, reset=0)  # For hhmmss (6 digits)
        day_digit_cnt = Signal(3, reset=0)  # Expecting 2 digits for day
        month_digit_cnt = Signal(3, reset=0)  # Expecting 2 digits for month
        year_digit_cnt = Signal(3, reset=0)  # Expecting 4 digits for year
        talker_id_cnt = Signal(1, reset=0) # Talker ID is always 2 symbols

        # Temporary registers for accumulating parsed values.
        time_hours_temp = Signal(8, reset=0)
        time_minutes_temp = Signal(8, reset=0)
        time_seconds_temp = Signal(8, reset=0)
        day_temp = Signal(8, reset=0)
        month_temp = Signal(8, reset=0)
        year_temp = Signal(16, reset=0)

        # Checksum status strobes
        self.o_valid = Signal()
        self.o_error = Signal()
        # left commented, because might be useful later
        # self.debug_counter = Signal(4, reset_less=True)

        # Signal to prevent checksum checking for unrelated messages
        dont_check = Signal(reset=0)

        self.sync += [
            pps_reg.eq(self.pps),
            pps_reg_1.eq(pps_reg),
            pps_reg_2.eq(pps_reg_1),
            # Rising edge of pps
            # Reset valid if pps happens (valid is outdated)
            If((pps_reg_1 == 1) & (pps_reg_2 == 0), [
                self.time_valid.eq(0),
            ]).Else([
                # Set valid if the checksum was valid
                If(self.o_valid == 1, [
                    self.time_valid.eq(1)
                    # Reset valid if the checksum was invalid
                ]).Elif(self.o_error == 1,[
                    self.time_valid.eq(0)
                ])
            ])
        ]
        

        # FSM states:
        #   IDLE: Wait for start of header.
        #   MATCH_HEADER: Compare incoming characters with "$GPZDA,"
        #   PARSE_TIME: Read 6 digits (hhmmss) from the time field.
        #   PARSE_DAY: Read the day field (2 digits).
        #   PARSE_MONTH: Read the month field (2 digits).
        #   PARSE_YEAR: Read the year field (4 digits).
        fsm = FSM(reset_state="IDLE")
        self.submodules.fsm = fsm

        # IDLE: Wait for the first character of the header.
        fsm.act("IDLE",
                # Reset talker id counter
                NextValue(talker_id_cnt,0),
                # Assume received packet is ZDA until proven otherwise
                NextValue(dont_check, 0),
                If(self.sink.valid,
                   If(self.sink.data == ord("$"),
                      # NextValue(header_index, 1),
                      NextValue(header_index, 0),
                      NextState("SKIP_TALKER_ID")
                      )
                   )
                )

        # SKIP_TALKER_ID: skips talker id.
        # talker ID should be two symbols. talker_id_cnt=0 skips the first symbol, talker_id_cnt=1 skips the second
        fsm.act("SKIP_TALKER_ID",[
            If(self.sink.valid,[
                If(talker_id_cnt >= 1,[
                    NextState("MATCH_HEADER")
                ]).Else([
                    NextValue(talker_id_cnt,talker_id_cnt+1)
                ])
            ])
        ])

        # MATCH_HEADER: Compare each incoming character with the header.
        fsm.act("MATCH_HEADER",
                If(self.sink.valid,
                   If(header_index < HEADER_LEN,
                      If(self.sink.data == HEADER[header_index],
                         NextValue(header_index, header_index + 1),
                         If(header_index == HEADER_LEN - 1,
                            # Full header matched; move to time field.
                            NextState("PARSE_TIME"),
                            # NextValue(self.debug_counter,self.debug_counter + 1),
                            NextValue(time_digit_cnt, 0)
                            )
                         ).Else(
                          # Mismatch: reset and go back to IDLE.
                          NextState("IDLE"),
                          NextValue(header_index, 0),
                          # Abort checksum checking
                          NextValue(dont_check, 1),
                      )
                      )
                   )
                )

        # PARSE_TIME: Read 6 digits (hhmmss) from the time field.
        fsm.act("PARSE_TIME", [
            If(self.sink.valid,
               If(time_digit_cnt < 6, [
                   Case(time_digit_cnt, {
                       0: [NextValue(time_hours_temp, (self.sink.data - 48) * 10)],
                       1: [NextValue(time_hours_temp, time_hours_temp + (self.sink.data - 48))],
                       2: [NextValue(time_minutes_temp, (self.sink.data - 48) * 10)],
                       3: [NextValue(time_minutes_temp, time_minutes_temp + (self.sink.data - 48))],
                       4: [NextValue(time_seconds_temp, (self.sink.data - 48) * 10)],
                       5: [NextValue(time_seconds_temp, time_seconds_temp + (self.sink.data - 48))]
                   }),
                   NextValue(time_digit_cnt, time_digit_cnt + 1)]
                  ).Else([
                   If(self.sink.valid,
                      If(self.sink.data == ord(","), [
                          # After time field, move to parse day field.
                          NextState("PARSE_DAY"),
                          NextValue(day_digit_cnt, 0)]
                         )
                      )
               ]
               )
               )]
                )

        # PARSE_DAY: Read day field (2 digits expected).
        fsm.act("PARSE_DAY",
                If(self.sink.valid,
                   If(day_digit_cnt < 2,
                      NextValue(day_temp, day_temp * 10 + (self.sink.data - 48)),
                      NextValue(day_digit_cnt, day_digit_cnt + 1)
                      ).Else(
                       # After two digits, expect a comma then move to parse month.
                       If(self.sink.data == ord(",")),
                       NextState("PARSE_MONTH"),
                       NextValue(month_digit_cnt, 0)
                   )
                   )
                )

        # PARSE_MONTH: Read month field (2 digits expected).
        fsm.act("PARSE_MONTH",
                If(self.sink.valid,
                   If(month_digit_cnt < 2,
                      NextValue(month_temp, month_temp * 10 + (self.sink.data - 48)),
                      NextValue(month_digit_cnt, month_digit_cnt + 1)
                      ).Else(
                       # After month field, expect a comma then move to parse year.
                       If(self.sink.data == ord(",")),
                       NextState("PARSE_YEAR"),
                       NextValue(year_digit_cnt, 0)
                   )
                   )
                )

        # PARSE_YEAR: Read year field (4 digits expected).
        fsm.act("PARSE_YEAR",
                If(self.sink.valid,
                   If(year_digit_cnt < 4,
                      NextValue(year_temp, year_temp * 10 + (self.sink.data - 48)),
                      NextValue(year_digit_cnt, year_digit_cnt + 1)
                      ).Else([
                       # Reset temporary registers and go back to IDLE for next message.
                       NextState("WAIT_CHECKSUM")
                       ]
                   )
                   )
                )

        # WAIT_CHECKSUM: wait for checksum calculation to finish
        fsm.act("WAIT_CHECKSUM", [
            If(self.o_valid == 1, [
                # Update all CSR registers.
                NextValue(self.time_hours, time_hours_temp),
                NextValue(self.time_minutes, time_minutes_temp),
                NextValue(self.time_seconds, time_seconds_temp),
                NextValue(self.time_day, day_temp),
                NextValue(self.time_month, month_temp),
                NextValue(self.time_year, year_temp),
                NextState("IDLE"),
                NextValue(header_index, 0),
                NextValue(time_hours_temp, 0),
                NextValue(time_minutes_temp, 0),
                NextValue(time_seconds_temp, 0),
                NextValue(day_temp, 0),
                NextValue(month_temp, 0),
                NextValue(year_temp, 0)
            ]).Elif(self.o_error == 1,[
                NextState("IDLE"),
                NextValue(header_index, 0),
                NextValue(time_hours_temp, 0),
                NextValue(time_minutes_temp, 0),
                NextValue(time_seconds_temp, 0),
                NextValue(day_temp, 0),
                NextValue(month_temp, 0),
                NextValue(year_temp, 0)
            ])

        ])
        # ------------------
        # Checksum checking
        # ------------------

        # Internal signals for checksum calculation.
        self.computed_checksum = computed_checksum = Signal(8, reset=0)
        self.received_checksum = received_checksum = Signal(8)
        nibble = Signal(4)

        # FSM to process incoming data.
        self.submodules.fsm2 = FSM(reset_state="IDLE")
        fsm2 = self.fsm2

        # IDLE: Wait for start-of-message marker '$'.
        fsm2.act("IDLE",
                 # self.o_valid.eq(0),
                 # self.o_error.eq(0),
                 NextValue(self.o_valid, 0),
                 NextValue(self.o_error, 0),
                 If(self.sink.valid & (self.sink.data == ord("$")),
                    NextValue(computed_checksum, 0),  # Reset checksum accumulator.
                    NextState("DATA")
                    )
                 )

        # DATA: XOR-accumulate incoming bytes until '*' is received.
        fsm2.act("DATA",
                 # Calculate checksum only if received packet is ZDA
                 If(dont_check == 0, [
                     If(self.sink.valid,
                        If(self.sink.data == ord("*"),
                           NextState("CHECKSUM1")
                           ).Else(
                            NextValue(computed_checksum, computed_checksum ^ self.sink.data)
                        )
                        )]).Else([
                     # Received packet is not ZDA, no need to check checksum
                     NextState("IDLE")
                 ])
                 )

        # CHECKSUM1: Convert first ASCII hex digit to a 4-bit nibble.
        fsm2.act("CHECKSUM1",
                 If(self.sink.valid,
                    If((self.sink.data >= ord("0")) & (self.sink.data <= ord("9")),
                       NextValue(nibble, self.sink.data - ord("0"))
                       ).Elif((self.sink.data >= ord("A")) & (self.sink.data <= ord("F")),
                              NextValue(nibble, self.sink.data - ord("A") + 10)
                              ),
                    NextState("CHECKSUM2")
                    )
                 )

        # CHECKSUM2: Convert second ASCII hex digit, combine with previous nibble,
        fsm2.act("CHECKSUM2",
                 If(self.sink.valid,
                    If((self.sink.data >= ord("0")) & (self.sink.data <= ord("9")),
                       NextValue(received_checksum, (nibble << 4) | (self.sink.data - ord("0")))
                       ).Elif((self.sink.data >= ord("A")) & (self.sink.data <= ord("F")),
                              NextValue(received_checksum, (nibble << 4) | (self.sink.data - ord("A") + 10))
                              ),
                    NextState("COMPARE_CHECKSUMS")
                    )
                 )

        # COMPARE_CHECKSUMS: compare received and calculated checksums
        fsm2.act("COMPARE_CHECKSUMS", [
            If(computed_checksum == received_checksum,
            # If(self.debug_counter == 15,
               NextValue(self.o_valid, 1),
               ).Else(
                NextValue(self.o_error, 1),
            ),
            NextState("PULSE")
        ])

        # PULSE: allow a single cycle for o_valid and o_error, before returning to reset state
        fsm2.act("PULSE", [
            NextState("IDLE")
        ])
