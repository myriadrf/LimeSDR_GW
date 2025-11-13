#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect.axi.axi_stream import AXIStreamInterface
from litex.soc.interconnect.csr            import CSRStatus, CSRStorage, CSRField
from litex.soc.interconnect                import stream

from gateware.common import *

# RX Path Top --------------------------------------------------------------------------------------

class RXPathTop(LiteXModule):
    def __init__(self, platform, fpgacfg_manager=None,
        # RX parameters
        RX_IQ_WIDTH                  = 12,
        S_AXIS_IQSMPLS_BUFFER_WORDS  = 16,
        M_AXIS_IQPACKET_BUFFER_WORDS = 512,
        int_clk_domain               = "lms_rx",
        m_clk_domain                 = "lms_rx",
        s_clk_domain                 = "lms_rx",
        soc_has_timesource           = False,
        sink_width                   = 64,
        source_width                 = 64,
        use_channel_combiner         = True,
        bypass_packets               = False,
        ):

        assert fpgacfg_manager is not None
        assert sink_width in [64, 128], f"Invalid sink_width: {sink_width}. Must be 64 (two channels) or 128 (four channels)."


        self.platform              = platform

        self.sink                  = AXIStreamInterface(sink_width, clock_domain=s_clk_domain)
        self.source                = AXIStreamInterface(source_width, clock_domain=m_clk_domain)

        self.rx_pct_fifo_aclrn_req = Signal()

        self.pct_hdr_cap           = Signal()

        # Sample NR.
        self.smpl_nr_cnt           = Signal(64)
        self.smpl_nr_cnt_out       = Signal(64)

        # Flag Control.
        self.tx_pct_loss_flg       = Signal()

        # Sample Compare.
        self.smpl_cnt_en           = Signal()

        self.pkt_size = CSRStorage(16, reset=253,
            description="Packet Size in bytes, "
        )

        # # #

        # Signals.
        # --------

        # RX IQ Stream.

        # sync.
        s_clk_rst_n              = Signal()
        self.s_clk_rst_n         = s_clk_rst_n
        s_clk_rst                = Signal()
        m_clk_rst_n              = Signal()
        int_clk_rst_n            = Signal()
        int_clk_rst              = Signal()
        self.int_clk_smpl_nr_clr = Signal()
        self.int_clk_ch_en       = Signal(4)
        self.s_clk_ch_en         = Signal(4)
        self.int_clk_mimo_en     = Signal()
        mimo_en                  = Signal()
        ddr_en                   = Signal()
        s_smpl_width             = Signal(2)

        # IQ Stream Combiner To Rx Path Top.
        iq_to_bit_pack_tdata         = Signal(64)
        self.iq_to_bit_pack_tvalid   = Signal()
        iq_to_bit_pack_tready        = Signal()
        iq_to_bit_pack_tkeep         = Signal(8)
        bit_pack_to_nto1_tdata       = Signal(64)
        self.bit_pack_to_nto1_tvalid = Signal()
        self.bit_pack_to_nto1_tlast  = Signal()

        pct_hdr_0               = Signal(64)
        self.pct_hdr_1          = Signal(64)

        self.iqpacket_axis = iqpacket_axis           = stream.Endpoint([("data", 128)])

        iqpacket_wr_data_count  = Signal(10)
        bp_sample_nr_counter    = Signal(64)
        pkt_size                = Signal(16)
        self.pkt_size_debug = pkt_size


        #debug
        self.debug_write_pointer    = Signal(5)
        self.debug_read_pointer     = Signal(5)
        self.debug_wrusedw          = Signal(5)
        self.debug_pipeline_en      = Signal(3)
        self.iqpacket_wr_data_count = iqpacket_wr_data_count

        self.comb += s_clk_rst.eq(~s_clk_rst_n)
        self.comb += int_clk_rst.eq(~int_clk_rst_n)

        self.num_of_channels = Signal(32)
        self.sample_counter_1_inc_val = Signal(32)

        self.bp_sample_nr_counter = bp_sample_nr_counter

        self.s_smpl_width = s_smpl_width

        self.s_clk_rst = s_clk_rst


        # Sample Counter, used for TX synchronization logic
        self.sample_counter_0 = Instance("counter64",
                                           # Clk/Reset.
                                           i_clk        = ClockSignal(s_clk_domain),  # S_AXIS_IQSMPLS_ACLK
                                           i_rst        = s_clk_rst,  # S_AXIS_IQSMPLS_ARESETN
                                           # AXI Stream Slave
                                           i_inc_en     = self.sink.valid & self.sink.ready,
                                           i_inc_val    = Constant(1, 32),
                                           i_ld         = Constant(0, 1),
                                           i_ld_val     = Constant(0, 64),
                                           o_count_o    = self.smpl_nr_cnt,
                                           )

        self.sample_counter_0_conv = add_vhd2v_converter(self.platform,
                                                           instance=self.sample_counter_0,
                                                           files=[
                                                               "gateware/LimeDFB/rx_path_top/src/counter64.vhd"],
                                                           )
        ## Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.sample_counter_0)


        if soc_has_timesource:
            # Timestamp related
            # TODO: This all should probably be moved to a separate module
            # TODO: If int_clk_rst_n is not lms_rx or equivalent, this doesn't work, because reset from int_clk_rst_n is used in lms_rx domain
            self.timestamp_settings = CSRStorage(size=1,description="Timestamp Settings", fields=[
                CSRField("TS_SEL", size=1, offset=0, description="Timestamp selector",reset=0, values=[
                    ("``0b0``", "Classic Timestamp."),
                    ("``0b1``", "PPS Counter+Clock counter mixed Timestamp."),
                ])
            ])
            self.pps                   = Signal()
            pps_reg                    = Signal()
            pps_reg_1                  = Signal()
            pps_reg_2                  = Signal()

            self.pps_rising            = Signal()
            self.timestamp_pps_counter = Signal(32)
            self.timestamp_clk_count   = Signal(32)
            self.timestamp_mixed       = timestamp_mixed = Signal(64)
            self.comb += [
                timestamp_mixed[0:32].eq(self.timestamp_clk_count),
                timestamp_mixed[32:64].eq(self.timestamp_pps_counter),
            ]
            # TODO: find a way to pass a clock domain to this instead of hardcoding lms_rx
            self.sync.lms_rx +=[
                pps_reg.eq(self.pps),
                pps_reg_1.eq(pps_reg),
                pps_reg_2.eq(pps_reg_1),
                self.pps_rising.eq(pps_reg_1 & ~pps_reg_2),

                ## Rules for clock counter
                # TS_SEL does not use clock counter, so keep it in reset
                If((self.timestamp_settings.fields.TS_SEL == 0) | (int_clk_rst_n == 0) ,[
                    self.timestamp_clk_count.eq(0),
                    self.timestamp_pps_counter.eq(0),
                ]).Elif(self.timestamp_settings.fields.TS_SEL == 1,[
                    # If clock counter is in use, reset at pps
                    If((self.pps_rising == 1),[
                        self.timestamp_clk_count.eq(0),
                        self.timestamp_pps_counter.eq(self.timestamp_pps_counter + 1),
                        # Else keep counting
                    ]).Else([
                        self.timestamp_clk_count.eq(self.timestamp_clk_count + 1),
                    ])
                ]).Else([
                    # Keep count in reset if undefined
                    self.timestamp_clk_count.eq(0),
                ]),
            ]

        if platform.name.startswith("limesdr_mini"):
            tx_pct_loss_sync = Signal()
            self.specials += MultiReg(self.tx_pct_loss_flg, tx_pct_loss_sync, odomain=s_clk_domain)
            self.comb += [
                # Packet Header 0
                pct_hdr_0.eq(Cat(Constant(0, 3), tx_pct_loss_sync, Constant(0, 12), 0x060504030201)), # FIXME: 0:15: isn't 0 and 16:63 differs for XTRX
                pkt_size.eq(Constant(4096 // 16, 16)),              # 256 * 128b = 4096Bytes
            ]
        else:
            self.comb += [
                # Packet Header 0
                pct_hdr_0.eq(0x7766554433221100),
                pkt_size.eq(Cat(Constant(0, 3), self.pkt_size.storage)[7:]),
            ]



        #-----------------------------------------------------------------------------------------------------------
        # Stage I - Channel combiner and bit_width selector
        # ----------------------------------------------------------------------------------------------------------
        chnl_combiner = ChannelCombiner(platform, s_clk_rst_n, self.s_clk_ch_en)
        chnl_combiner = stream.BufferizeEndpoints({"sink": stream.DIR_SINK}, True, True)(chnl_combiner)
        chnl_combiner = stream.BufferizeEndpoints({"source": stream.DIR_SOURCE}, True, True)(chnl_combiner)
        chnl_combiner = ClockDomainsRenamer(s_clk_domain)(chnl_combiner)
        self.chnl_combiner = chnl_combiner

        self.bit_width_selector = ClockDomainsRenamer(s_clk_domain)(BitwidthSelector(platform))

        self.comb += [
            self.bit_width_selector.sel.eq(Mux(s_smpl_width == 2, 1, 0)),
            self.bit_width_selector.rst.eq(~s_clk_rst_n),
        ]

        #-----------------------------------------------------------------------------------------------------------
        # Stage II - Sample counter end packetizer (Data2PacketsFSM)
        # ----------------------------------------------------------------------------------------------------------
        if not bypass_packets:
            # ----------------------------------------------------------------------------------------------------------
            # Second counter instance
            base_inc_val = Signal(32)

            self.sync.sync += self.num_of_channels.eq(
                sum(self.s_clk_ch_en[i] for i in range(len(self.s_clk_ch_en))))

            cases = {
                0: base_inc_val.eq(Constant(0, 32)),
                1: base_inc_val.eq(Constant(4, 32)),
                2: base_inc_val.eq(Constant(2, 32)),
                4: base_inc_val.eq(Constant(1, 32)),
                "default": base_inc_val.eq(Constant(0, 32)),
            }

            self.comb += Case(self.num_of_channels, cases)

            self.sync.sync += [
                If (s_smpl_width == 2,
                    self.sample_counter_1_inc_val.eq(base_inc_val*4),
                ).Else(
                    self.sample_counter_1_inc_val.eq(base_inc_val),
                )

            ]

            self.sample_counter_1 = Instance("counter64",
                                             i_clk=ClockSignal(s_clk_domain),
                                             i_rst=s_clk_rst,
                                             i_inc_en=self.bit_width_selector.source.valid & self.bit_width_selector.source.ready & self.bit_width_selector.source.last,
                                             i_inc_val=self.sample_counter_1_inc_val,  # example: increment by 2
                                             i_ld=Constant(0, 1),
                                             i_ld_val=Constant(0, 64),
                                             o_count_o=bp_sample_nr_counter
                                             )

            self.sample_counter_1_conv = add_vhd2v_converter(self.platform,
                                                             instance=self.sample_counter_1,
                                                             files=[
                                                                 "gateware/LimeDFB/rx_path_top/src/counter64.vhd"],
                                                             )
            ## Removed Instance to avoid multiple definition
            self._fragment.specials.remove(self.sample_counter_1)

            # ----------------------------------------------------------------------------------------------------------
            # Packetizer
            data2packets_fsm = Data2PacketsFSM(
                platform    = platform,
                rst_n       = s_clk_rst_n,      # Pass in the reset
                pkt_size    = pkt_size,         # Pass in the size signal
                pct_hdr_0   = pct_hdr_0,        # Pass in header 0
                pct_hdr_1   = self.pct_hdr_1    # Pass in header 1
            )

            # --- 3. Wrap it in the correct Clock Domain ---
            data2packets_fsm = ClockDomainsRenamer(s_clk_domain)(data2packets_fsm)
            self.data2packets_fsm = data2packets_fsm


        #-----------------------------------------------------------------------------------------------------------
        # Stage III - Converting source width and CDC (source_ep_conv->source_ep_cdc)
        # ----------------------------------------------------------------------------------------------------------
        # 128b to 256b converter
        self.source_ep_conv = ep_conv = ResetInserter()(
            ClockDomainsRenamer(s_clk_domain)(stream.Converter(128, source_width)))

        # Clock domain crossing FIFO
        source_ep_cdc      = stream.AsyncFIFO([("data", source_width), ("keep", source_width//8)], 128)
        source_ep_cdc      = ClockDomainsRenamer({"write": s_clk_domain, "read": m_clk_domain})(source_ep_cdc)
        self.source_ep_cdc = source_ep_cdc



        #-----------------------------------------------------------------------------------------------------------
        # Final pipeline sink -> chnl_combiner -> bit_width_selector -> (data2packets_fsm) -> source_ep_conv -> source_ep_cdc
        # ----------------------------------------------------------------------------------------------------------

        # sink -> chnl_combiner -> bit_width_selector.sink

        # Connect sink -> chnl_combiner.sink
        self.comb += [
            self.sink.connect(self.chnl_combiner.sink, keep=["valid", "ready"], omit=["data", "keep"]),
        ]

        # If sink is 64b (2 channels) assign upper bits to 0
        if len(self.sink.data) == 128:
            self.comb += self.chnl_combiner.sink.data.eq(self.sink.data)
        else:
            self.comb += self.chnl_combiner.sink.data.eq(Cat(self.sink.data, Constant(0, 64)))


        self.comb += [
            self.chnl_combiner.source.connect(self.bit_width_selector.sink, keep=["data", "keep", "valid", "ready"]),
        ]

        # bit_width_selector.source -> (data2packets_fsm) ->  source_ep_conv.sink
        if not bypass_packets:
            self.comb += [
                self.bit_width_selector.source.connect(self.data2packets_fsm.sink),
                self.data2packets_fsm.source.connect(self.source_ep_conv.sink, omit=["keep", "last"]),
            ]
        else:
            self.comb += [
                self.bit_width_selector.source.connect(self.source_ep_conv.sink, omit=["keep", "last"]),
            ]

        # source_ep_conv.soruce-> source_ep_cdc -> source
        self.comb += [
            self.source_ep_conv.reset.eq(~s_clk_rst_n),
            self.source_ep_conv.source.connect(self.source_ep_cdc.sink, omit=["keep", "last"]),
            self.source_ep_cdc.source.connect(self.source),
        ]

        if not bypass_packets:
            if soc_has_timesource:
                self.comb += [
                    If(self.timestamp_settings.fields.TS_SEL == 1,[
                        self.pct_hdr_1.eq(timestamp_mixed),
                        self.smpl_nr_cnt_out.eq(timestamp_mixed),
                    ]).Else([
                        # Default to classic timestamp
                        self.pct_hdr_1.eq(bp_sample_nr_counter),
                        self.smpl_nr_cnt_out.eq(self.smpl_nr_cnt),
                    ])
                ]
            else:
                self.comb += [
                    self.pct_hdr_1.eq(bp_sample_nr_counter),
                    self.smpl_nr_cnt_out.eq(self.smpl_nr_cnt),
                ]


        # CDC. -------------------------------------------------------------------------------------

        reset_n = Signal()
        # if platform.name.startswith("limesdr_mini"):
        reset_n = fpgacfg_manager.rx_en
        # else:
        #     reset_n = fpgacfg_manager.tx_en

        if s_clk_domain == "sys":
            self.comb += [
                s_clk_rst_n.eq(               reset_n),
                self.rx_pct_fifo_aclrn_req.eq(s_clk_rst_n),
                mimo_en.eq(                   fpgacfg_manager.mimo_int_en),
                ddr_en.eq(                    fpgacfg_manager.ddr_en),
                s_smpl_width.eq(              fpgacfg_manager.smpl_width),
            ]
        else:
            self.specials += [
                MultiReg(reset_n,                     s_clk_rst_n,                s_clk_domain, reset=0),
                MultiReg(s_clk_rst_n,                 self.rx_pct_fifo_aclrn_req, s_clk_domain, reset=0),
                MultiReg(fpgacfg_manager.mimo_int_en, mimo_en,                    s_clk_domain),
                MultiReg(fpgacfg_manager.ddr_en,      ddr_en,                     s_clk_domain),
                MultiReg(fpgacfg_manager.smpl_width,  s_smpl_width,               s_clk_domain),
            ]

        if int_clk_domain == "sys":
            self.comb += [
                int_clk_rst_n.eq(           reset_n),
                self.int_clk_ch_en.eq(      fpgacfg_manager.ch_en),
                self.int_clk_mimo_en.eq(    fpgacfg_manager.mimo_int_en),
                self.int_clk_smpl_nr_clr.eq(fpgacfg_manager.smpl_nr_clr),
            ]
        else:
            self.specials += [
                MultiReg(reset_n,                     int_clk_rst_n,            int_clk_domain, reset=0),
                MultiReg(fpgacfg_manager.ch_en,       self.int_clk_ch_en,       int_clk_domain, reset=0),
                MultiReg(fpgacfg_manager.mimo_int_en, self.int_clk_mimo_en,     int_clk_domain, reset=0),
                MultiReg(fpgacfg_manager.smpl_nr_clr, self.int_clk_smpl_nr_clr, int_clk_domain, reset=0),
            ]

        self.specials += [
            MultiReg(fpgacfg_manager.ch_en, self.s_clk_ch_en, s_clk_domain, reset=0),
        ]

        if m_clk_domain == "sys":
            self.comb += m_clk_rst_n.eq(reset_n)
        else:
            self.specials += MultiReg(reset_n, m_clk_rst_n, m_clk_domain, reset=1)


# ---------------------------------------------------------------------------------
#  Helper Class: FourChannelCombiner
# ---------------------------------------------------------------------------------
class FourChannelCombiner(LiteXModule):
    """
    A LiteXModule wrapper for the VHDL AXIS_CHNL_COMBINER.
    """
    def __init__(self, platform, s_clk_domain, s_clk_rst_n, s_clk_ch_en, s_smpl_width):
        self.sink    = stream.Endpoint([("data", 128), ("keep", 16)])
        self.source  = stream.Endpoint([("data", 128), ("keep", 16)])

        self.axis_chnl_combiner = ChannelCombiner(platform, s_clk_domain, s_clk_rst_n, s_clk_ch_en)

        self.bit_width_selector = ClockDomainsRenamer(s_clk_domain)(BitwidthSelector(platform))

        self.comb += [
            self.sink.connect(self.axis_chnl_combiner.sink, keep=["data", "keep", "valid", "ready"]),
            self.axis_chnl_combiner.source.connect(self.bit_width_selector.sink,
                                                   keep=["data", "keep", "valid", "ready"]),
            self.bit_width_selector.sel.eq(Mux(s_smpl_width == 2, 1, 0)),
            self.bit_width_selector.source.connect(self.source)
        ]


# ---------------------------------------------------------------------------------
#  Helper Class: ChannelCombiner
# ---------------------------------------------------------------------------------
class ChannelCombiner(LiteXModule):
    """
    A LiteXModule wrapper for the VHDL AXIS_CHNL_COMBINER.
    """
    def __init__(self, platform, s_clk_rst_n, s_clk_ch_en):

        self.sink    = stream.Endpoint([("data", 128), ("keep", 16)])
        self.source  = stream.Endpoint([("data", 128), ("keep", 16)])


        self.platform = platform

        #debug
        self.debug_write_pointer    = Signal(5)
        self.debug_read_pointer     = Signal(5)
        self.debug_wrusedw          = Signal(5)
        self.debug_pipeline_en      = Signal(3)

        self.s_clk_rst_n            = s_clk_rst_n
        self.s_clk_ch_en            = s_clk_ch_en

        self.axis_chnl_combiner = Instance("AXIS_CHNL_COMBINER",
                                           # Clk/Reset.
                                           i_ACLK           = ClockSignal(),     # S_AXIS_IQSMPLS_ACLK
                                           i_ARESETN        = self.s_clk_rst_n,  # S_AXIS_IQSMPLS_ARESETN
                                           # AXI Stream Slave
                                           i_S_AXIS_TVALID  = self.sink.valid,
                                           o_S_AXIS_TREADY  = self.sink.ready,
                                           i_S_AXIS_TDATA   = self.sink.data,
                                           i_S_AXIS_TKEEP   = self.sink.keep,
                                           i_S_AXIS_TUSER   = self.s_clk_ch_en,
                                           i_S_AXIS_TLAST   = self.sink.last,
                                           # AXI Stream Master
                                           o_M_AXIS_TVALID  = self.source.valid,
                                           i_M_AXIS_TREADY  = self.source.ready,
                                           o_M_AXIS_TDATA   = self.source.data,
                                           o_M_AXIS_TKEEP   = self.source.keep,
                                           o_M_AXIS_TLAST   = self.source.last,
                                           o_debug_write_pointer    = self.debug_write_pointer,
                                           o_debug_read_pointer     = self.debug_read_pointer,
                                           o_debug_wrusedw          = self.debug_wrusedw,
                                           o_debug_pipeline_en      = self.debug_pipeline_en,
                                           )

        self.axis_chnl_combiner_conv = add_vhd2v_converter(self.platform,
                                                           instance=self.axis_chnl_combiner,
                                                           files=[
                                                               "gateware/LimeDFB/rx_path_top/src/axis_chnl_combiner.vhd"],
                                                           )
        ## Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.axis_chnl_combiner)

# ---------------------------------------------------------------------------------
#  Helper Class: BitwidthSelector
# ---------------------------------------------------------------------------------
class BitwidthSelector(LiteXModule):
    """
    Selects between 16-bit (passthrough) and 12-bit (gearboxed) datapaths.

    All internal logic runs in the clock domain this module is
    instantiated in.
        Path Details:
                               [ sink ]
                                  | [128b]
                                  v
                          [    ep_demux    ]------- sel
                           0              1
                           |              | [128b]
                           |              |
                           |              | [96b] - Slicing (8x12b) & Cat()
                           |              v
                           |       [   ep_gearbox   ]
                           |              | [128b]
                           v              v
                           0              1
                         [      source      ]-------- sel
                                   |
                                   v [128b]
    """
    def __init__(self, platform):

        self.sink    = stream.Endpoint([("data", 128), ("keep", 16)])
        self.source  = stream.Endpoint([("data", 128), ("keep", 16)])

        # Source fifo for storing samples and handling backpressure
        source_fifo = ResetInserter()((stream.SyncFIFO([("data", 128), ("keep", 16)], depth=256, buffered=True)))
        self.source_fifo = source_fifo


        self.sel = Signal()
        self.rst = Signal()

        # DeMux to select between 16bit and 12bit
        ep_demux = stream.Demultiplexer([("data", 128), ("keep", 16)], 2, False)
        ep_demux = stream.BufferizeEndpoints({"source0": stream.DIR_SOURCE, "source1": stream.DIR_SOURCE})(ep_demux)
        self.ep_demux = ep_demux

        # Bit packing gearbox 96b to 128
        self.ep_gearbox = ep_gearbox = ResetInserter()(stream.Gearbox(96, 128, False))

        # Mux to select between 16bit and 12bit
        ep_mux = stream.Multiplexer([("data", 128), ("keep", 16)], 2, False)
        #ep_mux = stream.BufferizeEndpoints({"source" : stream.DIR_SOURCE})(ep_mux)
        self.ep_mux = ep_mux

        # --- TLAST Generation Logic ---
        # This counter tracks the input words to the gearbox (0, 1, 2, 3)
        self.tlast_cnt = tlast_cnt = Signal(2)
        self.tlast_cnt_max = tlast_cnt_max = Signal(2)

        self.comb += [
            If (self.sel == 0,
                self.tlast_cnt_max.eq(0)
                ).Else(
                self.tlast_cnt_max.eq(2)
            )
        ]

        # We increment the counter synchronously on every successful word
        # transfer into the gearbox.
        self.sync += [
            If(self.rst,
                # Reset the counter to 0
                tlast_cnt.eq(0)
            ).Elif(self.source_fifo.sink.ready & self.source_fifo.sink.valid,
               # When counter is 3, reset to 0, else increment
               tlast_cnt.eq(Mux(tlast_cnt == tlast_cnt_max, 0, tlast_cnt + 1))
               )
        ]
        # --- End TLAST Logic ---

        #data_slices = [ep_demux.source1.data[i * 16: i * 16 + 12] for i in range(8)]
        data_slices = [ep_demux.source1.data[i * 16 + 4:(i + 1) * 16] for i in range(8)]
        self.comb += [

            ep_demux.sel.eq(self.sel),
            ep_mux.sel.eq(self.sel),

            # Sink --> Demultiplexer sink
            self.sink.connect(ep_demux.sink),

            # Demultiplexer source0 --> Multiplexer sink0
            ep_demux.source0.connect(ep_mux.sink0),

            # Demultiplexer source1 --> Gearbox
            # Connect valid, ready and data signals separately to slice required 96b data
            ep_gearbox.reset.eq(self.rst),
            ep_gearbox.sink.valid.eq(ep_demux.source1.valid),
            ep_demux.source1.ready.eq(ep_gearbox.sink.ready),
            ep_gearbox.sink.data.eq(Cat(*data_slices)),

            # Gearbox -> multiplexer sink1
            ep_gearbox.source.connect(ep_mux.sink1),

            # Multiplexer -> source_fifo
            self.source_fifo.reset.eq(self.rst),
            ep_mux.source.connect(self.source_fifo.sink),
            # Generate 'last' combinatorially based on the counter
            self.source_fifo.sink.last.eq(tlast_cnt == tlast_cnt_max),
            self.source_fifo.source.connect(self.source),

        ]


# ---------------------------------------------------------------------------------
#  Helper Class: Data2PacketsFSM
# ---------------------------------------------------------------------------------
class Data2PacketsFSM(LiteXModule):
    """
    A LiteXModule wrapper for the VHDL DATA2PACKETS_FSM.

    This module converts an AXI-Stream of samples into a packetized
    AXI-Stream, adding headers. All internal logic runs in the
    clock domain this module is instantiated in.
    """

    def __init__(self, platform, rst_n, pkt_size, pct_hdr_0, pct_hdr_1,
                 sink_buffer_size=4096,
                 source_buffer_size=8192):
        # --- Public I/O Endpoints ---

        # Slave/Sink: Receives sample data
        # The 'keep' signal is not used by the VHDL, so we omit it.
        # .connect() will handle this automatically.
        self.sink = stream.Endpoint([("data", 128), ("keep", 16)])

        # Master/Source: Outputs packetized data
        self.source = stream.Endpoint([("data", 128), ("keep", 16)])

        # Source fifo for storing formed packets and handle backpressure
        source_fifo = ResetInserter()((stream.SyncFIFO([("data", 128), ("keep", 16)], depth=512, buffered=True)))
        self.source_fifo = source_fifo

        # source_fifo.source -> source
        self.comb += [
            self.source_fifo.source.connect(self.source),
            self.source_fifo.reset.eq(~rst_n),
        ]

        # --- Public Output Signals ---

        # FIFO data count (this is an output from the VHDL)
        self.wr_data_count_axis = Signal(10)

        # --- Public Debug Signals ---
        self.drop_samples = Signal()
        self.wr_header = Signal()
        self.state = Signal(4)
        self.wr_cnt = Signal(16)


        # --- VHDL Instance ---
        # sink_fifo.source ->  DATA2PACKETS_FSM -> source_fifo.sink
        vhdl_inst = Instance("DATA2PACKETS_FSM",
                             # Clk/Reset.
                             # ClockSignal() will be mapped by ClockDomainsRenamer
                             i_ACLK=ClockSignal(),
                             i_ARESET_N=rst_n,

                             # Header & Size Inputs
                             i_PCT_SIZE=pkt_size,
                             i_PCT_HDR_0=pct_hdr_0,
                             i_PCT_HDR_1=pct_hdr_1,

                             # AXIS Slave (connects to self.sink)
                             i_S_AXIS_TVALID=self.sink.valid,
                             o_S_AXIS_TREADY=self.sink.ready,
                             i_S_AXIS_TDATA=self.sink.data,
                             i_S_AXIS_TLAST=self.sink.last,

                             # AXIS Master (connects to self.source)
                             o_M_AXIS_TVALID=self.source_fifo.sink.valid,
                             i_M_AXIS_TREADY=self.source_fifo.sink.ready,
                             o_M_AXIS_TDATA=self.source_fifo.sink.data,
                             o_M_AXIS_TLAST=self.source_fifo.sink.last,

                             # Data Count Output
                             i_WR_DATA_COUNT_AXIS=self.source_fifo.level,

                             # Debug Outputs
                             o_DBG_DROP_SAMPLES=self.drop_samples,
                             o_DBG_WR_HEADER=self.wr_header,
                             #o_DBG_STATE=self.state,
                             #o_DBG_WR_CNT=self.wr_cnt,
                             )

        # --- VHDL Converter Setup ---
        self.specials += vhdl_inst
        self.data2packets_fsm_conv = add_vhd2v_converter(platform,
                                                         instance=vhdl_inst,
                                                         files=[
                                                             "gateware/LimeDFB_LiteX/rx_path_top/src/data2packets_fsm.vhd"]
                                                         )
        self._fragment.specials.remove(vhdl_inst)





