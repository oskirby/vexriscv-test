
module vextest_tbx (
        input  pin_clk,

        output pin_led,

        output [3:0] debug
    );

    localparam WB_ADDR_WIDTH = 32;
    localparam WB_DATA_WIDTH = 32;
    localparam WB_SEL_WIDTH = (WB_DATA_WIDTH / 8);

    wire clk;
    wire clk_locked;

    // Use an icepll generated pll
    pll pll48( .clock_in(pin_clk), .clock_out(clk), .locked( clk_locked ) );

    // Instantiate the boot ROM.
    wire [WB_ADDR_WIDTH-1:0] wb_bootrom_addr;
    wire [WB_DATA_WIDTH-1:0] wb_bootrom_rdata;
    wire [WB_DATA_WIDTH-1:0] wb_bootrom_wdata;
    wire                     wb_bootrom_we;
    wire [WB_SEL_WIDTH-1:0]  wb_bootrom_sel;
    wire                     wb_bootrom_ack;
    wire                     wb_bootrom_cyc;
    wire                     wb_bootrom_stb;
    bootrom#(
        .AW(WB_ADDR_WIDTH),
        .DW(WB_DATA_WIDTH)
    ) vexbootrom(
        .wb_clk_i(clk),
        .wb_reset_i(1'b0),
        .wb_adr_i(wb_bootrom_addr),
        .wb_dat_i(wb_bootrom_wdata),
        .wb_dat_o(wb_bootrom_rdata),
        .wb_we_i(wb_bootrom_we),
        .wb_sel_i(wb_bootrom_sel),
        .wb_ack_o(wb_bootrom_ack),
        .wb_cyc_i(wb_bootrom_cyc),
        .wb_stb_i(wb_bootrom_stb),
    );

    // Instantiate the SRAM.
    wire [WB_ADDR_WIDTH-1:0] wb_sram_addr;
    wire [WB_DATA_WIDTH-1:0] wb_sram_rdata;
    wire [WB_DATA_WIDTH-1:0] wb_sram_wdata;
    wire                     wb_sram_we;
    wire [WB_SEL_WIDTH-1:0]  wb_sram_sel;
    wire                     wb_sram_ack;
    wire                     wb_sram_cyc;
    wire                     wb_sram_stb;
    wbsram#(
        .AW(WB_ADDR_WIDTH),
        .DW(WB_DATA_WIDTH)
    ) vexsram(
        .wb_clk_i(clk),
        .wb_reset_i(1'b0),
        .wb_adr_i(wb_sram_addr),
        .wb_dat_i(wb_sram_wdata),
        .wb_dat_o(wb_sram_rdata),
        .wb_we_i(wb_sram_we),
        .wb_sel_i(wb_sram_sel),
        .wb_ack_o(wb_sram_ack),
        .wb_cyc_i(wb_sram_cyc),
        .wb_stb_i(wb_sram_stb)
    );

    // Instruction Bus wishbone signals.
    wire [WB_ADDR_WIDTH-1:0] wb_ibus_addr;
    wire [WB_DATA_WIDTH-1:0] wb_ibus_rdata;
    wire [WB_DATA_WIDTH-1:0] wb_ibus_wdata;
    wire                     wb_ibus_we;
    wire [WB_SEL_WIDTH-1:0]  wb_ibus_sel;
    wire                     wb_ibus_ack;
    wire                     wb_ibus_cyc;
    wire                     wb_ibus_stb;
    wire                     wb_ibus_err;

    // Data Bus wishbone signals.
    wire [WB_ADDR_WIDTH-1:0] wb_dbus_addr;
    wire [WB_DATA_WIDTH-1:0] wb_dbus_rdata;
    wire [WB_DATA_WIDTH-1:0] wb_dbus_wdata;
    wire                     wb_dbus_we;
    wire [WB_SEL_WIDTH-1:0]  wb_dbus_sel;
    wire                     wb_dbus_ack;
    wire                     wb_dbus_cyc;
    wire                     wb_dbus_stb;
    wire                     wb_dbus_err;


    // Instantiate the Main CPU
    VexRiscv vexcore(
        .externalResetVector(32'h00000000),
        .timerInterrupt(1'b0),
        .softwareInterrupt(1'b0),
        .externalInterruptArray(32'h00000000),

        // Instruction Bus.
        .iBusWishbone_CYC(wb_ibus_cyc),
        .iBusWishbone_STB(wb_ibus_stb),
        .iBusWishbone_ACK(wb_ibus_ack),
        .iBusWishbone_WE(wb_ibus_we),
        .iBusWishbone_ADR(wb_ibus_addr),
        .iBusWishbone_DAT_MISO(wb_ibus_rdata),
        .iBusWishbone_DAT_MOSI(wb_ibus_wdata),
        .iBusWishbone_SEL(wb_ibus_sel),
        .iBusWishbone_ERR(wb_ibus_err),
        // Bursts are not supported.
        //.iBusWishbone_BTE(????),
        //.iBusWishbone_CTI(????), 

        // Data Bus.
        .dBusWishbone_CYC(wb_dbus_cyc),
        .dBusWishbone_STB(wb_dbus_stb),
        .dBusWishbone_ACK(wb_dbus_ack),
        .dBusWishbone_WE(wb_dbus_we),
        .dBusWishbone_ADR(wb_dbus_addr),
        .dBusWishbone_DAT_MISO(wb_dbus_rdata),
        .dBusWishbone_DAT_MOSI(wb_dbus_wdata),
        .dBusWishbone_SEL(wb_dbus_sel),
        .dBusWishbone_ERR(wb_dbus_err),
        // Bursts are not supported.
        //.dBusWishbone_BTE(????),
        //.dBusWishbone_CTI(????),

        .clk(clk),
        .reset(1'b0)
    );

    // Create the Wishbone crossbar.
    wbxbar#(
        .NM(2), // One port each for instruction and data access from the CPU.
        .NS(2), // For now, just one port for SRAM and one for the boot ROM.
        .AW(WB_ADDR_WIDTH),
        .DW(WB_DATA_WIDTH),
	    .SLAVE_ADDR({
			{ 4'b0000, {(WB_ADDR_WIDTH-4){1'b0}}},  // Base address of the boot ROM.
			{ 4'b0001, {(WB_ADDR_WIDTH-4){1'b0}}}   // Base address of the SRAM.
        }),
	    .SLAVE_MASK({
			{ 4'b1111, {(WB_ADDR_WIDTH-4){1'b0}}},  // Address mask of the boot ROM.
			{ 4'b1111, {(WB_ADDR_WIDTH-4){1'b0}}}   // Address mask of the SRAM.
        }),
    ) vexcrossbar(
        .i_clk(clk),
        .i_reset(1'b0),

        // Crossbar Master Ports.
        .i_mcyc  ({wb_ibus_cyc,   wb_dbus_cyc}),
        .i_mstb  ({wb_ibus_stb,   wb_dbus_cyc}),
        .i_mwe   ({wb_ibus_we,    wb_dbus_we}),
        .i_maddr ({wb_ibus_addr,  wb_dbus_addr}),
        .i_mdata ({wb_ibus_wdata, wb_dbus_wdata}),
        .i_msel  ({wb_ibus_sel,   wb_dbus_sel}),
        //.o_mstall(????)
        .o_mack  ({wb_ibus_ack,   wb_dbus_ack}),
        .o_merr  ({wb_ibus_err,   wb_dbus_err}),
        .o_mdata ({wb_ibus_rdata, wb_dbus_rdata}),

        // Crossbar Slave Ports.
        .o_scyc  ({wb_bootrom_cyc,   wb_sram_cyc}),
        .o_sstb  ({wb_bootrom_stb,   wb_sram_stb}),
        .o_swe   ({wb_bootrom_we,    wb_sram_we}),
        .o_saddr ({wb_bootrom_addr,  wb_sram_addr}),
        .o_sdata ({wb_bootrom_wdata, wb_sram_wdata}),
        .o_ssel  ({wb_bootrom_sel,   wb_sram_sel}),
        .i_sstall({1'b0,             1'b0}),
        .i_sack  ({wb_bootrom_ack,   wb_sram_ack}),
        .i_serr  ({1'b0,             1'b0}),
        .i_sdata ({wb_bootrom_rdata, wb_sram_rdata}),
    );

    assign debug = wb_dbus_rdata[3:0];

endmodule
