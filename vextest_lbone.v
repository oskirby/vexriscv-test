
module vextest_lbone (
    input refclk,
    output [3:0] led
);

localparam WB_DATA_WIDTH = 32;
localparam WB_SEL_WIDTH = (WB_DATA_WIDTH / 8);
localparam WB_ADDR_WIDTH = 32 - $clog2(WB_SEL_WIDTH);
localparam WB_MUX_WIDTH = 4;

// Clock Generation.
wire clk;
wire clk_locked;
pll pll48( .clkin(refclk), .clkout0(clk), .locked(clk_locked) );

// Reset Generation.
wire rst;
reg [3:0] por_delay = 4'b1111;
always @(posedge clk) begin
    if (clk_locked && por_delay != 0) por_delay <= por_delay - 1;
end
assign rst = (por_delay != 0);


// Wishbone connected LED driver.
wire [WB_ADDR_WIDTH-1:0] wb_ledpwm_addr;
wire [WB_DATA_WIDTH-1:0] wb_ledpwm_rdata;
wire [WB_DATA_WIDTH-1:0] wb_ledpwm_wdata;
wire                     wb_ledpwm_we;
wire [WB_SEL_WIDTH-1:0]  wb_ledpwm_sel;
wire                     wb_ledpwm_ack;
wire                     wb_ledpwm_cyc;
wire                     wb_ledpwm_stb;
wire [3:0]               wb_ledpwm_output;
wbledpwm#(
    .AW(WB_ADDR_WIDTH),
    .DW(WB_DATA_WIDTH),
    .NLEDS(4)
) vexledpwm(
    .wb_clk_i(clk),
    .wb_reset_i(rst),
    .wb_adr_i(wb_ledpwm_addr),
    .wb_dat_i(wb_ledpwm_wdata),
    .wb_dat_o(wb_ledpwm_rdata),
    .wb_we_i(wb_ledpwm_we),
    .wb_sel_i(wb_ledpwm_sel),
    .wb_ack_o(wb_ledpwm_ack),
    .wb_cyc_i(wb_ledpwm_cyc),
    .wb_stb_i(wb_ledpwm_stb),

    .leds(wb_ledpwm_output)
);
assign led = ~wb_ledpwm_output;

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
    .wb_reset_i(rst),
    .wb_adr_i(wb_bootrom_addr),
    .wb_dat_i(wb_bootrom_wdata),
    .wb_dat_o(wb_bootrom_rdata),
    .wb_we_i(wb_bootrom_we),
    .wb_sel_i(wb_bootrom_sel),
    .wb_ack_o(wb_bootrom_ack),
    .wb_cyc_i(wb_bootrom_cyc),
    .wb_stb_i(wb_bootrom_stb)
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
    .wb_reset_i(rst),
    .wb_adr_i(wb_sram_addr),
    .wb_dat_i(wb_sram_wdata),
    .wb_dat_o(wb_sram_rdata),
    .wb_we_i(wb_sram_we),
    .wb_sel_i(wb_sram_sel),
    .wb_ack_o(wb_sram_ack),
    .wb_cyc_i(wb_sram_cyc),
    .wb_stb_i(wb_sram_stb)
);

// Instruction Bus wishbone signals (classic)
wire [WB_ADDR_WIDTH-1:0] wbc_ibus_addr;
wire [WB_DATA_WIDTH-1:0] wbc_ibus_rdata;
wire [WB_DATA_WIDTH-1:0] wbc_ibus_wdata;
wire                     wbc_ibus_we;
wire [WB_SEL_WIDTH-1:0]  wbc_ibus_sel;
wire                     wbc_ibus_ack;
wire                     wbc_ibus_cyc;
wire                     wbc_ibus_stb;
wire                     wbc_ibus_err;
wire [1:0]               wbc_ibus_bte;
wire [2:0]               wbc_ibus_cti;

// Data Bus wishbone signals (classic)
wire [WB_ADDR_WIDTH-1:0] wbc_dbus_addr;
wire [WB_DATA_WIDTH-1:0] wbc_dbus_rdata;
wire [WB_DATA_WIDTH-1:0] wbc_dbus_wdata;
wire                     wbc_dbus_we;
wire [WB_SEL_WIDTH-1:0]  wbc_dbus_sel;
wire                     wbc_dbus_ack;
wire                     wbc_dbus_cyc;
wire                     wbc_dbus_stb;
wire                     wbc_dbus_err;
wire [1:0]               wbc_dbus_bte;
wire [2:0]               wbc_dbus_cti;

// Instantiate the Main CPU
VexRiscv vexcore(
    .externalResetVector(32'h00000000),
    .timerInterrupt(1'b0),
    .softwareInterrupt(1'b0),
    .externalInterruptArray(32'h00000000),

    // Instruction Bus.
    .iBusWishbone_CYC(wbc_ibus_cyc),
    .iBusWishbone_STB(wbc_ibus_stb),
    .iBusWishbone_ACK(wbc_ibus_ack),
    .iBusWishbone_WE(wbc_ibus_we),
    .iBusWishbone_ADR(wbc_ibus_addr),
    .iBusWishbone_DAT_MISO(wbc_ibus_rdata),
    .iBusWishbone_DAT_MOSI(wbc_ibus_wdata),
    .iBusWishbone_SEL(wbc_ibus_sel),
    .iBusWishbone_ERR(wbc_ibus_err),
    .iBusWishbone_BTE(wbc_ibus_bte),
    .iBusWishbone_CTI(wbc_ibus_cti), 

    // Data Bus.
    .dBusWishbone_CYC(wbc_dbus_cyc),
    .dBusWishbone_STB(wbc_dbus_stb),
    .dBusWishbone_ACK(wbc_dbus_ack),
    .dBusWishbone_WE(wbc_dbus_we),
    .dBusWishbone_ADR(wbc_dbus_addr),
    .dBusWishbone_DAT_MISO(wbc_dbus_rdata),
    .dBusWishbone_DAT_MOSI(wbc_dbus_wdata),
    .dBusWishbone_SEL(wbc_dbus_sel),
    .dBusWishbone_ERR(wbc_dbus_err),
    .dBusWishbone_BTE(wbc_dbus_bte),
    .dBusWishbone_CTI(wbc_dbus_cti),

    .clk(clk),
    .reset(rst)
);

generate if (0) begin
    // Convert the VexRiscV from classic to pipelined wishbone.
    wire [WB_ADDR_WIDTH-1:0] wb_ibus_addr;
    wire [WB_DATA_WIDTH-1:0] wb_ibus_rdata;
    wire [WB_DATA_WIDTH-1:0] wb_ibus_wdata;
    wire                     wb_ibus_we;
    wire [WB_SEL_WIDTH-1:0]  wb_ibus_sel;
    wire                     wb_ibus_ack;
    wire                     wb_ibus_cyc;
    wire                     wb_ibus_stb;
    wire                     wb_ibus_err;
    wire                     wb_ibus_stall;
    wbc2pipeline#(
        .AW(WB_ADDR_WIDTH),
        .DW(WB_DATA_WIDTH)
    ) vexibus_pipelined(
        .i_clk(clk),
        .i_reset(rst),

        .i_mcyc(wbc_ibus_cyc),
        .i_mstb(wbc_ibus_stb),
        .i_mwe(wbc_ibus_we),
        .i_maddr(wbc_ibus_addr),
        .i_mdata(wbc_ibus_wdata),
        .i_msel(wbc_ibus_sel),
        .o_mack(wbc_ibus_ack),
        .o_mdata(wbc_ibus_rdata),
        .o_merr(wbc_ibus_err),
        .i_mcti(wbc_ibus_cti),
        .i_mbte(wbc_ibus_bte),

        .o_scyc(wb_ibus_cyc),
        .o_sstb(wb_ibus_stb),
        .o_swe(wb_ibus_we),
        .o_saddr(wb_ibus_addr),
        .o_sdata(wb_ibus_wdata),
        .o_ssel(wb_ibus_sel),
        .i_sstall(wb_ibus_stall),
        .i_sack(wb_ibus_ack),
        .i_sdata(wb_ibus_rdata),
        .i_serr(wb_ibus_err)
    );

    wire [WB_ADDR_WIDTH-1:0] wb_dbus_addr;
    wire [WB_DATA_WIDTH-1:0] wb_dbus_rdata;
    wire [WB_DATA_WIDTH-1:0] wb_dbus_wdata;
    wire                     wb_dbus_we;
    wire [WB_SEL_WIDTH-1:0]  wb_dbus_sel;
    wire                     wb_dbus_ack;
    wire                     wb_dbus_cyc;
    wire                     wb_dbus_stb;
    wire                     wb_dbus_err;
    wire                     wb_dbus_stall;
    wbc2pipeline#(
        .AW(WB_ADDR_WIDTH),
        .DW(WB_DATA_WIDTH)
    ) vexdbus_pipelined(
        .i_clk(clk),
        .i_reset(rst),

        .i_mcyc(wbc_dbus_cyc),
        .i_mstb(wbc_dbus_stb),
        .i_mwe(wbc_dbus_we),
        .i_maddr(wbc_dbus_addr),
        .i_mdata(wbc_dbus_wdata),
        .i_msel(wbc_dbus_sel),
        .o_mack(wbc_dbus_ack),
        .o_mdata(wbc_dbus_rdata),
        .o_merr(wbc_dbus_err),
        .i_mcti(wbc_dbus_cti),
        .i_mbte(wbc_dbus_bte),

        .o_scyc(wb_dbus_cyc),
        .o_sstb(wb_dbus_stb),
        .o_swe(wb_dbus_we),
        .o_saddr(wb_dbus_addr),
        .o_sdata(wb_dbus_wdata),
        .o_ssel(wb_dbus_sel),
        .i_sstall(wb_dbus_stall),
        .i_sack(wb_dbus_ack),
        .i_sdata(wb_dbus_rdata),
        .i_serr(wb_dbus_err)
    );

    // Create the Wishbone crossbar.
    wbxbar#(
        .NM(2), // One port each for instruction and data access from the CPU.
        .NS(3), // One port for SRAM, boot ROM and PWM LED driver.
        .AW(WB_ADDR_WIDTH),
        .DW(WB_DATA_WIDTH),
        .SLAVE_ADDR({
            { 4'h0, {(WB_ADDR_WIDTH-4){1'b0}}},  // Base address of the boot ROM.
            { 4'h1, {(WB_ADDR_WIDTH-4){1'b0}}},  // Base address of the SRAM.
            { 4'h2, {(WB_ADDR_WIDTH-4){1'b0}}}   // Base address of the PWM driver.
        }),
        .SLAVE_MASK({
            { 4'b1111, {(WB_ADDR_WIDTH-4){1'b0}}},  // Address mask of the boot ROM.
            { 4'b1111, {(WB_ADDR_WIDTH-4){1'b0}}},  // Address mask of the SRAM.
            { 4'b1111, {(WB_ADDR_WIDTH-4){1'b0}}}   // Address mask of the PWM driver.
        })
    ) vexcrossbar (
        .i_clk(clk),
        .i_reset(rst),

        // Crossbar Master Ports.
        .i_mcyc  ({wb_ibus_cyc,   wb_dbus_cyc}),
        .i_mstb  ({wb_ibus_stb,   wb_dbus_cyc}),
        .i_mwe   ({wb_ibus_we,    wb_dbus_we}),
        .i_maddr ({wb_ibus_addr,  wb_dbus_addr}),
        .i_mdata ({wb_ibus_wdata, wb_dbus_wdata}),
        .i_msel  ({wb_ibus_sel,   wb_dbus_sel}),
        .o_mstall({wb_ibus_stall, wb_dbus_stall}),
        .o_mack  ({wb_ibus_ack,   wb_dbus_ack}),
        .o_merr  ({wb_ibus_err,   wb_dbus_err}),
        .o_mdata ({wb_ibus_rdata, wb_dbus_rdata}),

        // Crossbar Slave Ports.
        .o_scyc  ({wb_bootrom_cyc,   wb_sram_cyc,   wb_ledpwm_cyc}),
        .o_sstb  ({wb_bootrom_stb,   wb_sram_stb,   wb_ledpwm_stb}),
        .o_swe   ({wb_bootrom_we,    wb_sram_we,    wb_ledpwm_we}),
        .o_saddr ({wb_bootrom_addr,  wb_sram_addr,  wb_ledpwm_addr}),
        .o_sdata ({wb_bootrom_wdata, wb_sram_wdata, wb_ledpwm_wdata}),
        .o_ssel  ({wb_bootrom_sel,   wb_sram_sel,   wb_ledpwm_sel}),
        .i_sstall({1'b0,             1'b0,          1'b0}),
        .i_sack  ({wb_bootrom_ack,   wb_sram_ack,   wb_ledpwm_ack}),
        .i_serr  ({1'b0,             1'b0,          1'b0}),
        .i_sdata ({wb_bootrom_rdata, wb_sram_rdata, wb_ledpwm_rdata})
    );
end else begin
    // Create the Wishbone crossbar.
    wbcxbar#(
        .NM(2), // One port each for instruction and data access from the CPU.
        .NS(3), // One port for SRAM, boot ROM and PWM LED driver.
        .AW(WB_ADDR_WIDTH),
        .DW(WB_DATA_WIDTH),
        .MUXWIDTH(4),
        .SLAVE_MUX({
            { 4'h0 },  // Base address of the boot ROM.
            { 4'h1 },  // Base address of the SRAM.
            { 4'h2 }   // Base address of the PWM driver.
        })
    ) vexcrossbar (
        .i_clk(clk),
        .i_reset(rst),

        // Crossbar Master Ports.
        .i_mcyc  ({wbc_ibus_cyc,   wbc_dbus_cyc}),
        .i_mstb  ({wbc_ibus_stb,   wbc_dbus_cyc}),
        .i_mwe   ({wbc_ibus_we,    wbc_dbus_we}),
        .i_maddr ({wbc_ibus_addr,  wbc_dbus_addr}),
        .i_mdata ({wbc_ibus_wdata, wbc_dbus_wdata}),
        .i_msel  ({wbc_ibus_sel,   wbc_dbus_sel}),
        .o_mack  ({wbc_ibus_ack,   wbc_dbus_ack}),
        .o_merr  ({wbc_ibus_err,   wbc_dbus_err}),
        .o_mdata ({wbc_ibus_rdata, wbc_dbus_rdata}),

        // Crossbar Slave Ports.
        .o_scyc  ({wb_bootrom_cyc,   wb_sram_cyc,   wb_ledpwm_cyc}),
        .o_sstb  ({wb_bootrom_stb,   wb_sram_stb,   wb_ledpwm_stb}),
        .o_swe   ({wb_bootrom_we,    wb_sram_we,    wb_ledpwm_we}),
        .o_saddr ({wb_bootrom_addr,  wb_sram_addr,  wb_ledpwm_addr}),
        .o_sdata ({wb_bootrom_wdata, wb_sram_wdata, wb_ledpwm_wdata}),
        .o_ssel  ({wb_bootrom_sel,   wb_sram_sel,   wb_ledpwm_sel}),
        .i_sack  ({wb_bootrom_ack,   wb_sram_ack,   wb_ledpwm_ack}),
        .i_serr  ({1'b0,             1'b0,          1'b0}),
        .i_sdata ({wb_bootrom_rdata, wb_sram_rdata, wb_ledpwm_rdata})
    );
end endgenerate

endmodule
