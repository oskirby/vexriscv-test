
module vextest_tbx (
    input  pin_clk,
    output pin_led,
    inout pin_usb_p,
    input pin_usb_n,
    output pin_pu,

    output pin_sck,
    output pin_csel,
    inout pin_mosi,
    inout pin_miso,
    inout pin_wp,
    inout pin_hold,

    output [3:0] debug
);

localparam WB_DATA_WIDTH = 32;
localparam WB_SEL_WIDTH = (WB_DATA_WIDTH / 8);
localparam WB_ADDR_WIDTH = 32 - $clog2(WB_SEL_WIDTH);
localparam WB_MUX_WIDTH = 4;

// Clock Generation.
wire clk;
wire clk_locked;
pll pll48( .clock_in(pin_clk), .clock_out(clk), .locked( clk_locked ) );

// Reset Generation.
wire rst;
reg [3:0] por_delay = 4'b1111;
always @(posedge clk) begin
    if (clk_locked && por_delay != 0) por_delay <= por_delay - 1;
end
assign rst = (por_delay != 0);

// SPI Pinouts
wire spi_clk;
wire spi_csel;
wire [3:0] spi_dir;
wire [3:0] spi_din;
wire [3:0] spi_dout;
SB_IO #(.PIN_TYPE(6'b101010), .PULLUP(1'b0)) spi_io_d0(
    .D_OUT_0(spi_dout[0]), .OUTPUT_ENABLE(spi_dir[0]), .D_IN_0(spi_din[0]), .PACKAGE_PIN(pin_mosi)
);
SB_IO #(.PIN_TYPE(6'b101010), .PULLUP(1'b0)) spi_io_d1(
    .D_OUT_0(spi_dout[1]), .OUTPUT_ENABLE(spi_dir[1]), .D_IN_0(spi_din[1]), .PACKAGE_PIN(pin_miso)
);
SB_IO #(.PIN_TYPE(6'b101010), .PULLUP(1'b0)) spi_io_d2(
    .D_OUT_0(spi_dout[2]), .OUTPUT_ENABLE(spi_dir[2]), .D_IN_0(spi_din[2]), .PACKAGE_PIN(pin_wp)
);
SB_IO #(.PIN_TYPE(6'b101010), .PULLUP(1'b0)) spi_io_d3(
    .D_OUT_0(spi_dout[3]), .OUTPUT_ENABLE(spi_dir[3]), .D_IN_0(spi_din[3]), .PACKAGE_PIN(pin_hold)
);
assign flash_csel = spi_csel;

// Wishbone connected USB/Serial interface.
wire usb_p_tx;
wire usb_n_tx;
wire usb_p_rx;
wire usb_n_rx;
wire usb_tx_en;
wire dfu_detach;
wire [3:0] usb_debug;

wire [WB_ADDR_WIDTH-1:0] wb_serial_addr;
wire [WB_DATA_WIDTH-1:0] wb_serial_rdata;
wire [WB_DATA_WIDTH-1:0] wb_serial_wdata;
wire                     wb_serial_we;
wire [WB_SEL_WIDTH-1:0]  wb_serial_sel;
wire                     wb_serial_ack;
wire                     wb_serial_cyc;
wire                     wb_serial_stb;

// USB Serial Core.
wb_usb_serial#(
    .AW(WB_ADDR_WIDTH),
    .DW(WB_DATA_WIDTH)
) usb_serial(
    .wb_clk_i(clk),
    .wb_reset_i(rst),

    // Wishbone bus.
    .wb_adr_i(wb_serial_addr),
    .wb_dat_i(wb_serial_wdata),
    .wb_dat_o(wb_serial_rdata),
    .wb_we_i(wb_serial_we),
    .wb_sel_i(wb_serial_sel),
    .wb_ack_o(wb_serial_ack),
    .wb_cyc_i(wb_serial_cyc),
    .wb_stb_i(wb_serial_stb),

    // USB lines.
    .usb_clk(clk),
    .usb_p_tx(usb_p_tx),
    .usb_n_tx(usb_n_tx),
    .usb_p_rx(usb_p_rx),
    .usb_n_rx(usb_n_rx),
    .usb_tx_en(usb_tx_en),
    
    // DFU state and debug
    .dfu_detach(dfu_detach),
    .debug(usb_debug)
);
usb_phy_ice40 usb_phy(
    .pin_usb_p(pin_usb_p),
    .pin_usb_n(pin_usb_n),

    .usb_p_tx(usb_p_tx),
    .usb_n_tx(usb_n_tx),
    .usb_p_rx(usb_p_rx),
    .usb_n_rx(usb_n_rx),
    .usb_tx_en(usb_tx_en)
);
assign pin_pu = 1'b1;

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
assign pin_led = wb_ledpwm_output[0];

// Wishbone connected SPI flash.
wire [WB_ADDR_WIDTH-1:0] wb_spi_addr;
wire [WB_DATA_WIDTH-1:0] wb_spi_rdata;
wire [WB_DATA_WIDTH-1:0] wb_spi_wdata;
wire                     wb_spi_we;
wire [WB_SEL_WIDTH-1:0]  wb_spi_sel;
wire                     wb_spi_ack;
wire                     wb_spi_cyc;
wire                     wb_spi_stb;
wb_qspi_flash#(
    .AW(WB_ADDR_WIDTH),
    .DW(WB_DATA_WIDTH)
) vexspi(
    .wb_clk_i(clk),
    .wb_reset_i(rst),
    .wb_adr_i(wb_spi_addr),
    .wb_dat_i(wb_spi_wdata),
    .wb_dat_o(wb_spi_rdata),
    .wb_we_i(wb_spi_we),
    .wb_sel_i(wb_spi_sel),
    .wb_ack_o(wb_spi_ack),
    .wb_cyc_i(wb_spi_cyc),
    .wb_stb_i(wb_spi_stb),

    .spi_clk(spi_clk),
    .spi_sel(spi_csel),
    .spi_d_out(spi_dout),
    .spi_d_in(spi_din),
    .spi_d_dir(spi_dir)
);

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
generate if (0) begin
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
end else begin
    wb_spram_256k#(
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
end endgenerate

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

// Create the Wishbone crossbar.
wbcxbar#(
    .NM(2), // One port each for instruction and data access from the CPU.
    .NS(5), // One port for SRAM, boot ROM and PWM LED driver.
    .AW(WB_ADDR_WIDTH),
    .DW(WB_DATA_WIDTH),
    .MUXWIDTH(4),
    .SLAVE_MUX({
        { 4'h0 },  // Base address of the boot ROM.
        { 4'h1 },  // Base address of the SRAM.
        { 4'h2 },  // Base address of the PWM driver.
        { 4'h3 },  // Base address of QSPI Flash.
        { 4'h4 }   // Base address of the USB Serial interface.
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
    .o_scyc  ({wb_bootrom_cyc,   wb_sram_cyc,   wb_ledpwm_cyc,   wb_spi_cyc,   wb_serial_cyc}),
    .o_sstb  ({wb_bootrom_stb,   wb_sram_stb,   wb_ledpwm_stb,   wb_spi_stb,   wb_serial_stb}),
    .o_swe   ({wb_bootrom_we,    wb_sram_we,    wb_ledpwm_we,    wb_spi_wei,   wb_serial_we}),
    .o_saddr ({wb_bootrom_addr,  wb_sram_addr,  wb_ledpwm_addr,  wb_spi_addr,  wb_serial_addr}),
    .o_sdata ({wb_bootrom_wdata, wb_sram_wdata, wb_ledpwm_wdata, wb_spi_wdata, wb_serial_wdata}),
    .o_ssel  ({wb_bootrom_sel,   wb_sram_sel,   wb_ledpwm_sel,   wb_spi_sel,   wb_serial_sel}),
    .i_sack  ({wb_bootrom_ack,   wb_sram_ack,   wb_ledpwm_ack,   wb_spi_ack,   wb_serial_ack}),
    .i_serr  ({1'b0,             1'b0,          1'b0,            1'b0,         1'b0}),
    .i_sdata ({wb_bootrom_rdata, wb_sram_rdata, wb_ledpwm_rdata, wb_spi_rdata, wb_serial_rdata})
);

endmodule
