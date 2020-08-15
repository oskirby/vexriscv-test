module picotest_lbone (
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
wire wb_reset;
reg [3:0] por_delay = 4'b1111;
always @(posedge clk) begin
    if (clk_locked && por_delay != 0) por_delay <= por_delay - 1;
end
assign wb_reset = (por_delay != 0);


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
    .wb_reset_i(wb_reset),
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
    .wb_reset_i(wb_reset),
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
    .wb_reset_i(wb_reset),
    .wb_adr_i(wb_sram_addr),
    .wb_dat_i(wb_sram_wdata),
    .wb_dat_o(wb_sram_rdata),
    .wb_we_i(wb_sram_we),
    .wb_sel_i(wb_sram_sel),
    .wb_ack_o(wb_sram_ack),
    .wb_cyc_i(wb_sram_cyc),
    .wb_stb_i(wb_sram_stb)
);

// PicoRV32 Wishbone Bus.
wire [31:0]              wb_core_byte_addr;
wire [WB_ADDR_WIDTH-1:0] wb_core_addr;
reg  [WB_DATA_WIDTH-1:0] wb_core_rdata;
wire [WB_DATA_WIDTH-1:0] wb_core_wdata;
wire                     wb_core_we;
wire [WB_SEL_WIDTH-1:0]  wb_core_sel;
reg                      wb_core_ack;
wire                     wb_core_cyc;
wire                     wb_core_stb;
wire                     wb_core_err;
assign wb_core_addr = wb_core_byte_addr[31:32-WB_ADDR_WIDTH];

// Instantiate the Main CPU
picorv32_wb#(
    .PROGADDR_RESET(32'h00000000),  // Execution starts at the beginning of boot ROM.
    .PROGADDR_IRQ(32'h00000010),    // Interrupts jump to address 0x10 in boot ROM.
    .STACKADDR(32'h10000FFC)        // Stack beigns at the end of SRAM.
) picocore(
    .trap(),

    // Wishbone interface.
    .wb_rst_i(wb_reset),
    .wb_clk_i(clk),
    .wbm_adr_o(wb_core_byte_addr),
    .wbm_dat_o(wb_core_wdata),
    .wbm_dat_i(wb_core_rdata),
    .wbm_we_o(wb_core_we),
    .wbm_sel_o(wb_core_sel),
    .wbm_stb_o(wb_core_stb),
    .wbm_ack_i(wb_core_ack),
    .wbm_cyc_o(wb_core_cyc),

    // IRQ interface.
    .irq(32'h00000000),
    .eoi()
);

// Wishbone Address Decoding.
wire [3:0] wb_core_mux;
assign wb_core_mux = wb_core_addr[WB_ADDR_WIDTH-1:WB_ADDR_WIDTH-WB_MUX_WIDTH];

localparam WB_MUX_BOOTROM = 4'h0;
localparam WB_MUX_SRAM =    4'h1;
localparam WB_MUX_LEDPWM =  4'h2;

assign wb_bootrom_addr  = wb_core_addr;
assign wb_bootrom_wdata = wb_core_wdata;
assign wb_bootrom_we    = wb_core_we;
assign wb_bootrom_sel   = wb_core_sel;
assign wb_bootrom_cyc   = wb_core_cyc && (wb_core_mux == WB_MUX_BOOTROM);
assign wb_bootrom_stb   = wb_core_stb;

assign wb_sram_addr     = wb_core_addr;
assign wb_sram_wdata    = wb_core_wdata;
assign wb_sram_we       = wb_core_we;
assign wb_sram_sel      = wb_core_sel;
assign wb_sram_cyc      = wb_core_cyc && (wb_core_mux == WB_MUX_SRAM);
assign wb_sram_stb      = wb_core_stb;

assign wb_ledpwm_addr   = wb_core_addr;
assign wb_ledpwm_wdata  = wb_core_wdata;
assign wb_ledpwm_we     = wb_core_we;
assign wb_ledpwm_sel    = wb_core_sel;
assign wb_ledpwm_cyc    = wb_core_cyc && (wb_core_mux == WB_MUX_LEDPWM);
assign wb_ledpwm_stb    = wb_core_stb;

always @(*) begin
    case (wb_core_mux)
        WB_MUX_BOOTROM: begin
            wb_core_rdata = wb_bootrom_rdata;
            wb_core_ack = wb_bootrom_ack;
        end
        WB_MUX_SRAM: begin
            wb_core_rdata = wb_sram_rdata;
            wb_core_ack = wb_sram_ack;
        end
        WB_MUX_LEDPWM: begin
            wb_core_rdata = wb_ledpwm_rdata;
            wb_core_ack = wb_ledpwm_ack;
        end
        default: begin
            // Treat invalid memory like a black hole.
            wb_core_rdata = 32'hDEADBEEF;
            wb_core_ack = wb_core_cyc && wb_core_stb;
        end
    endcase
end

endmodule
