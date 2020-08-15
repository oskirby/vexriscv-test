
module picotest_tbx (
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
    wire [WB_ADDR_WIDTH-1:0] wb_core_addr;
    reg  [WB_DATA_WIDTH-1:0] wb_core_rdata;
    wire [WB_DATA_WIDTH-1:0] wb_core_wdata;
    wire                     wb_core_we;
    wire [WB_SEL_WIDTH-1:0]  wb_core_sel;
    reg                      wb_core_ack;
    wire                     wb_core_cyc;
    wire                     wb_core_stb;
    wire                     wb_core_err;

    // Instantiate the Main CPU
    picorv32_wb#(
        .STACKADDR(32'h10000000) // Stack beigns at the start of SRAM.
    ) picocore(
        .trap(),

        // Wishbone interface.
        .wb_rst_i(1'b0),
        .wb_clk_i(clk),
        .wbm_adr_o(wb_core_addr),
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
    wire [3:0] wb_core_addr_mux = wb_core_addr[WB_ADDR_WIDTH-1:WB_ADDR_WIDTH-4];

    assign wb_bootrom_addr  = wb_core_addr;
    assign wb_bootrom_wdata = wb_core_wdata;
    assign wb_bootrom_we    = wb_core_we;
    assign wb_bootrom_sel   = wb_core_sel;
    assign wb_bootrom_cyc   = wb_core_cyc && (wb_core_addr_mux == 4'b0000);
    assign wb_bootrom_stb   = wb_core_stb;

    assign wb_sram_addr     = wb_core_addr;
    assign wb_sram_wdata    = wb_core_wdata;
    assign wb_sram_we       = wb_core_we;
    assign wb_sram_sel      = wb_core_sel;
    assign wb_sram_cyc      = wb_core_cyc && (wb_core_addr_mux == 4'b0001);
    assign wb_sram_stb      = wb_core_stb;

    always @(*) begin
        case (wb_core_addr_mux)
            4'b0000: begin
                wb_core_rdata <= wb_bootrom_rdata;
                wb_core_ack <= wb_bootrom_ack;
            end
            4'b0001: begin
                wb_core_rdata <= wb_sram_rdata;
                wb_core_ack <= wb_sram_ack;
            end
            default: begin
                // Treat invalid memory like a black hole.
                wb_core_rdata <= 32'hDEADBEEF;
                wb_core_ack <= wb_core_cyc && wb_core_stb;
            end
        endcase
    end

    assign debug = wb_core_rdata[3:0];

endmodule
