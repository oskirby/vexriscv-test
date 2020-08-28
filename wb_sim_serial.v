`default_nettype none

module wb_sim_serial#(
	parameter	AW = 32,
    parameter   DW = 32,
    parameter   SIZE = 1024
) (
    input wire              wb_clk_i,
    input wire              wb_reset_i,

    // Wishbone interface.
    input wire [AW-1:0]     wb_adr_i,
    input wire [DW-1:0]     wb_dat_i,
    output reg [DW-1:0]     wb_dat_o,
    input wire              wb_we_i,
    input wire [DW/8-1:0]   wb_sel_i,
    output reg              wb_ack_o,
    input wire              wb_cyc_i,
    input wire              wb_stb_i,

    // Dummy Serial Interface.
    output reg  [7:0] uart_data,
    output reg        uart_valid,

    // DFU Detach signal.
    output wire dfu_detach,
    output wire [4:0] debug
);

// Mock out some stuff.
assign dfu_detach = 1'b0;
assign debug = 0;

// Local register addresses - stolen from the ubiquitous NS16650
localparam REG_USART_RHR = 8'h00;   // Receive Holding Register
localparam REG_USART_THR = 8'h00;   // Transmit Holding Register
localparam REG_USART_IER = 8'h01;   // Interrupt Enable Register
localparam REG_USART_ISR = 8'h02;   // Interrupt Status Register
localparam REG_USART_FCR = 8'h02;   // FIFO Control Register
localparam REG_USART_LCR = 8'h03;   // Line Control Register
localparam REG_USART_MCR = 8'h04;   // Modem Control Register
localparam REG_USART_LSR = 8'h05;   // Line Status Register
localparam REG_USART_MSR = 8'h06;   // Modem Status Register
localparam REG_USART_SCRATCH = 8'h07;
// Extra registers, accessible when DLAB=1
localparam REG_USART_DLL = 8'h10;
localparam REG_USART_DLM = 8'h11;
localparam REG_USART_PLD = 8'h15;

wire stb_valid;
wire [7:0] r_addr;
assign stb_valid = wb_cyc_i && wb_stb_i && !wb_ack_o;
assign r_addr = {wb_adr_i[7:0]};

// Read Port - return zeroes everywhere.
always @(posedge wb_clk_i) begin
    wb_ack_o <= stb_valid;
    wb_dat_o <= (r_addr == REG_USART_ISR) ? 2 : 0; // Fake out the THR-Empty status.
end

// Write Port
always @(posedge wb_clk_i) begin
    if (stb_valid && wb_we_i && wb_sel_i[0]) begin
        uart_data <= wb_dat_i[7:0];
        uart_valid <= 1'b1;
    end
    else begin
        uart_valid <= 1'b0;
    end
end

endmodule
