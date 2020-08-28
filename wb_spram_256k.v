module wb_spram_256k#(
    parameter   AW = 32,
    parameter   DW = 32,
    parameter   SIZE = (256 * 1024) / 16
) (
    // Wishbone interface.
    input           wb_clk_i,
    input           wb_reset_i,
    input  [AW-1:0] wb_adr_i,
    input  [DW-1:0] wb_dat_i,
    output [DW-1:0] wb_dat_o,
    input           wb_we_i,
    input  [DW/8-1:0] wb_sel_i,
    output reg      wb_ack_o,
    input           wb_cyc_i,
    input           wb_stb_i
);

localparam SIZE_BITS = $clog2(SIZE);
wire [SIZE_BITS-1:0] spram_addr;
assign spram_addr = wb_adr_i[SIZE_BITS-1:0];

wire stb_valid;
assign stb_valid = wb_cyc_i && wb_stb_i && ~wb_ack_o;
always @(posedge wb_clk_i) begin
    wb_ack_o <= stb_valid;
end

// Least significant 16-bits.
SB_SPRAM256KA sp_ram_low(
    .ADDRESS(spram_addr),
    .DATAIN(wb_dat_i[15:0]),
    .MASKWREN({wb_sel_i[0], wb_sel_i[0], wb_sel_i[1], wb_sel_i[1]}),
    .WREN(stb_valid && wb_we_i),
    .CHIPSELECT(wb_cyc_i),
    .CLOCK(wb_clk_i),
    .STANDBY(1'b0),
    .SLEEP(1'b0),
    .POWEROFF(1'b1),
    .DATAOUT(wb_dat_o[15:0])
);

// Most significant 16-bits.
generate if (DW > 16) begin
    SB_SPRAM256KA sp_ram_high(
        .ADDRESS(spram_addr),
        .DATAIN(wb_dat_i[DW-1:16]),
        .MASKWREN({wb_sel_i[2], wb_sel_i[2], wb_sel_i[3], wb_sel_i[3]}),
        .WREN(stb_valid && wb_we_i),
        .CHIPSELECT(wb_cyc_i),
        .CLOCK(wb_clk_i),
        .STANDBY(1'b0),
        .SLEEP(1'b0),
        .POWEROFF(1'b1),
        .DATAOUT(wb_dat_o[DW-1:16])
    );
end endgenerate

endmodule
