module wbcxbar#(
    parameter NM = 4,
    parameter NS = 8,
    parameter AW = 32,
    parameter DW = 32,
    parameter SW = DW/8,
    parameter MUXWIDTH = 3,
    parameter [NS*MUXWIDTH-1:0] SLAVE_MUX = {
        { 3'b111 },
        { 3'b110 },
        { 3'b101 },
        { 3'b100 },
        { 3'b011 },
        { 3'b010 },
        { 3'b001 },
        { 3'b000 }
    }
) (
    input i_clk,
    input i_reset,

    // Wishbone Master Signals.
    input [NM-1:0]      i_mcyc,
    input [NM-1:0]      i_mstb,
    input [NM-1:0]      i_mwe,
    input [NM*AW-1:0]   i_maddr,
    input [NM*DW-1:0]   i_mdata,
    input [NM*SW-1:0]   i_msel,
    output [NM-1:0]     o_mack,
    output [NM*DW-1:0]  o_mdata,
    output [NM-1:0]     o_merr,

    // Wishbone Slave Signals.
    output [NS-1:0]     o_scyc,
    output [NS-1:0]     o_sstb,
    output [NS-1:0]     o_swe,
    output [NS*AW-1:0]  o_saddr,
    output [NS*DW-1:0]  o_sdata,
    output [NS*SW-1:0] o_ssel,
    input [NS-1:0]      i_sack,
    input [NS*DW-1:0]   i_sdata,
    input [NS-1:0]      i_serr
);

localparam M_GRANT_BITS = $clog2(NS+1);
localparam M_GRANT_SIZE = (1 << M_GRANT_BITS);
localparam M_GRANT_NONE = {(M_GRANT_BITS){1'b1}};

localparam S_GRANT_BITS = $clog2(NM+1);
localparam S_GRANT_SIZE = (1 << S_GRANT_BITS);
localparam S_GRANT_NONE = {(S_GRANT_BITS){1'b1}};

// Muxing Selections.
wire [M_GRANT_BITS-1:0] m_decode[NM-1:0];   /* Decoded slave the master is requesting. */
reg [M_GRANT_BITS-1:0] m_grant[NM-1:0];     /* Slave to which a master has been granted access */

// Master input signals, padded for muxing.
wire            m_mcyc[S_GRANT_SIZE-1:0];
wire            m_mstb[S_GRANT_SIZE-1:0];
wire            m_mwe[S_GRANT_SIZE-1:0];
wire [AW-1:0]   m_maddr[S_GRANT_SIZE-1:0];
wire [DW-1:0]   m_mdata[S_GRANT_SIZE-1:0];
wire [SW-1:0]   m_msel[S_GRANT_SIZE-1:0];

// Slave input signals, padded for muxing.
wire            m_sack[M_GRANT_SIZE-1:0];
wire [DW-1:0]   m_sdata[M_GRANT_SIZE-1:0];
wire            m_serr[M_GRANT_SIZE-1:0];

genvar gM, gS;
integer iM, iS;
generate
    for (gM = 0; gM < NM; gM = gM + 1) begin
        // Wire inputs from master to the mux array.
        assign m_mcyc[gM]  = i_mcyc[gM];
        assign m_mstb[gM]  = i_mstb[gM];
        assign m_mwe[gM]   = i_mwe[gM];
        assign m_maddr[gM] = i_maddr[AW+(gM*AW)-1:(gM*AW)];
        assign m_mdata[gM] = i_mdata[DW+(gM*DW)-1:(gM*DW)];
        assign m_msel[gM]  = i_msel[SW+(gM*SW)-1:(gM*SW)];

        // Wire outputs to master from the mux array.
        assign o_mack[gM]                   = m_sack[m_grant[gM]];
        assign o_mdata[DW+(gM*DW)-1:gM*DW]  = m_sdata[m_grant[gM]];
        assign o_merr[gM]                   = m_serr[m_grant[gM]];

        // Decode the master address.
        wbcdecoder#(
            .ADDRWIDTH(AW),
            .MUXWIDTH(MUXWIDTH),
            .OUTWIDTH(M_GRANT_BITS),
            .SLAVE_MUX(SLAVE_MUX)
        ) m_decode_inst (
            .addr(m_maddr[gM]),
            .decode(m_decode[gM])
        );
    end
    // Fill the remainder of the mux array with empty data, to
    // set the un-selected state of the outputs to the slave.
    for (gM = NM; gM < S_GRANT_SIZE; gM = gM + 1) begin
        assign m_mcyc[gM]  = 1'b0;
        assign m_mstb[gM]  = 1'b0;
        assign m_mwe[gM]   = 1'b0;
        assign m_maddr[gM] = {(AW){1'b0}};
        assign m_mdata[gM] = {(DW){1'b0}};
        assign m_msel[gM]  = {(DW/8){1'b0}};
    end

    for (gS = 0; gS < NS; gS = gS + 1) begin
        // Select the reverse grant direction.
        reg [S_GRANT_BITS-1:0] s_grant;
        always @(*) begin
            s_grant = S_GRANT_NONE;
            for (iM = 0; iM < NM; iM = iM + 1) begin
                if (m_grant[iM] == gS) s_grant = iM[S_GRANT_BITS-1:0];
            end
        end

        // Wire inputs from slave to the mux array.
        assign m_sack[gS]  = i_sack[gS];
        assign m_sdata[gS] = i_sdata[DW+(gS*DW)-1:gS*DW];
        assign m_serr[gS]  = i_serr[gS];

        // Wire outputs to master from the mux array.
        assign o_scyc[gS]                   = m_mcyc[s_grant];
        assign o_sstb[gS]                   = m_mstb[s_grant];
        assign o_swe[gS]                    = m_mwe[s_grant];
        assign o_saddr[AW+(gS*AW)-1:gS*AW]  = m_maddr[s_grant];
        assign o_sdata[DW+(gS*DW)-1:gS*DW]  = m_mdata[s_grant];
        assign o_ssel[SW+(gS*SW)-1:gS*SW]   = m_msel[s_grant];
    end
    // Fill the remainder of the mux array with empty data, to
    // set the un-selected state of the outputs to the master.
    for (gS = NS; gS < M_GRANT_SIZE; gS = gS + 1) begin
        assign m_sdata[gS] = 32'h00000000;
        assign m_sack[gS]  = 1'b0;
        assign m_serr[gS]  = 1'b0;
    end

    // The MxS arbiter
    always @(posedge i_clk) begin
        for (iM = 0; iM < NM; iM = iM + 1) begin
            if (!i_mcyc[iM]) begin
                // Master is inactive, release the grant.
                m_grant[iM] <= M_GRANT_NONE;
            end
            else if (!o_scyc[m_decode[iM]]) begin
                // Slave is inactive, acquire the grant.
                m_grant[iM] <= m_decode[iM];
            end
        end
    end
endgenerate
endmodule
