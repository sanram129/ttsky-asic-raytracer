// =============================================================================
// Module: axis_choose
// Description: Deterministically chooses the minimum of three unsigned values.
//              FIX: Output is always 1-hot (exactly one axis selected) to avoid
//              multi-axis stepping on fixed-point ties.
// =============================================================================

module axis_choose #(
    parameter int W = 32  // Bit-width of input values
)(
    input  logic [W-1:0] a,           // First input value
    input  logic [W-1:0] b,           // Second input value
    input  logic [W-1:0] c,           // Third input value
    output logic [2:0]   step_mask,   // 1-hot mask: [2]=c, [1]=b, [0]=a (selected axis)
    output logic [1:0]   primary_sel  // Deterministic selector (tie-break: a, then b, then c)
);

    // Combinational logic to compute deterministic minimum and generate a 1-hot mask
    // step_mask encoding (1-hot):
    //   step_mask[0] = 1 when selecting a
    //   step_mask[1] = 1 when selecting b
    //   step_mask[2] = 1 when selecting c
    //
    // Tie-breaking priority (deterministic): a, then b, then c
    
    always_comb begin
        if (a <= b && a <= c) begin
            primary_sel = 2'd0;
        end else if (b <= c) begin
            primary_sel = 2'd1;
        end else begin
            primary_sel = 2'd2;
        end

        step_mask = (3'b001 << primary_sel);
    end

endmodule
