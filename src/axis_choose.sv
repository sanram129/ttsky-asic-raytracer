// =============================================================================
// Module: axis_choose
// Description: Identifies ALL inputs equal to the minimum of three unsigned values
//              Outputs a 3-bit mask indicating which inputs are minimal
// =============================================================================

module axis_choose #(
    parameter int W = 32  // Bit-width of input values
)(
    input  logic [W-1:0] a,           // First input value
    input  logic [W-1:0] b,           // Second input value
    input  logic [W-1:0] c,           // Third input value
    output logic [2:0]   step_mask,   // Mask: [2]=c, [1]=b, [0]=a (1 if equal to min)
    output logic [1:0]   primary_sel  // Deterministic selector (lowest index set in mask)
);

    // Internal signal to store the minimum value
    logic [W-1:0] min_val;
    
    // Combinational logic to compute minimum and generate mask
    // step_mask encoding:
    //   step_mask[0] = 1 if a == min(a,b,c)
    //   step_mask[1] = 1 if b == min(a,b,c)
    //   step_mask[2] = 1 if c == min(a,b,c)
    //
    // Examples:
    //   a < b < c      → step_mask = 3'b001 (only a is minimum)
    //   a == b < c     → step_mask = 3'b011 (both a and b are minimum)
    //   a == b == c    → step_mask = 3'b111 (all are minimum)
    
    always_comb begin
        // Step 1: Find the minimum value
        if (a <= b && a <= c) begin
            min_val = a;
        end
        else if (b <= c) begin
            min_val = b;
        end
        else begin
            min_val = c;
        end
        
        // Step 2: Set mask bits for all inputs equal to minimum
        step_mask[0] = (a == min_val);  // Check if a equals minimum
        step_mask[1] = (b == min_val);  // Check if b equals minimum
        step_mask[2] = (c == min_val);  // Check if c equals minimum
        
        // Step 3: Generate primary_sel (lowest index with bit set)
        // Priority order: a > b > c
        if (step_mask[0]) begin
            primary_sel = 2'd0;  // a is minimum
        end
        else if (step_mask[1]) begin
            primary_sel = 2'd1;  // b is minimum
        end
        else begin
            primary_sel = 2'd2;  // c is minimum
        end
    end

endmodule
