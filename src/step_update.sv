// =============================================================================
// Module: step_update
// Description: Voxel stepping datapath for 3D DDA raytracer
//              Performs conditional multi-axis updates based on step_mask
//              Optimized for low area and low power
// =============================================================================

module step_update #(
    parameter int W = 32  // Timer/accumulator bit width
)(
    // Current voxel indices (6-bit coordinates for bounds detection)
    input  logic [5:0]   ix,
    input  logic [5:0]   iy,
    input  logic [5:0]   iz,
    
    // Step direction signs: 1 => +1 step, 0 => -1 step
    input  logic         sx,
    input  logic         sy,
    input  logic         sz,
    
    // Current timer values (unsigned fixed-point)
    input  logic [W-1:0] next_x,
    input  logic [W-1:0] next_y,
    input  logic [W-1:0] next_z,
    
    // Timer increments (unsigned fixed-point)
    input  logic [W-1:0] inc_x,
    input  logic [W-1:0] inc_y,
    input  logic [W-1:0] inc_z,
    
    // Step control from axis_choose module
    // step_mask[0] = step X axis, [1] = step Y axis, [2] = step Z axis
    input  logic [2:0]   step_mask,
    
    // Primary axis selector from axis_choose: 0=X, 1=Y, 2=Z
    input  logic [1:0]   primary_sel,
    
    // Updated voxel indices
    output logic [5:0]   ix_next,
    output logic [5:0]   iy_next,
    output logic [5:0]   iz_next,
    
    // Updated timer values
    output logic [W-1:0] next_x_next,
    output logic [W-1:0] next_y_next,
    output logic [W-1:0] next_z_next,
    
    // Face crossing information
    output logic [2:0]   face_mask,       // Which axes stepped: [2]=Z, [1]=Y, [0]=X
    output logic [2:0]   primary_face_id  // Primary face: 0=X+, 1=X-, 2=Y+, 3=Y-, 4=Z+, 5=Z-
);

    // =========================================================================
    // Step Direction Computation
    // =========================================================================
    // Convert sign bits to signed step values: sx==1 means +1, sx==0 means -1
    // Using 7-bit signed to handle 6-bit unsigned coordinate arithmetic
    
    logic signed [6:0] step_x, step_y, step_z;
    
    always_comb begin
        // Compute ±1 step values based on sign bits
        step_x = sx ? 7'sd1 : -7'sd1;
        step_y = sy ? 7'sd1 : -7'sd1;
        step_z = sz ? 7'sd1 : -7'sd1;
    end
    
    // =========================================================================
    // Voxel Index Updates (6-bit coordinates for out-of-bounds detection)
    // =========================================================================
    // Update each axis only if corresponding step_mask bit is set
    // 6-bit allows: 31+1=32 (OOB) and 0-1=63 (OOB) to be detected
    // Otherwise pass through unchanged (reduces power)
    
    always_comb begin
        // X-axis: Update if step_mask[0] == 1
        if (step_mask[0]) begin
            ix_next = ix + step_x[5:0];  // 6-bit arithmetic, no wrapping
        end else begin
            ix_next = ix;  // Pass through unchanged
        end
        
        // Y-axis: Update if step_mask[1] == 1
        if (step_mask[1]) begin
            iy_next = iy + step_y[5:0];  // 6-bit arithmetic, no wrapping
        end else begin
            iy_next = iy;
        end
        
        // Z-axis: Update if step_mask[2] == 1
        if (step_mask[2]) begin
            iz_next = iz + step_z[5:0];  // 6-bit arithmetic, no wrapping
        end else begin
            iz_next = iz;
        end
    end
    
    // =========================================================================
    // Timer Updates (unsigned fixed-point addition)
    // =========================================================================
    // Add increment only when corresponding axis steps (saves power)
    
    always_comb begin
        // X-timer: Add inc_x if stepping X
        if (step_mask[0]) begin
            next_x_next = next_x + inc_x;
        end else begin
            next_x_next = next_x;
        end
        
        // Y-timer: Add inc_y if stepping Y
        if (step_mask[1]) begin
            next_y_next = next_y + inc_y;
        end else begin
            next_y_next = next_y;
        end
        
        // Z-timer: Add inc_z if stepping Z
        if (step_mask[2]) begin
            next_z_next = next_z + inc_z;
        end else begin
            next_z_next = next_z;
        end
    end
    
    // =========================================================================
    // Face Crossing Information
    // =========================================================================
    
    // Face mask directly reflects which axes stepped this cycle
    // face_mask[0] = X stepped, [1] = Y stepped, [2] = Z stepped
    assign face_mask = step_mask;
    
    // Primary face ID: Encodes the primary axis and direction
    // Encoding: 0=X+, 1=X-, 2=Y+, 3=Y-, 4=Z+, 5=Z-
    // Based on primary_sel (which axis) and corresponding sign bit (which direction)
    always_comb begin
        case (primary_sel)
            2'd0: primary_face_id = sx ? 3'd0 : 3'd1;  // X-axis: X+ or X-
            2'd1: primary_face_id = sy ? 3'd2 : 3'd3;  // Y-axis: Y+ or Y-
            2'd2: primary_face_id = sz ? 3'd4 : 3'd5;  // Z-axis: Z+ or Z-
            default: primary_face_id = 3'd0;           // Should not occur
        endcase
    end

endmodule
