`timescale 1ns/1ps
`default_nettype none

// =============================================================================
// Module: voxel_raytracer_core
// Description: Clocked voxel raytracer core with pipelined datapath
//              Integrates all traversal and memory modules with proper clocking
//
// PIPELINE STAGES (5 total):
//   Stage 1: Input registers (latch current position and timers)
//   Stage 2: axis_choose - determine which axis has minimum timer (combinational)
//   Stage 3: step_update - compute next position and updated timers (combinational)
//   Stage 4: bounds_check + voxel_addr_map - check bounds, compute RAM address
//   Stage 5: voxel_ram - synchronous read (2-cycle internal latency)
//
// IMPORTANT TIMING NOTES:
//   - Total latency: 5 cycles from step_valid_in to step_valid_out
//   - bounds_check operates on NEXT position (ix_s3/iy_s3/iz_s3)
//   - voxel_addr_map uses CURRENT position (ix_s3_curr/iy_s3_curr/iz_s3_curr)
//   - voxel_occupied_out reflects the voxel at the CURRENT input position from 5 cycles earlier
//   - out_of_bounds_out reflects bounds check of NEXT position from 5 cycles earlier
//
// RAM ADDRESS HANDLING:
//   - voxel_addr_s4 is combinational output from voxel_addr_map
//   - voxel_ram internally registers the address (raddr_q), then reads on next cycle
//   - Total RAM latency: 2 cycles (address register + data output register)
//
// =============================================================================
module voxel_raytracer_core #(
    parameter int W = 32,           // Timer width
    parameter int COORD_W = 6,      // Coordinate width
    parameter int MAX_VAL = 31,     // Max coordinate value
    parameter int ADDR_BITS = 15    // Memory address bits
)(
    // Clock and Reset
    input  logic                 clk,
    input  logic                 rst_n,
    
    // Ray Step Inputs (registered on entry) - 6-bit for bounds detection
    input  logic [5:0]           ix_in,
    input  logic [5:0]           iy_in,
    input  logic [5:0]           iz_in,
    input  logic                 sx_in,
    input  logic                 sy_in,
    input  logic                 sz_in,
    input  logic [W-1:0]         next_x_in,
    input  logic [W-1:0]         next_y_in,
    input  logic [W-1:0]         next_z_in,
    input  logic [W-1:0]         inc_x_in,
    input  logic [W-1:0]         inc_y_in,
    input  logic [W-1:0]         inc_z_in,
    input  logic                 step_valid_in,
    
    // Scene Loading Interface
    input  logic                 load_mode,
    input  logic                 load_valid,
    output logic                 load_ready,
    input  logic [ADDR_BITS-1:0] load_addr,
    input  logic                 load_data,
    output logic [ADDR_BITS:0]   write_count,
    output logic                 load_complete,
    
    // Ray Step Outputs (pipelined - available 5 cycles after inputs) - 6-bit
    output logic [5:0]           ix_out,
    output logic [5:0]           iy_out,
    output logic [5:0]           iz_out,
    output logic [W-1:0]         next_x_out,
    output logic [W-1:0]         next_y_out,
    output logic [W-1:0]         next_z_out,
    output logic [2:0]           face_mask_out,
    output logic [2:0]           primary_face_id_out,
    output logic                 out_of_bounds_out,
    output logic                 voxel_occupied_out,
    output logic                 step_valid_out
);

    // =========================================================================
    // Pipeline Stage 1: Input Registers
    // =========================================================================
    logic [5:0]  ix_s1, iy_s1, iz_s1;
    logic        sx_s1, sy_s1, sz_s1;
    logic [W-1:0] next_x_s1, next_y_s1, next_z_s1;
    logic [W-1:0] inc_x_s1, inc_y_s1, inc_z_s1;
    logic        valid_s1;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ix_s1 <= '0; iy_s1 <= '0; iz_s1 <= '0;
            sx_s1 <= '0; sy_s1 <= '0; sz_s1 <= '0;
            next_x_s1 <= '0; next_y_s1 <= '0; next_z_s1 <= '0;
            inc_x_s1 <= '0; inc_y_s1 <= '0; inc_z_s1 <= '0;
            valid_s1 <= '0;
        end else begin
            ix_s1 <= ix_in; iy_s1 <= iy_in; iz_s1 <= iz_in;
            sx_s1 <= sx_in; sy_s1 <= sy_in; sz_s1 <= sz_in;
            next_x_s1 <= next_x_in; next_y_s1 <= next_y_in; next_z_s1 <= next_z_in;
            inc_x_s1 <= inc_x_in; inc_y_s1 <= inc_y_in; inc_z_s1 <= inc_z_in;
            valid_s1 <= step_valid_in;
        end
    end
    
    // =========================================================================
    // Pipeline Stage 2: axis_choose (Combinational)
    // Purpose: Determine which axis to step based on minimum timer value
    // =========================================================================
    logic [2:0] step_mask_s2;
    logic [1:0] primary_sel_s2;
    
    axis_choose #(.W(W)) u_axis_choose (
        .a(next_x_s1),
        .b(next_y_s1),
        .c(next_z_s1),
        .step_mask(step_mask_s2),
        .primary_sel(primary_sel_s2)
    );
    
    // Register stage 2 outputs
    logic [5:0]  ix_s2, iy_s2, iz_s2;
    logic        sx_s2, sy_s2, sz_s2;
    logic [W-1:0] next_x_s2, next_y_s2, next_z_s2;
    logic [W-1:0] inc_x_s2, inc_y_s2, inc_z_s2;
    logic [2:0]  step_mask_s2_q;
    logic [1:0]  primary_sel_s2_q;
    logic        valid_s2;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ix_s2 <= '0; iy_s2 <= '0; iz_s2 <= '0;
            sx_s2 <= '0; sy_s2 <= '0; sz_s2 <= '0;
            next_x_s2 <= '0; next_y_s2 <= '0; next_z_s2 <= '0;
            inc_x_s2 <= '0; inc_y_s2 <= '0; inc_z_s2 <= '0;
            step_mask_s2_q <= '0;
            primary_sel_s2_q <= '0;
            valid_s2 <= '0;
        end else begin
            ix_s2 <= ix_s1; iy_s2 <= iy_s1; iz_s2 <= iz_s1;
            sx_s2 <= sx_s1; sy_s2 <= sy_s1; sz_s2 <= sz_s1;
            next_x_s2 <= next_x_s1; next_y_s2 <= next_y_s1; next_z_s2 <= next_z_s1;
            inc_x_s2 <= inc_x_s1; inc_y_s2 <= inc_y_s1; inc_z_s2 <= inc_z_s1;
            step_mask_s2_q <= step_mask_s2;
            primary_sel_s2_q <= primary_sel_s2;
            valid_s2 <= valid_s1;
        end
    end
    
    // =========================================================================
    // Pipeline Stage 3: step_update (Combinational)
    // Purpose: Compute next voxel position and updated timer values
    // =========================================================================
    logic [5:0]  ix_next_s3, iy_next_s3, iz_next_s3;
    logic [W-1:0] next_x_next_s3, next_y_next_s3, next_z_next_s3;
    logic [2:0]  face_mask_s3;
    logic [2:0]  primary_face_id_s3;
    
    step_update #(.W(W)) u_step_update (
        .ix(ix_s2),
        .iy(iy_s2),
        .iz(iz_s2),
        .sx(sx_s2),
        .sy(sy_s2),
        .sz(sz_s2),
        .next_x(next_x_s2),
        .next_y(next_y_s2),
        .next_z(next_z_s2),
        .inc_x(inc_x_s2),
        .inc_y(inc_y_s2),
        .inc_z(inc_z_s2),
        .step_mask(step_mask_s2_q),
        .primary_sel(primary_sel_s2_q),
        .ix_next(ix_next_s3),
        .iy_next(iy_next_s3),
        .iz_next(iz_next_s3),
        .next_x_next(next_x_next_s3),
        .next_y_next(next_y_next_s3),
        .next_z_next(next_z_next_s3),
        .face_mask(face_mask_s3),
        .primary_face_id(primary_face_id_s3)
    );
    
    // Register stage 3 outputs
    // CRITICAL: We need both CURRENT and NEXT positions!
    // - CURRENT position (ix_s3_curr) is used for voxel RAM lookup
    // - NEXT position (ix_s3) is used for bounds check and next iteration
    logic [5:0]  ix_s3_curr, iy_s3_curr, iz_s3_curr;  // CURRENT position (6-bit)
    logic [5:0]  ix_s3, iy_s3, iz_s3;                 // NEXT position (6-bit)
    logic [W-1:0] next_x_s3, next_y_s3, next_z_s3;
    logic [2:0]  face_mask_s3_q;
    logic [2:0]  primary_face_id_s3_q;
    logic        valid_s3;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ix_s3_curr <= '0; iy_s3_curr <= '0; iz_s3_curr <= '0;
            ix_s3 <= '0; iy_s3 <= '0; iz_s3 <= '0;
            next_x_s3 <= '0; next_y_s3 <= '0; next_z_s3 <= '0;
            face_mask_s3_q <= '0;
            primary_face_id_s3_q <= '0;
            valid_s3 <= '0;
        end else begin
            // Capture CURRENT position (from stage 2) for voxel lookup
            ix_s3_curr <= ix_s2;
            iy_s3_curr <= iy_s2;
            iz_s3_curr <= iz_s2;
            
            // Capture NEXT position (from step_update) for bounds check
            ix_s3 <= ix_next_s3;
            iy_s3 <= iy_next_s3;
            iz_s3 <= iz_next_s3;
            
            next_x_s3 <= next_x_next_s3;
            next_y_s3 <= next_y_next_s3;
            next_z_s3 <= next_z_next_s3;
            face_mask_s3_q <= face_mask_s3;
            primary_face_id_s3_q <= primary_face_id_s3;
            valid_s3 <= valid_s2;
        end
    end
    
    // =========================================================================
    // Pipeline Stage 4: bounds_check & voxel_addr_map (Combinational)
    // Purpose: Check if NEXT position is within bounds, compute RAM address for CURRENT position
    // 
    // FIXED: Now correctly uses separate positions for each purpose:
    // - bounds_check examines ix_s3/iy_s3/iz_s3 (NEXT position after stepping)
    // - voxel_addr_map uses ix_s3_curr/iy_s3_curr/iz_s3_curr (CURRENT position)
    // 
    // This ensures we check occupancy of the voxel we're AT, not the voxel we're GOING TO.
    // =========================================================================
    logic [5:0] bounds_ix, bounds_iy, bounds_iz;
    logic out_of_bounds_s4;
    logic [ADDR_BITS-1:0] voxel_addr_s4;
    
    // NEXT position is already 6-bit, use directly for bounds check
    // Check if NEXT position is out of bounds (32-63 are OOB)
    assign bounds_ix = ix_s3;
    assign bounds_iy = iy_s3;
    assign bounds_iz = iz_s3;
    
    bounds_check #(
        .COORD_W(COORD_W),
        .MAX_VAL(MAX_VAL)
    ) u_bounds_check (
        .ix(bounds_ix),
        .iy(bounds_iy),
        .iz(bounds_iz),
        .out_of_bounds(out_of_bounds_s4)
    );
    
    // Compute RAM address for CURRENT position (FIXED!)
    // Use lower 5 bits only for RAM addressing (grid is 32x32x32)
    voxel_addr_map #(
        .X_BITS(5),
        .Y_BITS(5),
        .Z_BITS(5),
        .MAP_ZYX(1'b1)
    ) u_voxel_addr_map (
        .x(ix_s3_curr[4:0]),  // Use CURRENT position lower 5 bits
        .y(iy_s3_curr[4:0]),
        .z(iz_s3_curr[4:0]),
        .addr(voxel_addr_s4)
    );
    
    // Register stage 4 outputs
    logic [5:0]  ix_s4, iy_s4, iz_s4;  // NEXT position (for output) - 6-bit
    logic [W-1:0] next_x_s4, next_y_s4, next_z_s4;
    logic [2:0]  face_mask_s4;
    logic [2:0]  primary_face_id_s4;
    logic        out_of_bounds_s4_q;
    logic [ADDR_BITS-1:0] voxel_addr_s4_q;
    logic        valid_s4;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ix_s4 <= '0; iy_s4 <= '0; iz_s4 <= '0;
            next_x_s4 <= '0; next_y_s4 <= '0; next_z_s4 <= '0;
            face_mask_s4 <= '0;
            primary_face_id_s4 <= '0;
            out_of_bounds_s4_q <= '0;
            voxel_addr_s4_q <= '0;
            valid_s4 <= '0;
        end else begin
            // Propagate NEXT position (ix_s3 is next position)
            ix_s4 <= ix_s3;
            iy_s4 <= iy_s3;
            iz_s4 <= iz_s3;
            next_x_s4 <= next_x_s3;
            next_y_s4 <= next_y_s3;
            next_z_s4 <= next_z_s3;
            face_mask_s4 <= face_mask_s3_q;
            primary_face_id_s4 <= primary_face_id_s3_q;
            out_of_bounds_s4_q <= out_of_bounds_s4;
            voxel_addr_s4_q <= voxel_addr_s4;
            valid_s4 <= valid_s3;
        end
    end
    
    // =========================================================================
    // Pipeline Stage 5: voxel_ram read & scene_loader_if (Synchronous)
    // Purpose: Read voxel occupancy from RAM, handle scene loading
    // RAM TIMING: voxel_ram has internal 2-cycle latency:
    //   Cycle N:   raddr input = voxel_addr_s4 (combinational from stage 4)
    //   Cycle N+1: RAM internally registers address (raddr_q)
    //   Cycle N+2: RAM reads mem[raddr_q] and registers output (rdata)
    // =========================================================================
    logic voxel_occupied_s5;
    logic we_ram;
    logic [ADDR_BITS-1:0] waddr_ram;
    logic wdata_ram;
    
    scene_loader_if #(
        .ADDR_BITS(ADDR_BITS),
        .ENABLE_COUNTER(1'b1)
    ) u_scene_loader_if (
        .clk(clk),
        .rst_n(rst_n),
        .load_mode(load_mode),
        .load_valid(load_valid),
        .load_ready(load_ready),
        .load_addr(load_addr),
        .load_data(load_data),
        .we(we_ram),
        .waddr(waddr_ram),
        .wdata(wdata_ram),
        .write_count(write_count),
        .load_complete(load_complete)
    );
    
    voxel_ram #(
        .ADDR_BITS(ADDR_BITS),
        .SYNC_READ(1'b1),
        .WRITE_FIRST(1'b1)
    ) u_voxel_ram (
        .clk(clk),
        .rst_n(rst_n),
        // RAM address input: combinational output from voxel_addr_map (stage 4)
        // The voxel_ram module internally registers this address as raddr_q,
        // then performs synchronous read on the next cycle. Total RAM latency
        // is 2 cycles: address register + data output register.
        .raddr(voxel_addr_s4),
        .rdata(voxel_occupied_s5),
        .we(we_ram),
        .waddr(waddr_ram),
        .wdata(wdata_ram)
    );
    
    // Register stage 5 outputs for final alignment
    logic [5:0]  ix_s5, iy_s5, iz_s5;
    logic [W-1:0] next_x_s5, next_y_s5, next_z_s5;
    logic [2:0]  face_mask_s5;
    logic [2:0]  primary_face_id_s5;
    logic        out_of_bounds_s5;
    logic        valid_s5;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ix_s5 <= '0; iy_s5 <= '0; iz_s5 <= '0;
            next_x_s5 <= '0; next_y_s5 <= '0; next_z_s5 <= '0;
            face_mask_s5 <= '0;
            primary_face_id_s5 <= '0;
            out_of_bounds_s5 <= '0;
            valid_s5 <= '0;
        end else begin
            ix_s5 <= ix_s4;
            iy_s5 <= iy_s4;
            iz_s5 <= iz_s4;
            next_x_s5 <= next_x_s4;
            next_y_s5 <= next_y_s4;
            next_z_s5 <= next_z_s4;
            face_mask_s5 <= face_mask_s4;
            primary_face_id_s5 <= primary_face_id_s4;
            out_of_bounds_s5 <= out_of_bounds_s4_q;
            valid_s5 <= valid_s4;
        end
    end
    
    // =========================================================================
    // Output Assignments
    // =========================================================================
    assign ix_out = ix_s5;
    assign iy_out = iy_s5;
    assign iz_out = iz_s5;
    assign next_x_out = next_x_s5;
    assign next_y_out = next_y_s5;
    assign next_z_out = next_z_s5;
    assign face_mask_out = face_mask_s5;
    assign primary_face_id_out = primary_face_id_s5;
    assign out_of_bounds_out = out_of_bounds_s5;
    assign voxel_occupied_out = voxel_occupied_s5;  // RAM output - synchronous read (2-cycle latency)
    assign step_valid_out = valid_s5;

endmodule

`default_nettype wire
