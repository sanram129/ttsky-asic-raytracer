`timescale 1ns/1ps
`default_nettype none

// =============================================================================
// Module: raytracer_top
// Description: Complete raytracer system integrating:
//              - ray_job_if: Job input interface
//              - step_control_fsm: FSM for stepping control
//              - voxel_raytracer_core: Pipelined datapath (axis_choose, step_update, 
//                bounds_check, voxel_addr_map, voxel_ram, scene_loader_if)
//
// This top-level module resolves all interface compatibility issues and
// connects the FSM control flow with the pipelined datapath.
//
// PIPELINE LATENCY:
//   The voxel_raytracer_core has a 5-cycle latency from step_valid_in to 
//   step_valid_out. The FSM waits for solid_valid (= step_valid_out) before
//   loading pipeline results. Pipeline stages:
//     Cycle 1: Input registers
//     Cycle 2: axis_choose (combinational) + register
//     Cycle 3: step_update (combinational) + register
//     Cycle 4: bounds_check + voxel_addr_map (combinational) + register
//     Cycle 5: voxel_ram read (synchronous) + register
//   
//   TIMING: When FSM is at position P:
//     - FSM outputs position P to pipeline
//     - 5 cycles later: pipeline returns next position (P+1) and voxel 
//       occupancy for position P
//     - FSM loads P+1 if voxel at P is empty, else terminates
//
// BOUNDS CHECKING:
//   The pipeline checks bounds on the NEXT position (after stepping), not
//   the current position. This prevents stepping into invalid memory before
//   detecting the bounds violation.
//
// =============================================================================
module raytracer_top #(
    // Coordinate and timer parameters
    parameter int COORD_WIDTH = 16,        // Voxel coordinate width for FSM results
                                           // (over-provisioned for future expansion)
    parameter int COORD_W = 6,             // Coordinate width for bounds check (6-bit)
    parameter int TIMER_WIDTH = 32,        // Timer value width (fixed-point)
    parameter int W = 32,                  // Timer width for pipeline (must match TIMER_WIDTH)
    parameter int MAX_VAL = 31,            // Max coordinate value (0..31 for 32^3 grid)
    
    // Address and step parameters
    parameter int ADDR_BITS = 15,          // Memory address bits (32^3 = 2^15)
    parameter int X_BITS = 6,              // X coordinate bits (6-bit for bounds detection)
    parameter int Y_BITS = 6,              // Y coordinate bits (6-bit for bounds detection)
    parameter int Z_BITS = 6,              // Z coordinate bits (6-bit for bounds detection)
    parameter int MAX_STEPS_BITS = 10,     // Max steps counter width
    parameter int STEP_COUNT_WIDTH = 16    // Step counter width for FSM
)(
    // Clock and Reset
    input  logic                          clk,
    input  logic                          rst_n,
    
    // =========================================================================
    // Ray Job Input Interface (from CPU/testbench)
    // =========================================================================
    input  logic                          job_valid,
    output logic                          job_ready,
    
    // Ray job parameters (Option B format)
    input  logic [X_BITS-1:0]             job_ix0,
    input  logic [Y_BITS-1:0]             job_iy0,
    input  logic [Z_BITS-1:0]             job_iz0,
    input  logic                          job_sx,
    input  logic                          job_sy,
    input  logic                          job_sz,
    input  logic [W-1:0]                  job_next_x,
    input  logic [W-1:0]                  job_next_y,
    input  logic [W-1:0]                  job_next_z,
    input  logic [W-1:0]                  job_inc_x,
    input  logic [W-1:0]                  job_inc_y,
    input  logic [W-1:0]                  job_inc_z,
    input  logic [MAX_STEPS_BITS-1:0]     job_max_steps,
    
    // =========================================================================
    // Scene Loading Interface (from CPU/testbench)
    // =========================================================================
    input  logic                          load_mode,
    input  logic                          load_valid,
    output logic                          load_ready,
    input  logic [ADDR_BITS-1:0]          load_addr,
    input  logic                          load_data,
    output logic [ADDR_BITS:0]            write_count,
    output logic                          load_complete,
    
    // =========================================================================
    // Ray Tracing Results (to CPU/testbench)
    // =========================================================================
    output logic                          ray_done,
    output logic                          ray_hit,
    output logic                          ray_timeout,
    output logic [COORD_WIDTH-1:0]        hit_voxel_x,
    output logic [COORD_WIDTH-1:0]        hit_voxel_y,
    output logic [COORD_WIDTH-1:0]        hit_voxel_z,
    output logic [2:0]                    hit_face_id,
    output logic [STEP_COUNT_WIDTH-1:0]   steps_taken
);

    // =========================================================================
    // Internal Signals
    // =========================================================================
    
    // Ray job interface outputs
    logic                          job_loaded;
    logic                          job_active;
    logic [X_BITS-1:0]             ix0_q;
    logic [Y_BITS-1:0]             iy0_q;
    logic [Z_BITS-1:0]             iz0_q;
    logic                          sx_q, sy_q, sz_q;
    logic [W-1:0]                  next_x_q, next_y_q, next_z_q;
    logic [W-1:0]                  inc_x_q, inc_y_q, inc_z_q;
    logic [MAX_STEPS_BITS-1:0]     max_steps_q;
    
    // FSM to core signals (current state for pipeline input)
    logic                          step_valid;
    logic [X_BITS-1:0]             current_ix;
    logic [Y_BITS-1:0]             current_iy;
    logic [Z_BITS-1:0]             current_iz;
    logic [W-1:0]                  current_next_x;
    logic [W-1:0]                  current_next_y;
    logic [W-1:0]                  current_next_z;
    
    // Core to FSM feedback (pipeline computed next state)
    logic [X_BITS-1:0]             next_ix;
    logic [Y_BITS-1:0]             next_iy;
    logic [Z_BITS-1:0]             next_iz;
    logic [W-1:0]                  next_next_x;
    logic [W-1:0]                  next_next_y;
    logic [W-1:0]                  next_next_z;
    logic [2:0]                    face_mask;
    logic [2:0]                    primary_face_id;
    logic                          out_of_bounds;      // Bounds check for NEXT position
    logic                          voxel_occupied;     // Occupancy of CURRENT position
    logic                          step_valid_out;     // Pipeline output valid (5 cycles after input)
    
    // FSM control signals
    logic                          fsm_ready;
    logic                          fsm_active;
    logic                          fsm_done;
    
    // =========================================================================
    // Module Instantiation: ray_job_if
    // =========================================================================
    ray_job_if #(
        .X_BITS(X_BITS),
        .Y_BITS(Y_BITS),
        .Z_BITS(Z_BITS),
        .W(W),
        .MAX_STEPS_BITS(MAX_STEPS_BITS)
    ) u_ray_job_if (
        .clk(clk),
        .rst_n(rst_n),
        
        // Load mode gating
        .load_mode(load_mode),
        
        // Job input handshake
        .job_valid(job_valid),
        .job_ready(job_ready),
        
        // Job fields
        .ix0(job_ix0),
        .iy0(job_iy0),
        .iz0(job_iz0),
        .sx(job_sx),
        .sy(job_sy),
        .sz(job_sz),
        .next_x(job_next_x),
        .next_y(job_next_y),
        .next_z(job_next_z),
        .inc_x(job_inc_x),
        .inc_y(job_inc_y),
        .inc_z(job_inc_z),
        .max_steps(job_max_steps),
        
        // Job done from FSM
        .job_done(fsm_done),
        
        // Registered outputs
        .job_loaded(job_loaded),
        .job_active(job_active),
        .ix0_q(ix0_q),
        .iy0_q(iy0_q),
        .iz0_q(iz0_q),
        .sx_q(sx_q),
        .sy_q(sy_q),
        .sz_q(sz_q),
        .next_x_q(next_x_q),
        .next_y_q(next_y_q),
        .next_z_q(next_z_q),
        .inc_x_q(inc_x_q),
        .inc_y_q(inc_y_q),
        .inc_z_q(inc_z_q),
        .max_steps_q(max_steps_q)
    );
    
    // =========================================================================
    // Module Instantiation: step_control_fsm
    // =========================================================================
    // Note: FSM uses different clock/reset naming (clock/reset vs clk/rst_n)
    
    step_control_fsm #(
        .X_BITS(X_BITS),
        .Y_BITS(Y_BITS),
        .Z_BITS(Z_BITS),
        .TIMER_WIDTH(TIMER_WIDTH),
        .STEP_COUNT_WIDTH(STEP_COUNT_WIDTH)
    ) u_step_control_fsm (
        .clock(clk),                    // Map clk -> clock
        .reset(~rst_n),                 // Map rst_n -> reset (inverted!)
        
        // Job control
        .job_loaded(job_loaded),
        .ready(fsm_ready),
        .active(fsm_active),            // Drives pipeline step_valid_in
        
        // Job parameters (direct connection for coordinate bits)
        .job_init_x(ix0_q),
        .job_init_y(iy0_q),
        .job_init_z(iz0_q),
        .job_timer_x({{(TIMER_WIDTH-W){1'b0}}, next_x_q}),  // Zero-extend if needed
        .job_timer_y({{(TIMER_WIDTH-W){1'b0}}, next_y_q}),
        .job_timer_z({{(TIMER_WIDTH-W){1'b0}}, next_z_q}),
        .max_steps({{(STEP_COUNT_WIDTH-MAX_STEPS_BITS){1'b0}}, max_steps_q}),
        
        // Voxel data from RAM (via core pipeline)
        // Note: voxel_occupied corresponds to the position from 5 cycles earlier
        .solid_bit(voxel_occupied),
        .solid_valid(step_valid_out),   // Pipeline output valid after 5 cycles
        
        // Bounds checking
        // Note: out_of_bounds checks the NEXT position (prevents invalid stepping)
        .out_of_bounds(out_of_bounds),
        
        // Pipeline computed values (loaded when solid_valid=1 and no termination)
        .pipeline_next_x(next_ix),
        .pipeline_next_y(next_iy),
        .pipeline_next_z(next_iz),
        .pipeline_next_timer_x({{(TIMER_WIDTH-W){1'b0}}, next_next_x}),
        .pipeline_next_timer_y({{(TIMER_WIDTH-W){1'b0}}, next_next_y}),
        .pipeline_next_timer_z({{(TIMER_WIDTH-W){1'b0}}, next_next_z}),
        .pipeline_face_id(primary_face_id),
        
        // Current state (for pipeline input)
        .current_voxel_x(current_ix),
        .current_voxel_y(current_iy),
        .current_voxel_z(current_iz),
        .current_timer_x(current_next_x),
        .current_timer_y(current_next_y),
        .current_timer_z(current_next_z),
        .steps_taken(steps_taken),
        
        // Termination outputs
        .done(fsm_done),
        .hit(ray_hit),
        .timeout(ray_timeout),
        
        // Result outputs (valid when done=1)
        .hit_voxel_x(hit_voxel_x),
        .hit_voxel_y(hit_voxel_y),
        .hit_voxel_z(hit_voxel_z),
        .face_id(hit_face_id)
    );
    
    // Map FSM done to output
    assign ray_done = fsm_done;
    
    // =========================================================================
    // Module Instantiation: voxel_raytracer_core
    // =========================================================================
    // The core contains the full 5-stage pipeline:
    //   Stage 1: Input registers
    //   Stage 2: axis_choose (determine which axis has minimum timer)
    //   Stage 3: step_update (compute next position and timers)
    //   Stage 4: bounds_check + voxel_addr_map (check bounds, compute address)
    //   Stage 5: voxel_ram (synchronous read) + output registers
    
    voxel_raytracer_core #(
        .W(W),
        .COORD_W(COORD_W),
        .MAX_VAL(MAX_VAL),
        .ADDR_BITS(ADDR_BITS)
    ) u_voxel_raytracer_core (
        .clk(clk),
        .rst_n(rst_n),
        
        // Ray step inputs (from FSM current state)
        .ix_in(current_ix),
        .iy_in(current_iy),
        .iz_in(current_iz),
        .sx_in(sx_q),
        .sy_in(sy_q),
        .sz_in(sz_q),
        .next_x_in(current_next_x[W-1:0]),
        .next_y_in(current_next_y[W-1:0]),
        .next_z_in(current_next_z[W-1:0]),
        .inc_x_in(inc_x_q),
        .inc_y_in(inc_y_q),
        .inc_z_in(inc_z_q),
        .step_valid_in(fsm_active),     // Valid when FSM is in RUNNING state
        
        // Scene loading interface (pass through)
        .load_mode(load_mode),
        .load_valid(load_valid),
        .load_ready(load_ready),
        .load_addr(load_addr),
        .load_data(load_data),
        .write_count(write_count),
        .load_complete(load_complete),
        
        // Ray step outputs (available 5 cycles after inputs)
        .ix_out(next_ix),               // Next voxel position (after stepping)
        .iy_out(next_iy),
        .iz_out(next_iz),
        .next_x_out(next_next_x),       // Next timer values (after stepping)
        .next_y_out(next_next_y),
        .next_z_out(next_next_z),
        .face_mask_out(face_mask),
        .primary_face_id_out(primary_face_id),
        .out_of_bounds_out(out_of_bounds),      // Bounds check for NEXT position
        .voxel_occupied_out(voxel_occupied),    // Occupancy for CURRENT position
        .step_valid_out(step_valid_out)         // Output valid flag (5 cycles later)
    );
    
    // =========================================================================
    // Integration Notes and Timing Documentation:
    // =========================================================================
    // 
    // ARCHITECTURE OVERVIEW:
    // 1. FSM (step_control_fsm) manages control flow and state transitions
    // 2. Core (voxel_raytracer_core) is a pipelined datapath that computes
    //    next positions using axis_choose and step_update modules
    // 3. Data flow: FSM feeds current position → pipeline computes next → 
    //    FSM loads results when valid
    // 4. Single DDA implementation (in pipeline) - no redundant computation
    // 
    // PIPELINE LATENCY AND TIMING:
    // - Pipeline has 5-cycle latency from step_valid_in to step_valid_out
    // - Stage 1: Input registers latch current position/timers
    // - Stage 2: axis_choose determines which axis to step (combinational + reg)
    // - Stage 3: step_update computes next position (combinational + reg)
    // - Stage 4: bounds_check and voxel_addr_map (combinational + reg)
    // - Stage 5: voxel_ram synchronous read + output register
    // 
    // TIMING EXAMPLE (FSM at position P):
    //   Cycle 0: FSM outputs current_ix/iy/iz = P, fsm_active = 1
    //   Cycle 1: Pipeline stage 1 latches position P
    //   Cycle 2: Pipeline stage 2 determines stepping axis
    //   Cycle 3: Pipeline stage 3 computes next position P+1
    //   Cycle 4: Pipeline stage 4 checks bounds for P+1, computes RAM address for P
    //   Cycle 5: Pipeline stage 5 reads voxel occupancy for position P
    //   Cycle 5: step_valid_out = 1, voxel_occupied = RAM[P], next_ix/iy/iz = P+1
    //   Cycle 5: FSM sees solid_valid=1, reads voxel_occupied and pipeline_next_*
    //   Cycle 6: If RAM[P] empty: FSM updates to position P+1
    //            If RAM[P] solid: FSM terminates with hit at position P
    // 
    // BOUNDS CHECKING TIMING:
    // - bounds_check examines the NEXT position (P+1), not current position (P)
    // - out_of_bounds indicates if stepping to P+1 would exceed grid bounds
    // - This prevents invalid memory accesses before bounds violation detection
    // - FSM terminates if out_of_bounds=1, preventing step to invalid position
    // 
    // WIDTH CONVERSIONS:
    // - COORD_WIDTH (16-bit) is over-provisioned for result outputs to allow
    //   future expansion beyond 32^3 grids
    // - Actual voxel coordinates are X_BITS/Y_BITS/Z_BITS (6-bit = 0..63, valid range 0..31)
    // - TIMER_WIDTH and W should match (both 32-bit in default configuration)
    // - Zero-extension used for width conversions when parameters differ
    //
    // FEATURES PROVIDED:
    // - Job loading and parameter latching via ray_job_if
    // - FSM-controlled stepping state machine (4 states: IDLE/INIT/RUNNING/FINISH)
    // - Pipelined DDA stepping (axis_choose + step_update)
    // - Voxel memory access with scene loading capability
    // - Bounds checking and occupancy detection
    // - Hit detection with face ID capture
    // - Timeout detection for rays that exceed max_steps
    // - Load mode gating to prevent job acceptance during scene loading

endmodule

`default_nettype wire
