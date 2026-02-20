`timescale 1ns/1ps
`default_nettype none

// ============================================================
// ray_job_if — Ray input interface + job register
//
// Accepts one "Option B" ray job from CPU/harness, latches it,
// and exposes registered fields to downstream stepper logic.
// Owns start/busy/done handshaking at the input side.
//
// Handshake (CPU -> ASIC):
//   job_valid from CPU
//   job_ready from ASIC (high when able to accept)
// Fields (suggested in the design doc):
//   ix0/iy0/iz0 (5b each), sx/sy/sz (1b), next_*/inc_* (W bits),
//   max_steps (MAX_STEPS_BITS)
//
// This version also gates job acceptance during load_mode.
// ============================================================
module ray_job_if #(
  parameter int X_BITS          = 5,
  parameter int Y_BITS          = 5,
  parameter int Z_BITS          = 5,
  parameter int W               = 24,
  parameter int MAX_STEPS_BITS  = 10
)(
  input  logic                   clk,
  input  logic                   rst_n,

  // Optional gating: when scene is being loaded, refuse jobs
  input  logic                   load_mode,

  // CPU/harness job input handshake
  input  logic                   job_valid,
  output logic                   job_ready,

  // Job fields (Option B)
  input  logic [X_BITS-1:0]      ix0,
  input  logic [Y_BITS-1:0]      iy0,
  input  logic [Z_BITS-1:0]      iz0,

  input  logic                   sx,
  input  logic                   sy,
  input  logic                   sz,

  input  logic [W-1:0]           next_x,
  input  logic [W-1:0]           next_y,
  input  logic [W-1:0]           next_z,

  input  logic [W-1:0]           inc_x,
  input  logic [W-1:0]           inc_y,
  input  logic [W-1:0]           inc_z,

  input  logic [MAX_STEPS_BITS-1:0] max_steps,

  // From downstream control: pulse when job has completed
  input  logic                   job_done,

  // Status to downstream
  output logic                   job_loaded,  // 1-cycle pulse when we latch a new job
  output logic                   job_active,  // held high while a job is in progress

  // Registered job fields (to step_control / datapath)
  output logic [X_BITS-1:0]      ix0_q,
  output logic [Y_BITS-1:0]      iy0_q,
  output logic [Z_BITS-1:0]      iz0_q,

  output logic                   sx_q,
  output logic                   sy_q,
  output logic                   sz_q,

  output logic [W-1:0]           next_x_q,
  output logic [W-1:0]           next_y_q,
  output logic [W-1:0]           next_z_q,

  output logic [W-1:0]           inc_x_q,
  output logic [W-1:0]           inc_y_q,
  output logic [W-1:0]           inc_z_q,

  output logic [MAX_STEPS_BITS-1:0] max_steps_q
);

  // Ready when not loading and either:
  //  - idle (job_active=0), or
  //  - completing this cycle (job_done=1) so we can accept a new one immediately.
  always_comb begin
    job_ready = (!load_mode) && (!job_active || job_done);
  end

  wire accept = job_valid && job_ready;

  // Latch + status
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      job_loaded  <= 1'b0;
      job_active  <= 1'b0;

      ix0_q       <= '0;
      iy0_q       <= '0;
      iz0_q       <= '0;
      sx_q        <= 1'b0;
      sy_q        <= 1'b0;
      sz_q        <= 1'b0;

      next_x_q    <= '0;
      next_y_q    <= '0;
      next_z_q    <= '0;

      inc_x_q     <= '0;
      inc_y_q     <= '0;
      inc_z_q     <= '0;

      max_steps_q <= '0;
    end else begin
      // default pulse low; assert for 1 cycle on accept
      job_loaded <= accept;

      // Update active flag:
      // - If we accept a job, we are active (even if job_done also high).
      // - Else if job_done (and no accept), become idle.
      if (accept) begin
        job_active <= 1'b1;
      end else if (job_done) begin
        job_active <= 1'b0;
      end

      // Latch fields on accept
      if (accept) begin
        ix0_q       <= ix0;
        iy0_q       <= iy0;
        iz0_q       <= iz0;

        sx_q        <= sx;
        sy_q        <= sy;
        sz_q        <= sz;

        next_x_q    <= next_x;
        next_y_q    <= next_y;
        next_z_q    <= next_z;

        inc_x_q     <= inc_x;
        inc_y_q     <= inc_y;
        inc_z_q     <= inc_z;

        max_steps_q <= max_steps;
      end
    end
  end

`ifdef SIM
  // Helpful sanity checks in simulation
  initial begin
    if (X_BITS != 5 || Y_BITS != 5 || Z_BITS != 5) begin
      $display("INFO(ray_job_if): Using non-5-bit voxel indices: X=%0d Y=%0d Z=%0d",
               X_BITS, Y_BITS, Z_BITS);
    end
  end
`endif

endmodule

`default_nettype wire
