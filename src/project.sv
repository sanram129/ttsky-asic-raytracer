/*
 * TinyTapeout wrapper for the IEEE UofT ASIC raytracer.
 *
 * Pin protocol (simple byte-wide register file):
 *   - ui_in[7:0]  : write data byte
 *   - uio_in[5:0] : register address (0..63)
 *   - uio_in[6]   : wr_stb (pulse high for 1 clk to write)
 *   - uio_in[7]   : rd_stb (pulse high for 1 clk to latch read address)
 *   - uo_out[7:0] : read data byte for the last latched read address
 *   - uio_out/uio_oe are unused (held at 0)
 *
 * Control register map (addresses are hex):
 *   0x00 CTRL  (write)
 *        bit0: load_mode (1 = scene load enabled)
 *        bit1: load_pulse (write 1 to emit a 1-cycle load_valid)
 *        bit2: job_start  (write 1 to request a job; held until accepted)
 *        bit3: clear_status (write 1 to clear sticky result flags)
 *
 * Scene load registers (write):
 *   0x01 load_addr[7:0]
 *   0x02 load_addr[14:8] in bits[6:0]
 *   0x03 load_data in bit[0]
 *
 * Job registers (write):
 *   0x10 ix0 (bits[5:0])
 *   0x11 iy0 (bits[5:0])
 *   0x12 iz0 (bits[5:0])
 *   0x13 sx/sy/sz in bits[2:0]
 *   0x14..0x17 next_x (32-bit little-endian)
 *   0x18..0x1B next_y
 *   0x1C..0x1F next_z
 *   0x20..0x23 inc_x
 *   0x24..0x27 inc_y
 *   0x28..0x2B inc_z
 *   0x2C..0x2D max_steps (10-bit, little-endian: [7:0] then [9:8])
 *
 * Result registers (read):
 *   0x30 status0: {2'b0, load_complete_sticky, load_ready, job_ready, ray_timeout_sticky, ray_hit_sticky, ray_done_sticky}
 *   0x31 hit_face_id (bits[2:0])
 *   0x32..0x33 steps_taken (16-bit little-endian)
 *   0x34..0x35 hit_voxel_x (16-bit little-endian)
 *   0x36..0x37 hit_voxel_y
 *   0x38..0x39 hit_voxel_z
 *   0x3A..0x3B write_count (16-bit little-endian)
 */

`default_nettype none

module tt_um_ieeeuoftasic_raytracer (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n      // reset_n (active low)
);

  // -------------------------------------------------------------------------
  // Host register file / protocol
  // -------------------------------------------------------------------------
  wire [5:0] reg_addr = uio_in[5:0];
  wire       wr_stb   = uio_in[6];
  wire       rd_stb   = uio_in[7];
  wire [7:0] wr_data  = ui_in;

  // Latch read address on rd_stb so uo_out is stable.
  logic [5:0] rd_addr_q;

  // Byte-addressable register storage for writable fields.
  logic [7:0] regfile [0:63];

  // Control / pulses
  logic load_mode_q;
  logic load_pulse_q;
  logic clear_status_q;
  logic job_req_pending_q;

  // Sticky captured results
  logic        ray_done_sticky_q;
  logic        ray_hit_sticky_q;
  logic        ray_timeout_sticky_q;
  logic        load_complete_sticky_q;
  logic [2:0]  hit_face_id_q;
  logic [15:0] steps_taken_q;
  logic [15:0] hit_voxel_x_q;
  logic [15:0] hit_voxel_y_q;
  logic [15:0] hit_voxel_z_q;

  // -------------------------------------------------------------------------
  // Assemble wide fields from regfile
  // -------------------------------------------------------------------------
  wire [14:0] load_addr_w = {regfile[8'h02][6:0], regfile[8'h01]};
  wire        load_data_w = regfile[8'h03][0];

  wire [5:0]  job_ix0_w = regfile[8'h10][5:0];
  wire [5:0]  job_iy0_w = regfile[8'h11][5:0];
  wire [5:0]  job_iz0_w = regfile[8'h12][5:0];
  wire        job_sx_w  = regfile[8'h13][0];
  wire        job_sy_w  = regfile[8'h13][1];
  wire        job_sz_w  = regfile[8'h13][2];

  wire [31:0] job_next_x_w = {regfile[8'h17], regfile[8'h16], regfile[8'h15], regfile[8'h14]};
  wire [31:0] job_next_y_w = {regfile[8'h1B], regfile[8'h1A], regfile[8'h19], regfile[8'h18]};
  wire [31:0] job_next_z_w = {regfile[8'h1F], regfile[8'h1E], regfile[8'h1D], regfile[8'h1C]};
  wire [31:0] job_inc_x_w  = {regfile[8'h23], regfile[8'h22], regfile[8'h21], regfile[8'h20]};
  wire [31:0] job_inc_y_w  = {regfile[8'h27], regfile[8'h26], regfile[8'h25], regfile[8'h24]};
  wire [31:0] job_inc_z_w  = {regfile[8'h2B], regfile[8'h2A], regfile[8'h29], regfile[8'h28]};
  wire [9:0]  job_max_steps_w = {regfile[8'h2D][1:0], regfile[8'h2C]};

  // -------------------------------------------------------------------------
  // Instantiate the raytracer RTL
  // -------------------------------------------------------------------------
  logic job_ready_w, load_ready_w;
  logic ray_done_w, ray_hit_w, ray_timeout_w;
  logic [15:0] hit_voxel_x_w, hit_voxel_y_w, hit_voxel_z_w;
  logic [2:0]  hit_face_id_w;
  logic [15:0] steps_taken_w;
  logic [15:0] write_count_w;
  logic        load_complete_w;

  // Job valid is held until accepted (job_ready high).
  wire job_valid_w = job_req_pending_q;
  // Load valid is a 1-cycle pulse (host can pulse per write).
  wire load_valid_w = load_pulse_q;

  raytracer_top #(
    .COORD_WIDTH(16),
    .COORD_W(6),
    .TIMER_WIDTH(32),
    .W(32),
    .MAX_VAL(31),
    .ADDR_BITS(15),
    .X_BITS(6),
    .Y_BITS(6),
    .Z_BITS(6),
    .MAX_STEPS_BITS(10),
    .STEP_COUNT_WIDTH(16)
  ) u_raytracer (
    .clk(clk),
    .rst_n(rst_n),

    .job_valid(job_valid_w),
    .job_ready(job_ready_w),
    .job_ix0(job_ix0_w),
    .job_iy0(job_iy0_w),
    .job_iz0(job_iz0_w),
    .job_sx(job_sx_w),
    .job_sy(job_sy_w),
    .job_sz(job_sz_w),
    .job_next_x(job_next_x_w),
    .job_next_y(job_next_y_w),
    .job_next_z(job_next_z_w),
    .job_inc_x(job_inc_x_w),
    .job_inc_y(job_inc_y_w),
    .job_inc_z(job_inc_z_w),
    .job_max_steps(job_max_steps_w),

    .load_mode(load_mode_q),
    .load_valid(load_valid_w),
    .load_ready(load_ready_w),
    .load_addr(load_addr_w),
    .load_data(load_data_w),
    .write_count(write_count_w),
    .load_complete(load_complete_w),

    .ray_done(ray_done_w),
    .ray_hit(ray_hit_w),
    .ray_timeout(ray_timeout_w),
    .hit_voxel_x(hit_voxel_x_w),
    .hit_voxel_y(hit_voxel_y_w),
    .hit_voxel_z(hit_voxel_z_w),
    .hit_face_id(hit_face_id_w),
    .steps_taken(steps_taken_w)
  );

  // -------------------------------------------------------------------------
  // Register writes, read address latch, and sticky result capture
  // -------------------------------------------------------------------------
  integer i;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_addr_q <= 6'd0;
      load_mode_q <= 1'b0;
      load_pulse_q <= 1'b0;
      clear_status_q <= 1'b0;
      job_req_pending_q <= 1'b0;

      ray_done_sticky_q <= 1'b0;
      ray_hit_sticky_q <= 1'b0;
      ray_timeout_sticky_q <= 1'b0;
      load_complete_sticky_q <= 1'b0;
      hit_face_id_q <= 3'd0;
      steps_taken_q <= 16'd0;
      hit_voxel_x_q <= 16'd0;
      hit_voxel_y_q <= 16'd0;
      hit_voxel_z_q <= 16'd0;

      for (i = 0; i < 64; i = i + 1) begin
        regfile[i] <= 8'h00;
      end
    end else begin
      // default pulses
      load_pulse_q   <= 1'b0;
      clear_status_q <= 1'b0;

      // latch read address (stable readback)
      if (rd_stb) begin
        rd_addr_q <= reg_addr;
      end

      // Handle host writes
      if (wr_stb) begin
        // Store all writable bytes by default (except read-only result space)
        if (reg_addr < 6'h30) begin
          regfile[reg_addr] <= wr_data;
        end

        // CTRL register side-effects
        if (reg_addr == 6'h00) begin
          load_mode_q <= wr_data[0];
          if (wr_data[1]) load_pulse_q <= 1'b1;
          if (wr_data[2]) job_req_pending_q <= 1'b1;
          if (wr_data[3]) clear_status_q <= 1'b1;
        end
      end

      // If job is accepted, drop the pending request
      if (job_req_pending_q && job_ready_w) begin
        // job_valid_w is job_req_pending_q, so (valid && ready) implies accept
        job_req_pending_q <= 1'b0;
      end

      // Clear sticky flags on request
      if (clear_status_q) begin
        ray_done_sticky_q       <= 1'b0;
        ray_hit_sticky_q        <= 1'b0;
        ray_timeout_sticky_q    <= 1'b0;
        load_complete_sticky_q  <= 1'b0;
      end

      // Sticky capture of load completion (useful if load_mode later drops)
      if (load_complete_w) begin
        load_complete_sticky_q <= 1'b1;
      end

      // Capture results on ray_done
      if (ray_done_w) begin
        ray_done_sticky_q    <= 1'b1;
        ray_hit_sticky_q     <= ray_hit_w;
        ray_timeout_sticky_q <= ray_timeout_w;
        hit_face_id_q        <= hit_face_id_w;
        steps_taken_q        <= steps_taken_w;
        hit_voxel_x_q        <= hit_voxel_x_w;
        hit_voxel_y_q        <= hit_voxel_y_w;
        hit_voxel_z_q        <= hit_voxel_z_w;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Readback mux (uo_out)
  // -------------------------------------------------------------------------
  logic [7:0] rd_data;
  always_comb begin
    unique case (rd_addr_q)
      6'h00: rd_data = regfile[6'h00];
      6'h01: rd_data = regfile[6'h01];
      6'h02: rd_data = regfile[6'h02];
      6'h03: rd_data = regfile[6'h03];

      // Job regs (readback)
      6'h10: rd_data = regfile[6'h10];
      6'h11: rd_data = regfile[6'h11];
      6'h12: rd_data = regfile[6'h12];
      6'h13: rd_data = regfile[6'h13];

      // Result window
      6'h30: rd_data = {2'b00,
                        load_complete_sticky_q,
                        load_ready_w,
                        job_ready_w,
                        ray_timeout_sticky_q,
                        ray_hit_sticky_q,
                        ray_done_sticky_q};
      6'h31: rd_data = {5'b0, hit_face_id_q};
      6'h32: rd_data = steps_taken_q[7:0];
      6'h33: rd_data = steps_taken_q[15:8];
      6'h34: rd_data = hit_voxel_x_q[7:0];
      6'h35: rd_data = hit_voxel_x_q[15:8];
      6'h36: rd_data = hit_voxel_y_q[7:0];
      6'h37: rd_data = hit_voxel_y_q[15:8];
      6'h38: rd_data = hit_voxel_z_q[7:0];
      6'h39: rd_data = hit_voxel_z_q[15:8];
      6'h3A: rd_data = write_count_w[7:0];
      6'h3B: rd_data = write_count_w[15:8];

      default: rd_data = regfile[rd_addr_q];
    endcase
  end

  assign uo_out  = rd_data;

  // Unused bidirectional pins must be tied off.
  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, 1'b0};

endmodule

`default_nettype wire
