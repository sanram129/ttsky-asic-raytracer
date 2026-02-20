`timescale 1ns/1ps
`default_nettype none

// ============================================================
// Module 7: scene_loader_if
// ============================================================
module scene_loader_if #(
  parameter int ADDR_BITS = 15,
  parameter bit ENABLE_COUNTER = 1'b1
)(
  input  logic                 clk,
  input  logic                 rst_n,

  input  logic                 load_mode,
  input  logic                 load_valid,
  output logic                 load_ready,
  input  logic [ADDR_BITS-1:0] load_addr,
  input  logic                 load_data,

  output logic                 we,
  output logic [ADDR_BITS-1:0] waddr,
  output logic                 wdata,

  output logic [ADDR_BITS:0]   write_count,
  output logic                 load_complete
);

  localparam int TOTAL_VOXELS = (1 << ADDR_BITS);

  always_comb begin
    load_ready = 1'b1;
    we    = load_mode && load_valid && load_ready;
    waddr = load_addr;
    wdata = load_data;
  end

  generate
    if (ENABLE_COUNTER) begin : g_counter
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          write_count   <= '0;
          load_complete <= 1'b0;
        end else begin
          if (!load_mode) begin
            write_count   <= '0;
            load_complete <= 1'b0;
          end else if (we) begin
            // Always increment counter when writing
            write_count <= write_count + 1'b1;
            // Set complete when we reach the last write
            if (write_count == (TOTAL_VOXELS - 1)) load_complete <= 1'b1;
          end
        end
      end
    end else begin : g_no_counter
      always_comb begin
        write_count   = '0;
        load_complete = 1'b0;
      end
    end
  endgenerate

endmodule

`default_nettype wire
