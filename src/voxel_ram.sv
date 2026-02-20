`timescale 1ns/1ps
`default_nettype none

// ============================================================
// Module 6: voxel_ram
//  FIXED: sync read now actually uses raddr_q, and WRITE_FIRST compares waddr==raddr_q.
// ============================================================
module voxel_ram #(
  parameter int ADDR_BITS = 15,
  parameter bit SYNC_READ = 1'b1,
  parameter bit WRITE_FIRST = 1'b1
)(
  input  logic                 clk,
  input  logic                 rst_n,

  input  logic [ADDR_BITS-1:0] raddr,
  output logic                 rdata,

  input  logic                 we,
  input  logic [ADDR_BITS-1:0] waddr,
  input  logic                 wdata
);

  localparam int DEPTH = 1 << ADDR_BITS;
  logic mem [0:DEPTH-1];

  logic [ADDR_BITS-1:0] raddr_q;

  // Initialize memory to all zeros for simulation
  initial begin
    for (int i = 0; i < DEPTH; i++) begin
      mem[i] = 1'b0;
    end
  end

  // Write (sync)
  always_ff @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
  end

  generate
    if (SYNC_READ) begin : g_sync_read
      // Register the address
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) raddr_q <= '0;
        else        raddr_q <= raddr;
      end

      // Register we to track write activity
      logic we_q;
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) we_q <= 1'b0;
        else        we_q <= we;
      end
      
      // Register the output (1-cycle latency from raddr -> rdata)
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          rdata <= 1'b0;
        end else begin
          // WRITE_FIRST: if writing to the address we just read, forward the write data
          if (WRITE_FIRST && we && (waddr == raddr_q)) begin
            rdata <= wdata;
          // If there was a write last cycle to a different address, re-read to ensure fresh data  
          end else if (we_q) begin
            rdata <= mem[raddr_q];
          end else begin
            rdata <= mem[raddr_q];
          end
        end
      end
    end else begin : g_comb_read
      always_comb rdata = mem[raddr];
    end
  endgenerate

endmodule

`default_nettype wire
