`timescale 1ns/1ps
`default_nettype none

// ============================================================
// Module 5: voxel_addr_map
// ============================================================
module voxel_addr_map #(
  parameter int X_BITS = 5,
  parameter int Y_BITS = 5,
  parameter int Z_BITS = 5,
  parameter bit MAP_ZYX = 1'b1
)(
  input  logic [X_BITS-1:0] x,
  input  logic [Y_BITS-1:0] y,
  input  logic [Z_BITS-1:0] z,
  output logic [X_BITS+Y_BITS+Z_BITS-1:0] addr
);

  localparam int ADDR_BITS = X_BITS + Y_BITS + Z_BITS;

  initial begin
    if (X_BITS < 1 || X_BITS > 16) $error("voxel_addr_map: X_BITS=%0d out of range [1:16]", X_BITS);
    if (Y_BITS < 1 || Y_BITS > 16) $error("voxel_addr_map: Y_BITS=%0d out of range [1:16]", Y_BITS);
    if (Z_BITS < 1 || Z_BITS > 16) $error("voxel_addr_map: Z_BITS=%0d out of range [1:16]", Z_BITS);
    if (ADDR_BITS > 32)            $error("voxel_addr_map: ADDR_BITS=%0d exceeds 32", ADDR_BITS);
  end

  always_comb begin
    if (MAP_ZYX) addr = {z, y, x};  // default: (z<<10)|(y<<5)|x for 5/5/5
    else         addr = {x, y, z};
  end

endmodule

`default_nettype wire
