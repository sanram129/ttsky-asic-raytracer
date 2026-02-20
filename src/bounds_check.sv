// =============================================================================
// Module: bounds_check
// Description: Voxel bounds checking for 3D DDA raytracer
//              Checks if any coordinate exceeds the maximum valid value
//              Optimized for minimal area and power
// =============================================================================

module bounds_check #(
    parameter int COORD_W = 6,   // Coordinate bit width (default 6 for future-proofing)
    parameter int MAX_VAL = 31   // Maximum valid coordinate value (0..31 inclusive)
)(
    // Voxel coordinates (unsigned)
    input  logic [COORD_W-1:0] ix,
    input  logic [COORD_W-1:0] iy,
    input  logic [COORD_W-1:0] iz,
    
    // Out-of-bounds flag: 1 if any coordinate > MAX_VAL, else 0
    output logic out_of_bounds
);

    // =========================================================================
    // Bounds Check Logic
    // =========================================================================
    // Check if any coordinate exceeds the maximum valid value
    // For COORD_W=5 and MAX_VAL=31: All 5-bit values are 0..31, so always in-bounds
    // For COORD_W=6 and MAX_VAL=31: Values 32..63 are out-of-bounds
    //
    // Uses simple unsigned comparisons and a 3-input OR gate for minimal area
    
    assign out_of_bounds = (ix > MAX_VAL) | (iy > MAX_VAL) | (iz > MAX_VAL);

endmodule
