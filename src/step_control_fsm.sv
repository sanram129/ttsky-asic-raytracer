// Step Control FSM for DDA Ray-Tracer
// Complete implementation with job loading, bounds checking, solid detection
// SKY130 Process Target

module step_control_fsm #(
    parameter int X_BITS = 5,           // X coordinate width
    parameter int Y_BITS = 5,           // Y coordinate width
    parameter int Z_BITS = 5,           // Z coordinate width
    parameter int TIMER_WIDTH = 32,     // Timer value width (fixed-point)
    parameter int STEP_COUNT_WIDTH = 16 // Step counter width
) (
    input  logic                          clock,
    input  logic                          reset,
    
    // Job control
    input  logic                          job_loaded,      // Job parameters loaded and ready
    output logic                          ready,           // Ready for new job
    output logic                          active,          // Actively ray tracing (drives pipeline valid)
    
    // Job parameters (loaded once at start)
    input  logic [X_BITS-1:0]             job_init_x,
    input  logic [Y_BITS-1:0]             job_init_y,
    input  logic [Z_BITS-1:0]             job_init_z,
    input  logic [TIMER_WIDTH-1:0]        job_timer_x,
    input  logic [TIMER_WIDTH-1:0]        job_timer_y,
    input  logic [TIMER_WIDTH-1:0]        job_timer_z,
    input  logic [STEP_COUNT_WIDTH-1:0]   max_steps,       // Maximum steps before timeout
    
    // Voxel data from RAM
    input  logic                          solid_bit,       // Current voxel is solid (from RAM)
    input  logic                          solid_valid,     // Solid bit data is valid
    
    // Bounds checking
    input  logic                          out_of_bounds,   // Current voxel is out of bounds
    
    // Pipeline computed values (from axis_choose + step_update)
    input  logic [X_BITS-1:0]             pipeline_next_x,
    input  logic [Y_BITS-1:0]             pipeline_next_y,
    input  logic [Z_BITS-1:0]             pipeline_next_z,
    input  logic [TIMER_WIDTH-1:0]        pipeline_next_timer_x,
    input  logic [TIMER_WIDTH-1:0]        pipeline_next_timer_y,
    input  logic [TIMER_WIDTH-1:0]        pipeline_next_timer_z,
    input  logic [2:0]                    pipeline_face_id,
    
    // Current state outputs (continuously updated)
    output logic [X_BITS-1:0]             current_voxel_x,
    output logic [Y_BITS-1:0]             current_voxel_y,
    output logic [Z_BITS-1:0]             current_voxel_z,
    output logic [TIMER_WIDTH-1:0]        current_timer_x,
    output logic [TIMER_WIDTH-1:0]        current_timer_y,
    output logic [TIMER_WIDTH-1:0]        current_timer_z,
    output logic [STEP_COUNT_WIDTH-1:0]   steps_taken,
    
    // Termination outputs
    output logic                          done,            // Ray trace complete
    output logic                          hit,             // Hit solid voxel
    output logic                          timeout,         // Exceeded max_steps
    
    // Result outputs (valid when done=1)
    output logic [X_BITS-1:0]             hit_voxel_x,
    output logic [Y_BITS-1:0]             hit_voxel_y,
    output logic [Z_BITS-1:0]             hit_voxel_z,
    output logic [2:0]                    face_id          // One-hot: [2]=Z, [1]=Y, [0]=X
);

    // FSM State Definition
    typedef enum logic [1:0] {
        IDLE           = 2'b00,   // Waiting for job
        INIT           = 2'b01,   // Initialize from job
        RUNNING        = 2'b10,   // Actively ray tracing (pipeline processing)
        FINISH         = 2'b11    // Output results
    } state_t;
    
    state_t current_state, next_state;
    
    // Internal state registers
    logic [X_BITS-1:0]           voxel_x_reg;
    logic [Y_BITS-1:0]           voxel_y_reg;
    logic [Z_BITS-1:0]           voxel_z_reg;
    logic [TIMER_WIDTH-1:0]      timer_x_reg;
    logic [TIMER_WIDTH-1:0]      timer_y_reg;
    logic [TIMER_WIDTH-1:0]      timer_z_reg;
    logic [STEP_COUNT_WIDTH-1:0] step_counter;
    
    // Job parameter registers (loaded once)
    logic [STEP_COUNT_WIDTH-1:0] max_steps_reg;
    
    // Result registers
    logic [X_BITS-1:0]           hit_x_reg;
    logic [Y_BITS-1:0]           hit_y_reg;
    logic [Z_BITS-1:0]           hit_z_reg;
    logic [2:0]                  face_reg;
    
    // Status flags
    logic                        hit_flag;
    logic                        timeout_flag;
    logic                        bounds_flag;
    
    // Previous voxel position to detect when position changes
    logic [X_BITS-1:0]           voxel_x_prev;
    logic [Y_BITS-1:0]           voxel_y_prev;
    logic [Z_BITS-1:0]           voxel_z_prev;
    
    //----------------------------------------------------------------------
    // State Register
    //----------------------------------------------------------------------
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end
    
    //----------------------------------------------------------------------
    // Next State Logic
    //----------------------------------------------------------------------
    always_comb begin
        next_state = current_state;
        
        case (current_state)
            IDLE: begin
                if (job_loaded) begin
                    next_state = INIT;
                end
            end
            
            INIT: begin
                // Initialize complete, start ray tracing
                next_state = RUNNING;
            end
            
            RUNNING: begin
                // Continue running until we have valid data AND termination condition
                if (solid_valid) begin
                    // Check immediate termination conditions (solid hit, bounds, timeout)
                    if (hit_flag || timeout_flag || bounds_flag || 
                        solid_bit || out_of_bounds || (step_counter >= max_steps_reg)) begin
                        next_state = FINISH;
                    end
                    // else stay in RUNNING, will update position on next cycle
                end
                // else stay in RUNNING, waiting for pipeline to produce valid data
            end
            
            FINISH: begin
                // Hold results for one cycle
                next_state = IDLE;
            end
            
            default: begin
                next_state = IDLE;
            end
        endcase
    end
    
    //----------------------------------------------------------------------
    // Datapath Logic
    //----------------------------------------------------------------------
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            voxel_x_reg     <= '0;
            voxel_y_reg     <= '0;
            voxel_z_reg     <= '0;
            timer_x_reg     <= '0;
            timer_y_reg     <= '0;
            timer_z_reg     <= '0;
            step_counter    <= '0;
            max_steps_reg   <= '0;
            hit_x_reg       <= '0;
            hit_y_reg       <= '0;
            hit_z_reg       <= '0;
            face_reg        <= 3'b000;
            hit_flag         <= 1'b0;
            timeout_flag     <= 1'b0;
            bounds_flag      <= 1'b0;
            voxel_x_prev     <= '0;
            voxel_y_prev     <= '0;
            voxel_z_prev     <= '0;
        end else begin
            case (current_state)
                IDLE: begin
                    // Reset flags
                    hit_flag         <= 1'b0;
                    timeout_flag     <= 1'b0;
                    bounds_flag      <= 1'b0;
                    step_counter     <= '0;
                    voxel_x_prev     <= '0;
                    voxel_y_prev     <= '0;
                    voxel_z_prev     <= '0;
                end
                
                INIT: begin
                    // Load job parameters
                    voxel_x_reg   <= job_init_x;
                    voxel_y_reg   <= job_init_y;
                    voxel_z_reg   <= job_init_z;
                    timer_x_reg   <= job_timer_x;
                    timer_y_reg   <= job_timer_y;
                    timer_z_reg   <= job_timer_z;
                    max_steps_reg <= max_steps;
                    step_counter  <= '0;
                end
                
                RUNNING: begin
                    // Track previous voxel position for change detection
                    voxel_x_prev <= voxel_x_reg;
                    voxel_y_prev <= voxel_y_reg;
                    voxel_z_prev <= voxel_z_reg;
                    
                    // When pipeline returns valid data, process it
                    if (solid_valid) begin
                        // Check termination conditions
                        if (solid_bit) begin
                            hit_flag  <= 1'b1;
                            hit_x_reg <= voxel_x_reg;  // Hit at CURRENT position
                            hit_y_reg <= voxel_y_reg;
                            hit_z_reg <= voxel_z_reg;
                            // On hit, report the face used to ENTER the current voxel.
                            // face_reg is updated when we ADVANCE to the next voxel, so it
                            // already corresponds to the entry face for voxel_*_reg.
                            // Do NOT overwrite it with pipeline_face_id (which corresponds
                            // to the next step out of the current voxel).
                        end
                        
                        if (step_counter >= max_steps_reg) begin
                            timeout_flag <= 1'b1;
                        end
                        
                        if (out_of_bounds) begin
                            bounds_flag <= 1'b1;
                        end
                        
                        // If not terminating, load next position for next iteration
                        if (!(hit_flag || timeout_flag || bounds_flag || solid_bit || out_of_bounds || (step_counter >= max_steps_reg))) begin
                            voxel_x_reg <= pipeline_next_x;
                            voxel_y_reg <= pipeline_next_y;
                            voxel_z_reg <= pipeline_next_z;
                            timer_x_reg <= pipeline_next_timer_x;
                            timer_y_reg <= pipeline_next_timer_y;
                            timer_z_reg <= pipeline_next_timer_z;
                            face_reg    <= pipeline_face_id;
                            // Increment counter only when position changes from previous cycle
                            if ((voxel_x_reg != voxel_x_prev) || (voxel_y_reg != voxel_y_prev) || (voxel_z_reg != voxel_z_prev)) begin
                                step_counter <= step_counter + 1'b1;
                            end
                        end
                    end
                end
                
                FINISH: begin
                    // Hold final values
                end
                
                default: begin
                    // Do nothing
                end
            endcase
        end
    end
    
    //----------------------------------------------------------------------
    // Output Logic
    //----------------------------------------------------------------------
    always_comb begin
        // Control signals
        ready = (current_state == IDLE);
        active = (current_state == RUNNING);  // Drives pipeline valid signal
        
        // Termination signals
        done    = (current_state == FINISH);
        hit     = hit_flag;
        timeout = timeout_flag;
        
        // Current state outputs (always visible)
        current_voxel_x = voxel_x_reg;
        current_voxel_y = voxel_y_reg;
        current_voxel_z = voxel_z_reg;
        current_timer_x = timer_x_reg;
        current_timer_y = timer_y_reg;
        current_timer_z = timer_z_reg;
        steps_taken     = step_counter;
        
        // Result outputs (final hit location)
        hit_voxel_x = hit_x_reg;
        hit_voxel_y = hit_y_reg;
        hit_voxel_z = hit_z_reg;
        face_id     = face_reg;
    end

endmodule