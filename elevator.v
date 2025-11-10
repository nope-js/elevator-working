`timescale 1ns/1ps
module elevator #(
    parameter N_FLOORS = 4,
    parameter FLOOR_BITS = 2,    // log2(N_FLOORS) for N_FLOORS=4 -> 2
    parameter DOOR_TIMER = 50    // cycles door remains open
)(
    input  wire                          clk,
    input  wire                          rst,            // synchronous active-high reset
    input  wire [N_FLOORS-1:0]          inside_req,     // request from inside for each floor
    input  wire [N_FLOORS-1:0]          up_call,        // hall up calls (for floors 0..N-2)
    input  wire [N_FLOORS-1:0]          down_call,      // hall down calls (for floors 1..N-1)
    output reg  [FLOOR_BITS-1:0]        current_floor,
    output reg                           motor_up,
    output reg                           motor_down,
    output reg                           door_open,
    output reg                           direction       // 1 == up, 0 == down/idle
);

    // internal request memory (latches all requests until cleared)
    reg [N_FLOORS-1:0] reqs;

    // states
    typedef enum reg [1:0] {IDLE=2'b00, MOVING_UP=2'b01, MOVING_DOWN=2'b10, DOOR=2'b11} state_t;
    state_t state, next_state;

    // door timer
    reg [$clog2(DOOR_TIMER+1)-1:0] door_cnt;

    integer i;

    // helper functions (combinational checks)
    function automatic bit any_request_above(input [FLOOR_BITS-1:0] floor, input [N_FLOORS-1:0] r);
        integer idx;
        begin
            any_request_above = 0;
            for (idx = floor + 1; idx < N_FLOORS; idx = idx + 1)
                if (r[idx]) any_request_above = 1;
        end
    endfunction

    function automatic bit any_request_below(input [FLOOR_BITS-1:0] floor, input [N_FLOORS-1:0] r);
        integer idx;
        begin
            any_request_below = 0;
            for (idx = 0; idx < floor; idx = idx + 1)
                if (r[idx]) any_request_below = 1;
        end
    endfunction

    // combine all inputs into requests memory on each clock
    always @(posedge clk) begin
        if (rst) begin
            reqs <= {N_FLOORS{1'b0}};
        end else begin
            // OR in all active buttons/calls into reqs
            reqs <= reqs | inside_req | up_call | down_call;
            // Note: calls from invalid positions (e.g. up_call at top) are OK if user sets them,
            // they will simply be treated as requests for that floor.
        end
    end

    // State register
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            current_floor <= 0;
            door_open <= 0;
            motor_up <= 0;
            motor_down <= 0;
            direction <= 1'b1; // prefer up on reset
            door_cnt <= 0;
        end else begin
            state <= next_state;
            // motor/door/current_floor updates happen in combinational block below via nonblocking style
        end
    end

    // Next-state and outputs (combinational-ish but synchronized through state register)
    always @(*) begin
        // defaults
        motor_up = 0;
        motor_down = 0;
        door_open = 0;
        next_state = state;
        // copy current floor to local var (can't modify reg in combinational assign outside clock)
        // We'll determine transitions and output control.
        case (state)
            IDLE: begin
                // choose a direction if there is any request
                if (reqs != 0) begin
                    // if there is request at current floor, open door
                    if (reqs[current_floor]) begin
                        next_state = DOOR;
                    end else begin
                        // pick direction: if any above then up, else down
                        if (any_request_above(current_floor, reqs)) begin
                            next_state = MOVING_UP;
                            direction = 1'b1;
                            motor_up = 1;
                        end else if (any_request_below(current_floor, reqs)) begin
                            next_state = MOVING_DOWN;
                            direction = 1'b0;
                            motor_down = 1;
                        end else begin
                            next_state = IDLE;
                        end
                    end
                end else begin
                    next_state = IDLE;
                end
            end

            MOVING_UP: begin
                direction = 1'b1;
                motor_up = 1;
                motor_down = 0;
                // when reaching next clock, floor will be incremented in sequential block below
                // check if we should stop upon arrival (handled after floor increment)
                next_state = MOVING_UP;
            end

            MOVING_DOWN: begin
                direction = 1'b0;
                motor_down = 1;
                motor_up = 0;
                next_state = MOVING_DOWN;
            end

            DOOR: begin
                // open door, stop motors
                door_open = 1;
                motor_up = 0;
                motor_down = 0;
                next_state = DOOR;
            end

            default: begin
                next_state = IDLE;
            end
        endcase
    end

    // Movement and door-timer sequential logic (synchronous)
    always @(posedge clk) begin
        if (rst) begin
            current_floor <= 0;
            door_cnt <= 0;
        end else begin
            case (state)
                IDLE: begin
                    // if next_state decided to move, increment/decrement floor by one per cycle
                    if (next_state == MOVING_UP) begin
                        if (current_floor < N_FLOORS-1)
                            current_floor <= current_floor + 1;
                    end else if (next_state == MOVING_DOWN) begin
                        if (current_floor > 0)
                            current_floor <= current_floor - 1;
                    end else if (next_state == DOOR) begin
                        door_cnt <= 0;
                        // clear requests at this floor
                        reqs[current_floor] <= 0;
                    end
                end

                MOVING_UP: begin
                    // Move one floor per cycle (simple model)
                    if (current_floor < N_FLOORS-1)
                        current_floor <= current_floor + 1;

                    // if we arrived at a floor that has a request, open door
                    if (reqs[current_floor + 1]) begin
                        // We incremented current_floor above, so check new floor index
                        reqs[current_floor + 1] <= 0;
                        next_state <= DOOR; // request door open at next cycle
                        // set door counter next cycle
                        door_cnt <= 0;
                        // set state change will occur on next posedge via state <= next_state
                    end else begin
                        // continue moving if more requests above; else check below to reverse
                        if (!any_request_above(current_floor, reqs)) begin
                            if (any_request_below(current_floor, reqs)) begin
                                // reverse
                                next_state <= MOVING_DOWN;
                            end else begin
                                next_state <= IDLE;
                            end
                        end
                    end
                end

                MOVING_DOWN: begin
                    if (current_floor > 0)
                        current_floor <= current_floor - 1;

                    // arrived at new floor? check request
                    if (reqs[current_floor - 1]) begin
                        reqs[current_floor - 1] <= 0;
                        next_state <= DOOR;
                        door_cnt <= 0;
                    end else begin
                        if (!any_request_below(current_floor, reqs)) begin
                            if (any_request_above(current_floor, reqs)) begin
                                next_state <= MOVING_UP;
                            end else begin
                                next_state <= IDLE;
                            end
                        end
                    end
                end

                DOOR: begin
                    // door open for DOOR_TIMER cycles then go IDLE or to next direction
                    if (door_cnt < DOOR_TIMER) begin
                        door_cnt <= door_cnt + 1;
                        // clear requests for this floor (redundant but safe)
                        reqs[current_floor] <= 0;
                    end else begin
                        // decide next action
                        if (any_request_above(current_floor, reqs)) begin
                            next_state <= MOVING_UP;
                        end else if (any_request_below(current_floor, reqs)) begin
                            next_state <= MOVING_DOWN;
                        end else begin
                            next_state <= IDLE;
                        end
                        door_cnt <= 0;
                    end
                end

                default: begin
                    // do nothing
                end
            endcase
        end
    end

endmodule
