`timescale 1ns/1ps
module tb_elevator;
    parameter N_FLOORS = 4;
    reg clk;
    reg rst;
    reg [N_FLOORS-1:0] inside_req;
    reg [N_FLOORS-1:0] up_call;
    reg [N_FLOORS-1:0] down_call;

    wire [1:0] current_floor;
    wire motor_up, motor_down, door_open, direction;

    elevator #(.N_FLOORS(N_FLOORS), .FLOOR_BITS(2), .DOOR_TIMER(20)) U (
        .clk(clk), .rst(rst),
        .inside_req(inside_req), .up_call(up_call), .down_call(down_call),
        .current_floor(current_floor),
        .motor_up(motor_up), .motor_down(motor_down),
        .door_open(door_open), .direction(direction)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz-like simple clock for simulation
    end

    initial begin
        // init
        rst = 1;
        inside_req = 0;
        up_call = 0;
        down_call = 0;
        #20;
        rst = 0;

        // scenario 1: someone on floor 2 calls down
        #30;
        down_call = 4'b0100; // floor 2
        #10;
        down_call = 0;

        // scenario 2: while elevator moves, inside someone presses floor 0
        #200;
        inside_req = 4'b0001; // request floor 0
        #10;
        inside_req = 0;

        // scenario 3: two calls in opposite sides
        #300;
        up_call = 4'b0010; // floor 1 up
        down_call = 4'b1000; // floor 3 down
        #10;
        up_call = 0;
        down_call = 0;

        // run for a while then finish
        #2000;
        $finish;
    end

    // simple monitor
    initial begin
        $display("time\tfloor\tup\tdown\tdoor\tdir");
        $monitor("%0t\t%0d\t%b\t%b\t%b\t%b", $time, current_floor, motor_up, motor_down, door_open, direction);
    end
endmodule
