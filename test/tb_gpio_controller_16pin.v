`timescale 1ns / 1ps

module tb_gpio_controller_16pin;

    // Clock and reset
    reg clk;
    reg rst_n;
    
    // GPIO pins (bidirectional)
    wire [15:0] gpio_pins;
    reg [15:0] gpio_pins_drive;
    reg [15:0] gpio_pins_oe;
    
    // Memory interface
    reg [7:0] Mem_Addr;
    reg [31:0] Mem_WrData;
    wire [31:0] Mem_RdData;
    reg MemWrite;
    
    // Interrupt
    wire gpio_interrupt;
    
    // Alternate function signals
    reg uart_tx_out, uart_tx_en, uart_rx_en;
    wire uart_rx_in;
    
    reg spi_mosi_out, spi_sck_out, spi_ss_out;
    reg spi_mosi_en, spi_sck_en, spi_ss_en, spi_miso_en;
    wire spi_miso_in;
    
    wire i2c_sda, i2c_scl;
    reg i2c_sda_en, i2c_scl_en;
    
    // Test control signals
    integer test_case;
    integer error_count;
    
    // Instantiate the Device Under Test (DUT)
    gpio_controller_16pin dut (
        .clk(clk),
        .rst_n(rst_n),
        .gpio_pins(gpio_pins),
        .Mem_Addr(Mem_Addr),
        .Mem_WrData(Mem_WrData),
        .Mem_RdData(Mem_RdData),
        .MemWrite(MemWrite),
        .gpio_interrupt(gpio_interrupt),
        .uart_tx_out(uart_tx_out),
        .uart_rx_in(uart_rx_in),
        .uart_tx_en(uart_tx_en),
        .uart_rx_en(uart_rx_en),
        .spi_mosi_out(spi_mosi_out),
        .spi_sck_out(spi_sck_out),
        .spi_ss_out(spi_ss_out),
        .spi_miso_in(spi_miso_in),
        .spi_mosi_en(spi_mosi_en),
        .spi_sck_en(spi_sck_en),
        .spi_ss_en(spi_ss_en),
        .spi_miso_en(spi_miso_en),
        .i2c_sda(i2c_sda),
        .i2c_scl(i2c_scl),
        .i2c_sda_en(i2c_sda_en),
        .i2c_scl_en(i2c_scl_en)
    );
    
    // Bidirectional GPIO pin control
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : gpio_control
            assign gpio_pins[i] = gpio_pins_oe[i] ? gpio_pins_drive[i] : 1'bz;
        end
    endgenerate
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz clock
    end
    
    // Register addresses (same as in the module comments)
    localparam GPIO_DATA_OUT    = 8'h00;
    localparam GPIO_DATA_IN     = 8'h04;
    localparam GPIO_DIRECTION   = 8'h08;
    localparam GPIO_PULLUP_EN   = 8'h0C;
    localparam GPIO_SET         = 8'h10;
    localparam GPIO_CLEAR       = 8'h14;
    localparam GPIO_INT_ENABLE  = 8'h18;
    localparam GPIO_INT_TYPE    = 8'h1C;
    localparam GPIO_INT_POLARITY = 8'h20;
    localparam GPIO_INT_STATUS  = 8'h24;
    localparam GPIO_PIN_MUX     = 8'h28;
    
    // Test tasks
    task reset_system;
        begin
            $display("=== RESET SYSTEM ===");
            rst_n = 0;
            MemWrite = 0;
            Mem_Addr = 8'h00;
            Mem_WrData = 32'h00000000;
            gpio_pins_drive = 16'h0000;
            gpio_pins_oe = 16'h0000;
            uart_tx_out = 1'b0;
            uart_tx_en = 1'b0;
            uart_rx_en = 1'b0;
            spi_mosi_out = 1'b0;
            spi_sck_out = 1'b0;
            spi_ss_out = 1'b0;
            spi_mosi_en = 1'b0;
            spi_sck_en = 1'b0;
            spi_ss_en = 1'b0;
            spi_miso_en = 1'b0;
            i2c_sda_en = 1'b0;
            i2c_scl_en = 1'b0;
            @(posedge clk);
            @(posedge clk);
            rst_n = 1;
            @(posedge clk);
            @(posedge clk);
        end
    endtask
    
    task write_register(input [7:0] addr, input [31:0] data);
        begin
            @(posedge clk);
            Mem_Addr = addr;
            Mem_WrData = data;
            MemWrite = 1;
            @(posedge clk);
            MemWrite = 0;
            @(posedge clk);
        end
    endtask
    
    task read_register(input [7:0] addr, output [31:0] data);
        begin
            @(posedge clk);
            Mem_Addr = addr;
            MemWrite = 0;
            @(posedge clk);
            data = Mem_RdData;
            @(posedge clk);
        end
    endtask
    
    task check_value(input [31:0] expected, input [31:0] actual, input [200:0] test_name);
        begin
            if (expected !== actual) begin
                $display("ERROR: %s - Expected: 0x%08h, Got: 0x%08h", test_name, expected, actual);
                error_count = error_count + 1;
            end else begin
                $display("PASS: %s - Value: 0x%08h", test_name, actual);
            end
        end
    endtask
    
    // Test basic GPIO functionality
    task test_basic_gpio;
        reg [31:0] read_data;
        begin
            $display("\n=== TEST: Basic GPIO Operations ===");
            
            // Test 1: Configure pins 0-7 as outputs, 8-15 as inputs
            write_register(GPIO_DIRECTION, 32'h000000FF);
            read_register(GPIO_DIRECTION, read_data);
            check_value(32'h000000FF, read_data, "GPIO Direction Register");
            
            // Test 2: Write to output pins
            write_register(GPIO_DATA_OUT, 32'h000000AA);
            read_register(GPIO_DATA_OUT, read_data);
            check_value(32'h000000AA, read_data, "GPIO Data Out Register");
            
            // Test 3: Check that outputs appear on pins
            @(posedge clk);
            @(posedge clk);
            if (gpio_pins[7:0] !== 8'hAA) begin
                $display("ERROR: GPIO output pins - Expected: 0xAA, Got: 0x%02h", gpio_pins[7:0]);
                error_count = error_count + 1;
            end else begin
                $display("PASS: GPIO output pins showing correct values");
            end
            
            // Test 4: Drive input pins and read them
            gpio_pins_drive[15:8] = 8'h55;
            gpio_pins_oe[15:8] = 8'hFF;
            @(posedge clk);
            @(posedge clk);
            read_register(GPIO_DATA_IN, read_data);
            if (read_data[15:8] !== 8'h55) begin
                $display("ERROR: GPIO input read - Expected: 0x55, Got: 0x%02h", read_data[15:8]);
                error_count = error_count + 1;
            end else begin
                $display("PASS: GPIO input pins read correctly");
            end
        end
    endtask
    
    // Test atomic bit operations
    task test_atomic_operations;
        reg [31:0] read_data;
        begin
            $display("\n=== TEST: Atomic Bit Operations ===");
            
            // Initialize data_out to known value
            write_register(GPIO_DATA_OUT, 32'h000000F0);
            
            // Test SET operation
            write_register(GPIO_SET, 32'h00000005);  // Set bits 0 and 2
            @(posedge clk);
            @(posedge clk);
            read_register(GPIO_DATA_OUT, read_data);
            check_value(32'h000000F5, read_data, "Atomic SET operation");
            
            // Test CLEAR operation
            write_register(GPIO_CLEAR, 32'h00000030); // Clear bits 4 and 5
            @(posedge clk);
            @(posedge clk);
            read_register(GPIO_DATA_OUT, read_data);
            check_value(32'h000000C5, read_data, "Atomic CLEAR operation");
            
            // Test that SET and CLEAR registers read as 0
            read_register(GPIO_SET, read_data);
            check_value(32'h00000000, read_data, "SET register reads as 0");
            
            read_register(GPIO_CLEAR, read_data);
            check_value(32'h00000000, read_data, "CLEAR register reads as 0");
        end
    endtask
    
    // Test interrupt functionality
    task test_interrupts;
        reg [31:0] read_data;
        begin
            $display("\n=== TEST: Interrupt Functionality ===");
            
            // Configure pin 8 for rising edge interrupt
            write_register(GPIO_DIRECTION, 32'h000000FF); // Pin 8 as input
            write_register(GPIO_INT_TYPE, 32'h00000100);     // Pin 8 edge triggered
            write_register(GPIO_INT_POLARITY, 32'h00000100); // Pin 8 rising edge
            write_register(GPIO_INT_ENABLE, 32'h00000100);   // Enable pin 8 interrupt
            
            // Clear any pending interrupts
            write_register(GPIO_INT_STATUS, 32'h0000FFFF);
            
            // Check interrupt is initially cleared
            if (gpio_interrupt !== 1'b0) begin
                $display("ERROR: Interrupt should be low initially");
                error_count = error_count + 1;
            end
            
            // Generate rising edge on pin 8
            gpio_pins_drive[8] = 1'b0;
            gpio_pins_oe[8] = 1'b1;
            @(posedge clk);
            @(posedge clk);
            gpio_pins_drive[8] = 1'b1;  // Create rising edge
            @(posedge clk);
            @(posedge clk);
            
            // Check interrupt status
            read_register(GPIO_INT_STATUS, read_data);
            if (read_data[8] !== 1'b1) begin
                $display("ERROR: Interrupt status bit 8 not set - Got: 0x%08h", read_data);
                error_count = error_count + 1;
            end else begin
                $display("PASS: Interrupt status bit 8 set correctly");
            end
            
            // Check global interrupt signal
            if (gpio_interrupt !== 1'b1) begin
                $display("ERROR: Global interrupt signal should be high");
                error_count = error_count + 1;
            end else begin
                $display("PASS: Global interrupt signal asserted");
            end
            
            // Clear interrupt by writing 1 to status bit
            write_register(GPIO_INT_STATUS, 32'h00000100);
            @(posedge clk);
            read_register(GPIO_INT_STATUS, read_data);
            check_value(32'h00000000, read_data, "Interrupt cleared");
            
            // Check global interrupt is now low
            if (gpio_interrupt !== 1'b0) begin
                $display("ERROR: Global interrupt should be low after clear");
                error_count = error_count + 1;
            end else begin
                $display("PASS: Global interrupt cleared");
            end
        end
    endtask
    
    // Test pin multiplexing
    task test_pin_multiplexing;
        reg [31:0] read_data;
        begin
            $display("\n=== TEST: Pin Multiplexing ===");
            
            // Test UART multiplexing on pins 0,1
            write_register(GPIO_PIN_MUX, 32'h00000005); // Pin 0,1 = UART (01 binary)
            read_register(GPIO_PIN_MUX, read_data);
            check_value(32'h00000005, read_data, "UART Pin Mux Setting");
            
            // Enable UART TX on pin 0
            uart_tx_out = 1'b1;
            uart_tx_en = 1'b1;
            @(posedge clk);
            @(posedge clk);
            
            if (gpio_pins[0] !== 1'b1) begin
                $display("ERROR: UART TX not appearing on pin 0");
                error_count = error_count + 1;
            end else begin
                $display("PASS: UART TX correctly routed to pin 0");
            end
            
            // Test UART RX on pin 1
            gpio_pins_drive[1] = 1'b0;
            gpio_pins_oe[1] = 1'b1;
            @(posedge clk);
            @(posedge clk);
            
            if (uart_rx_in !== 1'b0) begin
                $display("ERROR: UART RX not reading from pin 1");
                error_count = error_count + 1;
            end else begin
                $display("PASS: UART RX correctly reading from pin 1");
            end
            
            // Test SPI multiplexing on pins 2,3,4,5
            write_register(GPIO_PIN_MUX, 32'h00000AA0); // Pins 2,3,4,5 = SPI (10 binary)
            
            spi_mosi_out = 1'b1;
            spi_sck_out = 1'b1;
            spi_ss_out = 1'b0;
            spi_mosi_en = 1'b1;
            spi_sck_en = 1'b1;
            spi_ss_en = 1'b1;
            @(posedge clk);
            @(posedge clk);
            
            if (gpio_pins[2] !== 1'b1 || gpio_pins[3] !== 1'b1 || gpio_pins[4] !== 1'b0) begin
                $display("ERROR: SPI signals not correctly routed");
                error_count = error_count + 1;
            end else begin
                $display("PASS: SPI signals correctly routed");
            end
            
            // Test SPI MISO input
            gpio_pins_drive[5] = 1'b1;
            gpio_pins_oe[5] = 1'b1;
            @(posedge clk);
            @(posedge clk);
            
            if (spi_miso_in !== 1'b1) begin
                $display("ERROR: SPI MISO not reading correctly");
                error_count = error_count + 1;
            end else begin
                $display("PASS: SPI MISO reading correctly");
            end
        end
    endtask
    
    // Test pull-up functionality (behavioral)
    task test_pullups;
        reg [31:0] read_data;
        begin
            $display("\n=== TEST: Pull-up Configuration ===");
            
            // Enable pull-ups on pins 10-12
            write_register(GPIO_PULLUP_EN, 32'h00001C00);
            read_register(GPIO_PULLUP_EN, read_data);
            check_value(32'h00001C00, read_data, "Pull-up Enable Register");
            
            $display("PASS: Pull-up configuration stored (behavioral test only)");
        end
    endtask
    
    // Main test sequence
    initial begin
        $display("Starting Advanced GPIO Controller Testbench");
        $display("=============================================");
        
        // Initialize variables
        test_case = 0;
        error_count = 0;
        
        // Run all tests
        reset_system();
        test_basic_gpio();
        test_atomic_operations();
        test_interrupts();
        test_pin_multiplexing();
        test_pullups();
        
        // Final results
        $display("\n=============================================");
        $display("TEST SUMMARY:");
        $display("=============================================");
        if (error_count == 0) begin
            $display("ALL TESTS PASSED! ✓");
        end else begin
            $display("TESTS FAILED: %0d errors found ✗", error_count);
        end
        $display("=============================================");
        
        #100;
        $finish;
    end
    
    // Monitor key signals
    initial begin
        $monitor("Time=%0t | gpio_interrupt=%b | gpio_pins[7:0]=0x%02h | gpio_pins[15:8]=0x%02h", 
                 $time, gpio_interrupt, gpio_pins[7:0], gpio_pins[15:8]);
    end
    
    // Optional: Generate VCD file for waveform viewing
    initial begin
        $dumpfile("gpio_controller_tb.vcd");
        $dumpvars(0, tb_gpio_controller_16pin);
    end

endmodule
