`timescale 1ns / 1ps

module gpio_controller_16pin (
    input wire clk,
    input wire rst_n,
    
    // 16-pin GPIO interface
    inout wire [15:0] gpio_pins,
    
    // Memory-mapped register interface
    input wire [7:0] Mem_Addr,
    input wire [31:0] Mem_WrData,
    output reg [31:0] Mem_RdData,
    input wire MemWrite,
    
    // Interrupt output
    output wire gpio_interrupt,
    
    // Alternate function interfaces
    // UART
    input wire uart_tx_out,
    output wire uart_rx_in,
    input wire uart_tx_en,
    input wire uart_rx_en,
    
    // SPI
    input wire spi_mosi_out,
    input wire spi_sck_out,
    input wire spi_ss_out,
    output wire spi_miso_in,
    input wire spi_mosi_en,
    input wire spi_sck_en,
    input wire spi_ss_en,
    input wire spi_miso_en,
    
    // I2C
    inout wire i2c_sda,
    inout wire i2c_scl,
    input wire i2c_sda_en,
    input wire i2c_scl_en
);

    // Core GPIO registers 
    reg [15:0] gpio_data_out;
    reg [15:0] gpio_data_in;
    reg [15:0] gpio_direction;
    reg [15:0] gpio_pullup_en;
    
    // Atomic bit manipulation registers
    reg [15:0] gpio_set;       // Write 1 to set bits (auto-clear)
    reg [15:0] gpio_clear;     // Write 1 to clear bits (auto-clear)
    
    // Interrupt control registers
    reg [15:0] gpio_int_enable;    // Interrupt enable per pin
    reg [15:0] gpio_int_type;      // 0=level, 1=edge
    reg [15:0] gpio_int_polarity;  // 0=low/falling, 1=high/rising
    reg [15:0] gpio_int_status;    // Interrupt status (W1C - write 1 to clear)
    
    // Pin multiplexing control (2 bits per pin = 32 bits total)
    reg [31:0] gpio_pin_mux;       // 00=GPIO, 01=UART, 10=SPI, 11=I2C
    
    // Internal signals
    reg [15:0] gpio_data_in_prev;  // For edge detection
    wire [15:0] gpio_int_pending;
    wire [15:0] pin_mux_out;       // Output from pin mux
    wire [15:0] pin_mux_oe;        // Output enable from pin mux
    reg [15:0] gpio_set_pulse;     // One-cycle pulse for SET
    reg [15:0] gpio_clear_pulse;   // One-cycle pulse for CLEAR
    
    // Pin multiplexing logic
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : pin_mux_control
            wire [1:0] mux_sel = gpio_pin_mux[2*i+1:2*i];
            
            // Output multiplexer
            assign pin_mux_out[i] = 
                (mux_sel == 2'b00) ? gpio_data_out[i] :          // GPIO
                (mux_sel == 2'b01) ? (                           // UART
                    (i == 0) ? uart_tx_out :                     // Pin 0 = UART TX
                    (i == 1) ? 1'b1 :                           // Pin 1 = UART RX (input only)
                    gpio_data_out[i]                            // Other pins default to GPIO
                ) :
                (mux_sel == 2'b10) ? (                           // SPI  
                    (i == 2) ? spi_mosi_out :                   // Pin 2 = SPI MOSI
                    (i == 3) ? spi_sck_out :                    // Pin 3 = SPI SCK
                    (i == 4) ? spi_ss_out :                     // Pin 4 = SPI SS
                    (i == 5) ? 1'b1 :                           // Pin 5 = SPI MISO (input only)
                    gpio_data_out[i]                            // Other pins default to GPIO
                ) :
                (mux_sel == 2'b11) ? (                           // I2C
                    (i == 6) ? 1'bz :                           // Pin 6 = I2C SDA (bidirectional)
                    (i == 7) ? 1'bz :                           // Pin 7 = I2C SCL (bidirectional)
                    gpio_data_out[i]                            // Other pins default to GPIO
                ) : gpio_data_out[i];
            
            // Output enable multiplexer
            assign pin_mux_oe[i] = 
                (mux_sel == 2'b00) ? gpio_direction[i] :         // GPIO
                (mux_sel == 2'b01) ? (                           // UART
                    (i == 0) ? uart_tx_en :                      // Pin 0 = UART TX
                    (i == 1) ? 1'b0 :                           // Pin 1 = UART RX (always input)
                    gpio_direction[i]                           // Other pins use GPIO direction
                ) :
                (mux_sel == 2'b10) ? (                           // SPI
                    (i == 2) ? spi_mosi_en :                    // Pin 2 = SPI MOSI
                    (i == 3) ? spi_sck_en :                     // Pin 3 = SPI SCK
                    (i == 4) ? spi_ss_en :                      // Pin 4 = SPI SS
                    (i == 5) ? 1'b0 :                           // Pin 5 = SPI MISO (always input)
                    gpio_direction[i]                           // Other pins use GPIO direction
                ) :
                (mux_sel == 2'b11) ? (                           // I2C (open-drain, controlled externally)
                    (i == 6) ? i2c_sda_en :                     // Pin 6 = I2C SDA
                    (i == 7) ? i2c_scl_en :                     // Pin 7 = I2C SCL
                    gpio_direction[i]                           // Other pins use GPIO direction
                ) : gpio_direction[i];
            
            // Tri-state pin control
            assign gpio_pins[i] = pin_mux_oe[i] ? pin_mux_out[i] : 1'bz;
        end
    endgenerate
    
    // Route alternate function inputs
    assign uart_rx_in = (gpio_pin_mux[3:2] == 2'b01) ? gpio_pins[1] : 1'b1;  // Pin 1 = UART RX
    assign spi_miso_in = (gpio_pin_mux[11:10] == 2'b10) ? gpio_pins[5] : 1'b0; // Pin 5 = SPI MISO
    
    // I2C bidirectional handling
    assign i2c_sda = (gpio_pin_mux[13:12] == 2'b11) ? gpio_pins[6] : 1'bz;     // Pin 6 = I2C SDA
    assign i2c_scl = (gpio_pin_mux[15:14] == 2'b11) ? gpio_pins[7] : 1'bz;     // Pin 7 = I2C SCL
    
    // Interrupt generation logic
    generate
        for (i = 0; i < 16; i = i + 1) begin : interrupt_gen
            assign gpio_int_pending[i] = gpio_int_enable[i] & (
                gpio_int_type[i] ? 
                    // Edge-triggered
                    (gpio_int_polarity[i] ? 
                        (~gpio_data_in_prev[i] & gpio_data_in[i]) :  // Rising edge
                        (gpio_data_in_prev[i] & ~gpio_data_in[i])    // Falling edge
                    ) :
                    // Level-triggered  
                    (gpio_int_polarity[i] ? 
                        gpio_data_in[i] :      // High level
                        ~gpio_data_in[i]       // Low level
                    )
            );
        end
    endgenerate
    
    // Global interrupt output 
    assign gpio_interrupt = |gpio_int_status;
    
    //Input sampling 
    always @(posedge clk) begin
        if (!rst_n) begin
            gpio_data_in <= 16'h0000;
            gpio_data_in_prev <= 16'h0000;
        end else begin
            gpio_data_in_prev <= gpio_data_in;
            gpio_data_in <= gpio_pins;
        end
    end
    
    // Register write operations and atomic operations
    always @(posedge clk) begin
        if (!rst_n) begin
            gpio_data_out <= 16'h0000;
            gpio_direction <= 16'h0000;
            gpio_pullup_en <= 16'h0000;
            gpio_set <= 16'h0000;
            gpio_clear <= 16'h0000;
            gpio_set_pulse <= 16'h0000;
            gpio_clear_pulse <= 16'h0000;
            gpio_int_enable <= 16'h0000;
            gpio_int_type <= 16'h0000;
            gpio_int_polarity <= 16'h0000;
            gpio_int_status <= 16'h0000;
            gpio_pin_mux <= 32'h00000000;
        end else begin
            // Clear pulse signals by default
            gpio_set_pulse <= 16'h0000;
            gpio_clear_pulse <= 16'h0000;
          
            // Updating interrupt status - set bits when interrupts occur
            gpio_int_status <= gpio_int_status | gpio_int_pending;
            
            // Register write operations
            if (MemWrite) begin
                case (Mem_Addr[7:2])
                    6'h00: begin // GPIO_DATA_OUT - normal write
                        gpio_data_out <= Mem_WrData[15:0];
                    end
                    6'h02: gpio_direction <= Mem_WrData[15:0];     // GPIO_DIRECTION
                    6'h03: gpio_pullup_en <= Mem_WrData[15:0];     // GPIO_PULLUP_EN
                    6'h04: begin // GPIO_SET - atomic set operation
                        gpio_set_pulse <= Mem_WrData[15:0];
                        gpio_data_out <= gpio_data_out | Mem_WrData[15:0];
                    end
                    6'h05: begin // GPIO_CLEAR - atomic clear operation
                        gpio_clear_pulse <= Mem_WrData[15:0];
                        gpio_data_out <= gpio_data_out & ~Mem_WrData[15:0];
                    end
                    6'h06: gpio_int_enable <= Mem_WrData[15:0];    // GPIO_INT_ENABLE
                    6'h07: gpio_int_type <= Mem_WrData[15:0];      // GPIO_INT_TYPE
                    6'h08: gpio_int_polarity <= Mem_WrData[15:0];  // GPIO_INT_POLARITY
                    6'h09: gpio_int_status <= gpio_int_status & ~Mem_WrData[15:0]; // GPIO_INT_STATUS (W1C)
                    6'h0A: gpio_pin_mux <= Mem_WrData[31:0];       // GPIO_PIN_MUX
                endcase
            end
            
            // Auto-clear atomic operation registers (they should always read as 0)
            gpio_set <= 16'h0000;
            gpio_clear <= 16'h0000;
        end
    end
    
    // Read operations 
    always @(*) begin
        case (Mem_Addr[7:2])
            6'h00: Mem_RdData = {16'h0, gpio_data_out};      // GPIO_DATA_OUT
            6'h01: Mem_RdData = {16'h0, gpio_data_in};       // GPIO_DATA_IN
            6'h02: Mem_RdData = {16'h0, gpio_direction};     // GPIO_DIRECTION
            6'h03: Mem_RdData = {16'h0, gpio_pullup_en};     // GPIO_PULLUP_EN
            6'h04: Mem_RdData = 32'h0;                       // GPIO_SET (always reads 0)
            6'h05: Mem_RdData = 32'h0;                       // GPIO_CLEAR (always reads 0)
            6'h06: Mem_RdData = {16'h0, gpio_int_enable};    // GPIO_INT_ENABLE
            6'h07: Mem_RdData = {16'h0, gpio_int_type};      // GPIO_INT_TYPE
            6'h08: Mem_RdData = {16'h0, gpio_int_polarity};  // GPIO_INT_POLARITY
            6'h09: Mem_RdData = {16'h0, gpio_int_status};    // GPIO_INT_STATUS
            6'h0A: Mem_RdData = gpio_pin_mux;                // GPIO_PIN_MUX
            default: Mem_RdData = 32'h00000000;
        endcase
    end

endmodule
