#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL   // 16 MHz
#define SCL_CLOCK 100000L  // I2C clock in Hz
// DS1621 working with 100kHz I2C clock page ... of the datasheet

// I2C operation types
#define I2C_WRITE 0
#define I2C_READ 1

// DS1621
#define DS1621_ADDRESS 0x48  // I2C address of the DS1621 (0x48)
// I2C Bit Rate

// MCP23017
#define MCP23017_ADDR 0x20  // I2C address of the MCP23017 (shifted left by 1 for R/W bit)
#define IODIRA 0x00         // Port A I/O direction register
#define IODIRB 0x01         // Port B I/O direction register
#define GPIOA 0x12          // Port A register

// Define bit patterns for digits 0-9 (assuming common cathode)
const uint8_t digitMap[10] = {
  0b00111111,  // 0 (segments a, b, c, d, e, f on)
  0b00000110,  // 1 (segments b, c on)
  0b01011011,  // 2 (segments a, b, d, e, g on)
  0b01001111,  // 3 (segments a, b, c, d, g on)
  0b01100110,  // 4 (segments b, c, f, g on)
  0b01101101,  // 5 (segments a, c, d, f, g on)
  0b01111101,  // 6 (segments a, c, d, e, f, g on)
  0b00000111,  // 7 (segments a, b, c on)
  0b01111111,  // 8 (all segments on)
  0b01101111   // 9 (segments a, b, c, d, f, g on)
};

// Function Prototypes
// UART functions
void uart_init(unsigned int baud);       // Initialize UART
void uart_transmit(unsigned char data);  // Transmit data via UART
void uart_print(const char *str);        // Print string via UART

// Timer functions
void init_timer0();  // Initialize Timer0
unsigned long millis();

// I2C functions
void i2c_init(void);                 // Initialize I2C
void i2c_start(void);                // Start I2C communication
void i2c_stop(void);                 // Stop I2C communication
void i2c_write_byte(uint8_t data);   // Write byte to I2C
uint8_t i2c_read_byte(uint8_t ack);  // Read byte from I2C

void i2c_write_byte_lcd(uint8_t data);
void pulse_enable(uint8_t data);


// MCP23017 functions
void mcp23017_write(uint8_t reg, uint8_t data);
void displayDigit(uint8_t digit, uint8_t port);
void init_seven_segment(void);

// DS1621 functions
void ds1621_init(void);
int16_t get_temperature(void);

// Initialize I2C
void i2c_init(void) {
  TWSR = 0x00;  // Prescaler = 1
  TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
}

// Send I2C start condition
void i2c_start(void) {
  // TWCR: TWI Control Register
  // TWINT: TWI Interrupt Flag
  // TWSTA: TWI Start Condition Bit
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  // Wait for TWINT Flag set. This indicates that the start condition has been transmitted.
  while (!(TWCR & (1 << TWINT)))
    ;
}

// Send I2C stop condition
void i2c_stop(void) {
  // TWCR: TWI Control Register
  // TWINT: TWI Interrupt Flag
  // TWSTO: TWI Stop Condition Bit
  // TWEN: TWI Enable Bit
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

// Write a byte to I2C
void i2c_write_byte(uint8_t data) {
  // TWDR: TWI Data Register
  TWDR = data;
  // TWCR: TWI Control Register
  // TWINT: TWI Interrupt Flag
  // TWEN: TWI Enable Bit
  TWCR = (1 << TWINT) | (1 << TWEN);
  // Wait for TWINT Flag set. This indicates that the data has been transmitted, and ACK/NACK has been received.
  while (!(TWCR & (1 << TWINT)))
    ;
}

// Read a byte from I2C
uint8_t i2c_read_byte(uint8_t ack) {
  // TWCR: TWI Control Register
  // TWINT: TWI Interrupt Flag
  // TWEA: TWI Enable Acknowledge Bit
  // TWEN: TWI Enable Bit
  TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
  while (!(TWCR & (1 << TWINT)))
    ;
  return TWDR;
}

// Initialize DS1621
void ds1621_init(void) {
  i2c_start();
  i2c_write_byte((DS1621_ADDRESS << 1) | I2C_WRITE);
  i2c_write_byte(0xAC);  // Access Configuration Register
  i2c_write_byte(0x00);  // Continuous conversion
  i2c_stop();

  // Start temperature conversion
  i2c_start();
  i2c_write_byte((DS1621_ADDRESS << 1) | I2C_WRITE);
  i2c_write_byte(0xEE);  // Start conversion
  i2c_stop();
}

// Read temperature from DS1621
int16_t get_temperature(void) {
  i2c_start();
  i2c_write_byte((DS1621_ADDRESS << 1) | I2C_WRITE);
  i2c_write_byte(0xAA);  // Read temperature

  // Repeated start
  i2c_start();
  i2c_write_byte((DS1621_ADDRESS << 1) | I2C_READ);

  uint8_t t_msb = i2c_read_byte(1);  // Read MSB, send ACK
  uint8_t t_lsb = i2c_read_byte(0);  // Read LSB, send NACK
  i2c_stop();

  // Calculate temperature in tenths of a degree
  int16_t raw_t = (int8_t)t_msb << 1 | t_lsb >> 7;
  return raw_t * 10 / 2;
}

// UART Initialization
void uart_init(unsigned int baud) {
  unsigned int ubrr = F_CPU / 16 / baud - 1;
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

// UART Transmit
void uart_transmit(unsigned char data) {
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = data;
}

// UART Print String
void uart_print(const char *str) {
  while (*str) {
    uart_transmit(*str++);
  }
}

void i2c_write_byte_lcd(uint8_t data) {
  i2c_start();
  i2c_write_byte(I2C_WRITE);
  i2c_write_byte(data);
  i2c_stop();
}

void pulse_enable(uint8_t data) {
  i2c_write_byte_lcd(data);
  _delay_ms(1);  // Enable pulse must be >450ns according to datasheet HD44780U p25
  i2c_write_byte_lcd(data);
  _delay_ms(50);  // Commands need >37us to settle according to datasheet HD44780U
}

// Write to MCP23017 register
void mcp23017_write(uint8_t reg, uint8_t data) {
  i2c_start();
  i2c_write_byte(MCP23017_ADDR << 1);  // Address with write bit
  i2c_write_byte(reg);                 // Register address
  i2c_write_byte(data);                // Data
  i2c_stop();
}

// Display digit on specified port
void displayDigit(uint8_t digit, uint8_t port) {
  if (digit > 9)
    return;  // Invalid digit, ignore
  mcp23017_write(port, digitMap[digit]);
}

// Initialize the MCP23017 for 7-segment displays
void init_seven_segment(void) {
  // Configure both ports as outputs
  mcp23017_write(IODIRA, 0x00);
  _delay_ms(20);
  mcp23017_write(IODIRB, 0x00);
  _delay_ms(20);
  i2c_start();
  i2c_write_byte(MCP23017_ADDR << 1);  // Address with write bit
  i2c_write_byte(GPIOA);               // Register address
  i2c_write_byte(digitMap[0]);         // Data A
  i2c_write_byte(digitMap[0]);         // Data B
  i2c_stop();
}

// Interrupt Service Routine for Timer0 compare match
unsigned long previousMillis;
volatile int on = 0;  // shared volatile variable,
// make sure the compiler does not optimize it away

volatile unsigned long timer0_millis = 0;
unsigned long period = 60000;  // 60 second


void init_timer0() {
  // Set Timer0 to CTC mode
  TCCR0A = (1 << WGM01);

  // Set prescaler to 64
  TCCR0B = (1 << CS01) | (1 << CS00);

  // Set compare value for 1ms interrupt at 16MHz
  OCR0A = 249;

  // Enable Timer0 compare match interrupt
  TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
  timer0_millis++;
}

unsigned long millis() {
  unsigned long m;
  uint8_t oldSREG = SREG;

  // Disable interrupts while reading timer0_millis
  cli();
  m = timer0_millis;
  SREG = oldSREG;

  return m;
}

int main(void) {
  // Initialize UART for serial output
  uart_init(9600);

  // Initialize I2C
  i2c_init();

  // Initialize DS1621
  ds1621_init();

  // Configure MCP23017 ports as outputs
  init_seven_segment();

  // port D as inputs ( why? ) - all the pins in Port D are set to INPUT mode.
  DDRD = 0;
  /*
      EIMSK - External Interrupt Mask Register
      see data sheet: which interrupt is enabled?
      INT0 - 2 0x0002 INT0 External Interrupt Request 0
      Set HIGH to INT0 - Pin 2
    */
  EIMSK = (1 << INT0);
  /*
      EICRA - External Interrupt Control Register A
      see data sheet: when does the interrupt react?
      External Interrupt Control Register A (EICRA) define whether the external interrupt is activated
      on rising and/or falling edge of the INT0 pin or level sensed. Activity on the pin will cause an
      interrupt request even if INT0 is configured as an output.
    */
  EICRA = 0b00000011;
  /*
      why ?
      sei() - SEts the global interrupt flag
    */
  init_timer0();  // Add this line
  sei();

  int16_t c_temp = get_temperature();
  int16_t f_temp = c_temp * 9 / 5 + 320;

  char buffer_F[20];
  char buffer_C[20];
  char temperature[10];

  // Format Celsius
  snprintf(buffer_C, sizeof(buffer_C), "Temp: %c%02d.%1d C",
           (c_temp < 0 ? '-' : ' '), abs(c_temp / 10), abs(c_temp % 10));
  // Format F
  snprintf(buffer_F, sizeof(buffer_F), "Temp: %c%02d.%1d F",
           (f_temp < 0 ? '-' : ' '), abs(f_temp / 10), abs(f_temp % 10));

  snprintf(temperature, sizeof(temperature), "%c%02d.%1d", (c_temp < 0 ? '-' : ' '), abs(c_temp / 10), abs(c_temp % 10));

  // Print temperature
  uart_print(temperature);
  uart_print("\n");

  while (1) {

    unsigned long currentMillis = millis();
    // Get temperature

    c_temp = get_temperature();
    f_temp = c_temp * 9 / 5 + 320;

    i2c_start();
    i2c_write_byte(MCP23017_ADDR << 1);  // Address with write bit
    i2c_write_byte(GPIOA);               // Register address
    if (on) {
      i2c_write_byte(digitMap[(int)abs(f_temp / 10) - (f_temp / 100 * 10)]);  // Data A
      i2c_write_byte(digitMap[(int)abs(f_temp / 100)]);                       // Data B
    } else {
      i2c_write_byte(digitMap[(int)abs(c_temp / 10) - (c_temp / 100 * 10)]);  // Data A
      i2c_write_byte(digitMap[(int)abs(c_temp / 100)]);                       // Data B
    }
    i2c_stop();

    // Format Celsius
    snprintf(buffer_C, sizeof(buffer_C), "Temp: %c%02d.%1d C", (c_temp < 0 ? '-' : ' '), abs(c_temp / 10), abs(c_temp % 10));
    // Format F
    snprintf(buffer_F, sizeof(buffer_F), "Temp: %c%02d.%1d F", (f_temp < 0 ? '-' : ' '), abs(f_temp / 10), abs(f_temp % 10));

    snprintf(temperature, sizeof(temperature), "%c%02d.%1d", (c_temp < 0 ? '-' : ' '), abs(c_temp / 10), abs(c_temp % 10));

    if (currentMillis - previousMillis >= period) {
      // Print temperature
      uart_print(temperature);
      uart_print("\n");
      previousMillis = currentMillis;
    }
  }

  return 0;
}


ISR(INT0_vect) {
  _delay_ms(20);  // short delay to avoid bouncing
  if (on)         // toggle on
    on = 0;
  else
    on = 1;
}