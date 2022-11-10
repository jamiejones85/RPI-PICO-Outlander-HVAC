#include "hvac.h"
#include "can.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

#include "ssd1306.h"

// Define pins for SPI (to CAN)
#define SPI_PORT spi0
#define SPI_MISO 16
#define SPI_CS 17
#define SPI_CLK 18
#define SPI_MOSI 19
#define CAN_INT 20
#define CAN_CLK 21  // 8MHz clock for CAN
#define CAN_SLEEP 22


// GPIO pins
#define IN1 14
#define IN2 13
#define IN3 12
#define IN4 10
#define OUT1 9
#define OUT2 6
#define OUT3 4
#define OUT4 2

#define MAXTEMP 85
#define MINTEMP 40

uint16_t data_timer;
bool heating = false;
bool hvPresent = false;
bool ledOn = false;
int16_t currentTemperature = 0;
int16_t desiredTemperature = 0;
bool enabled = false;
bool contactorsClosed = false;
bool heaterComms = false;
uint16_t data_timer;

uint16_t power = 0;
char inverterStatus = 0x0;
ssd1306_t disp;
uint16_t result;

void setup_gpios(void) {
    i2c_init(i2c0, 400000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);
}

void SPI_configure() {
  spi_init(SPI_PORT, 1000000);
  spi_set_format(SPI_PORT, 8, 0, 0, SPI_MSB_FIRST);
  gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
  gpio_init(SPI_CS);
  gpio_set_dir(SPI_CS, GPIO_OUT);
  gpio_put(SPI_CS, 1);
}


void CAN_reg_write(uint8_t reg, uint8_t val) {
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_WRITE, reg, val}, 3);
  gpio_put(SPI_CS, 1);
}

void CAN_reg_modify(uint8_t reg, uint8_t mask, uint8_t val) {
  gpio_put(SPI_CS, 0);
  busy_wait_us(2);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_MODIFY, reg, mask, val}, 4);
  busy_wait_us(2);
  gpio_put(SPI_CS, 1);
}

uint8_t CAN_reg_read(uint8_t reg) {
  uint8_t data;
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_READ, reg}, 2);
  spi_read_blocking(SPI_PORT, 0, &data, 1);
  gpio_put(SPI_CS, 1);
  return (data);
}

void CAN_receive_buf(uint8_t n) {
  uint8_t rtr = (CAN_reg_read(REG_RXBnSIDL(n)) & FLAG_SRR) ? true : false;

  if (!rtr) {
    uint8_t received_data[8];
    for (int i = 0; i < 8; i++)
      received_data[i] = CAN_reg_read(REG_RXBnD0(n) + i);
    if (!n) {
      // 0x398
      //Heater status
      if (received_data[5] == 0x00) {
        heating = false;
        power = 0;
      } else if (received_data[5] > 0) {
        heating = true;
      }
      //hv status
      if (received_data[6] == 0x09) {
        hvPresent = false;
      } else if (received_data[6] == 0x00) {
        hvPresent = true;
      }

      //temperatures
      unsigned int temp1 = received_data[3] - 40;
      unsigned int temp2 = received_data[4] - 40;
      if (temp2 > temp1) {
        currentTemperature = temp2;
      } else {
        currentTemperature = temp1;
      }
      data_timer = 0;

    } else {
      // 0x02
      inverterStatus = received_data[0];
    }
  }
  CAN_reg_modify(REG_CANINTF, FLAG_RXnIF(n), 0x00);

}

void CAN_receive() {
  uint8_t intf = CAN_reg_read(REG_CANINTF);
  if (intf & FLAG_RXnIF(0))
    CAN_receive_buf(0);
  if (intf & FLAG_RXnIF(1))
    CAN_receive_buf(1);
}


void CAN_transmit(uint8_t ext, uint32_t id, uint8_t* data, uint8_t length) {
  if (ext) {
    CAN_reg_write(REG_TXBnEID0(0), id);
    CAN_reg_write(REG_TXBnEID8(0), id >> 8);
    CAN_reg_write(REG_TXBnSIDL(0),
                  ((id >> 16) & 3) | FLAG_EXIDE | (((id >> 18) & 7) << 5));
    CAN_reg_write(REG_TXBnSIDH(0), id >> 21);
  } else {
    CAN_reg_write(REG_TXBnSIDH(0), id >> 3);  // Set CAN ID
    CAN_reg_write(REG_TXBnSIDL(0), id << 5);  // Set CAN ID
    CAN_reg_write(REG_TXBnEID8(0), 0x00);     // Extended ID
    CAN_reg_write(REG_TXBnEID0(0), 0x00);     // Extended ID
  }

  CAN_reg_write(REG_TXBnDLC(0), length);  // Frame length

  for (int i = 0; i < length; i++) {  // Write the frame data
    CAN_reg_write(REG_TXBnD0(0) + i, data[i]);
  }

  CAN_reg_write(REG_TXBnCTRL(0), 0x08);  // Start sending
  busy_wait_us(1000);                    // Allow up to 1ms to transmit
  CAN_reg_write(REG_TXBnCTRL(0), 0);     // Stop sending
  CAN_reg_modify(REG_CANINTF, FLAG_TXnIF(0), 0x00);  // Clear interrupt flag
}

void CAN_reset() {
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_RESET}, 1);
  gpio_put(SPI_CS, 1);
  busy_wait_us(100);
}

void CAN_configure() {
  // Configure speed to 500kbps based on 8MHz Crystal
  // Magic constants from
  // https://github.com/sandeepmistry/arduino-CAN/blob/master/src/MCP2515.cpp
  CAN_reg_write(REG_CNF1, 0x00);
  CAN_reg_write(REG_CNF2, 0x90);
  CAN_reg_write(REG_CNF3, 0x02);

  // Enable Filters
  CAN_reg_write(REG_RXBnCTRL(0), 0);  // Enable filters, no rollover
  CAN_reg_write(REG_RXBnCTRL(1), 0);
  // Set masks RXM0 and RXM1 to exact match (0x7FF)
  for (int n = 0; n < 2; n++) {
    uint32_t mask = 0x7FF;
    CAN_reg_write(REG_RXMnSIDH(n), mask >> 3);
    CAN_reg_write(REG_RXMnSIDL(n), mask << 5);
    CAN_reg_write(REG_RXMnEID8(n), 0);
    CAN_reg_write(REG_RXMnEID0(n), 0);
  }
  // Set up filters RXF0 and RFX2 to match 2 addresses
  uint32_t addr = 0x398;
  CAN_reg_write(REG_RXFnSIDH(0), addr >> 3);
  CAN_reg_write(REG_RXFnSIDL(0), addr << 5);
  CAN_reg_write(REG_RXFnEID8(0), 0);
  CAN_reg_write(REG_RXFnEID0(0), 0);

  addr = 0x02;
  CAN_reg_write(REG_RXFnSIDH(2), addr >> 3);
  CAN_reg_write(REG_RXFnSIDL(2), addr << 5);
  CAN_reg_write(REG_RXFnEID8(2), 0);
  CAN_reg_write(REG_RXFnEID0(3), 0);

  // Enable receive interrupts
  // CAN_reg_write(REG_CANINTE, 3);

  // Set normal operation mode
  CAN_reg_write(REG_CANCTRL, MODE_NORMAL);
}

bool ms_10_callback(struct repeating_timer *t) {
    CAN_transmit(0, 0x285, (uint8_t[]){0x00, 0x00, 0x14, 0x21, 0x90, 0xFE, 0x0C, 0x10}, 8);

    if (enabled && contactorsClosed  && currentTemperature < desiredTemperature) {
        if (currentTemperature < desiredTemperature - 5) {
            power = 2;
            CAN_transmit(0, 0x188, (uint8_t[]){0x03, 0x50, 0xA2, 0x4D, 0x00, 0x00, 0x00, 0x00}, 8);
        } else {
            power = 1;
            CAN_transmit(0, 0x188, (uint8_t[]){0x03, 0x50, 0x32, 0x4D, 0x00, 0x00, 0x00, 0x00}, 8);
        }

    }
    CAN_receive();
    data_timer++;
    if (data_timer > 500) {
        heaterComms = false;
    } else {
        heaterComms = true;
    }
    return true;
}

void show_off() {
    ssd1306_clear(&disp);
    ssd1306_draw_string(&disp, 38, 10, 3, "OFF");
    ssd1306_show(&disp);
}

void show_nocomms() {
    ssd1306_clear(&disp);
    ssd1306_draw_string(&disp, 8, 10, 2, "No Comms");
    ssd1306_show(&disp);
}

void show_precharge() {
    ssd1306_clear(&disp);
    ssd1306_draw_string(&disp, 8, 10, 2, "Pre Char");
    ssd1306_show(&disp);
}

void show_temperature() {
    ssd1306_clear(&disp);

    if (power == 2) {
      ssd1306_draw_square(&disp, 0, 0, 10, 32); // full power
    } else if (power == 1) {
      ssd1306_draw_square(&disp, 0, 16, 10, 16);
    }
    char buffOut[5];
    sprintf(buffOut, "%d/%d", desiredTemperature, currentTemperature);
    ssd1306_draw_string(&disp, 17, 10, 3, buffOut);

    ssd1306_show(&disp);
}

bool ms_1000_callback(struct repeating_timer *t) {
    gpio_put(PICO_DEFAULT_LED_PIN, ledOn);
    ledOn = !ledOn;

    result = adc_read();

    if (result > 500) {
        enabled = true;
        int input_range = 4095 - 500;
        int output_range = MAXTEMP - MINTEMP;
        desiredTemperature = (result - 500)*output_range / input_range + MINTEMP;

    } else {
        enabled = false;
    }

    contactorsClosed = inverterStatus > 0x00;
    if (!contactorsClosed) {
        enabled = false;
    }

    if (inverterStatus == 0) {
        show_precharge();
    } else if (!heaterComms) {
        show_nocomms();
    } else if (enabled) {
        show_temperature();
    } else {
        show_off();
    }

    return true;
}

bool ms_50_callback(struct repeating_timer *t) {
    return true;
}

int main() {
  // Set system clock to 80MHz
  set_sys_clock_khz(80000, true);
  stdio_init_all();

  busy_wait_ms(1000);
  printf("Startup\n");

  // Configure CAN transceiver sleep line
  gpio_init(CAN_SLEEP);
  gpio_set_dir(CAN_SLEEP, GPIO_OUT);
  gpio_put(CAN_SLEEP, 0);  // Logic low to wake transceiver

  // Output 8MHz square wave on CAN_CLK pin
  clock_gpio_init(CAN_CLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 10);

   // Configure SPI to communicate with CAN
  SPI_configure();
  // Set up CAN to receive messages
  CAN_reset();
  CAN_configure();

  setup_gpios();

  struct repeating_timer timer10;
  add_repeating_timer_ms(10, ms_10_callback, NULL, &timer10);

  // struct repeating_timer timer50;
  // add_repeating_timer_ms(50, ms_50_callback, NULL, &timer50);

  struct repeating_timer timer1000;
  add_repeating_timer_ms(1000, ms_1000_callback, NULL, &timer1000);

  disp.external_vcc=false;
  ssd1306_init(&disp, 128, 32, 0x3C, i2c0);
  ssd1306_clear(&disp);


  while (1) {

  }
}