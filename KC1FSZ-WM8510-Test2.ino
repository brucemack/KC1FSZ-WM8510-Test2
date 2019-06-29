// Test Sketch for WM8510
// Bruce MacKinnon KC1FSZ 29-June-2019
//
#define PIN_WM8510_SCLK 3
#define PIN_WM8510_SDIN 4
#define PIN_WM8510_CSB 5

#define I2S_MCLK_FRACT 9
#define I2S_MCLK_DIVIDE 423

// WM8510 Control masks
#define WM8510_03_SPKNEN      (0b001000000)
#define WM8510_03_SPKPEN      (0b000100000)
#define WM8510_03_SPKMIXEN    (0b000000100)
#define WM8510_03_DACEN       (0b000000001)

#define WM8510_04_WL_00       (0b000000000)   // 16 bit
#define WM8510_04_FMT_10      (0b000010000)   // I2S 

#define WM8510_06_CLKSEL      (0b100000000)   // PLL

#define WM8510_2C_MVBSEL      (0b100000000)
#define WM8510_2C_MICPNINPPGA (0b000000010)
#define WM8510_2C_MICP2INPPGA (0b000000001)

void strobePin(int pin) {
  digitalWrite(pin,1);
  digitalWrite(pin,0);
}

void writeWM8510Bit(unsigned int v) {
  digitalWrite(PIN_WM8510_SDIN,(v != 0) ? 1 : 0);
  strobePin(PIN_WM8510_SCLK);
}

void writeWM8510Word(unsigned int v,unsigned int wordSize) {
  unsigned int mask = 1 << (wordSize - 1);
  for (unsigned int i = 0; i < wordSize; i++) {
    // Bitwise operation.  Looking for a non-zero
    writeWM8510Bit(v & mask);
    v = v << 1; 
  }
}

// The WM8510 has registers that are addressed using 7 bits.  Data 
// length is 9 bits.
void writeWM8510Register(unsigned int addr,unsigned int data) {
  digitalWrite(PIN_WM8510_CSB,0);
  writeWM8510Word(addr,7);
  writeWM8510Word(data,9);  
  digitalWrite(PIN_WM8510_CSB,1);
}

void setup() {

  pinMode(13,OUTPUT);

  // Flash hello
  digitalWrite(13,1);
  delay(500);
  digitalWrite(13,0);
  delay(500);
  digitalWrite(13,1);
  delay(500);
  digitalWrite(13,0);
  delay(500);

  // Setup the WM8510 control pins
  pinMode(PIN_WM8510_SCLK,OUTPUT);
  pinMode(PIN_WM8510_SDIN,OUTPUT);
  pinMode(PIN_WM8510_CSB,OUTPUT);
  digitalWrite(PIN_WM8510_SCLK,0);
  digitalWrite(PIN_WM8510_SDIN,0);
  // Normally high
  digitalWrite(PIN_WM8510_CSB,1);

  // I2S Configuration
  // Configure the Teensy 3.2 pins per the reference and wiring
  PORTC_PCR3 = PORT_PCR_MUX(6); // Alt 6 is BLCK - T3.2 pin 9
  PORTC_PCR6 = PORT_PCR_MUX(6); // Alt 6 is MCLK - T3.2 pin 11
  PORTC_PCR1 = PORT_PCR_MUX(6); // Alt 6 is TXD0 - T3.2 pin 22
  PORTC_PCR2 = PORT_PCR_MUX(6); // Alt 6 is LRCLK - T3.2 pin 23
  //PORTC_PCR5 = PORT_PCR_MUX(4); // Alt 4 is RXD0 - T3.2 pin 13
  // System gating control register.  I2S clock gate control is enabled.
  SIM_SCGC6 |= SIM_SCGC6_I2S;
  // The MCLK is sourced from the system clock | The MCLK is enabled
  I2S0_MCR = I2S_MCR_MICS(0) | I2S_MCR_MOE;
  // Divide down the system clock to get the MCLK
  I2S0_MDR = I2S_MDR_FRACT(I2S_MCLK_FRACT) | I2S_MDR_DIVIDE(I2S_MCLK_DIVIDE);
  // No mask - we are sending both channels of audio
  I2S0_TMR = 0;
  // Set the high water mark for the FIFO.  This determines when the interupt
  // needs to fire.
  I2S0_TCR1 = I2S_TCR1_TFW(1);
  // Asynchronous mode | Bit Clock Active Low | Master Clock 1 Selected |
  // Bit Clock generated internally (master) | Bit Clock Divide
  // Setting DIV=3 means (3+1)*2=8 division of MCLK
  I2S0_TCR2 = I2S_TCR2_SYNC(0) | I2S_TCR2_BCP | I2S_TCR2_MSEL(1) |
     I2S_TCR2_BCD | I2S_TCR2_DIV(3); 
  // Transmit channel 0 is enabled | Start of word flag = 0
  I2S0_TCR3 = I2S_TCR3_TCE | I2S_TCR3_WDFL(0);
  // Frame size = 2 | Sync width = 16 bit clocks | MSB first |
  // Frame sync asserts one bit before the first bit of the frame |
  // Frame sync is active low | Frame sync is generated internally
  I2S0_TCR4 = I2S_TCR4_FRSZ(1) | I2S_TCR4_SYWD(15) | I2S_TCR4_MF |
    I2S_TCR4_FSE | 
    I2S_TCR4_FSP | I2S_TCR4_FSD;
  // Word N Width = 16 | Word 0 Width = 16 | First Bit Shifted = 16
  I2S0_TCR5 = I2S_TCR5_WNW(15) | I2S_TCR5_W0W(15) | I2S_TCR5_FBT(15);
  // Software reset
  I2S0_TCSR = I2S_TCSR_SR;
  // Transmit enabled | Bit clock enabled | FIFO request interrupt enable
  I2S0_TCSR = I2S_TCSR_TE | I2S_TCSR_BCE | I2S_TCSR_FRIE;
  // Enable the system-level interrupt for I2S
  NVIC_ENABLE_IRQ(IRQ_I2S0_TX);

  // WN8510 Power on sequence
  // Wait for supply voltage to settle
  delay(50);
  // Set MICBEN=1, BIASEN=1, VMIDEL[1:0]
  writeWM8510Register(0x01,0b000011011);
  // Wait for VMID to settle
  delay(2000);
  // Speaker output enabled P/N | speaker mixer | DAC enabled 
  writeWM8510Register(0x03,
    WM8510_03_SPKNEN | WM8510_03_SPKPEN | WM8510_03_SPKMIXEN | WM8510_03_DACEN);
  // Word Length 16 | I2S format 
  writeWM8510Register(0x04,
    WM8510_04_WL_00 | WM8510_04_FMT_10); 
  // Clock source MCLK | scaling factor 1
  writeWM8510Register(0x06,0);
  // Sample rate to 8kHz
  writeWM8510Register(0x07,0b000001010);
  // Set microphone bias level and hook up MICN and MICP
  writeWM8510Register(0x2c,
    WM8510_2C_MVBSEL | WM8510_2C_MICPNINPPGA | WM8510_2C_MICP2INPPGA);
}

void loop() {
  /*
  // Toggle MICBIAS level
  writeWM8510Register(0x2c,0b000000011);
  delay(2000);
  writeWM8510Register(0x2c,0b100000011);
  delay(2000);
  */
}

uint32_t frameCounter = 0;
uint32_t counter = 0;
uint32_t diag0 = 0;

/*
 * Utility function that looks at the read/write ponters on the 
 * FIFO to determine if it is completely full yet.  The FIFO
 * pointers are 3-bit values.
 */
bool isFIFOFull() {
    // Read the FIFO pointers
    uint32_t v = I2S0_TFR0;
    uint32_t wfp = (v & 0b11110000000000000000) >> 16;
    uint32_t rfp = (v & 0b1111);
    // Full happens when the pointers are the same except for the MSB.
    if ((wfp & 0b111) == (rfp & 0b111) && wfp != rfp) {
      return true;
    } else {
      return false;
    }
}

/**
 * This function writes as much as possible into the FIFO.
 */
void tryWrite() {
  // Loop
  while (!isFIFOFull()) {
    frameCounter++;
    // Data counter only advances at the start of a frame.  This
    // means that the same value is being written into both 
    // channels.
    if (frameCounter % 2 == 0) {
      counter++;
    }
    // Write a value.  This automatically advances the write pointer
    I2S0_TDR0 = counter & 0xffff;    
  }
}

// This gets called whenever the FIFO interupt is raised
void i2s0_tx_isr(void) {
  cli();
  // Replenish the FIFO as quickly as possible
  tryWrite();
  // Lower frequency flashing (approximately 1 Hz)
  if (++diag0 % 8000 > 4000) {
    digitalWriteFast(13,1);
  } else {
    digitalWriteFast(13,0);
  }
  sei();
}
