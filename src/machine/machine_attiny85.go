//go:build attiny85
// +build attiny85

package machine

import (
	"device/avr"
	"runtime/volatile"
)

const (
	PB0 Pin = iota
	PB1
	PB2
	PB3
	PB4
	PB5
)

// getPortMask returns the PORTx register and mask for the pin.
func (p Pin) getPortMask() (*volatile.Register8, uint8) {
	// Very simple for the attiny85, which only has a single port.
	return avr.PORTB, 1 << uint8(p)
}

// I2C pins for ATtiny85 (directly on PORTB)
const (
	SDA_PIN Pin = PB0 // USI SDA
	SCL_PIN Pin = PB2 // USI SCL
)

// I2C on ATtiny85 using USI (Universal Serial Interface)
type I2C struct {
}

// I2C0 is the only I2C interface on ATtiny85
var I2C0 *I2C = nil

// I2CConfig is used to store config info for I2C.
type I2CConfig struct {
	Frequency uint32
}

// Configure sets up the USI for I2C (TWI) communication
func (i2c *I2C) Configure(config I2CConfig) error {
	// Enable pull-ups on SDA and SCL to set high as released state
	avr.PORTB.SetBits((1 << SDA_PIN) | (1 << SCL_PIN))

	// Enable SCL and SDA as outputs
	avr.DDRB.SetBits((1 << SDA_PIN) | (1 << SCL_PIN))

	// Preload data register with "released level" data (all 1s)
	avr.USIDR.Set(0xFF)

	// Configure USI:
	// - Disable interrupts (USISIE=0, USIOIE=0)
	// - Two-wire mode (USIWM1=1, USIWM0=0)
	// - Software clock strobe (USICS1=1, USICS0=0, USICLK=1)
	// - Don't toggle clock yet (USITC=0)
	avr.USICR.Set(avr.USICR_USIWM1 | avr.USICR_USICS1 | avr.USICR_USICLK)

	// Clear all flags and reset counter
	avr.USISR.Set(avr.USISR_USISIF | avr.USISR_USIOIF | avr.USISR_USIPF | avr.USISR_USIDC)

	return nil
}

// Tx performs a full I2C transaction.
// It sends the data in w to the device at addr, then reads len(r) bytes into r.
func (i2c *I2C) Tx(addr uint16, w, r []byte) error {
	if len(w) > 0 {
		// Send START and address with write bit
		if !i2c.start() {
			return errI2CSignalStartTimeout
		}

		// Send address with write bit (bit 0 = 0)
		if !i2c.writeByte(uint8(addr<<1) | 0x00) {
			return errI2CAckExpected
		}

		// Send data bytes
		for _, b := range w {
			if !i2c.writeByte(b) {
				return errI2CAckExpected
			}
		}
	}

	if len(r) > 0 {
		// Send (repeated) START and address with read bit
		if !i2c.start() {
			return errI2CSignalStartTimeout
		}

		// Send address with read bit (bit 0 = 1)
		if !i2c.writeByte(uint8(addr<<1) | 0x01) {
			return errI2CAckExpected
		}

		// Read data bytes
		for i := range r {
			// Send ACK for all bytes except the last one (send NACK)
			ack := i < len(r)-1
			r[i] = i2c.readByte(ack)
		}
	}

	// Send STOP condition if we did any communication
	if len(w) > 0 || len(r) > 0 {
		i2c.stop()
	}

	return nil
}

// start generates an I2C START condition
func (i2c *I2C) start() bool {
	// Release SCL to ensure that (repeated) Start can be performed
	avr.PORTB.SetBits(1 << SCL_PIN)

	// Wait for SCL to go high (with timeout for clock stretching)
	for !avr.PORTB.HasBits(1 << SCL_PIN) {
	}

	// Short delay (>4.7µs for 100kHz I2C)
	i2c.delay()

	// Generate Start Condition: pull SDA LOW while SCL is high
	avr.PORTB.ClearBits(1 << SDA_PIN)

	// Short delay (>4.0µs)
	i2c.delay()

	// Pull SCL LOW
	avr.PORTB.ClearBits(1 << SCL_PIN)

	// Release SDA (will be controlled by data transfer)
	avr.PORTB.SetBits(1 << SDA_PIN)

	// Verify start condition was detected
	return avr.USISR.HasBits(avr.USISR_USISIF)
}

// stop generates an I2C STOP condition
func (i2c *I2C) stop() bool {
	// Pull SDA low
	avr.PORTB.ClearBits(1 << SDA_PIN)

	// Release SCL
	avr.PORTB.SetBits(1 << SCL_PIN)

	// Wait for SCL to go high
	for !avr.PINB.HasBits(1 << SCL_PIN) {
	}

	// Short delay
	i2c.delay()

	// Release SDA (goes high while SCL is high = STOP condition)
	avr.PORTB.SetBits(1 << SDA_PIN)

	// Short delay
	i2c.delay()

	// Verify stop condition was detected
	return avr.USISR.HasBits(avr.USISR_USIPF)
}

// writeByte sends a byte and returns true if ACK was received
func (i2c *I2C) writeByte(data uint8) bool {
	// Pull SCL low
	avr.PORTB.ClearBits(1 << SCL_PIN)

	// Load data into USI data register
	avr.USIDR.Set(data)

	// Transfer 8 bits
	i2c.transfer(0x00) // Counter = 0, will overflow after 16 edges (8 bits)

	// Set SDA as input for ACK
	avr.DDRB.ClearBits(1 << SDA_PIN)

	// Read ACK bit (1 bit transfer)
	ack := i2c.transfer(0x0E) // Counter = 14, will overflow after 2 edges (1 bit)

	// Return true if ACK received (bit 0 = 0)
	return (ack & 0x01) == 0
}

// readByte receives a byte and sends ACK (true) or NACK (false)
func (i2c *I2C) readByte(ack bool) uint8 {
	// Set SDA as input
	avr.DDRB.ClearBits(1 << SDA_PIN)

	// Read 8 bits
	data := i2c.transfer(0x00)

	// Prepare ACK/NACK
	if ack {
		avr.USIDR.Set(0x00) // ACK = SDA low
	} else {
		avr.USIDR.Set(0xFF) // NACK = SDA high
	}

	// Send ACK/NACK (1 bit)
	i2c.transfer(0x0E)

	return data
}

// transfer clocks data in/out using USI
// counterValue is the initial value for the 4-bit counter (0x00 for 8 bits, 0x0E for 1 bit)
func (i2c *I2C) transfer(counterValue uint8) uint8 {
	// Set USISR: clear flags and set counter
	avr.USISR.Set(avr.USISR_USISIF | avr.USISR_USIOIF | avr.USISR_USIPF | avr.USISR_USIDC | counterValue)

	// Prepare control register value for clocking:
	// - Two-wire mode (USIWM1=1)
	// - Software clock strobe (USICS1=1, USICLK=1)
	// - Toggle clock (USITC=1)
	clockCmd := avr.USICR_USIWM1 | avr.USICR_USICS1 | avr.USICR_USICLK | avr.USICR_USITC

	// Clock until counter overflow
	for !avr.USISR.HasBits(avr.USISR_USIOIF) {
		// Delay for SCL low time
		i2c.delay()

		// Generate positive SCL edge
		avr.USICR.Set(clockCmd)

		// Wait for SCL to go high (clock stretching support)
		for !avr.PINB.HasBits(1 << SCL_PIN) {
		}

		// Delay for SCL high time
		i2c.delay()

		// Generate negative SCL edge
		avr.USICR.Set(clockCmd)
	}

	// Short delay
	i2c.delay()

	// Read data
	data := avr.USIDR.Get()

	// Release SDA (set high)
	avr.USIDR.Set(0xFF)

	// Set SDA as output
	avr.DDRB.SetBits(1 << SDA_PIN)

	return data
}

// delay provides timing for I2C (~5µs for 100kHz operation)
// Calculates the number of delay cycles based on CPU frequency
func (i2c *I2C) delay() {
	// For 100kHz I2C, we need ~5µs delay (T2_TWI > 4.7µs)
	// cycles = CPUFrequency * 5µs = CPUFrequency / 200000
	// Each loop iteration is approximately 3 cycles (dec, brne, nop)
	cycles := CPUFrequency() / 200000 / 3
	for i := uint32(0); i < cycles; i++ {
		avr.Asm("nop")
	}
}
