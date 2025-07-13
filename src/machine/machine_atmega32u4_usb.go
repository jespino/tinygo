//go:build atmega32u4

package machine

import (
	"device/avr"
	"machine/usb"
	"runtime/interrupt"
	"unsafe"
)

const NumberOfUSBEndpoints = 7

var (
	// USB endpoint configuration
	endPoints = []uint32{
		usb.CONTROL_ENDPOINT:  usb.ENDPOINT_TYPE_CONTROL,
		usb.CDC_ENDPOINT_ACM:  (usb.ENDPOINT_TYPE_INTERRUPT | usb.EndpointIn),
		usb.CDC_ENDPOINT_OUT:  (usb.ENDPOINT_TYPE_BULK | usb.EndpointOut),
		usb.CDC_ENDPOINT_IN:   (usb.ENDPOINT_TYPE_BULK | usb.EndpointIn),
		usb.HID_ENDPOINT_IN:   (usb.ENDPOINT_TYPE_DISABLE), // Interrupt In
		usb.HID_ENDPOINT_OUT:  (usb.ENDPOINT_TYPE_DISABLE), // Interrupt Out
		usb.MIDI_ENDPOINT_IN:  (usb.ENDPOINT_TYPE_DISABLE), // Bulk In
		usb.MIDI_ENDPOINT_OUT: (usb.ENDPOINT_TYPE_DISABLE), // Bulk Out
	}
)

// Configure the USB peripheral. The config is here for compatibility with the UART interface.
func (dev *USBDevice) Configure(config UARTConfig) {
	if dev.initcomplete {
		return
	}

	state := interrupt.Disable()
	defer interrupt.Restore(state)

	// Initialize USB controller
	initUSBController()

	// Enable USB interrupts
	interrupt.New(avr.USB_GEN_vect, handleUSBIRQ)

	// Enable endpoint interrupts
	interrupt.New(avr.USB_COM_vect, handleUSBEndpointIRQ)

	dev.initcomplete = true
}

// initUSBController initializes the ATmega32U4 USB controller
func initUSBController() {
	// Enable USB clock (PLL)
	configurePLL()

	// Enable USB pad regulator
	avr.UHWCON.SetBits(1 << avr.UVREGE)

	// Enable USB controller
	avr.USBCON.SetBits(1 << avr.USBE)

	// Attach to USB bus
	avr.UDCON.ClearBits(1 << avr.DETACH)

	// Enable USB interrupts
	avr.UDIEN.SetBits(1<<avr.EORSTE | 1<<avr.SOFE)

	// Configure control endpoint (EP0)
	initEndpoint(0, usb.ENDPOINT_TYPE_CONTROL)
}

// configurePLL sets up the PLL for USB operation
func configurePLL() {
	// Configure PLL prescaler for 16MHz crystal
	// PLL input divider = 16MHz / 8 = 2MHz
	avr.PLLFRQ.Set(0x4A) // PDIV = 0x0A (divide by 11), PLLTM = 0x2, PLLUSB = 1

	// Enable PLL
	avr.PLLCSR.SetBits(1 << avr.PLLE)

	// Wait for PLL to lock
	for !avr.PLLCSR.HasBits(1 << avr.PLOCK) {
		// Wait for PLL lock
	}
}

// initEndpoint initializes a USB endpoint
func initEndpoint(ep uint32, epType uint32) {
	// Select endpoint
	avr.UENUM.Set(uint8(ep))

	// Enable endpoint
	avr.UECONX.SetBits(1 << avr.EPEN)

	// Configure endpoint type and direction
	var cfg0 uint8
	var cfg1 uint8

	switch epType & 0x03 {
	case usb.ENDPOINT_TYPE_CONTROL:
		cfg0 = avr.EP_TYPE_CONTROL << avr.EPTYPE0
		cfg1 = avr.EP_SIZE_64<<avr.EPSIZE0 | avr.EP_BANK_SINGLE<<avr.EPBK0 | 1<<avr.ALLOC
	case usb.ENDPOINT_TYPE_BULK:
		cfg0 = avr.EP_TYPE_BULK << avr.EPTYPE0
		cfg1 = avr.EP_SIZE_64<<avr.EPSIZE0 | avr.EP_BANK_SINGLE<<avr.EPBK0 | 1<<avr.ALLOC
	case usb.ENDPOINT_TYPE_INTERRUPT:
		cfg0 = avr.EP_TYPE_INTERRUPT << avr.EPTYPE0
		cfg1 = avr.EP_SIZE_64<<avr.EPSIZE0 | avr.EP_BANK_SINGLE<<avr.EPBK0 | 1<<avr.ALLOC
	case usb.ENDPOINT_TYPE_ISOCHRONOUS:
		cfg0 = avr.EP_TYPE_ISOCHRONOUS << avr.EPTYPE0
		cfg1 = avr.EP_SIZE_64<<avr.EPSIZE0 | avr.EP_BANK_SINGLE<<avr.EPBK0 | 1<<avr.ALLOC
	}

	// Set endpoint direction for non-control endpoints
	if epType&0x03 != usb.ENDPOINT_TYPE_CONTROL {
		if epType&usb.EndpointIn != 0 {
			cfg0 |= avr.EP_DIR_IN << avr.EPDIR
		} else {
			cfg0 |= avr.EP_DIR_OUT << avr.EPDIR
		}
	}

	// Configure endpoint
	avr.UECFG0X.Set(cfg0)
	avr.UECFG1X.Set(cfg1)

	// Check if configuration is OK
	if !avr.UESTA0X.HasBits(1 << avr.CFGOK) {
		// Configuration failed
		return
	}

	// Enable endpoint interrupts
	if ep == 0 {
		avr.UEIENX.SetBits(1 << avr.RXSTPI)
	} else {
		avr.UEIENX.SetBits(1<<avr.TXINI | 1<<avr.RXOUTI)
	}
}

// handleUSBIRQ handles USB general interrupts
func handleUSBIRQ(interrupt.Interrupt) {
	// Check for end of reset
	if avr.UDINT.HasBits(1 << avr.EORSTI) {
		avr.UDINT.ClearBits(1 << avr.EORSTI)

		// Reset USB configuration
		usbConfiguration = 0

		// Reconfigure control endpoint
		initEndpoint(0, usb.ENDPOINT_TYPE_CONTROL)
	}

	// Check for start of frame
	if avr.UDINT.HasBits(1 << avr.SOFI) {
		avr.UDINT.ClearBits(1 << avr.SOFI)
		// Handle start of frame if needed
	}

	// Check for suspend
	if avr.UDINT.HasBits(1 << avr.SUSPI) {
		avr.UDINT.ClearBits(1 << avr.SUSPI)
		// Handle suspend if needed
	}

	// Check for wake-up
	if avr.UDINT.HasBits(1 << avr.WAKEUPI) {
		avr.UDINT.ClearBits(1 << avr.WAKEUPI)
		// Handle wake-up if needed
	}
}

// handleUSBEndpointIRQ handles USB endpoint interrupts
func handleUSBEndpointIRQ(interrupt.Interrupt) {
	// Check each endpoint for pending interrupts
	for ep := uint8(0); ep < NumberOfUSBEndpoints; ep++ {
		// Select endpoint
		avr.UENUM.Set(ep)

		// Check for setup packet on control endpoint
		if ep == 0 && avr.UEINTX.HasBits(1<<avr.RXSTPI) {
			handleSetupPacket()
		}

		// Check for received data
		if avr.UEINTX.HasBits(1 << avr.RXOUTI) {
			handleEndpointRx(ep)
		}

		// Check for transmitted data
		if avr.UEINTX.HasBits(1 << avr.TXINI) {
			handleEndpointTx(ep)
		}
	}
}

// handleSetupPacket handles setup packets on control endpoint
func handleSetupPacket() {
	// Read setup packet
	setup := parseUSBSetupPacket()

	// Clear setup interrupt
	avr.UEINTX.ClearBits(1 << avr.RXSTPI)

	ok := false
	if (setup.BmRequestType & usb.REQUEST_TYPE) == usb.REQUEST_STANDARD {
		// Standard Requests
		ok = handleStandardSetup(setup)
	} else {
		// Class Interface Requests
		if setup.WIndex < uint16(len(usbSetupHandler)) && usbSetupHandler[setup.WIndex] != nil {
			ok = usbSetupHandler[setup.WIndex](setup)
		}
	}

	if !ok {
		// Stall endpoint
		avr.UECONX.SetBits(1 << avr.STALLRQ)
	}
}

// parseUSBSetupPacket parses a setup packet from the USB controller
func parseUSBSetupPacket() usb.Setup {
	var setup usb.Setup

	// Read setup packet data from endpoint FIFO
	setup.BmRequestType = avr.UEDATX.Get()
	setup.BRequest = avr.UEDATX.Get()
	setup.WValueL = avr.UEDATX.Get()
	setup.WValueH = avr.UEDATX.Get()
	setup.WIndex = uint16(avr.UEDATX.Get()) | (uint16(avr.UEDATX.Get()) << 8)
	setup.WLength = uint16(avr.UEDATX.Get()) | (uint16(avr.UEDATX.Get()) << 8)

	return setup
}

// handleEndpointRx handles received data on endpoints
func handleEndpointRx(ep uint8) {
	// Select endpoint
	avr.UENUM.Set(uint8(ep))

	// Get byte count
	count := avr.UEBCLX.Get()
	if count > 0 {
		// Read data from FIFO
		data := make([]byte, count)
		for i := uint8(0); i < count; i++ {
			data[i] = avr.UEDATX.Get()
		}

		// Call handler if available
		if usbRxHandler[ep] != nil {
			usbRxHandler[ep](data)
		}
	}

	// Clear OUT interrupt
	avr.UEINTX.ClearBits(1 << avr.RXOUTI)

	// Release FIFO
	avr.UEINTX.ClearBits(1 << avr.FIFOCON)
}

// handleEndpointTx handles transmitted data on endpoints
func handleEndpointTx(ep uint8) {
	// Select endpoint
	avr.UENUM.Set(uint8(ep))

	// Clear IN interrupt
	avr.UEINTX.ClearBits(1 << avr.TXINI)

	// Call handler if available
	if usbTxHandler[ep] != nil {
		usbTxHandler[ep]()
	}
}

// SendUSBInPacket sends a packet on an IN endpoint
func SendUSBInPacket(ep uint32, data []byte) {
	// Select endpoint
	avr.UENUM.Set(uint8(ep))

	// Wait for endpoint to be ready
	for !avr.UEINTX.HasBits(1 << avr.TXINI) {
		// Wait for TX ready
	}

	// Write data to FIFO
	for _, b := range data {
		avr.UEDATX.Set(b)
	}

	// Send packet
	avr.UEINTX.ClearBits(1 << avr.TXINI)
	avr.UEINTX.ClearBits(1 << avr.FIFOCON)
}

// SendZlp sends a zero-length packet
func SendZlp() {
	SendUSBInPacket(0, []byte{})
}

// ReceiveUSBControlPacket receives a control packet
func ReceiveUSBControlPacket() ([usb.EndpointPacketSize]byte, error) {
	// Select control endpoint
	avr.UENUM.Set(0)

	// Wait for data
	for !avr.UEINTX.HasBits(1 << avr.RXOUTI) {
		// Wait for RX ready
	}

	// Read data
	var data [usb.EndpointPacketSize]byte
	count := avr.UEBCLX.Get()
	for i := uint8(0); i < count && i < usb.EndpointPacketSize; i++ {
		data[i] = avr.UEDATX.Get()
	}

	// Clear OUT interrupt
	avr.UEINTX.ClearBits(1 << avr.RXOUTI)

	return data, nil
}

// handleUSBSetAddress handles USB set address requests
func handleUSBSetAddress(setup usb.Setup) bool {
	// Set address but don't enable yet
	avr.UDADDR.Set(setup.WValueL)

	// Send status
	SendZlp()

	// Wait for status to be sent
	for !avr.UEINTX.HasBits(1 << avr.TXINI) {
		// Wait
	}

	// Enable address
	avr.UDADDR.SetBits(1 << avr.ADDEN)
	
	return true
}

// AckUsbOutTransfer acknowledges an OUT transfer
func AckUsbOutTransfer(ep uint32) {
	// Select endpoint
	avr.UENUM.Set(uint8(ep))

	// Clear OUT interrupt
	avr.UEINTX.ClearBits(1 << avr.RXOUTI)

	// Release FIFO
	avr.UEINTX.ClearBits(1 << avr.FIFOCON)
}

// sendViaEPIn sends data via an IN endpoint
func sendViaEPIn(ep uint32, data *byte, count int) {
	// Select endpoint
	avr.UENUM.Set(uint8(ep))

	// Wait for endpoint to be ready
	for !avr.UEINTX.HasBits(1 << avr.TXINI) {
		// Wait for TX ready
	}

	// Write data to FIFO
	ptr := uintptr(unsafe.Pointer(data))
	for i := 0; i < count; i++ {
		avr.UEDATX.Set(*(*uint8)(unsafe.Pointer(ptr)))
		ptr++
	}

	// Send packet
	avr.UEINTX.ClearBits(1 << avr.TXINI)
	avr.UEINTX.ClearBits(1 << avr.FIFOCON)
}

// USB constants
const (
	usb_VID = 0x2886 // Default vendor ID
	usb_PID = 0x802d // Default product ID
)

// EnterBootloader resets the system to enter the bootloader
func EnterBootloader() {
	// Disable interrupts
	state := interrupt.Disable()
	defer interrupt.Restore(state)

	// Disconnect from USB
	avr.UDCON.SetBits(1 << avr.DETACH)

	// Reset to bootloader
	avr.UDCON.SetBits(1 << avr.RSTCPU)
}

// sendUSBPacket sends a USB packet on the specified endpoint
func sendUSBPacket(ep uint32, data []byte, maxsize uint16) {
	l := uint16(len(data))
	if 0 < maxsize && maxsize < l {
		l = maxsize
	}

	// Select endpoint
	avr.UENUM.Set(uint8(ep))

	// Wait for endpoint to be ready
	for !avr.UEINTX.HasBits(1 << avr.TXINI) {
		// Wait for TX ready
	}

	// Write data to FIFO
	for i := uint16(0); i < l; i++ {
		avr.UEDATX.Set(data[i])
	}

	// Send packet
	avr.UEINTX.ClearBits(1 << avr.TXINI)
	avr.UEINTX.ClearBits(1 << avr.FIFOCON)
}

// Reset system when DTR is low and baud rate is 1200
func handleBootloaderReset() {
	// This would be called from the CDC handler when DTR goes low
	// and baud rate is set to 1200 bps
	EnterBootloader()
}
