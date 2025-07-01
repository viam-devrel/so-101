package main

import (
	"fmt"
	"log"
	"time"

	"go.bug.st/serial"
)

func main() {
	// Open serial port
	mode := &serial.Mode{
		BaudRate: 1000000,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	port, err := serial.Open("/dev/tty.usbmodem5A4B0464471", mode)
	if err != nil {
		log.Fatal(err)
	}
	defer port.Close()

	fmt.Println("Connected to SO-101 - Testing READ commands")

	// Set read timeout
	port.SetReadTimeout(1 * time.Second)

	// Try to read servo model/version info
	fmt.Println("Test 1: Reading servo model info...")

	// Read model number (address 0x00, 2 bytes)
	readPacket := []byte{
		0xFF, 0xFF, // Header
		0x01, // Servo ID 1
		0x04, // Length
		0x02, // Read command
		0x00, // Starting address (model number)
		0x02, // Number of bytes to read
		0x00, // Checksum placeholder
	}

	// Calculate checksum
	checksum := byte(0)
	for i := 2; i < len(readPacket)-1; i++ {
		checksum += readPacket[i]
	}
	readPacket[len(readPacket)-1] = ^checksum

	fmt.Printf("Sending read packet: %X\n", readPacket)

	// Clear buffer first
	port.ResetInputBuffer()

	// Send packet
	n, err := port.Write(readPacket)
	if err != nil {
		fmt.Printf("Write error: %v\n", err)
	} else {
		fmt.Printf("Sent %d bytes\n", n)
	}

	// Try to read response
	time.Sleep(100 * time.Millisecond)
	response := make([]byte, 20)
	n, err = port.Read(response)
	if err != nil {
		fmt.Printf("Read error: %v\n", err)
	} else {
		fmt.Printf("Received %d bytes: %X\n", n, response[:n])
	}

	time.Sleep(1 * time.Second)

	// Test 2: Try reading current position
	fmt.Println("Test 2: Reading current position...")

	posReadPacket := []byte{
		0xFF, 0xFF, // Header
		0x01, // Servo ID 1
		0x04, // Length
		0x02, // Read command
		0x24, // Present position address
		0x02, // Number of bytes to read
		0x00, // Checksum placeholder
	}

	checksum = byte(0)
	for i := 2; i < len(posReadPacket)-1; i++ {
		checksum += posReadPacket[i]
	}
	posReadPacket[len(posReadPacket)-1] = ^checksum

	fmt.Printf("Sending position read: %X\n", posReadPacket)

	port.ResetInputBuffer()

	n, err = port.Write(posReadPacket)
	if err != nil {
		fmt.Printf("Write error: %v\n", err)
	} else {
		fmt.Printf("Sent %d bytes\n", n)
	}

	time.Sleep(100 * time.Millisecond)
	n, err = port.Read(response)
	if err != nil {
		fmt.Printf("Read error: %v\n", err)
	} else {
		fmt.Printf("Received %d bytes: %X\n", n, response[:n])
	}

	time.Sleep(1 * time.Second)

	// Test 3: Try different servo IDs (maybe they're not 1,2,3,4,5)
	fmt.Println("Test 3: Scanning for servo IDs...")

	for id := 1; id <= 10; id++ {
		fmt.Printf("Trying servo ID %d...\n", id)

		pingPacket := []byte{
			0xFF, 0xFF, // Header
			byte(id), // Servo ID
			0x02,     // Length
			0x01,     // Ping command
			0x00,     // Checksum placeholder
		}

		checksum = byte(0)
		for i := 2; i < len(pingPacket)-1; i++ {
			checksum += pingPacket[i]
		}
		pingPacket[len(pingPacket)-1] = ^checksum

		port.ResetInputBuffer()
		port.Write(pingPacket)

		time.Sleep(50 * time.Millisecond)
		n, err = port.Read(response)
		if err == nil && n > 0 {
			fmt.Printf("  Servo ID %d responded: %X\n", id, response[:n])
		}
	}

	fmt.Println("Scan complete.")
}
