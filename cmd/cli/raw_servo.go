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

	fmt.Println("Connected to SO-101")

	// Clear buffers
	port.ResetInputBuffer()
	port.ResetOutputBuffer()

	// Try different SO-ARM packet formats

	// Test 1: Simple position command for servo 1
	fmt.Println("Test 1: Trying simple position command...")

	// Format: [Header][ID][Length][Command][Position_L][Position_H][Checksum]
	// Try moving servo 1 to center position (2048)
	packet1 := []byte{
		0xFF, 0xFF, // Header
		0x01,       // Servo ID 1
		0x05,       // Length (command + 2 position bytes + checksum)
		0x03,       // Write command
		0x1E,       // Goal position low address
		0x00, 0x08, // Position 2048 (center) - low byte, high byte
		0x00, // Checksum placeholder
	}

	// Calculate checksum
	checksum := byte(0)
	for i := 2; i < len(packet1)-1; i++ {
		checksum += packet1[i]
	}
	packet1[len(packet1)-1] = ^checksum // Bitwise NOT

	fmt.Printf("Sending packet: %X\n", packet1)
	n, err := port.Write(packet1)
	if err != nil {
		fmt.Printf("Write error: %v\n", err)
	} else {
		fmt.Printf("Sent %d bytes\n", n)
	}

	time.Sleep(2 * time.Second)

	// Test 2: Try different packet format
	fmt.Println("Test 2: Trying alternative format...")

	// Maybe SO-101 uses different structure
	packet2 := []byte{
		0xFF, 0xFF, // Header
		0x01,       // Servo ID 1
		0x07,       // Length
		0x03,       // Write Data
		0x1E,       // Starting address (Goal Position L)
		0x02,       // Data length (2 bytes for position)
		0x00, 0x08, // Position 2048
		0x00, // Checksum
	}

	// Calculate checksum
	checksum = byte(0)
	for i := 2; i < len(packet2)-1; i++ {
		checksum += packet2[i]
	}
	packet2[len(packet2)-1] = ^checksum

	fmt.Printf("Sending packet: %X\n", packet2)
	n, err = port.Write(packet2)
	if err != nil {
		fmt.Printf("Write error: %v\n", err)
	} else {
		fmt.Printf("Sent %d bytes\n", n)
	}

	time.Sleep(2 * time.Second)

	// Test 3: Try simple movement with speed
	fmt.Println("Test 3: Trying movement with speed...")

	// Set moving speed first
	speedPacket := []byte{
		0xFF, 0xFF, // Header
		0x01,       // Servo ID 1
		0x07,       // Length
		0x03,       // Write Data
		0x20,       // Moving speed address
		0x02,       // Data length
		0x00, 0x02, // Speed (512)
		0x00, // Checksum
	}

	checksum = byte(0)
	for i := 2; i < len(speedPacket)-1; i++ {
		checksum += speedPacket[i]
	}
	speedPacket[len(speedPacket)-1] = ^checksum

	fmt.Printf("Setting speed: %X\n", speedPacket)
	port.Write(speedPacket)
	time.Sleep(100 * time.Millisecond)

	// Then send position
	posPacket := []byte{
		0xFF, 0xFF, // Header
		0x01,       // Servo ID 1
		0x07,       // Length
		0x03,       // Write Data
		0x1E,       // Goal position address
		0x02,       // Data length
		0x00, 0x09, // Position 2304 (slight movement)
		0x00, // Checksum
	}

	checksum = byte(0)
	for i := 2; i < len(posPacket)-1; i++ {
		checksum += posPacket[i]
	}
	posPacket[len(posPacket)-1] = ^checksum

	fmt.Printf("Setting position: %X\n", posPacket)
	port.Write(posPacket)

	fmt.Println("Tests complete. Did you see any movement?")
	time.Sleep(3 * time.Second)
}
