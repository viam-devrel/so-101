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

	fmt.Println("Testing direct servo communication...")

	// Test 1: Move servo 1 to position 35000 (slight movement from center 32768)
	fmt.Println("Test 1: Moving servo 1 to position 35000...")

	// Feetech sync write packet for Goal_Position (address 42)
	// [0xFF][0xFF][0xFE][Length][0x83][42][2][ID][Pos_L][Pos_H][Checksum]
	position := 35000
	packet := []byte{
		0xFF, 0xFF, // Header
		0xFE,                         // Broadcast ID for sync write
		0x07,                         // Length (instruction + addr + data_len + id + pos_l + pos_h + checksum)
		0x83,                         // Sync write instruction
		42,                           // Goal_Position address
		2,                            // Data length (2 bytes per servo)
		1,                            // Servo ID 1
		byte(position & 0xFF),        // Position low byte
		byte((position >> 8) & 0xFF), // Position high byte
	}

	// Calculate checksum (ID + Length + Instruction + Address + DataLen + ServoID + PosL + PosH)
	checksum := byte(0)
	checksum += byte(0xFE)                   // ID
	checksum += 0x07                         // Length
	checksum += 0x83                         // Instruction
	checksum += 42                           // Address
	checksum += 2                            // Data length
	checksum += 1                            // Servo ID
	checksum += byte(position & 0xFF)        // Position low byte
	checksum += byte((position >> 8) & 0xFF) // Position high byte
	checksum = ^checksum                     // Bitwise NOT
	packet = append(packet, checksum)

	fmt.Printf("Sending packet: %X\n", packet)

	// Clear buffers
	port.ResetInputBuffer()
	port.ResetOutputBuffer()

	// Send packet
	n, err := port.Write(packet)
	if err != nil {
		fmt.Printf("Write error: %v\n", err)
	} else {
		fmt.Printf("Sent %d bytes\n", n)
	}

	fmt.Println("Waiting 3 seconds for movement...")
	time.Sleep(3 * time.Second)

	// Test 2: Move back to center
	fmt.Println("Test 2: Moving servo 1 back to center (32768)...")

	position = 32768
	packet = []byte{
		0xFF, 0xFF, // Header
		0xFE,                         // Broadcast ID for sync write
		0x07,                         // Length
		0x83,                         // Sync write instruction
		42,                           // Goal_Position address
		2,                            // Data length (2 bytes per servo)
		1,                            // Servo ID 1
		byte(position & 0xFF),        // Position low byte
		byte((position >> 8) & 0xFF), // Position high byte
	}

	checksum = byte(0)
	checksum += byte(0xFE)                   // ID
	checksum += 0x07                         // Length
	checksum += 0x83                         // Instruction
	checksum += 42                           // Address
	checksum += 2                            // Data length
	checksum += 1                            // Servo ID
	checksum += byte(position & 0xFF)        // Position low byte
	checksum += byte((position >> 8) & 0xFF) // Position high byte
	checksum = ^checksum
	packet = append(packet, checksum)

	fmt.Printf("Sending packet: %X\n", packet)

	port.ResetInputBuffer()
	port.ResetOutputBuffer()

	n, err = port.Write(packet)
	if err != nil {
		fmt.Printf("Write error: %v\n", err)
	} else {
		fmt.Printf("Sent %d bytes\n", n)
	}

	fmt.Println("Test complete. Did you see any movement?")
	time.Sleep(2 * time.Second)
}
