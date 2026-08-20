// Command servo_info reports identity and firmware registers for every servo on a bus.
//
//	go run cmd/cli/servo_info.go -port=/dev/cu.usbmodemXXXX
package main

import (
	"context"
	"flag"
	"fmt"
	"os"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
	port := flag.String("port", "", "serial port")
	ids := flag.String("ids", "1,2,3,4,5,6", "comma-separated servo IDs")
	flag.Parse()
	if *port == "" {
		fmt.Fprintln(os.Stderr, "error: -port is required")
		os.Exit(1)
	}

	bus, err := feetech.NewBus(feetech.BusConfig{
		Port: *port, BaudRate: 1000000, Protocol: feetech.ProtocolSTS,
		Timeout: time.Second, MinCommandGap: time.Millisecond,
	})
	if err != nil {
		fmt.Fprintf(os.Stderr, "error: open %s: %v\n", *port, err)
		os.Exit(1)
	}
	defer bus.Close()

	ctx := context.Background()
	var idList []int
	for _, c := range *ids {
		if c >= '1' && c <= '9' {
			idList = append(idList, int(c-'0'))
		}
	}

	fmt.Printf("%-4s %-8s %-14s %-10s %-9s %-8s %s\n",
		"id", "ping", "model_number", "firmware", "mode", "temp_C", "volts")
	for _, id := range idList {
		s := feetech.NewServo(bus, id, &feetech.ModelSTS3215)

		model, err := s.Ping(ctx)
		if err != nil {
			fmt.Printf("%-4d %-8s %v\n", id, "FAIL", err)
			continue
		}

		read := func(r feetech.Register) string {
			b, err := bus.ReadRegister(ctx, id, r.Address, int(r.Size))
			if err != nil {
				return "err"
			}
			if r.Size == 1 {
				return fmt.Sprintf("%d", b[0])
			}
			return fmt.Sprintf("%d", bus.Protocol().DecodeWord(b))
		}

		// feetech.RegFirmwareVersion is {Address: 0, Size: 1} and so reports only the
		// major. The minor lives at address 1; read both.
		fw := "?"
		if b, err := bus.ReadRegister(ctx, id, 0, 2); err == nil {
			fw = fmt.Sprintf("%d.%d", b[0], b[1])
		}
		mode := read(feetech.RegOperatingMode)
		temp := read(feetech.RegPresentTemp)
		volt := read(feetech.RegPresentVoltage) // tenths of a volt

		voltV := volt
		if b, err := bus.ReadRegister(ctx, id, feetech.RegPresentVoltage.Address, 1); err == nil {
			voltV = fmt.Sprintf("%.1f", float64(b[0])/10)
		}
		fmt.Printf("%-4d %-8s %-14d %-10s %-9s %-8s %s\n", id, "ok", model, fw, mode, temp, voltV)
	}
}
