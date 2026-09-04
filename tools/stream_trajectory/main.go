// Command stream_trajectory replays an arm-recorder session through
// MoveThroughJointPositionsStreamed, so time-scheduled streaming can be measured on hardware.
//
//	go run ./tools/stream_trajectory -address <machine> -arm follower-arm -session nod.json
package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"math"
	"os"
	"os/signal"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
)

// session is the arm-recorder on-disk schema (frames are radians; no per-frame timestamps).
type session struct {
	FrequencyHz float64     `json:"frequency_hz"`
	JointCount  int         `json:"joint_count"`
	Frames      [][]float64 `json:"frames"`
}

func main() {
	address := flag.String("address", "", "machine address")
	apiKey := flag.String("api-key", os.Getenv("VIAM_API_KEY"), "API key ($VIAM_API_KEY)")
	apiKeyID := flag.String("api-key-id", os.Getenv("VIAM_API_KEY_ID"), "API key ID ($VIAM_API_KEY_ID)")
	armName := flag.String("arm", "arm", "arm component name")
	path := flag.String("session", "", "arm-recorder session JSON")
	hz := flag.Float64("hz", 100, "densify to this rate; at or below the recording's rate streams it as recorded")
	batch := flag.Int("batch", 10, "points per batch")
	flag.Parse()
	if *address == "" || *path == "" || *batch < 1 || *hz <= 0 {
		flag.Usage()
		os.Exit(2)
	}

	raw, err := os.ReadFile(*path)
	if err != nil {
		log.Fatal(err)
	}
	var sess session
	if err := json.Unmarshal(raw, &sess); err != nil {
		log.Fatalf("parse %s: %v", *path, err)
	}
	if sess.FrequencyHz <= 0 || len(sess.Frames) == 0 {
		log.Fatalf("%s: need frequency_hz > 0 and at least one frame", *path)
	}
	frames, rate := densify(sess.Frames, sess.FrequencyHz, *hz)
	points := toPoints(frames, rate)

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()
	logger := logging.NewLogger("stream_trajectory")
	machine, err := client.New(ctx, *address, logger, client.WithDialOptions(
		rpc.WithEntityCredentials(*apiKeyID, rpc.Credentials{Type: rpc.CredentialsTypeAPIKey, Payload: *apiKey})))
	if err != nil {
		log.Fatal(err)
	}
	defer func() { _ = machine.Close(context.Background()) }()
	a, err := arm.FromRobot(machine, *armName)
	if err != nil {
		log.Fatal(err)
	}

	batches := make(chan []arm.TrajectoryPoint)
	responses := make(chan arm.Response)
	go func() {
		defer close(batches)
		for i := 0; i < len(points); i += *batch {
			select {
			case batches <- points[i:min(i+*batch, len(points))]:
			case <-ctx.Done():
				return
			}
		}
	}()
	// Drain acks concurrently: sending every batch before reading any can wedge on gRPC
	// flow control.
	acks := 0
	drained := make(chan struct{})
	go func() {
		defer close(drained)
		for range responses {
			acks++
		}
	}()

	started := time.Now()
	// The client does not close responses (rdk components/arm/client.go: "the caller closes
	// responses once we have returned"), so we close it here after the call returns.
	err = a.MoveThroughJointPositionsStreamed(ctx, batches, responses, nil)
	close(responses)
	<-drained

	fmt.Printf("%d points at %.0f Hz, %d acks, trajectory %.2fs, wall %.2fs, err=%v\n",
		len(points), rate, acks, points[len(points)-1].Time.Seconds(), time.Since(started).Seconds(), err)
	if err != nil {
		os.Exit(1)
	}
}

// densify linearly resamples frames recorded at fromHz to toHz. At or below fromHz it
// returns the input unchanged.
func densify(frames [][]float64, fromHz, toHz float64) ([][]float64, float64) {
	if toHz <= fromHz || len(frames) < 2 {
		return frames, fromHz
	}
	total := float64(len(frames)-1) / fromHz
	n := int(math.Floor(total*toHz+1e-9)) + 1
	out := make([][]float64, n)
	last := len(frames) - 1
	for k := range out {
		pos := float64(k) / toHz * fromHz // fractional source index
		i := int(math.Floor(pos))
		if i >= last {
			out[k] = frames[last]
			continue
		}
		f := pos - float64(i)
		row := make([]float64, len(frames[i]))
		for j := range row {
			row[j] = frames[i][j]*(1-f) + frames[i+1][j]*f
		}
		out[k] = row
	}
	return out, toHz
}

// toPoints stamps frame i at i/rate seconds.
func toPoints(frames [][]float64, rate float64) []arm.TrajectoryPoint {
	points := make([]arm.TrajectoryPoint, len(frames))
	for i, f := range frames {
		pos := make([]referenceframe.Input, len(f))
		copy(pos, f)
		points[i] = arm.TrajectoryPoint{
			Time:      time.Duration(float64(i) / rate * float64(time.Second)),
			Positions: pos,
		}
	}
	return points
}
