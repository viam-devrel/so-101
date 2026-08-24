package servocmd

import "testing"

func TestWaitArg(t *testing.T) {
	cases := []struct {
		name string
		cmd  map[string]any
		want bool
	}{
		{"nil defaults to true", nil, true},
		{"missing key defaults to true", map[string]any{"speed": 1.0}, true},
		{"explicit false", map[string]any{"wait": false}, false},
		{"explicit true", map[string]any{"wait": true}, true},
		{"non-bool ignored", map[string]any{"wait": "no"}, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			if got := WaitArg(tc.cmd); got != tc.want {
				t.Fatalf("WaitArg(%v) = %v, want %v", tc.cmd, got, tc.want)
			}
		})
	}
}
