package servocmd

import "testing"

func TestParseWaitExtra(t *testing.T) {
	cases := []struct {
		name  string
		extra map[string]any
		want  bool
	}{
		{"nil defaults to true", nil, true},
		{"missing key defaults to true", map[string]any{"speed": 1.0}, true},
		{"explicit false", map[string]any{"wait": false}, false},
		{"explicit true", map[string]any{"wait": true}, true},
		{"non-bool ignored", map[string]any{"wait": "no"}, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			if got := ParseWaitExtra(tc.extra); got != tc.want {
				t.Fatalf("ParseWaitExtra(%v) = %v, want %v", tc.extra, got, tc.want)
			}
		})
	}
}
