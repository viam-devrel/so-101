package arm

import "testing"

func TestParseWaitExtra(t *testing.T) {
	cases := []struct {
		name  string
		extra map[string]interface{}
		want  bool
	}{
		{"nil defaults to true", nil, true},
		{"missing key defaults to true", map[string]interface{}{"speed": 1.0}, true},
		{"explicit false", map[string]interface{}{"wait": false}, false},
		{"explicit true", map[string]interface{}{"wait": true}, true},
		{"non-bool ignored", map[string]interface{}{"wait": "no"}, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			if got := parseWaitExtra(tc.extra); got != tc.want {
				t.Fatalf("parseWaitExtra(%v) = %v, want %v", tc.extra, got, tc.want)
			}
		})
	}
}
