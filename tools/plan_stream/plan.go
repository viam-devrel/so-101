package main

import "fmt"

// armWaypoints pulls armName's rows (radians) out of a motion `plan` DoCommand response,
// which arrives as []any of map[string]any (frame) of []any of float64. A step whose arm
// slice is empty is skipped; a step with no arm key at all is an error.
func armWaypoints(planResp any, armName string) ([][]float64, error) {
	steps, ok := planResp.([]any)
	if !ok {
		return nil, fmt.Errorf("plan: want []any, got %T", planResp)
	}
	var out [][]float64
	for i, step := range steps {
		frames, ok := step.(map[string]any)
		if !ok {
			return nil, fmt.Errorf("plan[%d]: want map[string]any, got %T", i, step)
		}
		raw, ok := frames[armName]
		if !ok {
			return nil, fmt.Errorf("plan[%d]: no frame %q (have %v)", i, armName, keys(frames))
		}
		list, ok := raw.([]any)
		if !ok {
			return nil, fmt.Errorf("plan[%d][%q]: want []any, got %T", i, armName, raw)
		}
		if len(list) == 0 {
			continue
		}
		row := make([]float64, len(list))
		for j, v := range list {
			f, ok := v.(float64)
			if !ok {
				return nil, fmt.Errorf("plan[%d][%q][%d]: want float64, got %T", i, armName, j, v)
			}
			row[j] = f
		}
		out = append(out, row)
	}
	if len(out) == 0 {
		return nil, fmt.Errorf("plan has no waypoints for arm %q", armName)
	}
	return out, nil
}

func keys(m map[string]any) []string {
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	return out
}
