package controller

import (
	"syscall"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestTolerateCondition(t *testing.T) {
	rejection := &feetech.ServoError{ID: 6, Op: "read", Status: feetech.ErrChecksum}
	mixed := &feetech.ServoError{ID: 6, Op: "read", Status: feetech.ErrOverload | feetech.ErrRange}
	noResponse := &feetech.ServoError{ID: 6, Op: "sync_read", Err: feetech.ErrNoResponse}
	transport := &feetech.CommError{Op: "read", Err: syscall.EIO}

	cases := map[string]struct {
		in        error
		wantFlags feetech.StatusError
		wantErr   error // nil means "must be nil"
	}{
		"nil":                 {nil, 0, nil},
		"bare condition flag": {feetech.ErrOverload, feetech.ErrOverload, nil},
		"servo error with condition": {
			&feetech.ServoError{ID: 6, Op: "read", Status: feetech.ErrOverheat}, feetech.ErrOverheat, nil},
		"sync read error ORs per-servo flags": {
			&feetech.SyncReadError{Op: "sync_read", Status: map[int]feetech.StatusError{
				2: feetech.ErrOverload, 5: feetech.ErrOverheat}},
			feetech.ErrOverload | feetech.ErrOverheat, nil},
		"rejection flag":                 {rejection, 0, rejection},
		"condition mixed with rejection": {mixed, 0, mixed},
		"missing servo":                  {noResponse, 0, noResponse},
		"transport failure":              {transport, 0, transport},
	}
	for name, tc := range cases {
		t.Run(name, func(t *testing.T) {
			flags, err := tolerateCondition(tc.in)
			assert.Equal(t, tc.wantFlags, flags)
			if tc.wantErr == nil {
				require.NoError(t, err)
			} else {
				assert.Same(t, tc.wantErr, err, "the original error must pass through untouched")
			}
		})
	}
}
