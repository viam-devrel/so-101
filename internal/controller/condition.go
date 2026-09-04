package controller

import "github.com/hipsterbrown/feetech-servo/feetech"

// tolerateCondition keeps a read whose only error is a servo condition flag (overload,
// overheat, voltage, angle limit): the payload is valid, so the flags come back and the
// error does not. Transport failures and request rejections pass through unchanged.
// Every read path in this package routes its bus error through here.
func tolerateCondition(err error) (feetech.StatusError, error) {
	if flags, ok := feetech.ConditionStatus(err); ok {
		return flags, nil
	}
	return 0, err
}
