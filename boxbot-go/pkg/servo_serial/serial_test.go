package servo_serial

import (
	"github.com/stretchr/testify/assert"
	"testing"
)

func TestMapping(t *testing.T) {
	assert.InDelta(t, 500, mapAngleToMicroseconds(0), 0.001)
	assert.InDelta(t, 1000, mapAngleToMicroseconds(45), 0.001)
	assert.InDelta(t, 1500, mapAngleToMicroseconds(90), 0.001)
	assert.InDelta(t, 2000, mapAngleToMicroseconds(135), 0.001)
	assert.InDelta(t, 2500, mapAngleToMicroseconds(180), 0.001)
}
