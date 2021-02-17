package main

import (
	"github.com/kelseyhightower/envconfig"
	"github.com/pkg/errors"
)

//TODO(timanema): Get correct values for these things (cruise speed, homing offset, indefinite speed etc..)
type Config struct {
	BaseMotorEnabled         bool    `default:"true" split_words:"true"`
	BaseMotorDevice          string  `default:"/dev/ttyUSB0" split_words:"true"`
	BaseMotorID              int32   `default:"0" split_words:"true"`
	BaseMotorCruiseSpeed     float32 `default:"200" split_words:"true"`
	BaseMotorMaxSpeed        float32 `default:"400" split_words:"true"`
	BaseMotorHomingOffset    float32 `default:"3124" split_words:"true"`
	BaseMotorHomingSpeed     float32 `default:"200" split_words:"true"`
	BaseMotorReversePosition bool    `default:"true" split_words:"true"`
	BaseMotorIndefiniteSpeed float32 `default:"200" split_words:"true"`

	TowerMotorEnabled         bool    `default:"true" split_words:"true"`
	TowerMotorDevice          string  `default:"/dev/ttyUSB1" split_words:"true"`
	TowerMotorID              int32   `default:"1" split_words:"true"`
	TowerMotorCruiseSpeed     float32 `default:"300" split_words:"true"`
	TowerMotorMaxSpeed        float32 `default:"1500" split_words:"true"`
	TowerMotorHomingSpeed     float32 `default:"300" split_words:"true"`
	TowerMotorReversePosition bool    `default:"true" split_words:"true"`
	TowerMotorIndefiniteSpeed float32 `default:"200" split_words:"true"`

	ArmMotorEnabled         bool    `default:"true" split_words:"true"`
	ArmMotorDevice          string  `default:"/dev/ttyUSB2" split_words:"true"`
	ArmMotorID              int32   `default:"2" split_words:"true"`
	ArmMotorCruiseSpeed     float32 `default:"800" split_words:"true"`
	ArmMotorMaxSpeed        float32 `default:"1000" split_words:"true"`
	ArmMotorHomingSpeed     float32 `default:"400" split_words:"true"`
	ArmMotorReversePosition bool    `default:"true" split_words:"true"`
	ArmMotorIndefiniteSpeed float32 `default:"400" split_words:"true"`
}

func ReadConfig() (Config, error) {
	var c Config
	err := envconfig.Process("RCS_STEPPER", &c)
	if err != nil {
		return c, errors.Wrap(err, "failed to process environment variables")
	}

	return c, nil
}

/*
Stack Left Top:
x: -0.3275 / -0.3335
y: 0.897 / 0.897
w: -0.069813

z: 1.18
then:
disable 0 and 2 for moveit, then move down 20 cm, then move up 20 cm, then enable
movot 0 and 2 for moveit


Conveyor belt:
x: -1.01
y: 0
w: 1.57

z: 1.38

then: drop

then:
y: -0.02

then:
z: -0.3

Stack right top:
x: 0.1425
y: 0.937
z: 1.28

w: -0.139626
*/
