package main

import (
	"github.com/kelseyhightower/envconfig"
	"github.com/pkg/errors"
)

type Config struct {
	GripperDevice string  `default:"/dev/ttyUSB0" split_words:"true"`
	DeltaXSearch  float64 `default:"0.1" split_words:"true"`
	DeltaYSearch  float64 `default:"0.1" split_words:"true"`
}

func ReadConfig() (Config, error) {
	var c Config
	err := envconfig.Process("RCS_GRIPPER", &c)
	if err != nil {
		return c, errors.Wrap(err, "failed to process environment variables")
	}

	return c, nil
}
