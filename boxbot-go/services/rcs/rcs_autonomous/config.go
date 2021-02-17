package main

import (
	"github.com/kelseyhightower/envconfig"
	"github.com/pkg/errors"
)

type Config struct {
	EnableGripperTest        bool    `default:"false" split_words:"true"`
	EnableGripperTestHome    bool    `default:"false" split_words:"true"`
	GripperDevice            string  `default:"/dev/ttyUSB5" split_words:"true"`
	DeltaXSearch             float64 `default:"0.05" split_words:"true"`
	DeltaYSearch             float64 `default:"0.05" split_words:"true"`
	GripperHoverCartonHeight float64 `default:"0.03" split_words:"true"`
	GripHeight               float64 `default:"0.15" split_words:"true"`
}

func ReadConfig() (Config, error) {
	var c Config
	err := envconfig.Process("RCS_AUTONOMOUS", &c)
	if err != nil {
		return c, errors.Wrap(err, "failed to process environment variables")
	}

	return c, nil
}
