package main

import (
	"github.com/kelseyhightower/envconfig"
	"github.com/pkg/errors"
)

type Config struct {
	ServoDevice       string  `default:"/dev/ttyACM0" split_words:"true"`
	ServoMotorID      int32   `default:"3" split_words:"true"`
	ServoSpeed        float64 `default:"5" split_words:"true"`
	ServoHomeLocation float64 `default:"90" split_words:"true"`
}

func ReadConfig() (Config, error) {
	var c Config
	err := envconfig.Process("RCS_SERVO", &c)
	if err != nil {
		return c, errors.Wrap(err, "failed to process environment variables")
	}

	return c, nil
}
