package main

import (
	"github.com/kelseyhightower/envconfig"
	"github.com/pkg/errors"
)

type Config struct {
	RelayGracePeriodSeconds    int    `default:"2" split_words:"true"`
	EmergencyRetryDelaySeconds int    `default:"3" split_words:"true"`
	WebsocketHost              string `default:"localhost" split_words:"true"`
	WebsocketPort              int    `default:"8082" split_words:"true"`
}

func ReadConfig() (Config, error) {
	var c Config
	err := envconfig.Process("RAS_INTERFACE", &c)
	if err != nil {
		return c, errors.Wrap(err, "failed to process environment variables")
	}

	return c, nil
}
