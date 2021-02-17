package main

import (
	"github.com/kelseyhightower/envconfig"
	"github.com/pkg/errors"
)

type Config struct {
	EnableShutdown        bool `default:"false" split_words:"true"`
	RCSInitTimeoutSeconds int  `default:"60" split_words:"true"`
}

func ReadConfig() (Config, error) {
	var c Config
	err := envconfig.Process("RAS_BASE", &c)
	if err != nil {
		return c, errors.Wrap(err, "failed to process environment variables")
	}

	return c, nil
}
