package log

import (
	"boxbot-go/pkg/msgs"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"io"
	"log"
)

type Logger interface {
	io.Closer
	Logf(level msgs.LogLevel, msg string, a ...interface{})
	LogLocalf(level msgs.LogLevel, msg string, a ...interface{})
}

type logger struct {
	pub  *goroslib.Publisher
	name string

	closed *atomic.Bool
}

func NewLogger(n *goroslib.Node, name string) (Logger, error) {
	var err error
	logger := &logger{
		name:   name,
		closed: atomic.NewBool(false),
	}

	logger.pub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n,
		Topic: msgs.LogTopic.Topic,
		Msg:   msgs.LogTopic.Msg,
	})
	if err != nil {
		return nil, errors.Wrapf(err, "unable to create %s publisher", msgs.LogTopic.Topic)
	}

	return logger, nil
}

func (l *logger) Logf(level msgs.LogLevel, msg string, a ...interface{}) {
	msg = fmt.Sprintf(msg, a...)

	if !l.closed.Load() {
		l.pub.Write(&msgs.LogEntry{
			Node:    l.name,
			Level:   level,
			Message: msg,
		})
	}

	l.LogLocalf(level, msg)
}

func (l *logger) LogLocalf(level msgs.LogLevel, msg string, a ...interface{}) {
	log.Printf("%v: %v\n", level, fmt.Sprintf(msg, a...))
}

func (l *logger) Close() error {
	l.closed.Store(true)
	return errors.Wrapf(l.pub.Close(), "failed to close %s publisher", msgs.LogTopic.Topic)
}
