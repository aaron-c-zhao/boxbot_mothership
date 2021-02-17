package main

import (
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	"boxbot-go/pkg/node"
	"boxbot-go/pkg/srvs/srv_ras"
	"bytes"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/dgraph-io/badger/v2"
	"github.com/pkg/errors"
	"strconv"
	"sync"
	"time"
)

const indexSeparatorChar = 46 // 46 == .
var (
	entryPrefix = []byte{0x42}
	indexPrefix = []byte{0x78}
	leasePrefix = []byte{0x54}
)

type logging struct {
	n              *goroslib.Node
	getSrv, delSrv *goroslib.ServiceProvider
	logSub         *goroslib.Subscriber

	c Config

	logger log.Logger

	api *LogWebApi

	close, done chan struct{}
	closeLock   sync.Once

	db  *badger.DB
	seq *badger.Sequence
}

func NewNode(c node.Config) (_ node.Node, err error) {
	n := &logging{
		close: make(chan struct{}, 1),
		done:  make(chan struct{}),
	}

	// Read config
	n.c, err = ReadConfig()
	if err != nil {
		return nil, errors.Wrap(err, "unable to read config")
	}

	// Create node
	if n.n, err = goroslib.NewNode(goroslib.NodeConf{
		MasterAddress: fmt.Sprintf("%v:%v", c.RosHost, c.RosPort),
		Name:          node.LoggingServiceName,
		Namespace:     node.RasNamespace,
	}); err != nil {
		return nil, errors.Wrap(err, "unable to create ros node")
	}

	// Create logger
	n.logger, err = log.NewLogger(n.n, node.LoggingServiceName)
	if err != nil {
		return nil, errors.Wrap(err, "unable to create logger")
	}

	// Create services
	if n.getSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_ras.GetLogServiceName,
		Srv:      &srv_ras.GetLogService{},
		Callback: n.getLogs,
	}); err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_ras.GetLogServiceName)
	}

	if n.delSrv, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n.n,
		Name:     srv_ras.DeleteLogServiceName,
		Srv:      &srv_ras.DeleteLogService{},
		Callback: n.deleteLogs,
	}); err != nil {
		return nil, errors.Wrapf(err, "unable to create %s service", srv_ras.DeleteLogServiceName)
	}

	// Create subscriber
	if n.logSub, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n.n,
		Topic:    msgs.LogTopic.Topic,
		Callback: n.handleLog,
	}); err != nil {
		return nil, errors.Wrapf(err, "unable to create %s subscriber", msgs.LogTopic.Topic)
	}

	// Open DB
	if err = n.openDB(); err != nil {
		return nil, errors.Wrap(err, "unable to open database")
	}

	// Start web API
	n.api = NewLogWebApi(n.getLogs, n.deleteLogs)
	go func() {
		//TODO(timanema): Error handling, and have web host address somewhere
		_ = n.api.ListenAndServe(fmt.Sprintf("%s:%d", n.c.ApiHost, n.c.ApiPort), "localhost:8080")
	}()

	// Start self checks
	go node.PingSelf(n.n, n.close, n.done, node.LoggingServiceName)

	return n, err
}

func (n *logging) openDB() (err error) {
	if n.db, err = badger.Open(badger.DefaultOptions("logging_db")); err != nil {
		return errors.Wrap(err, "unable to open logging_db")
	}

	if n.seq, err = n.db.GetSequence(leasePrefix, 500); err != nil {
		return errors.Wrap(err, "unable to get a id sequence lease")
	}

	return
}

func (n *logging) insertEntry(txn *badger.Txn, e *msgs.LogEntry) ([]byte, error) {
	next, err := n.seq.Next()
	if err != nil {
		return nil, errors.Wrap(err, "failed to get next id from sequence")
	}

	key := make([]byte, 9) // len of prefix + uint64
	key[0] = entryPrefix[0]
	binary.LittleEndian.PutUint64(key[1:], next)

	val, err := json.Marshal(e)
	if err != nil {
		return nil, errors.Wrap(err, "failed to marshal log entry")
	}

	if err := txn.Set(key, val); err != nil {
		return nil, errors.Wrap(err, "failed to set log_entry value")
	}

	return key, nil
}

func findSeparatorIndex(key []byte) int {
	// 11 is used, because at time of writing that was the minimum amount of bytes needed for prefix+date,
	// so it definitely will never be less.
	for i := 11; i < len(key); i++ {
		if key[i] == indexSeparatorChar {
			return i
		}
	}

	return -1
}

func (n *logging) insertSecondaryIndex(txn *badger.Txn, e *msgs.LogEntry, key []byte) error {
	timestamp := strconv.Itoa(int(e.Timestamp))

	// Secondary index is timestamp + . + key
	index := make([]byte, 0, len(timestamp)+10) // 10 = len of prefix+uint64+separator
	index = append(index, indexPrefix...)
	index = append(index, timestamp[:]...)
	index = append(index, indexSeparatorChar)
	index = append(index, key[:]...)

	return txn.Set(index, nil)
}

func (n *logging) handleLog(e *msgs.LogEntry) {
	e.Timestamp = time.Now().Unix()

	n.logger.LogLocalf(msgs.DEBUG, "adding log entry %v", e)
	err := n.db.Update(func(txn *badger.Txn) error {
		key, err := n.insertEntry(txn, e)
		if err != nil {
			return errors.Wrap(err, "failed to insert log data")
		}

		return errors.Wrap(n.insertSecondaryIndex(txn, e, key), "failed to insert index")
	})
	if err != nil {
		n.logger.LogLocalf(msgs.WARN, "failed to persist log entry: %v", err)
	}
}

func timestampIteration(txn *badger.Txn, from, to int, f func(k []byte) error) error {
	opts := badger.DefaultIteratorOptions
	opts.PrefetchValues = false
	it := txn.NewIterator(opts)
	defer it.Close()
	start := append(indexPrefix, []byte(strconv.Itoa(from))...)
	end := append(indexPrefix, []byte(strconv.Itoa(to+1))...)
	for it.Seek(start); it.ValidForPrefix(indexPrefix); it.Next() {
		k := it.Item().Key()

		if bytes.Compare(k, end) >= 0 {
			break
		}

		if err := f(k); err != nil {
			return err
		}
	}

	return nil
}

func (n *logging) timestampLookup(from, to int) ([][]byte, error) {
	res := make([][]byte, 0, 20)
	err := n.db.View(func(txn *badger.Txn) error {
		return timestampIteration(txn, from, to, func(k []byte) error {
			v := append([]byte(nil), k[findSeparatorIndex(k)+1:]...)
			res = append(res, v)

			return nil
		})
	})
	if err != nil {
		return nil, errors.Wrap(err, "failed to query log indexes")
	}

	return res, nil
}

func (n *logging) getLogs(r *srv_ras.GetLogReq) *srv_ras.GetLogRes {
	// Convert node slice to map for easy lookups
	nodes := make(map[string]struct{})
	for _, r := range r.Nodes {
		nodes[r] = struct{}{}
	}

	// Convert level slice to map for easy lookups
	levels := make(map[string]struct{})
	for _, l := range r.Levels {
		levels[l] = struct{}{}
	}

	ids, err := n.timestampLookup(int(r.From), int(r.To))
	if err != nil {
		msg := errors.Wrap(err, "failed to query database for indexes").Error()
		n.logger.Logf(msgs.WARN, msg)

		return &srv_ras.GetLogRes{
			Error: msg,
		}
	}

	res := make([]msgs.LogEntry, 0, len(ids))
	err = n.db.View(func(txn *badger.Txn) error {
		for _, key := range ids {
			item, err := txn.Get(key)
			if err != nil {
				return err
			}

			var e msgs.LogEntry
			if err := item.Value(func(val []byte) error {
				return json.Unmarshal(val, &e)
			}); err != nil {
				return err
			}

			// TODO(timanema): Check if this can go faster, but doubt it
			if _, ok := nodes[e.Node]; !ok {
				continue
			}

			if _, ok := levels[e.Level]; !ok {
				continue
			}

			res = append(res, e)
		}

		return nil
	})
	if err != nil {
		msg := errors.Wrap(err, "failed to query database for entries").Error()
		n.logger.Logf(msgs.WARN, msg)

		return &srv_ras.GetLogRes{
			Error: msg,
		}
	}

	return &srv_ras.GetLogRes{
		Result: res,
	}
}

func (n *logging) deleteLogs(r *srv_ras.DeleteLogReq) *srv_ras.DeleteLogRes {
	// Convert node slice to map for easy lookups
	nodes := make(map[string]struct{})
	for _, r := range r.Nodes {
		nodes[r] = struct{}{}
	}

	// Convert level slice to map for easy lookups
	levels := make(map[string]struct{})
	for _, l := range r.Levels {
		levels[l] = struct{}{}
	}

	deleted := 0
	err := n.db.Update(func(txn *badger.Txn) error {
		return timestampIteration(txn, int(r.From), int(r.To), func(k []byte) error {
			key := append([]byte(nil), k...)
			item, err := txn.Get(key[findSeparatorIndex(key)+1:])
			if err != nil {
				return err
			}

			var e msgs.LogEntry
			if err := item.Value(func(val []byte) error {
				return json.Unmarshal(val, &e)
			}); err != nil {
				return err
			}

			// TODO(timanema): Check if this can go faster, but doubt it
			if _, ok := nodes[e.Node]; !ok {
				return nil
			}

			if _, ok := levels[e.Level]; !ok {
				return nil
			}

			deleted += 1

			if err := txn.Delete(key); err != nil {
				return err
			}

			return txn.Delete(key[findSeparatorIndex(key)+1:])
		})
	})
	if err != nil {
		msg := errors.Wrap(err, "failed to query log indexes").Error()
		n.logger.Logf(msgs.WARN, msg)

		return &srv_ras.DeleteLogRes{
			Error: msg,
		}
	}

	return &srv_ras.DeleteLogRes{
		Deleted: uint32(deleted),
	}
}

func (n *logging) Done() <-chan struct{} {
	return n.done
}

func (n *logging) Close() (err error) {
	n.closeLock.Do(func() {
		if err = errors.Wrap(n.logger.Close(), "failed to close logger"); err != nil {
			return
		}
		if err = errors.Wrap(n.seq.Release(), "failed to release sequence"); err != nil {
			return
		}
		if err = errors.Wrap(n.db.Close(), "failed to close database"); err != nil {
			return
		}
		if err = errors.Wrapf(n.logSub.Close(), "failed to close %s subscriber", msgs.LogTopic.Topic); err != nil {
			return
		}
		if err = errors.Wrapf(n.getSrv.Close(), "failed to close %s service", srv_ras.GetLogServiceName); err != nil {
			return
		}
		if err = errors.Wrapf(n.delSrv.Close(), "failed to close %s service", srv_ras.DeleteLogServiceName); err != nil {
			return
		}
		if err = errors.Wrap(n.n.Close(), "failed to close ros node"); err != nil {
			return
		}

		close(n.close)
	})

	if err != nil {
		return err
	}

	return nil
}
