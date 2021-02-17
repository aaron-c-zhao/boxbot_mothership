package ws

import (
	"boxbot-go/pkg/log"
	"boxbot-go/pkg/msgs"
	"github.com/gorilla/websocket"
	"net/http"
	"sync"
)

var upgrader = websocket.Upgrader{CheckOrigin: func(r *http.Request) bool {
	return true
}}

type WebSocketServer struct {
	close chan struct{}

	latch        interface{}
	listeners    map[chan interface{}]struct{}
	listenersMut sync.RWMutex

	logger log.Logger
}

func New(logger log.Logger, initialValue interface{}) *WebSocketServer {
	return &WebSocketServer{
		close:        make(chan struct{}),
		listeners:    make(map[chan interface{}]struct{}),
		latch:        initialValue,
		listenersMut: sync.RWMutex{},
		logger:       logger,
	}
}

func (w *WebSocketServer) handleConnection(res http.ResponseWriter, req *http.Request) {
	conn, err := upgrader.Upgrade(res, req, nil)
	if err != nil {
		w.logger.Logf(msgs.WARN, "unable to upgrade websocket connection: %v", err)
		return
	}
	defer conn.Close()

	// Send latched message, if set
	if w.latch != nil {
		if err := conn.WriteJSON(w.latch); err != nil {
			w.logger.Logf(msgs.WARN, "unable to write to websocket: %v (attempted to write %v)", err, w.latch)
			return
		}
	}

	msg := make(chan interface{}, 5)
	w.listenersMut.Lock()
	w.listeners[msg] = struct{}{}
	w.listenersMut.Unlock()

close:
	for {
		select {
		case <-w.close:
			break close
		case m := <-msg:
			if m == nil {
				break close
			}

			if err := conn.WriteJSON(m); err != nil {
				w.logger.Logf(msgs.WARN, "unable to write to websocket: %v (attempted to write %v)", err, m)
				break close
			}
		}
	}

	w.listenersMut.Lock()
	delete(w.listeners, msg)
	w.listenersMut.Unlock()
}

func (w *WebSocketServer) ListenAndServe(address string) error {
	http.HandleFunc("/", w.handleConnection)
	return http.ListenAndServe(address, nil)
}

func (w *WebSocketServer) Publish(msg interface{}) {
	w.listenersMut.RLock()
	defer w.listenersMut.RUnlock()

	for c := range w.listeners {
		select {
		case c <- msg:
		default:
		}
	}

	w.latch = msg
}

func (w *WebSocketServer) Close() error {
	close(w.close)
	return nil
}
