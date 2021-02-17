package main

import (
	"boxbot-go/pkg/srvs/srv_ras"
	"context"
	"encoding/json"
	"fmt"
	"github.com/gorilla/mux"
	"github.com/rs/cors"
	"net/http"
	"strconv"
	"time"
)

const (
	nodesQueryParam  = "node"
	levelsQueryParam = "level"
)

type GetFunc func(*srv_ras.GetLogReq) *srv_ras.GetLogRes
type DelFunc func(*srv_ras.DeleteLogReq) *srv_ras.DeleteLogRes

type LogWebApi struct {
	r   *mux.Router
	srv *http.Server

	getCb GetFunc
	delCb DelFunc
}

type logParams struct {
	from, to      int64
	nodes, levels []string
}

func NewLogWebApi(getCb GetFunc, delCb DelFunc) *LogWebApi {
	return &LogWebApi{
		getCb: getCb,
		delCb: delCb,
	}
}

func (l *LogWebApi) retrieveParams(w http.ResponseWriter, r *http.Request) (logParams, error) {
	if err := r.ParseForm(); err != nil {
		http.Error(w, "failed to parse form", http.StatusBadRequest)
		return logParams{}, err
	}

	from, err := strconv.Atoi(r.FormValue("from"))
	if err != nil {
		http.Error(w, "from query param has to be a number", http.StatusBadRequest)
		return logParams{}, err
	}

	to, err := strconv.Atoi(r.FormValue("to"))
	if err != nil {
		http.Error(w, "to query param has to be a number", http.StatusBadRequest)
		return logParams{}, err
	}

	return logParams{
		from:   int64(from),
		to:     int64(to),
		nodes:  r.Form[nodesQueryParam],
		levels: r.Form[levelsQueryParam],
	}, nil
}

func (l *LogWebApi) handleGetLogs(w http.ResponseWriter, r *http.Request) {
	p, err := l.retrieveParams(w, r)
	if err != nil {
		return
	}

	if l.getCb == nil {
		http.Error(w, "no defined get logging callback, this is a bug", http.StatusInternalServerError)
		return
	}

	res := l.getCb(&srv_ras.GetLogReq{
		From:   p.from,
		To:     p.to,
		Nodes:  p.nodes,
		Levels: p.levels,
	})

	if len(res.Error) > 0 {
		http.Error(w, fmt.Sprintf("error while calling service: %v", res.Error), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(res); err != nil {
		http.Error(w, fmt.Sprintf("unable to encode response: %v", err), http.StatusInternalServerError)
	}
}

func (l *LogWebApi) handleDelLogs(w http.ResponseWriter, r *http.Request) {
	p, err := l.retrieveParams(w, r)
	if err != nil {
		return
	}

	if l.delCb == nil {
		http.Error(w, "no defined del logging callback, this is a bug", http.StatusInternalServerError)
		return
	}

	res := l.delCb(&srv_ras.DeleteLogReq{
		From:   p.from,
		To:     p.to,
		Nodes:  p.nodes,
		Levels: p.levels,
	})

	if len(res.Error) > 0 {
		http.Error(w, fmt.Sprintf("error while calling service: %v", res.Error), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(res); err != nil {
		http.Error(w, fmt.Sprintf("unable to encode response: %v", err), http.StatusInternalServerError)
	}
}

func (l *LogWebApi) ListenAndServe(address, webSrc string) error {
	l.r = mux.NewRouter()

	l.r.HandleFunc("/logging/get", l.handleGetLogs).Methods(http.MethodGet)
	l.r.HandleFunc("/logging/del", l.handleDelLogs).Methods(http.MethodDelete)

	c := cors.New(cors.Options{
		AllowedOrigins:   []string{webSrc, address},
		AllowedMethods:   []string{http.MethodGet, http.MethodDelete, http.MethodOptions},
		AllowCredentials: true,
	}).Handler(l.r)

	l.srv = &http.Server{
		Addr:         address,
		WriteTimeout: time.Second * 15,
		ReadTimeout:  time.Second * 15,
		IdleTimeout:  time.Second * 60,
		Handler:      c,
	}

	return l.srv.ListenAndServe()
}

func (l *LogWebApi) Close() error {
	return l.srv.Shutdown(context.TODO())
}
