package ws

type DisplayState struct {
	Error   bool   `json:"error"`
	Message string `json:"message"`
	Hint    string `json:"hint"`
}

type Notification struct {
	Id      uint32 `json:"id"`
	Message string `json:"message"`
}
