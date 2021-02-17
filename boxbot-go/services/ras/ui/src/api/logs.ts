export enum LogLevel {
    ERROR = "ERROR",
    WARNING = "WARNING",
    INFO = "INFO",
    DEBUG = "DEBUG",
    TRACE = "TRACE",
}

export type LogEntry = {
    id: number;
    timestamp: Date;
    level: LogLevel;
    message: string;
};
