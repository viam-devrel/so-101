import type { Logger } from '$lib/types';

type LogLevel = 'debug' | 'info' | 'warn' | 'error';

interface LogEntry {
	level: LogLevel;
	message: string;
	timestamp: string;
	data?: any;
}

class LoggerService implements Logger {
	private isDev = import.meta.env.DEV;
	private logs: LogEntry[] = [];
	private maxLogs = 1000; // Keep last 1000 logs in memory

	private log(level: LogLevel, message: string, data?: any): void {
		const timestamp = new Date().toISOString();
		const entry: LogEntry = {
			level,
			message,
			timestamp,
			data
		};

		// Store in memory for debugging
		this.logs.push(entry);
		if (this.logs.length > this.maxLogs) {
			this.logs.shift();
		}

		// Console output based on environment and level
		const formattedMessage = `[${level.toUpperCase()}] ${timestamp} - ${message}`;

		switch (level) {
			case 'debug':
				if (this.isDev) {
					console.log(formattedMessage, data || '');
				}
				break;
			case 'info':
				if (this.isDev) {
					console.info(formattedMessage, data || '');
				}
				break;
			case 'warn':
				console.warn(formattedMessage, data || '');
				break;
			case 'error':
				console.error(formattedMessage, data || '');
				// In production, you might want to send errors to a service like Sentry
				if (!this.isDev && typeof window !== 'undefined') {
					// Example: Send to error reporting service
					// this.reportError(message, data);
				}
				break;
		}
	}

	debug(message: string, data?: any): void {
		this.log('debug', message, data);
	}

	info(message: string, data?: any): void {
		this.log('info', message, data);
	}

	warn(message: string, data?: any): void {
		this.log('warn', message, data);
	}

	error(message: string, error?: Error): void {
		const errorData = error
			? {
					name: error.name,
					message: error.message,
					stack: error.stack
				}
			: undefined;
		this.log('error', message, errorData);
	}

	// Utility methods for debugging
	getLogs(): LogEntry[] {
		return [...this.logs];
	}

	getLogsForLevel(level: LogLevel): LogEntry[] {
		return this.logs.filter((log) => log.level === level);
	}

	clearLogs(): void {
		this.logs = [];
	}

	// For debugging in browser console
	exportLogs(): string {
		return JSON.stringify(this.logs, null, 2);
	}
}

// Export singleton instance
export const logger = new LoggerService();

// Named export for specific use cases
export { LoggerService };
