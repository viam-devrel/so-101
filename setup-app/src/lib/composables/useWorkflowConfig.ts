import { page } from '$app/state';
import type { SensorConfig, SessionState } from '$lib/types';
import { logger } from '$lib/utils/logger';
import { parseConnectionFromCookies } from '$lib/utils/connection';

/**
 * Composable for managing workflow configuration across all workflow pages
 * Handles URL parameters and session storage consistently
 */
export function useWorkflowConfig() {
	/**
	 * Parse sensor configuration from URL search parameters
	 */
	function getSensorConfigFromURL(): SensorConfig | null {
		try {
			const urlParams = page.url.searchParams;
			const partId = urlParams.get('part');
			const sensorName = urlParams.get('sensor');

			if (partId && sensorName) {
				logger.debug('Found sensor config in URL parameters', { partId, sensorName });
				return { partId, sensorName };
			}

			logger.debug('No sensor config found in URL parameters');
			return null;
		} catch (error) {
			logger.error('Error parsing URL parameters', error as Error);
			return null;
		}
	}

	/**
	 * Parse sensor configuration from session storage
	 * Only returns valid data that's less than 1 hour old
	 */
	function getSensorConfigFromSession(): SensorConfig | null {
		// Declared out here so the catch below can still clear the entry.
		let sessionKey = '';
		try {
			if (typeof window === 'undefined') {
				return null; // SSR guard
			}

			// Inside the guard: parseConnectionFromCookies reads window.location.
			const { machineId } = parseConnectionFromCookies();
			sessionKey = `so101-setup-state-${machineId}`;

			const stored = sessionStorage.getItem(sessionKey);
			if (!stored) {
				logger.debug('No session storage data found');
				return null;
			}

			const sessionState: SessionState = JSON.parse(stored);

			// Check if data is less than 1 hour old. A missing or non-numeric timestamp counts
			// as expired, not immortal: `Date.now() - <non-number>` is NaN, and every NaN
			// comparison is false, so an age check alone would treat it as fresh forever.
			const oneHourMs = 3600000;
			const timestamp = sessionState.timestamp;
			const isExpired =
				typeof timestamp !== 'number' ||
				!Number.isFinite(timestamp) ||
				timestamp <= 0 ||
				Date.now() - timestamp > oneHourMs;

			if (isExpired) {
				logger.debug('Session storage data expired, clearing');
				sessionStorage.removeItem(sessionKey);
				return null;
			}

			if (sessionState.sensorConfig) {
				logger.debug('Found valid sensor config in session storage', sessionState.sensorConfig);
				return sessionState.sensorConfig;
			}

			return null;
		} catch (error) {
			logger.warn('Error parsing session storage data, clearing', error as Error);
			if (typeof window !== 'undefined' && sessionKey) {
				sessionStorage.removeItem(sessionKey);
			}
			return null;
		}
	}

	/**
	 * Initialize sensor configuration with fallback strategy:
	 * 1. Try URL parameters first
	 * 2. Fall back to session storage
	 * 3. Return null if neither available
	 */
	function initializeSensorConfig(): {
		sensorConfig: SensorConfig | null;
		source: 'url' | 'session' | 'none';
	} {
		// Try URL parameters first (highest priority)
		let config = getSensorConfigFromURL();
		if (config) {
			return { sensorConfig: config, source: 'url' };
		}

		// Fall back to session storage
		config = getSensorConfigFromSession();
		if (config) {
			return { sensorConfig: config, source: 'session' };
		}

		// No configuration available
		logger.warn('No sensor configuration available from URL or session storage');
		return { sensorConfig: null, source: 'none' };
	}

	return {
		getSensorConfigFromURL,
		getSensorConfigFromSession,
		initializeSensorConfig
	};
}
