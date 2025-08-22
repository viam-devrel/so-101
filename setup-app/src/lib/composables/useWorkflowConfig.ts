import { page } from '$app/state';
import type { SensorConfig, SessionState } from '$lib/types';
import { logger } from '$lib/utils/logger';

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
		try {
			if (typeof window === 'undefined') {
				return null; // SSR guard
			}

			const stored = sessionStorage.getItem('so101-setup-state');
			if (!stored) {
				logger.debug('No session storage data found');
				return null;
			}

			const sessionState: SessionState = JSON.parse(stored);

			// Check if data is less than 1 hour old
			const oneHourMs = 3600000;
			const isExpired = sessionState.timestamp && Date.now() - sessionState.timestamp > oneHourMs;

			if (isExpired) {
				logger.debug('Session storage data expired, clearing');
				sessionStorage.removeItem('so101-setup-state');
				return null;
			}

			if (sessionState.sensorConfig) {
				logger.debug('Found valid sensor config in session storage', sessionState.sensorConfig);
				return sessionState.sensorConfig;
			}

			return null;
		} catch (error) {
			logger.warn('Error parsing session storage data, clearing', error as Error);
			if (typeof window !== 'undefined') {
				sessionStorage.removeItem('so101-setup-state');
			}
			return null;
		}
	}

	/**
	 * Save sensor configuration to session storage
	 */
	function saveSensorConfigToSession(
		sensorConfig: SensorConfig,
		completedWorkflows: string[] = []
	): void {
		try {
			if (typeof window === 'undefined') {
				return; // SSR guard
			}

			const sessionState: SessionState = {
				sensorConfig,
				completedWorkflows,
				timestamp: Date.now()
			};

			sessionStorage.setItem('so101-setup-state', JSON.stringify(sessionState));
			logger.debug('Saved sensor config to session storage', sessionState);
		} catch (error) {
			logger.error('Failed to save to session storage', error as Error);
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

	/**
	 * Update session storage with completed workflow
	 */
	function markWorkflowCompleted(workflowType: string): void {
		try {
			if (typeof window === 'undefined') {
				return;
			}

			const stored = sessionStorage.getItem('so101-setup-state');
			if (!stored) {
				return;
			}

			const sessionState: SessionState = JSON.parse(stored);
			const completedWorkflows = sessionState.completedWorkflows || [];

			if (!completedWorkflows.includes(workflowType)) {
				completedWorkflows.push(workflowType);
				sessionState.completedWorkflows = completedWorkflows;
				sessionState.timestamp = Date.now();

				sessionStorage.setItem('so101-setup-state', JSON.stringify(sessionState));
				logger.info('Marked workflow as completed', { workflowType, completedWorkflows });
			}
		} catch (error) {
			logger.error('Failed to mark workflow as completed', error as Error);
		}
	}

	/**
	 * Clear all session data
	 */
	function clearSessionData(): void {
		try {
			if (typeof window !== 'undefined') {
				sessionStorage.removeItem('so101-setup-state');
				logger.info('Cleared session storage data');
			}
		} catch (error) {
			logger.error('Failed to clear session data', error as Error);
		}
	}

	/**
	 * Get list of completed workflows from session
	 */
	function getCompletedWorkflows(): string[] {
		try {
			if (typeof window === 'undefined') {
				return [];
			}

			const stored = sessionStorage.getItem('so101-setup-state');
			if (!stored) {
				return [];
			}

			const sessionState: SessionState = JSON.parse(stored);
			return sessionState.completedWorkflows || [];
		} catch (error) {
			logger.warn('Failed to get completed workflows', error as Error);
			return [];
		}
	}

	return {
		getSensorConfigFromURL,
		getSensorConfigFromSession,
		saveSensorConfigToSession,
		initializeSensorConfig,
		markWorkflowCompleted,
		clearSessionData,
		getCompletedWorkflows
	};
}
