// @vitest-environment jsdom
import { describe, it, expect, vi, beforeEach } from 'vitest';

const MACHINE_ID = 'test-machine';

vi.mock('$app/state', () => ({
	page: { url: new URL('http://localhost/') }
}));

vi.mock('$lib/utils/connection', () => ({
	parseConnectionFromCookies: vi.fn(() => ({
		connectionDetails: null,
		machineId: MACHINE_ID,
		error: null
	}))
}));

import { page } from '$app/state';
import { useWorkflowConfig } from './useWorkflowConfig';
import type { SensorConfig } from '$lib/types';

// The landing page (src/routes/+page.svelte) writes
// `sessionStorage.setItem(\`so101-setup-state-${machineId}\`, ...)`. This constant is that
// same derivation, spelled out independently -- if useWorkflowConfig's key ever drifts from
// it, restore silently breaks (this is the "mismatch silently broke restore" bug class).
const SESSION_KEY = `so101-setup-state-${MACHINE_ID}`;

const sensorConfig: SensorConfig = { partId: 'main', sensorName: 'my-sensor' };

function writeSessionRaw(body: Record<string, unknown>) {
	sessionStorage.setItem(SESSION_KEY, JSON.stringify(body));
}

function writeSession(config: SensorConfig, timestamp: number) {
	writeSessionRaw({ sensorConfig: config, completedWorkflows: [], timestamp });
}

beforeEach(() => {
	sessionStorage.clear();
	(page as { url: URL }).url = new URL('http://localhost/');
});

describe('getSensorConfigFromURL', () => {
	it('reads part+sensor from the URL', () => {
		(page as { url: URL }).url = new URL('http://localhost/?part=main&sensor=my-sensor');
		const { getSensorConfigFromURL } = useWorkflowConfig();
		expect(getSensorConfigFromURL()).toEqual(sensorConfig);
	});

	it('is null when sensor is missing', () => {
		(page as { url: URL }).url = new URL('http://localhost/?part=main');
		const { getSensorConfigFromURL } = useWorkflowConfig();
		expect(getSensorConfigFromURL()).toBeNull();
	});

	it('is null when part is missing', () => {
		(page as { url: URL }).url = new URL('http://localhost/?sensor=my-sensor');
		const { getSensorConfigFromURL } = useWorkflowConfig();
		expect(getSensorConfigFromURL()).toBeNull();
	});
});

describe('getSensorConfigFromSession', () => {
	it('round-trips a value written exactly the way the landing page writes it', () => {
		writeSession(sensorConfig, Date.now());
		const { getSensorConfigFromSession } = useWorkflowConfig();
		expect(getSensorConfigFromSession()).toEqual(sensorConfig);
	});

	it('is null with nothing stored', () => {
		const { getSensorConfigFromSession } = useWorkflowConfig();
		expect(getSensorConfigFromSession()).toBeNull();
	});

	it('is null and clears storage once older than 1 hour', () => {
		writeSession(sensorConfig, Date.now() - 3600001);
		const { getSensorConfigFromSession } = useWorkflowConfig();
		expect(getSensorConfigFromSession()).toBeNull();
		expect(sessionStorage.getItem(SESSION_KEY)).toBeNull();
	});

	it('is still valid one millisecond under the 1 hour boundary', () => {
		writeSession(sensorConfig, Date.now() - 3599999);
		const { getSensorConfigFromSession } = useWorkflowConfig();
		expect(getSensorConfigFromSession()).toEqual(sensorConfig);
	});

	it('treats a missing timestamp as expired, not immortal', () => {
		writeSessionRaw({ sensorConfig, completedWorkflows: [] });
		const { getSensorConfigFromSession } = useWorkflowConfig();
		expect(getSensorConfigFromSession()).toBeNull();
	});

	it('treats a non-numeric timestamp as expired', () => {
		writeSessionRaw({ sensorConfig, completedWorkflows: [], timestamp: 'yesterday' });
		const { getSensorConfigFromSession } = useWorkflowConfig();
		expect(getSensorConfigFromSession()).toBeNull();
	});

	it('treats a zero/negative timestamp as expired', () => {
		writeSessionRaw({ sensorConfig, completedWorkflows: [], timestamp: 0 });
		const { getSensorConfigFromSession } = useWorkflowConfig();
		expect(getSensorConfigFromSession()).toBeNull();
	});

	it('clears and returns null on unparsable JSON', () => {
		sessionStorage.setItem(SESSION_KEY, '{not json');
		const { getSensorConfigFromSession } = useWorkflowConfig();
		expect(getSensorConfigFromSession()).toBeNull();
		expect(sessionStorage.getItem(SESSION_KEY)).toBeNull();
	});
});

describe('initializeSensorConfig', () => {
	it('prefers URL over session', () => {
		writeSession({ partId: 'session', sensorName: 'session-sensor' }, Date.now());
		(page as { url: URL }).url = new URL('http://localhost/?part=url&sensor=url-sensor');
		const { initializeSensorConfig } = useWorkflowConfig();
		expect(initializeSensorConfig()).toEqual({
			sensorConfig: { partId: 'url', sensorName: 'url-sensor' },
			source: 'url'
		});
	});

	it('falls back to session when the URL has nothing', () => {
		writeSession({ partId: 'session', sensorName: 'session-sensor' }, Date.now());
		const { initializeSensorConfig } = useWorkflowConfig();
		expect(initializeSensorConfig()).toEqual({
			sensorConfig: { partId: 'session', sensorName: 'session-sensor' },
			source: 'session'
		});
	});

	it('is none when neither URL nor session has anything', () => {
		const { initializeSensorConfig } = useWorkflowConfig();
		expect(initializeSensorConfig()).toEqual({ sensorConfig: null, source: 'none' });
	});
});
