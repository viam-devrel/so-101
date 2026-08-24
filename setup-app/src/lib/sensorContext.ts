import { getContext, setContext } from 'svelte';
import type { SensorContext } from './types';

// Symbol key (not the string 'sensor') so a missing provider can't quietly resolve to some
// unrelated same-named context; combined with the guard below it turns a missing
// <SensorProvider> into a clear error instead of an unchecked-cast `undefined` typed as
// SensorContext that fails later with an opaque TypeError.
const SENSOR_CONTEXT_KEY = Symbol('sensor');

export function setSensorContext(context: SensorContext): void {
	setContext(SENSOR_CONTEXT_KEY, context);
}

// Call at component init only (getContext's own requirement) -- never inside $derived/$effect.
export function useSensor(): SensorContext {
	const context = getContext<SensorContext | undefined>(SENSOR_CONTEXT_KEY);
	if (!context) {
		throw new Error('useSensor() must be called inside <SensorProvider>');
	}
	return context;
}
