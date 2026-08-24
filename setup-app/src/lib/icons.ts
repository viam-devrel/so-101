// Shared icon path data. Consolidates the SVG paths that were duplicated (2-14x each) across
// step components and routes. Render via `Icon.svelte`, never inline -- that component picks
// fill vs. stroke from `mode`, so a caller can't mismatch them (heroicons "outline" paths are
// hairline strokes with zero fill-area; filling one with fill-rule=evenodd silently drops the
// thin interior marks -- e.g. the exclamation mark or checkmark -- leaving a blob).
//
// Usage: import { ICONS, type IconName } from '$lib/icons'; then <Icon name="warningTriangle" />

export type IconMode = 'outline' | 'solid';

export interface IconDef {
	viewBox: string;
	mode: IconMode;
	paths: string[];
}

export const ICONS = {
	// Outline (heroicons v1 outline convention: fill="none" stroke="currentColor")
	warningTriangle: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: [
			'M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z'
		]
	},
	checkCircle: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: ['M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z']
	},
	infoCircle: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: ['M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z']
	},
	gear: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: [
			'M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z M15 12a3 3 0 11-6 0 3 3 0 016 0z'
		]
	},
	arrowDown: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: ['M19 14l-7 7m0 0l-7-7m7 7V3']
	},
	targetLocation: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: [
			'M17.657 16.657L13.414 20.9a1.998 1.998 0 01-2.827 0l-4.244-4.243a8 8 0 1111.314 0z',
			'M15 11a3 3 0 11-6 0 3 3 0 016 0z'
		]
	},
	download: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: [
			'M8 7H5a2 2 0 00-2 2v9a2 2 0 002 2h14a2 2 0 002-2V9a2 2 0 00-2-2h-3m-1 4l-3-3m0 0l-3 3m3-3v12'
		]
	},
	clock: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: ['M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z']
	},
	search: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: ['M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z']
	},
	refresh: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: [
			'M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15'
		]
	},
	barChart: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: [
			'M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z'
		]
	},
	arrowLeft: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: ['M10 19l-7-7m0 0l7-7m-7 7h18']
	},
	clipboardList: {
		viewBox: '0 0 24 24',
		mode: 'outline',
		paths: [
			'M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2m-3 7h3m-3 4h3m-6-4h.01M9 16h.01'
		]
	},

	// Solid (heroicons v1 solid convention: fill="currentColor", fill-rule="evenodd")
	warningTriangleSolid: {
		viewBox: '0 0 20 20',
		mode: 'solid',
		paths: [
			'M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l6.518 11.59c.75 1.334-.213 2.98-1.742 2.98H3.48c-1.53 0-2.492-1.646-1.743-2.98l6.52-11.59zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z'
		]
	},
	checkSmall: {
		viewBox: '0 0 20 20',
		mode: 'solid',
		paths: [
			'M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z'
		]
	},
	arrowRight: {
		viewBox: '0 0 20 20',
		mode: 'solid',
		paths: [
			'M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z'
		]
	},
	xCircleSolid: {
		viewBox: '0 0 20 20',
		mode: 'solid',
		paths: [
			'M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z'
		]
	},
	alertCircleSolid: {
		viewBox: '0 0 20 20',
		mode: 'solid',
		paths: [
			'M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7 4a1 1 0 11-2 0 1 1 0 012 0zm-1-9a1 1 0 00-1 1v4a1 1 0 102 0V6a1 1 0 00-1-1z'
		]
	},
	close: {
		viewBox: '0 0 20 20',
		mode: 'solid',
		paths: [
			'M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z'
		]
	},
	play: {
		viewBox: '0 0 20 20',
		mode: 'solid',
		paths: [
			'M10 18a8 8 0 100-16 8 8 0 000 16zM9.555 7.168A1 1 0 008 8v4a1 1 0 001.555.832l3-2a1 1 0 000-1.664l-3-2z'
		]
	},
	stop: {
		viewBox: '0 0 20 20',
		mode: 'solid',
		paths: [
			'M10 18a8 8 0 100-16 8 8 0 000 16zM8 7a1 1 0 00-1 1v4a1 1 0 001 1h4a1 1 0 001-1V8a1 1 0 00-1-1H8z'
		]
	}
} satisfies Record<string, IconDef>;

export type IconName = keyof typeof ICONS;
