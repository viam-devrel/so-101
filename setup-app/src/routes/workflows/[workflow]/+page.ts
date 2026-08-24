import type { EntryGenerator } from './$types';

// The three workflow values are a closed, known set (WorkflowType in $lib/types) -- list
// them so adapter-static's prerenderer can emit each path at build time instead of needing
// to crawl an <a href> that discovers them (the app only reaches these routes via goto()).
// An out-of-set value is handled at runtime in +page.svelte, not rejected here.
export const entries: EntryGenerator = () => [
	{ workflow: 'motor-setup' },
	{ workflow: 'calibration' },
	{ workflow: 'full-setup' }
];
