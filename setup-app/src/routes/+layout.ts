import type { LayoutLoad } from './$types';

// Disable SSR and prerendering for client-side only operation
export const ssr = false;
export const prerender = false;
export const trailingSlash = 'ignore';

export const load: LayoutLoad = (event) => {
	console.log({ event });
	return {};
};
