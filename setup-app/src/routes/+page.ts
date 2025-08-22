import type { PageLoad } from './$types';

export const trailingSlash = 'ignore';

export const load: PageLoad = ({ params, ...rest }) => {
	console.log({ params, rest });
	return {};
};
