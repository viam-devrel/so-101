import tailwindcss from '@tailwindcss/vite';
import { sveltekit } from '@sveltejs/kit/vite';
import { svelteTesting } from '@testing-library/svelte/vite';
// vitest/config re-exports vite's defineConfig with a `test` field added to its type -- the
// build/dev/preview paths (vite/vite-plugin-svelte/sveltekit) are unaffected.
import { defineConfig } from 'vitest/config';

export default defineConfig({
	// svelteTesting's config hook is a no-op unless process.env.VITEST is set (it switches
	// Svelte's resolve condition to 'browser' and wires up @testing-library/svelte's
	// auto-cleanup) -- harmless to include unconditionally.
	plugins: [tailwindcss(), sveltekit(), svelteTesting()],
	optimizeDeps: {
		include: ['@viamrobotics/sdk', '@viamrobotics/svelte-sdk', 'js-cookie']
	},
	define: {
		global: 'globalThis' // Required for Viam SDK browser compatibility
	},
	server: {
		fs: {
			allow: ['..'] // Allow access to node_modules
		}
	},
	test: {
		// Pure logic by default (fast); files touching window/sessionStorage/DOM opt into jsdom
		// with a `// @vitest-environment jsdom` docblock.
		environment: 'node',
		include: ['src/**/*.{test,spec}.ts']
	}
});
