import tailwindcss from '@tailwindcss/vite';
import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig } from 'vite';

export default defineConfig({
  plugins: [tailwindcss(), sveltekit()],
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
  }
});
