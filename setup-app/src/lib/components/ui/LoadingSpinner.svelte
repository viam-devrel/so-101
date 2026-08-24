<script lang="ts">
	// Usage: <LoadingSpinner size="sm" text="Saving..." />
	// The spinner graphic is decorative (aria-hidden); role="status" on the
	// wrapper announces `text` to screen readers when it changes.
	interface Props {
		size?: 'sm' | 'md' | 'lg';
		color?: 'blue' | 'green' | 'red' | 'gray' | 'white';
		text?: string;
		inline?: boolean;
		className?: string;
	}

	let { size = 'md', color = 'blue', text, inline = false, className = '' }: Props = $props();

	const sizeClasses = {
		sm: 'h-4 w-4',
		md: 'h-6 w-6',
		lg: 'h-12 w-12'
	};

	const colorClasses = {
		blue: 'border-blue-600',
		green: 'border-green-600',
		red: 'border-red-600',
		gray: 'border-gray-600',
		// For spinners sitting on a saturated button fill (primary/success/danger).
		white: 'border-white'
	};

	const spinnerClass = `animate-spin rounded-full border-b-2 ${sizeClasses[size]} ${colorClasses[color]} ${className}`;
</script>

<div class={inline ? 'inline-flex items-center' : 'flex items-center justify-center'} role="status">
	<div class={spinnerClass} aria-hidden="true"></div>
	{#if text}
		<span class="ml-2 text-gray-700">{text}</span>
	{:else}
		<span class="sr-only">Loading</span>
	{/if}
</div>
