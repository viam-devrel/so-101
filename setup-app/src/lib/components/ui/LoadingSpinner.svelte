<script lang="ts">
	interface Props {
		size?: 'sm' | 'md' | 'lg';
		color?: 'blue' | 'green' | 'red' | 'gray';
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
		gray: 'border-gray-600'
	};

	const spinnerClass = `animate-spin rounded-full border-b-2 ${sizeClasses[size]} ${colorClasses[color]} ${className}`;
</script>

<div class={inline ? 'inline-flex items-center' : 'flex items-center justify-center'}>
	<div class={spinnerClass}></div>
	{#if text}
		<span class="ml-2 text-gray-700">{text}</span>
	{/if}
</div>
