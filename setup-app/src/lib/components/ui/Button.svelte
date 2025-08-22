<script lang="ts">
	import LoadingSpinner from './LoadingSpinner.svelte';

	interface Props {
		variant?: 'primary' | 'secondary' | 'success' | 'danger' | 'ghost';
		size?: 'sm' | 'md' | 'lg';
		disabled?: boolean;
		loading?: boolean;
		fullWidth?: boolean;
		type?: 'button' | 'submit' | 'reset';
		onclick?: () => void;
		className?: string;
		children?: any;
	}

	let {
		variant = 'primary',
		size = 'md',
		disabled = false,
		loading = false,
		fullWidth = false,
		type = 'button',
		onclick,
		className = '',
		children
	}: Props = $props();

	const baseClasses =
		'inline-flex items-center justify-center border font-medium rounded-md focus:outline-none focus:ring-2 focus:ring-offset-2 transition-colors duration-200 disabled:opacity-50 disabled:cursor-not-allowed';

	const variantClasses = {
		primary: 'border-transparent text-white bg-blue-600 hover:bg-blue-700 focus:ring-blue-500',
		secondary: 'border-gray-300 text-gray-700 bg-white hover:bg-gray-50 focus:ring-blue-500',
		success: 'border-transparent text-white bg-green-600 hover:bg-green-700 focus:ring-green-500',
		danger: 'border-transparent text-white bg-red-600 hover:bg-red-700 focus:ring-red-500',
		ghost: 'border-transparent text-gray-700 bg-transparent hover:bg-gray-100 focus:ring-gray-500'
	};

	const sizeClasses = {
		sm: 'px-3 py-2 text-sm',
		md: 'px-4 py-2 text-sm',
		lg: 'px-6 py-3 text-base'
	};

	const buttonClass = `${baseClasses} ${variantClasses[variant]} ${sizeClasses[size]} ${fullWidth ? 'w-full' : ''} ${className}`;

	const isDisabled = disabled || loading;

	function handleClick() {
		if (!isDisabled && onclick) {
			onclick();
		}
	}
</script>

<button {type} class={buttonClass} disabled={isDisabled} onclick={handleClick}>
	{#if loading}
		<LoadingSpinner
			size="sm"
			color={variant === 'primary' || variant === 'success' || variant === 'danger'
				? 'gray'
				: 'blue'}
			inline
		/>
		<span class="ml-2">
			{@render children?.()}
		</span>
	{:else}
		{@render children?.()}
	{/if}
</button>
