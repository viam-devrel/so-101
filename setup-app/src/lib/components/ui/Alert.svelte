<script lang="ts">
	interface Props {
		type?: 'info' | 'success' | 'warning' | 'error';
		variant?: 'info' | 'success' | 'warning' | 'error';
		title?: string;
		dismissible?: boolean;
		children?: any;
	}

	let { type, variant, title, dismissible = false, children }: Props = $props();

	// Support both 'type' and 'variant' props for flexibility
	const alertType = variant || type || 'info';
	let isVisible = $state(true);

	const typeConfig = {
		info: {
			containerClasses: 'bg-blue-50 border border-blue-200 text-blue-800',
			iconClasses: 'text-blue-600',
			titleClasses: 'text-blue-800',
			icon: 'M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z'
		},
		success: {
			containerClasses: 'bg-green-50 border border-green-200 text-green-800',
			iconClasses: 'text-green-600',
			titleClasses: 'text-green-800',
			icon: 'M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z'
		},
		warning: {
			containerClasses: 'bg-yellow-50 border border-yellow-200 text-yellow-800',
			iconClasses: 'text-yellow-600',
			titleClasses: 'text-yellow-800',
			icon: 'M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z'
		},
		error: {
			containerClasses: 'bg-red-50 border border-red-200 text-red-800',
			iconClasses: 'text-red-600',
			titleClasses: 'text-red-800',
			icon: 'M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7 4a1 1 0 11-2 0 1 1 0 012 0zm-1-9a1 1 0 00-1 1v4a1 1 0 102 0V6a1 1 0 00-1-1z'
		}
	};

	const config = typeConfig[alertType];

	function dismiss() {
		isVisible = false;
	}
</script>

{#if isVisible}
	<div class="rounded-md p-4 {config.containerClasses}">
		<div class="flex items-start">
			<svg class="w-5 h-5 {config.iconClasses} mt-0.5" fill="currentColor" viewBox="0 0 20 20">
				<path fill-rule="evenodd" d={config.icon} clip-rule="evenodd" />
			</svg>
			<div class="ml-3 flex-1">
				{#if title}
					<h3 class="text-sm font-medium {config.titleClasses} mb-2">
						{title}
					</h3>
				{/if}
				<div class="text-sm">
					{@render children?.()}
				</div>
			</div>
			{#if dismissible}
				<button
					onclick={dismiss}
					class="ml-3 -mx-1.5 -my-1.5 rounded-md p-1.5 hover:bg-black hover:bg-opacity-10 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-offset-transparent focus:ring-gray-600"
				>
					<span class="sr-only">Dismiss</span>
					<svg class="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
						<path
							fill-rule="evenodd"
							d="M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z"
							clip-rule="evenodd"
						/>
					</svg>
				</button>
			{/if}
		</div>
	</div>
{/if}
