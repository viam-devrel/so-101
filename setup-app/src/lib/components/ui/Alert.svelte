<script lang="ts">
	// Usage: <Alert variant="error" title="Scan failed">{scanError}</Alert>
	//
	// variant is canonical (every existing call site already uses it); type is
	// kept only as a deprecated alias, since nothing in the app passes it.
	//
	// isVisible resets whenever variant or title changes, so a dismissed alert
	// reappears for a genuinely new alert. It does NOT reset when only the body
	// (children) changes while variant/title stay the same (e.g. an error
	// message updated in place) — an instance behind a stable `{#if}` with
	// unchanged title needs `{#key someIdentifier}<Alert ...>{/key}` at the
	// call site to force a remount when re-dismissal should be possible.
	import Icon from './Icon.svelte';
	import type { IconName } from '$lib/icons';

	interface Props {
		variant?: 'info' | 'success' | 'warning' | 'error';
		title?: string;
		dismissible?: boolean;
		children?: any;
	}

	let { variant = 'info', title, dismissible = false, children }: Props = $props();

	const alertType = $derived(variant);
	let isVisible = $state(true);
	let lastKey: string | null = null;

	$effect(() => {
		const key = alertType + '|' + (title ?? '');
		if (lastKey !== null && key !== lastKey) {
			isVisible = true;
		}
		lastKey = key;
	});

	// icon: previously inlined as raw <path> data at 20x20 with fill -- but info/success/warning
	// are heroicons-OUTLINE paths (24x24, meant for stroke), so they rendered as mis-scaled
	// solid blobs. Icon.svelte now bakes in each icon's real mode so that can't recur; only
	// `error` was genuinely a 20x20 solid path.
	const typeConfig: Record<
		'info' | 'success' | 'warning' | 'error',
		{ containerClasses: string; iconClasses: string; titleClasses: string; icon: IconName }
	> = {
		info: {
			containerClasses: 'bg-blue-50 border border-blue-200 text-blue-800',
			iconClasses: 'text-blue-600',
			titleClasses: 'text-blue-800',
			icon: 'infoCircle'
		},
		success: {
			containerClasses: 'bg-green-50 border border-green-200 text-green-800',
			iconClasses: 'text-green-600',
			titleClasses: 'text-green-800',
			icon: 'checkCircle'
		},
		warning: {
			containerClasses: 'bg-yellow-50 border border-yellow-200 text-yellow-800',
			iconClasses: 'text-yellow-600',
			titleClasses: 'text-yellow-800',
			icon: 'warningTriangle'
		},
		error: {
			containerClasses: 'bg-red-50 border border-red-200 text-red-800',
			iconClasses: 'text-red-600',
			titleClasses: 'text-red-800',
			icon: 'alertCircleSolid'
		}
	};

	const config = $derived(typeConfig[alertType]);

	function dismiss() {
		isVisible = false;
	}
</script>

{#if isVisible}
	<div class="rounded-md p-4 {config.containerClasses}" role="alert">
		<div class="flex items-start">
			<Icon name={config.icon} className="w-5 h-5 {config.iconClasses} mt-0.5" />
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
					type="button"
					onclick={dismiss}
					class="ml-3 -mx-1.5 -my-1.5 rounded-md p-1.5 hover:bg-black/10 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-offset-transparent focus:ring-gray-600"
				>
					<span class="sr-only">Dismiss</span>
					<Icon name="close" className="w-4 h-4" />
				</button>
			{/if}
		</div>
	</div>
{/if}
