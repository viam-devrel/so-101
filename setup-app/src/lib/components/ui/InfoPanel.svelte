<!--
	Usage: <InfoPanel tone="warning" title="Manual Positioning Required">
	         Manually move each joint to the center of its range of motion.
	       </InfoPanel>
	       <InfoPanel tone="safety" title="Important Safety Information">
	         <p class="font-medium">Before proceeding, please ensure:</p>
	         <ul class="list-disc list-inside space-y-1 text-sm">
	           <li>The arm is securely mounted</li>
	         </ul>
	       </InfoPanel>

	Consolidates the hand-rolled `bg-{color}-50` callout boxes repeated ~20x across the step
	components (instructions, warnings, danger notices, the safety checklist). `tone` picks a
	literal Tailwind class string per variant (see comment below) so Tailwind's scanner can find
	them; `icon` overrides the tone's default icon, and omitting both hides the icon.
	Children are freeform -- callers vary between prose, a `<ul>`, or a small grid, so the body
	is not prescribed beyond spacing.
-->
<script lang="ts">
	import Icon from './Icon.svelte';
	import type { IconName } from '$lib/icons';

	type Tone = 'info' | 'success' | 'warning' | 'danger' | 'safety';

	interface Props {
		tone?: Tone;
		title?: string;
		icon?: IconName | null;
		className?: string;
		children?: any;
	}

	let { tone = 'info', title, icon, className = '', children }: Props = $props();

	// Literal Tailwind strings only -- an interpolated `bg-${tone}-50` is invisible to the
	// Tailwind v4 scanner (see so-101 CLAUDE.md).
	const TONES: Record<
		Tone,
		{ container: string; icon: string; title: string; body: string; defaultIcon: IconName | null }
	> = {
		info: {
			container: 'bg-blue-50 border border-blue-200',
			icon: 'text-blue-600',
			title: 'text-blue-900',
			body: 'text-blue-800',
			defaultIcon: 'infoCircle'
		},
		success: {
			container: 'bg-green-50 border border-green-200',
			icon: 'text-green-600',
			title: 'text-green-900',
			body: 'text-green-800',
			defaultIcon: 'checkCircle'
		},
		warning: {
			container: 'bg-yellow-50 border border-yellow-200',
			icon: 'text-yellow-600',
			title: 'text-yellow-900',
			body: 'text-yellow-800',
			defaultIcon: 'warningTriangle'
		},
		danger: {
			container: 'bg-orange-50 border border-orange-200',
			icon: 'text-orange-600',
			title: 'text-orange-900',
			body: 'text-orange-800',
			defaultIcon: 'warningTriangleSolid'
		},
		safety: {
			container: 'bg-amber-50 border-l-4 border-amber-400',
			icon: 'text-amber-600',
			title: 'text-amber-900',
			body: 'text-amber-800',
			defaultIcon: 'warningTriangle'
		}
	};

	const config = $derived(TONES[tone]);
	const resolvedIcon = $derived(icon === undefined ? config.defaultIcon : icon);
</script>

<div class="{config.container} rounded-lg p-4 {className}">
	<div class="flex items-start">
		{#if resolvedIcon}
			<div class="flex-shrink-0">
				<Icon name={resolvedIcon} className="h-6 w-6 {config.icon}" />
			</div>
		{/if}
		<div class="{resolvedIcon ? 'ml-3' : 'w-full'} {config.body}">
			{#if title}
				<h4 class="font-medium {config.title} mb-2">{title}</h4>
			{/if}
			{@render children?.()}
		</div>
	</div>
</div>
