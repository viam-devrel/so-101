<!--
	Usage:
	<Icon name="warningTriangle" className="h-6 w-6 text-amber-600" />
	<Icon name="checkCircle" className="w-5 h-5 text-green-600" label="Success" />

	Fill vs. stroke is decided here from `ICONS[name].mode`, not by the caller, so an outline
	icon can't be rendered with fill (see icons.ts for why that silently produces a blob).
	Decorative by default (aria-hidden); pass `label` for a meaningful standalone icon.
-->
<script lang="ts">
	import { ICONS, type IconName } from '$lib/icons';

	interface Props {
		name: IconName;
		className?: string;
		label?: string;
	}

	let { name, className = 'w-5 h-5', label }: Props = $props();

	const icon = $derived(ICONS[name]);
</script>

{#if icon}
	<svg
		class={className}
		viewBox={icon.viewBox}
		fill={icon.mode === 'solid' ? 'currentColor' : 'none'}
		stroke={icon.mode === 'solid' ? undefined : 'currentColor'}
		role={label ? 'img' : undefined}
		aria-hidden={label ? undefined : 'true'}
		aria-label={label}
	>
		{#each icon.paths as d}
			{#if icon.mode === 'solid'}
				<path fill-rule="evenodd" clip-rule="evenodd" {d} />
			{:else}
				<path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" {d} />
			{/if}
		{/each}
	</svg>
{/if}
