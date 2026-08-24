<!--
	Usage: <StatCard value={5} label="Motors calibrated" sublabel="of 6 total" />

	Matches StepComplete.svelte's summary-stat markup (`bg-white p-4 rounded-lg` + a big number
	over a label over an optional sub-label). `tone` colors the value to match whichever InfoPanel
	tone the card sits inside (StepComplete's only use today is green, inside its green summary).
-->
<script lang="ts">
	type Tone = 'green' | 'blue' | 'gray';

	interface Props {
		value: string | number;
		label: string;
		sublabel?: string;
		tone?: Tone;
		className?: string;
	}

	let { value, label, sublabel, tone = 'green', className = '' }: Props = $props();

	const VALUE_TONES: Record<Tone, string> = {
		green: 'text-green-600',
		blue: 'text-blue-600',
		gray: 'text-gray-900'
	};

	const valueClass = $derived(VALUE_TONES[tone]);
</script>

<div class="bg-white p-4 rounded-lg {className}">
	<div class="text-2xl font-bold {valueClass} mb-2">{value}</div>
	<div class="text-sm text-gray-600">{label}</div>
	{#if sublabel}
		<div class="text-xs text-gray-500 mt-1">{sublabel}</div>
	{/if}
</div>
