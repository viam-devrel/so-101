<script lang="ts">
	import type { StepProps, MotorScanBusResponse } from '$lib/types';
	import { MOTOR_SETUP_ORDER } from '$lib/constants';
	import { Button, LoadingSpinner, Alert } from '$lib/components/ui';

	interface Props {
		sendCommand: StepProps['sendCommand'];
	}

	let { sendCommand }: Props = $props();

	let scanning = $state(false);
	let scanResult = $state<MotorScanBusResponse | null>(null);
	let scanError = $state<string | null>(null);

	// Lookup by target ID, not name: scan results are keyed by ID, and this is the mapping
	// the panel renders against. Sourced from MOTOR_SETUP_ORDER so the joint names/IDs live
	// in exactly one place.
	const EXPECTED_BY_ID = new Map(MOTOR_SETUP_ORDER.map((motor) => [motor.targetId, motor]));

	const expectedRows = $derived.by(() => {
		if (!scanResult) return [];
		const foundById = new Map(scanResult.servos.map((servo) => [servo.id, servo]));
		return [...EXPECTED_BY_ID.entries()]
			.sort(([a], [b]) => a - b)
			.map(([id, motor]) => ({ id, description: motor.description, found: foundById.get(id) }));
	});

	const unexpectedServos = $derived(
		scanResult ? scanResult.servos.filter((servo) => servo.status === 'unexpected') : []
	);

	const missingCount = $derived(expectedRows.filter((row) => !row.found).length);

	async function scanBus() {
		if (scanning) return; // Button disables itself, but guard the async gap too.
		scanning = true;
		scanError = null;
		try {
			const result = (await sendCommand({
				command: 'motor_setup_scan_bus'
			})) as MotorScanBusResponse;
			if (!result?.servos) {
				throw new Error(result?.error || 'Bus scan returned an unexpected response');
			}
			scanResult = result;
		} catch (error) {
			scanError = error instanceof Error ? error.message : 'Bus scan failed';
			scanResult = null;
		} finally {
			scanning = false;
		}
	}
</script>

<div class="bg-white border border-gray-200 rounded-lg p-4">
	<div class="flex items-center justify-between mb-2">
		<h5 class="font-medium text-gray-900">Bus Scan</h5>
		<Button onclick={scanBus} disabled={scanning} variant="secondary" size="sm">
			{#if scanning}
				<LoadingSpinner size="sm" className="mr-2" />
				Scanning...
			{:else}
				Scan Bus
			{/if}
		</Button>
	</div>

	<p class="text-xs text-gray-500 mb-3">
		Sweeps every possible servo ID (1-253) and reports what actually answers -- real evidence
		instead of guesswork. Each silent ID costs a timeout, so this can take several seconds on a
		sparse bus.
	</p>

	{#if scanning}
		<div class="text-sm text-gray-600 flex items-center">
			<LoadingSpinner size="sm" className="mr-2" />
			Scanning the bus, this can take several seconds...
		</div>
	{:else if scanError}
		<Alert variant="error" title="Scan failed">
			{scanError}
		</Alert>
	{:else if scanResult}
		<div class="space-y-3">
			<p class="text-sm text-gray-700">
				{scanResult.servos_found} servo{scanResult.servos_found === 1 ? '' : 's'} responded on the bus{#if scanResult.unexpected_count > 0},
					{scanResult.unexpected_count}
					at {scanResult.unexpected_count === 1 ? 'an unexpected ID' : 'unexpected IDs'}{/if}.
			</p>

			<div class="grid grid-cols-2 sm:grid-cols-3 gap-2">
				{#each expectedRows as row (row.id)}
					<div
						class="border rounded p-2 text-xs {row.found
							? 'bg-green-50 border-green-300 text-green-800'
							: 'bg-red-50 border-red-300 text-red-800'}"
					>
						<div class="font-medium">{row.description}</div>
						<div>ID {row.id}: {row.found ? row.found.model : 'missing'}</div>
					</div>
				{/each}
			</div>

			{#if unexpectedServos.length > 0}
				<Alert variant="warning" title="Servos at unexpected IDs">
					<ul class="space-y-1">
						{#each unexpectedServos as servo (servo.id)}
							<li>ID {servo.id} ({servo.model}) is not one of the expected IDs 1-6.</li>
						{/each}
					</ul>
					<p class="mt-2">
						A motor that never got assigned during motor setup still answers on its factory ID.
						Re-run motor setup for it with only that motor plugged in.
					</p>
				</Alert>
			{/if}

			{#if missingCount > 0}
				<!-- The likelier daisy-chain mistake than a stray ID: a sweep sees at most one
				     responder per ID, so two motors sharing an ID is indistinguishable from one
				     simply being absent. Say so, rather than leaving the red tiles unexplained. -->
				<Alert variant="warning" title="No servo answered on the red IDs">
					<p>
						Either that motor is unpowered or unplugged, or two motors were assigned the same ID
						during setup -- the bus only ever hears one responder per ID, so the duplicate hides and
						the other joint's ID goes silent. Re-run motor setup for the red joints with only that
						motor plugged in.
					</p>
				</Alert>
			{/if}

			{#if missingCount === 0 && unexpectedServos.length === 0}
				<p class="text-sm text-green-700">
					All {expectedRows.length} expected IDs responded and nothing else did.
				</p>
			{/if}
		</div>
	{/if}
</div>
