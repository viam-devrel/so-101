<script lang="ts">
	import { goto } from '$app/navigation';
	import type { StepProps, CalibrationReadings, WorkflowType } from '$lib/types';
	import { getMachineRootPath } from '$lib/utils/connection';
	import { Button, Icon, InfoPanel, StatCard } from '$lib/components/ui';
	import type { IconName } from '$lib/icons';

	interface Props extends StepProps {
		workflowType?: WorkflowType;
	}

	let {
		sensorReadings,
		motorSetupResults,
		workflowType = 'full-setup',
		torqueOutcome
	}: Props = $props();

	// Get current sensor readings for final status
	const readings = $derived(sensorReadings.current.data as CalibrationReadings | undefined);
	const joints = $derived(readings?.joints || {});

	// Calculate completion statistics
	// "configured" (not "discovered") is the terminal step for a motor setup entry.
	const totalMotorsConfigured = $derived(
		Object.values(motorSetupResults).filter((result) => result.step === 'configured').length
	);
	const totalJointsCalibrated = $derived(Object.keys(joints).length);
	const completedJoints = $derived(
		Object.values(joints).filter((joint) => joint.is_completed).length
	);

	// Format current time for completion timestamp
	const completionTime = new Date().toLocaleString();

	// Per-workflow stat card content, keyed by the stat itself so the workflow map below
	// only needs to list which stats apply.
	const STATS = {
		motors: { label: 'Motors Configured', description: 'IDs assigned and baudrates set' },
		joints: { label: 'Joints Calibrated', description: 'Homing and ranges recorded' },
		ranges: { label: 'Complete Ranges', description: 'Full motion coverage achieved' }
	} as const;
	type StatKey = keyof typeof STATS;

	const statValues: Record<StatKey, number> = $derived({
		motors: totalMotorsConfigured,
		joints: totalJointsCalibrated,
		ranges: completedJoints
	});

	// Per-workflow "What Was Accomplished" content, keyed by section.
	interface Accomplishment {
		title: string;
		iconColor: string;
		icon?: IconName;
		items: string[];
	}
	const ACCOMPLISHMENTS: Record<'motor' | 'calibration', Accomplishment> = {
		motor: {
			title: 'Motor Configuration',
			iconColor: 'text-blue-600',
			icon: 'gear' as const,
			// Phrased around what verification proved, not around how many motors this wizard
			// configured: a motor that was already set up can be skipped, so the "Motors
			// Configured" stat above is legitimately below 6 while all six still answer.
			items: [
				'✅ All 6 servo motors responding at their assigned IDs',
				'✅ Communication baudrate set to 1,000,000 bps',
				'✅ Motor chain connectivity verified',
				'✅ Hardware communication established'
			]
		},
		calibration: {
			title: 'Calibration Process',
			iconColor: 'text-green-600',
			icon: 'barChart' as const,
			items: [
				'✅ Homing positions set for all joints',
				'✅ Full range of motion recorded',
				'✅ Joint limits saved to servo memory',
				'✅ Configuration file created and saved'
			]
		}
	} as const;
	type AccomplishmentKey = keyof typeof ACCOMPLISHMENTS;

	// Shared "next steps" entries. Torque is only touched by the calibration workflow
	// (components/calibration/workflow.go), so motor-setup must not claim it was re-enabled.
	// Which of these two renders is decided by torqueOutcome (set by StepCalibrationSave via
	// wizard state) in `content` below -- never claim torque is on when the save reported it
	// off.
	const NEXT_STEP_TORQUE_OK = {
		title: 'Motors Re-enabled',
		body: 'Servo holding torque has been restored. The arm will maintain its position and be ready for controlled movement.'
	};
	const NEXT_STEP_TORQUE_FAILED = {
		title: 'Torque Not Restored',
		body: 'The arm is still limp -- holding torque could not be re-enabled after saving. Support the arm before letting go, then power-cycle the servos or restart the machine to restore holding torque.'
	};
	const NEXT_STEP_INTEGRATION = {
		title: 'Viam Integration Ready',
		body: 'Your robot config now includes properly calibrated arm and gripper components that can be controlled through the Viam platform.'
	};
	const NEXT_STEP_PROGRAMMING = {
		title: 'Programming and Control',
		body: 'Use the Viam SDK (Python, TypeScript, Go, C++) or the Viam app control interface to program and operate your arm.'
	};
	// Replaces the surrounding "your arm is ready" copy when torque failed -- it would
	// contradict the NEXT_STEP_TORQUE_FAILED warning it frames.
	const TORQUE_FAILED_COPY = {
		nextStepsTitle: 'Before You Use the Arm',
		closingTitle: '⚠️ Calibration Saved — Arm Is Limp',
		closing:
			'The calibration itself is saved, but holding torque could not be re-enabled. Support the arm before letting go, then power-cycle the servos or restart the machine before controlling it.'
	};

	// Per-workflow content: which stats/accomplishments apply, and the headline copy.
	const WORKFLOW_CONTENT: Record<
		WorkflowType,
		{
			headline: string;
			subtitle: string;
			stats: StatKey[];
			accomplishments: AccomplishmentKey[];
			nextStepsTitle: string;
			nextSteps: { title: string; body: string }[];
			closingTitle: string;
			closing: string;
		}
	> = {
		'motor-setup': {
			headline: '🎉 Motors Configured!',
			subtitle: "Your SO-101 robotic arm's servo motors have been successfully configured.",
			stats: ['motors'],
			accomplishments: ['motor'],
			nextStepsTitle: 'What Comes Next',
			nextSteps: [
				{
					title: 'Motor IDs Assigned',
					body: 'Every servo now has the ID and 1,000,000 bps baudrate the arm and gripper components expect.'
				},
				{
					title: 'Calibration Still Needed',
					body: "Run the calibration workflow next to set homing positions and record each joint's range of motion. The arm is not ready for controlled movement until that is done."
				}
			],
			closingTitle: '🔧 Motors Ready — Calibration Next',
			closing:
				"You've successfully configured the motors. Run the calibration workflow next to record joint limits before controlling the arm."
		},
		calibration: {
			headline: '🎉 Calibration Complete!',
			subtitle: 'Your SO-101 robotic arm has been successfully calibrated.',
			stats: ['joints', 'ranges'],
			accomplishments: ['calibration'],
			nextStepsTitle: 'Your Arm is Ready!',
			nextSteps: [NEXT_STEP_TORQUE_OK, NEXT_STEP_INTEGRATION, NEXT_STEP_PROGRAMMING], // overridden by `content` below
			closingTitle: '🚀 Your SO-101 is Ready for Action!',
			closing:
				"You've successfully completed the calibration process. Your robotic arm is now calibrated and ready for precise control through the Viam platform."
		},
		'full-setup': {
			headline: '🎉 Setup Complete!',
			subtitle: 'Your SO-101 robotic arm has been successfully configured and calibrated.',
			stats: ['motors', 'joints', 'ranges'],
			accomplishments: ['motor', 'calibration'],
			nextStepsTitle: 'Your Arm is Ready!',
			nextSteps: [NEXT_STEP_TORQUE_OK, NEXT_STEP_INTEGRATION, NEXT_STEP_PROGRAMMING], // overridden by `content` below
			closingTitle: '🚀 Your SO-101 is Ready for Action!',
			closing:
				"You've successfully completed the setup process. Your robotic arm is now calibrated, configured, and ready for precise control through the Viam platform."
		}
	};

	// motor-setup never touches torque, so its nextSteps stand as declared above. For the other
	// two, substitute the real torque outcome for the placeholder -- and, when it failed, the
	// surrounding copy too. Default to the FAILED (cautious) copy on a still-null outcome rather
	// than assuming success, since this step is only reachable after calibration_save reports
	// success (see wizardProgress.ts / CalibrationWizard's stepCompletion), at which point
	// torqueOutcome is always already set.
	const content = $derived.by(() => {
		const base = WORKFLOW_CONTENT[workflowType];
		if (workflowType === 'motor-setup') return base;
		if (torqueOutcome?.enabled === true) {
			return {
				...base,
				nextSteps: [NEXT_STEP_TORQUE_OK, NEXT_STEP_INTEGRATION, NEXT_STEP_PROGRAMMING]
			};
		}
		return {
			...base,
			nextSteps: [NEXT_STEP_TORQUE_FAILED, NEXT_STEP_INTEGRATION, NEXT_STEP_PROGRAMMING],
			...TORQUE_FAILED_COPY
		};
	});

	// Static literal classes so Tailwind's scanner can find them (it cannot see
	// `md:grid-cols-{expr}`; those utilities only exist today because other files
	// happen to spell them out).
	const statsGridCols = $derived(
		{ 1: 'md:grid-cols-1', 2: 'md:grid-cols-2', 3: 'md:grid-cols-3' }[content.stats.length] ??
			'md:grid-cols-3'
	);
	const accomplishmentsGridCols = $derived(
		content.accomplishments.length > 1 ? 'md:grid-cols-2' : 'md:grid-cols-1'
	);
</script>

<div class="max-w-4xl mx-auto text-center">
	<!-- Success Header -->
	<div class="mb-8">
		<div class="w-24 h-24 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-6">
			<Icon name="checkCircle" className="w-12 h-12 text-green-600" />
		</div>
		<h3 class="text-3xl font-bold text-gray-900 mb-4">{content.headline}</h3>
		<p class="text-xl text-gray-600">
			{content.subtitle}
		</p>
	</div>

	<!-- Completion Summary -->
	<InfoPanel tone="success" icon={null} title="Setup Summary" className="mb-8">
		<div class="grid {statsGridCols} gap-6 text-center">
			{#each content.stats as statKey}
				<StatCard
					value={statValues[statKey]}
					label={STATS[statKey].label}
					sublabel={STATS[statKey].description}
					tone="green"
				/>
			{/each}
		</div>
		<div class="mt-4 text-xs text-gray-600">
			Completed: {completionTime}
		</div>
	</InfoPanel>

	<!-- What Was Accomplished -->
	<div class="bg-white border border-gray-200 rounded-lg p-6 mb-8 text-left">
		<h4 class="text-lg font-semibold text-gray-900 mb-4 text-center">What Was Accomplished</h4>

		<div class="grid {accomplishmentsGridCols} gap-6">
			{#each content.accomplishments as accomplishmentKey}
				{@const accomplishment = ACCOMPLISHMENTS[accomplishmentKey]}
				<div>
					<h5 class="font-medium text-gray-800 mb-3 flex items-center">
						{#if accomplishment.icon}
							<Icon
								name={accomplishment.icon}
								className="w-5 h-5 {accomplishment.iconColor} mr-2"
							/>
						{/if}
						{accomplishment.title}
					</h5>
					<ul class="text-sm text-gray-600 space-y-1">
						{#each accomplishment.items as item}
							<li>{item}</li>
						{/each}
					</ul>
				</div>
			{/each}
		</div>
	</div>

	<!-- Next Steps -->
	<div class="bg-blue-50 border border-blue-200 rounded-lg p-6 mb-8 text-left">
		<h4 class="text-lg font-semibold text-blue-900 mb-4 text-center">{content.nextStepsTitle}</h4>

		<div class="space-y-4">
			{#each content.nextSteps as nextStep, index}
				<div class="flex items-start">
					<div class="flex-shrink-0">
						<div
							class="w-6 h-6 bg-blue-600 text-white rounded-full flex items-center justify-center text-sm font-medium"
						>
							{index + 1}
						</div>
					</div>
					<div class="ml-3">
						<h5 class="font-medium text-blue-900">{nextStep.title}</h5>
						<p class="text-blue-800 text-sm">
							{nextStep.body}
						</p>
					</div>
				</div>
			{/each}
		</div>
	</div>

	<!-- Resources and Actions -->
	<div class="space-y-6">
		<!-- Resources -->
		<div class="bg-gray-50 p-6 rounded-lg">
			<h4 class="text-lg font-semibold text-gray-900 mb-4">Helpful Resources</h4>
			<div class="grid md:grid-cols-3 gap-4">
				<a
					href="https://docs.viam.com/components/arm/"
					target="_blank"
					rel="noopener noreferrer"
					class="block p-4 bg-white border border-gray-200 rounded-lg hover:bg-gray-50 transition-colors"
				>
					<h5 class="font-medium text-gray-900 mb-2">Arm Component Docs</h5>
					<p class="text-sm text-gray-600">
						Learn how to control robotic arms through the Viam platform
					</p>
				</a>

				<a
					href="https://docs.viam.com/components/gripper/"
					target="_blank"
					rel="noopener noreferrer"
					class="block p-4 bg-white border border-gray-200 rounded-lg hover:bg-gray-50 transition-colors"
				>
					<h5 class="font-medium text-gray-900 mb-2">Gripper Component Docs</h5>
					<p class="text-sm text-gray-600">Documentation for gripper control and programming</p>
				</a>

				<a
					href="https://docs.viam.com/sdks/"
					target="_blank"
					rel="noopener noreferrer"
					class="block p-4 bg-white border border-gray-200 rounded-lg hover:bg-gray-50 transition-colors"
				>
					<h5 class="font-medium text-gray-900 mb-2">SDK Documentation</h5>
					<p class="text-sm text-gray-600">Program your arm using Python, TypeScript, Go, or C++</p>
				</a>
			</div>
		</div>

		<!-- Actions. No link out to the machine's config page: ConnectionDetails only carries
		     the gRPC dial hostname and a `machineId` that is sometimes actually a name slug (see
		     parseConnectionFromCookies), never an org/location id, so an app.viam.com config URL
		     can't be built correctly from what this app knows -- guessing one would be worse than
		     omitting it. -->
		<div class="flex flex-col sm:flex-row gap-4 justify-center">
			<Button onclick={() => goto(getMachineRootPath())} variant="secondary" size="lg">
				<Icon name="arrowLeft" className="w-5 h-5 mr-3" />
				Back to Workflow Selection
			</Button>
		</div>

		<!-- Final Message -->
		<div class="bg-gradient-to-r from-blue-50 to-green-50 p-6 rounded-lg border">
			<div class="text-center">
				<h4 class="text-lg font-semibold text-gray-900 mb-2">
					{content.closingTitle}
				</h4>
				<p class="text-gray-600">
					{content.closing}
				</p>
			</div>
		</div>
	</div>
</div>
