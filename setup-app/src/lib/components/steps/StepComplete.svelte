<script lang="ts">
	import type { StepProps, CalibrationReadings, WorkflowType } from '$lib/types';

	interface Props extends StepProps {
		workflowType?: WorkflowType;
	}

	let { sensorReadings, motorSetupResults, workflowType = 'full-setup' }: Props = $props();

	// Get current sensor readings for final status
	const readings = $derived(sensorReadings.current.data as CalibrationReadings | undefined);
	const joints = $derived(readings?.joints || {});

	// Calculate completion statistics
	const totalMotorsConfigured = $derived(Object.keys(motorSetupResults).length);
	const totalJointsCalirated = $derived(Object.keys(joints).length);
	const completedJoints = $derived(
		Object.values(joints).filter((joint) => joint.is_completed).length
	);

	// Format current time for completion timestamp
	const completionTime = new Date().toLocaleString();
</script>

<div class="max-w-4xl mx-auto text-center">
	<!-- Success Header -->
	<div class="mb-8">
		<div class="w-24 h-24 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-6">
			<svg class="w-12 h-12 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
				<path
					stroke-linecap="round"
					stroke-linejoin="round"
					stroke-width="2"
					d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z"
				></path>
			</svg>
		</div>
		<h3 class="text-3xl font-bold text-gray-900 mb-4">🎉 Setup Complete!</h3>
		<p class="text-xl text-gray-600">
			Your SO-101 robotic arm has been successfully configured and calibrated.
		</p>
	</div>

	<!-- Completion Summary -->
	<div class="bg-green-50 border border-green-200 rounded-lg p-6 mb-8">
		<h4 class="text-lg font-semibold text-green-900 mb-4">Setup Summary</h4>
		<div class="grid md:grid-cols-3 gap-6 text-center">
			<div class="bg-white p-4 rounded-lg">
				<div class="text-2xl font-bold text-green-600 mb-2">{totalMotorsConfigured}</div>
				<div class="text-sm text-gray-600">Motors Configured</div>
				<div class="text-xs text-gray-500 mt-1">IDs assigned and baudrates set</div>
			</div>
			<div class="bg-white p-4 rounded-lg">
				<div class="text-2xl font-bold text-green-600 mb-2">{totalJointsCalirated}</div>
				<div class="text-sm text-gray-600">Joints Calibrated</div>
				<div class="text-xs text-gray-500 mt-1">Homing and ranges recorded</div>
			</div>
			<div class="bg-white p-4 rounded-lg">
				<div class="text-2xl font-bold text-green-600 mb-2">{completedJoints}</div>
				<div class="text-sm text-gray-600">Complete Ranges</div>
				<div class="text-xs text-gray-500 mt-1">Full motion coverage achieved</div>
			</div>
		</div>
		<div class="mt-4 text-xs text-gray-600">
			Completed: {completionTime}
		</div>
	</div>

	<!-- What Was Accomplished -->
	<div class="bg-white border border-gray-200 rounded-lg p-6 mb-8 text-left">
		<h4 class="text-lg font-semibold text-gray-900 mb-4 text-center">What Was Accomplished</h4>

		<div class="grid md:grid-cols-2 gap-6">
			<div>
				<h5 class="font-medium text-gray-800 mb-3 flex items-center">
					<svg
						class="w-5 h-5 text-blue-600 mr-2"
						fill="none"
						stroke="currentColor"
						viewBox="0 0 24 24"
					>
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z"
						></path>
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M15 12a3 3 0 11-6 0 3 3 0 016 0z"
						></path>
					</svg>
					Motor Configuration
				</h5>
				<ul class="text-sm text-gray-600 space-y-1">
					<li>✅ All 6 servo motors configured with correct IDs</li>
					<li>✅ Communication baudrate set to 1,000,000 bps</li>
					<li>✅ Motor chain connectivity verified</li>
					<li>✅ Hardware communication established</li>
				</ul>
			</div>

			<div>
				<h5 class="font-medium text-gray-800 mb-3 flex items-center">
					<svg
						class="w-5 h-5 text-green-600 mr-2"
						fill="none"
						stroke="currentColor"
						viewBox="0 0 24 24"
					>
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v4a2 2 0 01-2 2h-2a2 2 0 00-2-2z"
						></path>
					</svg>
					Calibration Process
				</h5>
				<ul class="text-sm text-gray-600 space-y-1">
					<li>✅ Homing positions set for all joints</li>
					<li>✅ Full range of motion recorded</li>
					<li>✅ Joint limits saved to servo memory</li>
					<li>✅ Configuration file created and saved</li>
				</ul>
			</div>
		</div>
	</div>

	<!-- Next Steps -->
	<div class="bg-blue-50 border border-blue-200 rounded-lg p-6 mb-8 text-left">
		<h4 class="text-lg font-semibold text-blue-900 mb-4 text-center">Your Arm is Ready!</h4>

		<div class="space-y-4">
			<div class="flex items-start">
				<div class="flex-shrink-0">
					<div
						class="w-6 h-6 bg-blue-600 text-white rounded-full flex items-center justify-center text-sm font-medium"
					>
						1
					</div>
				</div>
				<div class="ml-3">
					<h5 class="font-medium text-blue-900">Motors Re-enabled</h5>
					<p class="text-blue-800 text-sm">
						Servo holding torque has been restored. The arm will maintain its position and be ready
						for controlled movement.
					</p>
				</div>
			</div>

			<div class="flex items-start">
				<div class="flex-shrink-0">
					<div
						class="w-6 h-6 bg-blue-600 text-white rounded-full flex items-center justify-center text-sm font-medium"
					>
						2
					</div>
				</div>
				<div class="ml-3">
					<h5 class="font-medium text-blue-900">Viam Integration Ready</h5>
					<p class="text-blue-800 text-sm">
						Your robot config now includes properly calibrated arm and gripper components that can
						be controlled through the Viam platform.
					</p>
				</div>
			</div>

			<div class="flex items-start">
				<div class="flex-shrink-0">
					<div
						class="w-6 h-6 bg-blue-600 text-white rounded-full flex items-center justify-center text-sm font-medium"
					>
						3
					</div>
				</div>
				<div class="ml-3">
					<h5 class="font-medium text-blue-900">Programming and Control</h5>
					<p class="text-blue-800 text-sm">
						Use the Viam SDK (Python, TypeScript, Go, C++) or the Viam app control interface to
						program and operate your arm.
					</p>
				</div>
			</div>
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
					class="block p-4 bg-white border border-gray-200 rounded-lg hover:bg-gray-50 transition-colors"
				>
					<h5 class="font-medium text-gray-900 mb-2">Gripper Component Docs</h5>
					<p class="text-sm text-gray-600">Documentation for gripper control and programming</p>
				</a>

				<a
					href="https://docs.viam.com/sdks/"
					target="_blank"
					class="block p-4 bg-white border border-gray-200 rounded-lg hover:bg-gray-50 transition-colors"
				>
					<h5 class="font-medium text-gray-900 mb-2">SDK Documentation</h5>
					<p class="text-sm text-gray-600">Program your arm using Python, TypeScript, Go, or C++</p>
				</a>
			</div>
		</div>

		<!-- Actions -->
		<div class="flex flex-col sm:flex-row gap-4 justify-center">
			<button
				onclick={() => window.location.reload()}
				class="inline-flex items-center justify-center px-6 py-3 border border-gray-300 text-base font-medium rounded-md text-gray-700 bg-white hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500 transition-colors"
			>
				<svg class="w-5 h-5 mr-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
					<path
						stroke-linecap="round"
						stroke-linejoin="round"
						stroke-width="2"
						d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15"
					></path>
				</svg>
				Run Setup Again
			</button>
		</div>

		<!-- Final Message -->
		<div class="bg-gradient-to-r from-blue-50 to-green-50 p-6 rounded-lg border">
			<div class="text-center">
				<h4 class="text-lg font-semibold text-gray-900 mb-2">
					🚀 Your SO-101 is Ready for Action!
				</h4>
				<p class="text-gray-600">
					You've successfully completed the setup process. Your robotic arm is now calibrated,
					configured, and ready for precise control through the Viam platform.
				</p>
			</div>
		</div>
	</div>
</div>
