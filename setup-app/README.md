# SO-101 Setup Application

A SvelteKit web application (the `so101-setup` Viam application) that provides guided
motor-setup and calibration workflows for the SO-101 5-DOF robotic arm. It talks to the
`devrel:so101:calibration` sensor component via DoCommand to drive the underlying state
machine. It is bundled into `module.tar.gz` and served as a Viam App.

## Architecture

### Technology Stack

- **SvelteKit 2**: Web framework with TypeScript support
- **Svelte 5**: Frontend framework using modern runes (`$state`, `$derived`)
- **Tailwind CSS 4**: Utility-first CSS framework with forms and typography plugins
- **@viamrobotics/svelte-sdk**: Reactive Svelte integration for Viam SDK
- **@viamrobotics/sdk**: Core Viam TypeScript SDK for robot communication
- **js-cookie**: Cookie parsing for connection authentication

### Application Structure

- **Client-side only**: SSR disabled, runs entirely in browser
- **Cookie-based authentication**: connection details come from a per-machine cookie
- **Three independent workflows**, each its own route under `src/routes/workflows/`:
  motor setup (4 steps), calibration (6 steps), and full setup (8 steps, motor setup +
  calibration back to back)

### Routing

- `src/routes/+page.svelte` — landing page. Shows `SensorConfigForm` (a picker of the
  machine's `sensor` resources plus a Test Connection button; the part ID is not a field,
  it is hardcoded to `'main'`) until a connection test succeeds, then `WorkflowSelector`
  to choose a workflow. The chosen sensor config is passed to the workflow route as URL
  params and mirrored into `sessionStorage` (keyed `so101-setup-state-{machineId}`) so a
  refresh on the workflow page doesn't lose it.
- `src/routes/workflows/motor-setup/+page.svelte`,
  `src/routes/workflows/calibration/+page.svelte`,
  `src/routes/workflows/full-setup/+page.svelte` — one page per workflow. Each resolves
  the sensor config via `useWorkflowConfig` (URL params first, session storage fallback),
  then renders `<SensorProvider>` wrapping that workflow's wizard component.

### Connection Management (`src/routes/+layout.svelte`, `src/lib/utils/connection.ts`)

- `parseConnectionFromCookies` derives a machine ID from the URL path (it accepts
  `/machine/{name}/robot/{id}`, `/machine/{name}`, and `/robot/{id}` for local dev), then
  reads a cookie keyed by that machine ID with shape
  `{ apiKey: { id, key }, machineId, hostname }`.
- The layout turns that into a `dialConfigs` map for `ViamProvider`, always under the
  fixed key `'main'`.

### Sensor Context (`src/lib/components/SensorProvider.svelte`)

Wraps a workflow's wizard and sets a `'sensor'` Svelte context: the sensor client, a
polled `getReadings` query (1s interval), a `doCommand` mutation, a `sendCommand` helper
that raises user-friendly errors, and the `sensorConfig` it was given. Wizards and step
components read this context rather than talking to the SDK directly.

> **Known wart**: `SensorConfig.partId` is not a Viam part ID — it's the key into the
> `dialConfigs` map built in `+layout.svelte`, which is hardcoded to `'main'`. The field
> is named for a rename that hasn't happened yet.

### Wizards (`src/lib/components/*Wizard.svelte`)

`BaseWizard.svelte` is the shared chrome: progress bar, step breadcrumbs, the error
banner, and the previous/next footer. Each workflow wizard supplies its own step list and
switches over the current step name to render the matching step component:

- **MotorSetupWizard** (4 steps): overview → motor_setup → motor_verify → complete
- **CalibrationWizard** (6 steps): overview → calibration_start → calibration_homing →
  calibration_recording → calibration_save → complete
- **FullSetupWizard** (8 steps): overview → motor_setup → motor_verify →
  calibration_start → calibration_homing → calibration_recording → calibration_save →
  complete

### Step Components (`src/lib/components/steps/`)

Shared across the wizards above:

- **StepOverview**: introduction and safety information
- **StepMotorSetup**: motor ID discovery and configuration
- **StepMotorVerify**: motor verification and validation
- **StepCalibrationStart**: begin calibration workflow
- **StepCalibrationHoming**: manual positioning for homing
- **StepCalibrationRecording**: range recording with real-time progress
- **StepCalibrationSave**: save calibration data
- **StepComplete**: success confirmation

## Development Commands

```bash
pnpm install          # Install dependencies
pnpm dev              # Start development server (add -- --open to open a browser)
pnpm build            # Build production application
pnpm preview          # Preview production build
pnpm check            # Svelte type checking (svelte-check)
pnpm format           # Format code with Prettier
pnpm lint             # Check formatting with Prettier (no writes)
```

`pnpm run check`, `pnpm run lint`, and `pnpm run build` all run in CI on every pull
request (`.github/workflows/ci.yml`, on Node 22 / pnpm 11).

## Local Development with Viam CLI

To test the application with a live robot connection, use the Viam CLI proxy server:

```bash
viam module local-app-testing --app-url http://localhost:5173 --machine-id <machine-id>
```

Replace `<machine-id>` with your robot's machine ID from the Viam platform. The CLI stands
up a proxy on `http://localhost:8012` that simulates the Viam app server, so it supplies
the machine-scoped connection cookie the app reads. Browse to **localhost:8012**, not
5173 — hitting the dev server directly has no cookie and no machine ID in the path, so the
layout stops at "Connection Error".

### Prerequisites for Local Testing

1. **Viam CLI installed**: Follow [Viam CLI installation guide](https://docs.viam.com/appendix/cli/)
2. **Robot configured**: SO-101 calibration sensor component must be configured on your robot
3. **Network access**: Robot and development machine must have internet connectivity
4. **Serial permissions**: Proper USB serial port permissions on the robot computer

## Motor Setup Workflow

The application configures motors in reverse assembly order to avoid ID conflicts:

1. **Gripper** (servo 6) - End effector motor
2. **Wrist Roll** (servo 5) - Wrist rotation joint
3. **Wrist Flex** (servo 4) - Wrist flexion joint
4. **Elbow Flex** (servo 3) - Elbow joint
5. **Shoulder Lift** (servo 2) - Shoulder lift joint
6. **Shoulder Pan** (servo 1) - Base rotation joint

Per motor, `StepMotorSetup` issues `motor_setup_discover` then `motor_setup_assign_id`
(which also sets the 1 Mbaud baudrate). Verification is not per motor: `StepMotorVerify`
makes a single `motor_setup_verify` call covering all six.

## Calibration State Machine

The SO-101 calibration sensor implements a state machine:

- `idle` → `started` → `homing_position` → `range_recording` → `completed` → `idle`
  (`save_calibration` is what returns `completed` to `idle`); any failure sets `error`,
  which `reset` clears back to `idle`
- Step components read `calibration_state` out of the polled readings and gate their
  controls on it
- Real-time progress feedback during range recording phase

## Debugging

`src/lib/utils/logger.ts` buffers the last 1000 log entries in memory on the exported
`logger` singleton and exposes them via `getLogs()`, `getLogsForLevel(level)`,
`exportLogs()` (JSON string), and `clearLogs()`. Console output is narrower than the
buffer: `warn` and `error` always print, `debug` and `info` only when `import.meta.env.DEV`.

The buffer is currently unreachable. Nothing calls any of those four methods, they are not
declared on the `Logger` interface in `types.ts`, and the singleton is not attached to
`window` — so from a live browser console there is no way in short of a breakpoint or a
temporary import. Treat it as groundwork for a debug panel that does not exist yet, not as
a debugging tool you can use today.

## Safety Considerations

- `StepOverview` leads with a safety warning, workspace requirements, and a prerequisites
  checklist before any command is sent
- `StepCalibrationStart` warns that holding torque is about to be disabled, before the
  manual-positioning steps; `StepCalibrationSave` says when torque is restored
- `StepCalibrationStart` offers `abort`; the later calibration steps offer `reset`. The two
  motor-setup steps offer neither — there is no single always-available abort control
