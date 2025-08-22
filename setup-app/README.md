# SO-101 Setup Application

A SvelteKit web application that provides a setup wizard for the SO-101 5-DOF robotic arm. The app guides users through motor configuration and calibration workflows using the Viam TypeScript SDK to connect to robots and interact with the SO-101 calibration sensor component.

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
- **Cookie-based authentication**: Connection details from Viam platform cookies
- **URL routing**: `/robot/{machine-id}` where machine-id matches cookie key
- **Wizard-based UI**: 8-step workflow for setup process

### Core Components

#### Connection Management (`src/routes/+layout.svelte`)
- Parses connection details from browser cookies
- Creates `DialConf` for ViamProvider setup
- Handles connection errors and loading states
- Expected cookie structure: `{ apiKey: { id, key }, machineId, hostname }`

#### Setup Wizard (`src/lib/components/SetupWizard.svelte`)
- Orchestrates 8-step workflow with state management
- Progress tracking and navigation between steps
- Error handling with retry mechanisms
- Motor setup results persistence across steps

#### Step Components (`src/lib/components/steps/`)
- **StepOverview**: Introduction and safety information
- **StepMotorSetup**: Motor ID discovery and configuration
- **StepMotorVerify**: Motor verification and validation
- **StepCalibrationStart**: Begin calibration workflow
- **StepCalibrationHoming**: Manual positioning for homing
- **StepCalibrationRecording**: Range recording with real-time progress
- **StepCalibrationSave**: Save calibration data
- **StepComplete**: Success confirmation

## Development Commands

### Setup
```bash
pnpm install          # Install dependencies (preferred)
npm install           # Alternative package manager
```

### Development
```bash
pnpm dev              # Start development server
npm run dev           # Alternative with npm

# Start server and open in browser
pnpm dev -- --open
npm run dev -- --open
```

### Building
```bash
pnpm build            # Build production application
npm run build         # Alternative with npm

pnpm preview          # Preview production build
npm run preview       # Alternative with npm
```

### Code Quality
```bash
pnpm check            # Run Svelte type checking
npm run check         # Alternative with npm

pnpm format           # Format code with Prettier
npm run format        # Alternative with npm

pnpm lint             # Check code formatting
npm run lint          # Alternative with npm
```

## Local Development with Viam CLI

To test the application with a live robot connection, use the Viam CLI proxy server:

```bash
# Start the proxy server to connect local app to your robot
viam module local-app-testing --app-url http://localhost:5173 --machine-id <machine-id>
```

Replace `<machine-id>` with your robot's machine ID from the Viam platform.

This command:
1. Creates a secure tunnel between your local development server and the Viam platform
2. Allows the web app to authenticate and connect to your robot
3. Enables testing of the full setup workflow with real hardware

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

Each motor follows: discover → configure → verify pattern using sensor DoCommands.

## Calibration State Machine

The SO-101 calibration sensor implements a state machine:
- `idle` → `started` → `homing_position` → `range_recording` → `completed` → `idle`
- UI tracks state transitions and provides appropriate controls
- Real-time progress feedback during range recording phase

## Safety Considerations

- Prominent safety warnings throughout workflow
- Clear torque disable notifications during manual positioning
- Emergency abort functionality always available
- Workspace safety requirements prominently displayed

## Documentation

For comprehensive implementation details, see:
- `docs/so101-setup-wizard.md` - Complete implementation guide
- `docs/so101-api-reference.md` - Detailed API reference for sensor commands