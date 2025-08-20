# SO-101 Setup Wizard - Implementation Steering Document

## Project Overview

Build a SvelteKit web application that guides users through first-time setup of the SO-101 robotic arm, including motor configuration and calibration workflows. The app will use the Viam TypeScript and Svelte SDKs to connect to robots and interact with the SO-101 calibration sensor component.

## Technical Requirements

### Core Dependencies
- **SvelteKit**: Web framework
- **@viamrobotics/svelte-sdk**: Reactive layer over Viam SDK
- **@viamrobotics/sdk**: Core Viam TypeScript SDK  
- **@tanstack/svelte-query**: Query management (dependency of Svelte SDK)
- **js-cookie**: Cookie parsing for connection details
- **TailwindCSS**: Styling framework

### Browser Environment
- Client-side only (SSR disabled)
- Connection details provided via browser cookies from Viam platform
- URL format: `/robot/{machine-id}` where machine-id matches cookie key

## Application Architecture

### File Structure
```
src/
├── routes/
│   ├── +layout.svelte          # ViamProvider setup, connection parsing
│   ├── +layout.ts              # Disable SSR/prerendering  
│   └── +page.svelte            # Main wizard orchestration
├── lib/
│   ├── components/
│   │   ├── SetupWizard.svelte          # Main wizard component
│   │   └── steps/
│   │       ├── StepOverview.svelte     # Introduction and safety
│   │       ├── StepMotorSetup.svelte   # Motor ID configuration
│   │       ├── StepMotorVerify.svelte  # Motor verification
│   │       ├── StepCalibrationStart.svelte
│   │       ├── StepCalibrationHoming.svelte
│   │       ├── StepCalibrationRecording.svelte
│   │       ├── StepCalibrationSave.svelte
│   │       └── StepComplete.svelte
│   ├── types.ts                # TypeScript interfaces
│   └── utils/
│       └── connection.ts       # Connection parsing utilities
├── app.html                    # HTML template
└── app.css                     # Tailwind imports
```

### Configuration Files
- `svelte.config.js`: SvelteKit configuration with adapter
- `vite.config.ts`: Vite configuration for Viam SDK compatibility  
- `tailwind.config.js`: Tailwind CSS configuration
- `package.json`: Dependencies and scripts

## Implementation Plan

### Phase 1: Project Setup and Basic Structure

#### 1.1 Initialize SvelteKit Project
```bash
npm create svelte@latest so101-setup-wizard
cd so101-setup-wizard
npm install
```

#### 1.2 Install Dependencies
```bash
npm install @viamrobotics/svelte-sdk @viamrobotics/sdk @tanstack/svelte-query js-cookie
npm install -D @types/js-cookie tailwindcss autoprefixer postcss
```

#### 1.3 Configure Build Tools
- Set up Tailwind CSS with PostCSS
- Configure Vite for Viam SDK compatibility (global polyfills)
- Disable SSR and prerendering in SvelteKit config

### Phase 2: Connection Management and Layout

#### 2.1 Layout Component (`src/routes/+layout.svelte`)
**Purpose**: Parse connection details and provide ViamProvider

**Key Implementation Details**:
- Extract machine ID from URL path: `window.location.pathname.split("/")[2]`
- Parse connection cookie using js-cookie library
- Expected cookie structure:
  ```json
  {
    "apiKey": { "id": "key-id", "key": "key-secret" },
    "machineId": "machine-id", 
    "hostname": "robot-hostname.viam.cloud"
  }
  ```
- Create `DialConf` object for ViamProvider:
  ```typescript
  {
    host: hostname,
    credentials: {
      type: 'api-key',
      authEntity: apiKeyId, 
      payload: apiKeySecret,
    },
    signalingAddress: 'https://app.viam.com:443',
    disableSessions: false,
  }
  ```
- Handle connection errors gracefully with retry options
- Show loading states during connection parsing

#### 2.2 Connection Utilities (`src/lib/utils/connection.ts`)
- `parseConnectionFromCookies()`: Extract and validate connection details
- `createDialConfig()`: Transform details into DialConf format
- Error handling for missing/malformed cookies

### Phase 3: Sensor Client Integration

#### 3.1 Main Page (`src/routes/+page.svelte`)
**Purpose**: Create sensor client and pass to wizard

**Viam SDK Integration**:
```typescript
// Use Viam Svelte SDK hooks
const partID = 'so101-robot'; // Fixed part ID
const sensorName = 'so101-calibration'; // SO-101 calibration sensor name

// Create sensor client using SDK hook
const sensorClient = createResourceClient(
  SensorClient,
  () => partID,
  () => sensorName
);

// Create reactive query for sensor readings (1 second interval)
const sensorReadings = createResourceQuery(
  sensorClient, 
  'getReadings',
  { refetchInterval: 1000 }
);

// Create mutation for DoCommand calls
const doCommand = createResourceMutation(sensorClient, 'doCommand');
```

#### 3.2 Error Handling Strategy
- Wrap all sensor operations in try-catch blocks
- Display user-friendly error messages with retry options
- Log technical details to console for debugging
- Handle network timeouts and connection issues gracefully

### Phase 4: Wizard Framework

#### 4.1 Setup Wizard (`src/lib/components/SetupWizard.svelte`)
**Purpose**: Orchestrate 8-step workflow with progress tracking

**State Management**:
```typescript
const WORKFLOW_STEPS = [
  'overview', 'motor_setup', 'motor_verify', 
  'calibration_start', 'calibration_homing', 
  'calibration_recording', 'calibration_save', 'complete'
];

let currentStep = $state(0);
let error = $state<string | null>(null);  
let motorSetupResults = $state<Record<string, any>>({});
```

**Navigation**:
- Progress bar showing current step / total steps
- Previous/Next navigation (where appropriate)
- Step validation before advancement
- Error state handling with retry mechanisms

#### 4.2 Common Step Interface
All step components receive consistent props:
```typescript
interface StepProps {
  sensorClient: any;           // Sensor client reference
  sensorReadings: any;         // Reactive readings query
  doCommand: any;              // DoCommand mutation
  sendCommand: (cmd: any) => Promise<any>;  // Helper function
  error: string | null;        // Current error state
  setError: (error: string | null) => void;
  clearError: () => void;
  nextStep: () => void;
  prevStep: () => void;
  motorSetupResults: Record<string, any>;
  setMotorSetupResults: (results: Record<string, any>) => void;
}
```

### Phase 5: Motor Setup Implementation

#### 5.1 Motor Setup Step (`src/lib/components/steps/StepMotorSetup.svelte`)
**Purpose**: Configure each servo motor with correct ID

**Motor Configuration Order** (reverse assembly order to avoid ID conflicts):
```typescript
const MOTOR_SETUP_ORDER = [
  { name: 'gripper', targetId: 6, description: 'Gripper (end effector)' },
  { name: 'wrist_roll', targetId: 5, description: 'Wrist Roll Joint' },
  { name: 'wrist_flex', targetId: 4, description: 'Wrist Flex Joint' },
  { name: 'elbow_flex', targetId: 3, description: 'Elbow Flex Joint' },
  { name: 'shoulder_lift', targetId: 2, description: 'Shoulder Lift Joint' },
  { name: 'shoulder_pan', targetId: 1, description: 'Shoulder Pan Joint' }
];
```

**DoCommand Calls**:
1. **Discovery**: `{ command: 'motor_setup_discover', motor_name: 'gripper' }`
   - Expected response: `{ success: true, current_id: 1, target_id: 6, model: 'sts3215', found_baudrate: 57600 }`

2. **ID Assignment**: `{ command: 'motor_setup_assign_id', motor_name: 'gripper', current_id: 1, target_id: 6, current_baudrate: 57600 }`
   - Expected response: `{ success: true, status: 'Configured gripper' }`

**UI Requirements**:
- Grid showing all 6 motors with status indicators
- Current motor highlighted with detailed instructions
- Discovery and configuration buttons with loading states
- Progress tracking through motor setup results state

#### 5.2 Motor Verification Step (`src/lib/components/steps/StepMotorVerify.svelte`)
**Purpose**: Verify all motors are properly configured

**DoCommand Call**: `{ command: 'motor_setup_verify' }`
**Expected Response**:
```json
{
  "success": true,
  "motors": {
    "shoulder_pan": { "id": 1, "status": "ok", "model": "sts3215" },
    "shoulder_lift": { "id": 2, "status": "ok", "model": "sts3215" },
    // ... other motors
  },
  "status": "All motors verified successfully"
}
```

### Phase 6: Calibration Workflow Implementation

#### 6.1 Calibration Start (`src/lib/components/steps/StepCalibrationStart.svelte`)
**DoCommand**: `{ command: 'start' }`
**Effect**: Disables torque for manual positioning
**Next State**: `calibration_state: 'started'`

#### 6.2 Homing Position (`src/lib/components/steps/StepCalibrationHoming.svelte`)
**DoCommand**: `{ command: 'set_homing' }`
**User Action**: Manually position arm to center of range
**Next State**: `calibration_state: 'homing_position'`

#### 6.3 Range Recording (`src/lib/components/steps/StepCalibrationRecording.svelte`)
**DoCommands**:
- Start: `{ command: 'start_range_recording' }`
- Stop: `{ command: 'stop_range_recording' }`

**Real-time Progress Display**:
```typescript
// From sensor readings during recording
{
  calibration_state: 'range_recording',
  recording_time_seconds: 15.3,
  position_samples: 306,
  joints: {
    shoulder_pan: {
      id: 1,
      current_position: 2150,
      recorded_min: 758,
      recorded_max: 3292,
      is_completed: false
    }
    // ... other joints
  }
}
```

**UI Requirements**:
- Start/Stop recording buttons
- Real-time progress display (recording time, sample count)
- Joint-by-joint progress indicators
- Clear instructions for manual movement

#### 6.4 Save Calibration (`src/lib/components/steps/StepCalibrationSave.svelte`)
**DoCommand**: `{ command: 'save_calibration' }`
**Effect**: Writes calibration to servo registers and saves file
**Final State**: `calibration_state: 'idle'` (reset for next use)

### Phase 7: UI/UX Implementation

#### 7.1 Visual Design
- **Progress Bar**: Shows step X of Y with percentage completion
- **Step Cards**: Clean, card-based layout for each step
- **Status Indicators**: Color-coded status for motors and calibration stages
- **Loading States**: Spinner animations during operations
- **Error Display**: Red-bordered alerts with retry buttons

#### 7.2 Responsive Design  
- Mobile-friendly layout with responsive grids
- Touch-friendly button sizes
- Readable text at all screen sizes
- Grid layouts for motor status: 2 cols mobile, 3 cols desktop

#### 7.3 Color Scheme
- **Primary**: Blue (#2563eb) for main actions
- **Success**: Green (#16a34a) for completed states  
- **Warning**: Amber (#f59e0b) for safety notes
- **Error**: Red (#dc2626) for error states
- **Gray**: Various grays for neutral content

### Phase 8: Testing and Validation

#### 8.1 Component Testing
- Test each step component in isolation
- Verify proper state transitions between steps
- Test error handling and retry mechanisms
- Validate responsive design across screen sizes

#### 8.2 Integration Testing
- Test full workflow end-to-end
- Verify proper SDK integration and reactivity
- Test connection parsing and error scenarios
- Validate proper cleanup on navigation/refresh

## Implementation Resources

### Viam SDK Reference Materials

#### From `viam-svelte-sdk` Documentation:
```typescript
// Provider setup pattern
<ViamProvider {dialConfigs}>
  {@render children()}
</ViamProvider>

// Resource client creation
const client = createResourceClient(
  SensorClient,
  () => partID,
  () => name
);

// Reactive queries
const readings = createResourceQuery(client, 'getReadings', {
  refetchInterval: 1000
});

// Mutations for commands
const command = createResourceMutation(client, 'doCommand');
```

#### From Connection Sample Code:
```typescript
// Cookie parsing pattern
let machineCookieKey = window.location.pathname.split("/")[2];
({
  apiKey: { id: apiKeyId, key: apiKeySecret },
  machineId: machineId,
  hostname: host,
} = JSON.parse(Cookies.get(machineCookieKey)!));

// DialConf structure
const dialConfig = {
  host: hostname,
  credentials: {
    type: 'api-key',
    authEntity: apiKeyId,
    payload: apiKeySecret,
  },
  signalingAddress: 'https://app.viam.com:443',
  disableSessions: false,
};
```

### SO-101 Calibration API Reference

#### Motor Setup Commands (from calibration.go):
```json
// Discover single motor
{ "command": "motor_setup_discover", "motor_name": "gripper" }

// Assign motor ID  
{ 
  "command": "motor_setup_assign_id",
  "motor_name": "gripper",
  "current_id": 1,
  "target_id": 6, 
  "current_baudrate": 57600
}

// Verify all motors
{ "command": "motor_setup_verify" }

// Scan bus for debugging
{ "command": "motor_setup_scan_bus" }
```

#### Calibration Workflow Commands:
```json
// Start calibration (disables torque)
{ "command": "start" }

// Set homing position based on current arm position  
{ "command": "set_homing" }

// Begin range recording
{ "command": "start_range_recording" }

// Stop recording and calculate ranges
{ "command": "stop_range_recording" }

// Save to servo registers and file
{ "command": "save_calibration" }

// Utility commands
{ "command": "abort" }
{ "command": "reset" }
{ "command": "get_current_positions" }
```

#### Sensor Readings Structure:
```json
{
  "calibration_state": "idle|started|homing_position|range_recording|completed|error",
  "instruction": "Human-readable instruction text",
  "available_commands": ["start", "set_homing", ...],
  "servo_count": 5,
  "recording_time_seconds": 15.3,
  "position_samples": 306,
  "joints": {
    "shoulder_pan": {
      "id": 1,
      "current_position": 2150,
      "homing_offset": -103,
      "recorded_min": 758,
      "recorded_max": 3292,
      "is_completed": false
    }
  },
  "motor_setup": {
    "in_progress": false,
    "step": 0,
    "status": "Motor setup ready"
  }
}
```

## Implementation Steps

### Step 1: Project Foundation
1. Create SvelteKit project with TypeScript
2. Install and configure dependencies
3. Set up Tailwind CSS with proper purging
4. Configure Vite for Viam SDK (add global polyfills if needed)
5. Create basic file structure

### Step 2: Connection Layer
1. Implement `parseConnectionFromCookies()` utility
2. Create layout component with ViamProvider setup
3. Add connection error handling and loading states
4. Test connection parsing with mock cookie data

### Step 3: Sensor Integration
1. Create main page with sensor client setup
2. Implement reactive queries for sensor readings
3. Create DoCommand mutation wrapper
4. Test basic sensor communication

### Step 4: Wizard Framework  
1. Create SetupWizard component with step management
2. Implement progress bar and navigation
3. Create step component interface and props structure
4. Add error state management

### Step 5: Motor Setup Steps
1. Implement StepOverview with safety information
2. Create StepMotorSetup with discovery/assignment flow
3. Implement motor progress tracking and validation
4. Create StepMotorVerify with verification results display

### Step 6: Calibration Steps
1. Implement StepCalibrationStart with workflow initiation
2. Create StepCalibrationHoming with positioning instructions
3. Implement StepCalibrationRecording with real-time progress
4. Create StepCalibrationSave with summary and final save
5. Implement StepComplete with success confirmation

### Step 7: Polish and Testing
1. Add comprehensive error handling throughout
2. Implement loading states and smooth transitions
3. Test responsive design and accessibility
4. Add proper TypeScript types and validation
5. Test full workflow with simulated sensor responses

## Key Implementation Considerations

### State Management
- Use Svelte 5 runes (`$state`, `$derived`) for local component state
- Leverage Viam SDK's reactive queries for sensor data
- Maintain wizard state in parent component, pass down as props
- Store motor setup results to track progress across steps

### Error Handling Patterns
```typescript
const sendCommand = async (command: Record<string, any>) => {
  try {
    clearError();
    const result = await doCommand.current.mutateAsync(command);
    return result;
  } catch (err) {
    const errorMsg = err instanceof Error ? err.message : 'Unknown error occurred';
    setError(errorMsg);
    throw err;
  }
};
```

### Reactive Data Flow
- Sensor readings update automatically via `refetchInterval: 1000`
- UI components react to reading changes using `$derived`
- Progress indicators update based on current sensor state
- Step advancement triggered by state changes

### Safety Considerations
- Clear safety warnings on overview step
- Prominent torque disable notifications
- Emergency abort functionality available
- Clear workspace requirements
- Manual movement instructions with safety notes

## Motor Setup Workflow Details

### Phase A: Individual Motor Configuration
**Requirement**: Connect only one motor at a time to avoid ID conflicts

**For each motor in MOTOR_SETUP_ORDER**:
1. **Discovery Phase**:
   - User connects single motor to controller
   - App calls `motor_setup_discover` with motor name
   - Displays discovered motor details (current ID, model, baudrate)

2. **Configuration Phase**:
   - App calls `motor_setup_assign_id` with discovered and target IDs
   - Updates motor from default ID (usually 1) to target ID
   - Updates baudrate from 57600 to 1000000
   - Tracks completion in `motorSetupResults` state

3. **Progress Tracking**:
   - Visual grid showing all 6 motors
   - Color-coded status: pending (gray), current (blue), discovered (yellow), configured (green)
   - Current motor highlighted with detailed instructions

### Phase B: Verification
**Requirement**: All motors connected in daisy-chain

1. **Verification Call**: `motor_setup_verify`
2. **Result Processing**: Display status for each motor (ID, model, communication status)
3. **Success Criteria**: All motors respond with "ok" status
4. **Auto-advance**: Move to calibration workflow on successful verification

## Calibration Workflow Details

### State Machine Integration
The calibration sensor operates as a state machine. The UI must track and respond to state transitions:

**State Flow**: `idle` → `started` → `homing_position` → `range_recording` → `completed` → `idle`

### Real-time Progress Display
During range recording, show:
- Recording duration in seconds
- Number of position samples collected  
- Per-joint progress (range span, completion status)
- Visual indicators for joint movement coverage

### Data Validation
- Ensure all joints moved through sufficient range
- Display range spans for user verification
- Handle invalid ranges gracefully with clear instructions
- Provide retry mechanisms for insufficient coverage

## TypeScript Interfaces

```typescript
// src/lib/types.ts
export interface ConnectionDetails {
  apiKey: {
    id: string;
    key: string;
  };
  machineId: string;
  hostname: string;
}

export interface MotorSetupConfig {
  name: string;
  targetId: number;
  description: string;
}

export interface MotorSetupResult {
  motor_name: string;
  current_id: number;
  target_id: number;
  model: string;
  found_baudrate: number;
  step: 'discovered' | 'configured';
  success: boolean;
}

export interface CalibrationJoint {
  id: number;
  name: string;
  current_position: number;
  homing_offset: number;
  recorded_min: number;
  recorded_max: number;
  is_completed: boolean;
}

export interface CalibrationReadings {
  calibration_state: 'idle' | 'started' | 'homing_position' | 'range_recording' | 'completed' | 'error';
  instruction: string;
  available_commands: string[];
  servo_count: number;
  recording_time_seconds?: number;
  position_samples?: number;
  joints: Record<string, CalibrationJoint>;
  motor_setup: {
    in_progress: boolean;
    step: number;
    status: string;
  };
}
```

## Vite Configuration Requirements

```typescript
// vite.config.ts - Required for Viam SDK compatibility
export default defineConfig({
  plugins: [sveltekit()],
  optimizeDeps: {
    include: ['@viamrobotics/sdk', '@viamrobotics/svelte-sdk', 'js-cookie']
  },
  define: {
    global: 'globalThis', // Required for Viam SDK browser compatibility
  },
  server: {
    fs: {
      allow: ['..'] // Allow access to node_modules
    }
  }
});
```

## Deployment Considerations

### Build Configuration
- Ensure static builds for easy deployment  
- Include proper asset optimization
- Configure for client-side only operation (no SSR)

### Environment Setup
- Robot must have SO-101 calibration sensor component configured
- Serial port permissions must be properly set on robot computer
- Network connectivity between web app and robot required

### Integration Points
- App expects to be launched from Viam platform with proper cookies
- URL structure must match `/robot/{machine-id}` pattern
- Connection cookies must contain valid API credentials and robot hostname

## Success Criteria

### Functional Requirements
- [ ] Successfully connect to robot using cookie-based authentication
- [ ] Complete motor setup workflow for all 6 servos
- [ ] Execute full calibration workflow with real-time progress
- [ ] Save calibration data to robot successfully
- [ ] Handle all error conditions gracefully with recovery options

### User Experience Requirements  
- [ ] Clear, step-by-step guidance throughout setup process
- [ ] Responsive design working on desktop and mobile
- [ ] Loading states and progress indicators provide clear feedback
- [ ] Error messages are helpful and actionable
- [ ] Safety warnings are prominent and clear

### Technical Requirements
- [ ] Proper TypeScript types throughout application
- [ ] Reactive updates using Viam Svelte SDK patterns
- [ ] Efficient re-rendering and state management
- [ ] Clean component architecture with reusable patterns
- [ ] Comprehensive error boundaries and fallback handling

## Next Steps for Implementation

1. **Start with project scaffolding** - Set up SvelteKit project with dependencies
2. **Implement connection layer** - Cookie parsing and ViamProvider setup  
3. **Build wizard framework** - Step management and navigation
4. **Add motor setup** - Discovery, assignment, and verification flows
5. **Implement calibration** - Full calibration workflow with real-time updates
6. **Polish and test** - Error handling, responsive design, and end-to-end testing

This steering document provides the complete roadmap for building a production-ready SO-101 setup wizard using modern web technologies and the Viam platform.