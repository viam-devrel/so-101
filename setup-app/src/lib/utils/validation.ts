import type { ValidationResult } from '$lib/types';

/**
 * Validate sensor component name
 */
export function validateSensorName(name: string): ValidationResult {
	if (!name || !name.trim()) {
		return { isValid: false, error: 'Sensor name is required' };
	}

	const trimmed = name.trim();

	if (trimmed.length < 2) {
		return { isValid: false, error: 'Sensor name must be at least 2 characters long' };
	}

	if (trimmed.length > 50) {
		return { isValid: false, error: 'Sensor name must be less than 50 characters long' };
	}

	// Allow alphanumeric, hyphens, underscores, and periods
	if (!/^[a-zA-Z0-9_.-]+$/.test(trimmed)) {
		return {
			isValid: false,
			error: 'Sensor name can only contain letters, numbers, hyphens, underscores, and periods'
		};
	}

	return { isValid: true };
}

/**
 * Validate part ID
 */
export function validatePartId(partId: string): ValidationResult {
	if (!partId || !partId.trim()) {
		return { isValid: false, error: 'Part ID is required' };
	}

	const trimmed = partId.trim();

	if (trimmed.length < 1) {
		return { isValid: false, error: 'Part ID cannot be empty' };
	}

	if (trimmed.length > 100) {
		return { isValid: false, error: 'Part ID must be less than 100 characters long' };
	}

	// Allow alphanumeric, hyphens, underscores, and periods
	if (!/^[a-zA-Z0-9_.-]+$/.test(trimmed)) {
		return {
			isValid: false,
			error: 'Part ID can only contain letters, numbers, hyphens, underscores, and periods'
		};
	}

	return { isValid: true };
}

/**
 * Validate complete sensor configuration
 */
export function validateSensorConfig(partId: string, sensorName: string): ValidationResult {
	const partValidation = validatePartId(partId);
	if (!partValidation.isValid) {
		return partValidation;
	}

	const sensorValidation = validateSensorName(sensorName);
	if (!sensorValidation.isValid) {
		return sensorValidation;
	}

	return { isValid: true };
}
