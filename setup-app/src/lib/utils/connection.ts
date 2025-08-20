import Cookies from 'js-cookie';
import type { ConnectionDetails } from '../types';

/**
 * Parse connection details from browser cookies
 * Expected cookie structure: { apiKey: { id, key }, machineId, hostname }
 */
export function parseConnectionFromCookies(): {
  connectionDetails: ConnectionDetails | null;
  machineId: string | null;
  error: string | null;
} {
  try {
    // Extract machine ID from URL path
    // Supports both formats:
    // - /machine/{machine-name}/robot/{machine-id} (hosted environment)
    // - /machine/{machine-name} (direct access)
    // - /robot/{machine-id} (local dev)
    const pathParts = window.location.pathname.split('/').filter(part => part !== '');
    let machineId: string | null = null;

    // Check for /machine/{machine-name}/robot/{machine-id} pattern
    if (pathParts.length >= 4 && pathParts[0] === 'machine' && pathParts[2] === 'robot') {
      machineId = pathParts[3];
    }
    // Check for /machine/{machine-name} pattern (use machine-name as machine ID)
    else if (pathParts.length >= 2 && pathParts[0] === 'machine') {
      machineId = pathParts[1];
    }
    // Check for /robot/{machine-id} pattern (local dev)
    else if (pathParts.length >= 2 && pathParts[0] === 'robot') {
      machineId = pathParts[1];
    }

    if (!machineId) {
      return {
        connectionDetails: null,
        machineId: null,
        error: 'Machine ID not found in URL path. Expected format: /machine/{machine-name} or /machine/{machine-name}/robot/{machine-id} or /robot/{machine-id}'
      };
    }

    // Get connection cookie by machine ID
    const cookieData = Cookies.get(machineId);
    
    if (!cookieData) {
      return {
        connectionDetails: null,
        machineId,
        error: `Connection cookie not found for machine ID: ${machineId}`
      };
    }

    // Parse cookie JSON
    const connectionDetails: ConnectionDetails = JSON.parse(cookieData);

    // Validate required fields
    if (!connectionDetails.apiKey || !connectionDetails.apiKey.id || !connectionDetails.apiKey.key) {
      return {
        connectionDetails: null,
        machineId,
        error: 'Invalid cookie format: missing API key data'
      };
    }

    if (!connectionDetails.machineId || !connectionDetails.hostname) {
      return {
        connectionDetails: null,
        machineId,
        error: 'Invalid cookie format: missing machine ID or hostname'
      };
    }

    return {
      connectionDetails,
      machineId,
      error: null
    };
  } catch (error) {
    return {
      connectionDetails: null,
      machineId: null,
      error: error instanceof Error ? error.message : 'Failed to parse connection details'
    };
  }
}

/**
 * Create DialConf object for ViamProvider from connection details
 */
export function createDialConfig(connectionDetails: ConnectionDetails) {
  return {
    host: connectionDetails.hostname,
    credentials: {
      type: 'api-key',
      authEntity: connectionDetails.apiKey.id,
      payload: connectionDetails.apiKey.key,
    },
    signalingAddress: 'https://app.viam.com:443',
    disableSessions: false,
  };
}

/**
 * Get the base path for the application from current URL
 * Returns empty string for local dev, /machine/{machine-name} for hosted environment
 */
export function getBasePath(): string {
  const pathParts = window.location.pathname.split('/').filter(part => part !== '');
  
  // Check if we're in a /machine/{machine-name} environment
  if (pathParts.length >= 2 && pathParts[0] === 'machine') {
    return `/machine/${pathParts[1]}`;
  }
  
  // Local dev or other environment - no base path
  return '';
}

/**
 * Get user-friendly error message for connection errors
 */
export function getConnectionErrorMessage(error: string): string {
  if (error.includes('Machine ID not found')) {
    return 'Please navigate to this page from the Viam app with the correct robot URL.';
  }
  
  if (error.includes('Connection cookie not found')) {
    return 'Connection credentials not found. Please navigate to this page from the Viam app.';
  }
  
  if (error.includes('Invalid cookie format')) {
    return 'Connection credentials are invalid. Please refresh and try again from the Viam app.';
  }
  
  return `Connection error: ${error}`;
}