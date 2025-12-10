// Error handling and validation utilities for browser-based execution

// Custom error classes
export class DemoExecutionError extends Error {
  constructor(message: string, public code?: string) {
    super(message);
    this.name = 'DemoExecutionError';
  }
}

export class ValidationError extends Error {
  constructor(message: string, public field?: string) {
    super(message);
    this.name = 'ValidationError';
  }
}

export class NetworkError extends Error {
  constructor(message: string, public status?: number) {
    super(message);
    this.name = 'NetworkError';
  }
}

// Validation functions
export const validateCode = (code: string, dependencies: string[]): string[] => {
  const errors: string[] = [];

  if (!code || code.trim().length === 0) {
    errors.push('Code cannot be empty');
  }

  // Check for basic syntax issues based on dependencies
  if (dependencies.includes('p5.js')) {
    // Check for common p5.js issues
    if (code.includes('setup()') && !code.includes('function setup()')) {
      errors.push('p5.js setup must be defined as function setup()');
    }
    if (code.includes('draw()') && !code.includes('function draw()')) {
      errors.push('p5.js draw must be defined as function draw()');
    }
  }

  if (dependencies.includes('Three.js')) {
    // Check for common Three.js issues
    if (code.includes('THREE.Scene') && !code.includes('new THREE.Scene')) {
      errors.push('Three.js Scene must be instantiated with new THREE.Scene()');
    }
  }

  return errors;
};

export const validateDependencies = (dependencies: string[]): string[] => {
  const errors: string[] = [];
  const supportedDeps = ['p5.js', 'Three.js', 'Pyodide', 'TensorFlow.js', 'WebSerial', 'WebUSB'];

  for (const dep of dependencies) {
    if (!supportedDeps.includes(dep)) {
      errors.push(`Unsupported dependency: ${dep}`);
    }
  }

  return errors;
};

// Error handling wrapper for async operations
export const handleAsyncError = async <T>(
  asyncFn: () => Promise<T>,
  fallbackValue?: T
): Promise<T | undefined> => {
  try {
    return await asyncFn();
  } catch (error) {
    console.error('Async operation failed:', error);
    if (fallbackValue !== undefined) {
      return fallbackValue;
    }
    throw error;
  }
};

// Safe execution wrapper for browser environments
export const safeExecute = async <T>(
  fn: () => T,
  onError?: (error: Error) => void
): Promise<T | null> => {
  try {
    return await fn();
  } catch (error) {
    console.error('Execution error:', error);
    if (onError) {
      onError(error as Error);
    }
    return null;
  }
};

// Validation middleware for API requests
export const validateRequest = <T>(
  data: T,
  validator: (data: T) => string[]
): { isValid: boolean; errors: string[] } => {
  const errors = validator(data);
  return {
    isValid: errors.length === 0,
    errors
  };
};