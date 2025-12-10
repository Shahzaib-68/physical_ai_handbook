// Pyodide integration for Python execution in browser
import { getAppConfig } from '../utils/config';
import { DemoExecutionError, ValidationError } from '../utils/validation';

// Interface for Pyodide execution result
interface PyodideExecutionResult {
  output: string;
  error?: string;
  executionTime: number;
}

// Global variable to store Pyodide instance once loaded
let pyodideInstance: any = null;
let pyodideLoadingPromise: Promise<any> | null = null;

// Initialize Pyodide with necessary packages
export const initializePyodide = async (packages: string[] = ['numpy', 'matplotlib', 'scipy']): Promise<any> => {
  // If already initialized, return the instance
  if (pyodideInstance) {
    return pyodideInstance;
  }

  // If loading is in progress, wait for it to complete
  if (pyodideLoadingPromise) {
    return pyodideLoadingPromise;
  }

  // Create a new loading promise
  pyodideLoadingPromise = (async () => {
    try {
      // Dynamically import Pyodide
      // @ts-ignore
      const pyodide = await loadPyodide();
      
      // Load required packages
      if (packages.length > 0) {
        await pyodide.loadPackage(packages);
      }
      
      // Store the instance
      pyodideInstance = pyodide;
      return pyodide;
    } catch (error) {
      console.error('Failed to initialize Pyodide:', error);
      throw new DemoExecutionError(`Failed to initialize Pyodide: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  })();

  return pyodideLoadingPromise;
};

// Load Pyodide dynamically (in a real implementation, this would be from CDN)
const loadPyodide = async () => {
  // In a real implementation, we would load Pyodide from a CDN:
  // const { loadPyodide: loadPyodideActual } = await import('pyodide');
  // return await loadPyodideActual({ indexURL: 'https://cdn.jsdelivr.net/pyodide/v0.24.1/full/' });

  // For this implementation, we'll return a mock object that simulates Pyodide
  return createMockPyodide();
};

// Create a mock Pyodide object for this implementation
const createMockPyodide = () => {
  return {
    runPython: (code: string) => {
      // In a real implementation, this would execute Python code
      // For the mock, we'll just return a placeholder result
      console.log("Executing Python code:", code);
      return `Python output for: ${code.substring(0, 50)}...`;
    },
    loadPackage: async (packages: string[]) => {
      console.log("Loading Python packages:", packages);
      // In real implementation, would load packages
    },
    globals: {
      set: (name: string, value: any) => {
        console.log(`Setting global variable ${name} to`, value);
      },
      get: (name: string) => {
        console.log(`Getting global variable ${name}`);
        return undefined;
      }
    }
  };
};

// Execute Python code using Pyodide
export const executePythonCode = async (
  code: string,
  packages: string[] = []
): Promise<PyodideExecutionResult> => {
  try {
    if (!pyodideInstance) {
      await initializePyodide(packages);
    }

    const startTime = Date.now();
    const result = pyodideInstance.runPython(code);
    const endTime = Date.now();

    // In a real implementation, we'd capture output and errors from Pyodide
    const executionTime = endTime - startTime;

    return {
      output: result,
      executionTime
    };
  } catch (error) {
    console.error('Error executing Python code:', error);
    return {
      output: '',
      error: `Python execution error: ${error instanceof Error ? error.message : 'Unknown error'}`,
      executionTime: 0
    };
  }
};

// Validate Python code before execution
export const validatePythonCode = (code: string): string[] => {
  const errors: string[] = [];

  // Check for potentially dangerous code patterns
  const dangerousPatterns = [
    /import\s+os/,
    /import\s+sys/,
    /import\s+subprocess/,
    /import\s+shutil/,
    /import\s+socket/,
    /import\s+threading/,
    /import\s+multiprocessing/,
    /open\s*\(/,
    /exec\s*\(/,
    /file\s*\(/,
    /__import__\s*\(/,
  ];

  for (const pattern of dangerousPatterns) {
    if (pattern.test(code)) {
      errors.push(`Potentially unsafe Python pattern detected: ${pattern}`);
    }
  }

  // Check for long code that might execute for too long
  if (code.split('\n').length > 100) {
    errors.push('Python code exceeds maximum allowed lines (100)');
  }

  return errors;
};

// Safe Python executor with validation
export const safeExecutePython = async (
  code: string,
  packages: string[] = []
): Promise<PyodideExecutionResult> => {
  // Validate the code first
  const validationErrors = validatePythonCode(code);
  if (validationErrors.length > 0) {
    return {
      output: '',
      error: `Validation failed: ${validationErrors.join(', ')}`,
      executionTime: 0
    };
  }

  // Execute the code
  return await executePythonCode(code, packages);
};

// Check if Pyodide is supported in the current environment
export const isPyodideSupported = (): boolean => {
  // In a real implementation, we'd check for WebAssembly support
  return typeof WebAssembly !== 'undefined';
};

// Get available Python packages
export const getAvailablePyodidePackages = (): string[] => {
  // In a real implementation, this would return the list of pre-loaded packages
  return ['numpy', 'matplotlib', 'scipy', 'pandas', 'sympy'];
};