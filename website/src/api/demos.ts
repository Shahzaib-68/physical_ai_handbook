// Validation for demo execution
import { DemoExecutionError, ValidationError } from '../utils/validation';

// Validate that the code is safe to execute in browser environment
export const validateDemoCode = (code: string, dependencies: string[]): string[] => {
  const errors: string[] = [];

  // Check for potentially dangerous code patterns
  const dangerousPatterns = [
    /eval\s*\(/,  // eval function
    /new\s+Function\s*\(/,  // Function constructor
    /setTimeout\s*\(\s*["'].*["']/,  // String-based setTimeout
    /setInterval\s*\(\s*["'].*["']/,  // String-based setInterval
    /document\.cookie/,  // Cookie access
    /localStorage\.setItem/,  // Local storage write
    /sessionStorage\.setItem/,  // Session storage write
  ];

  for (const pattern of dangerousPatterns) {
    if (pattern.test(code)) {
      errors.push(`Potentially unsafe code pattern detected: ${pattern}`);
    }
  }

  // Validate dependencies are allowed
  const allowedDeps = ['p5.js', 'Three.js', 'Pyodide', 'TensorFlow.js', 'WebSerial', 'WebUSB'];
  for (const dep of dependencies) {
    if (!allowedDeps.includes(dep)) {
      errors.push(`Unsupported dependency: ${dep}`);
    }
  }

  return errors;
};

// Validate execution parameters
export const validateExecutionParams = (
  demoId: string,
  code: string,
  config: Record<string, any>
): string[] => {
  const errors: string[] = [];

  if (!demoId || typeof demoId !== 'string') {
    errors.push('Valid demo ID is required');
  }

  if (!code || typeof code !== 'string') {
    errors.push('Valid code is required');
  }

  if (typeof config !== 'object') {
    errors.push('Config must be an object');
  }

  // Validate code length limits
  if (code.length > 10000) { // 10KB limit
    errors.push('Code exceeds maximum allowed length (10KB)');
  }

  return errors;
};

// Validate execution environment
export const validateExecutionEnvironment = (): string[] => {
  const errors: string[] = [];

  // Check if we're in a browser environment
  if (typeof window === 'undefined') {
    errors.push('Execution environment must be a browser');
  }

  // In a real implementation, we would check for required browser features
  // and security policies here

  return errors;
};

// Main validation function to call before executing a demo
export const validateDemoExecution = (
  demoId: string,
  code: string,
  dependencies: string[],
  config: Record<string, any> = {}
): { isValid: boolean; errors: string[] } => {
  const allErrors = [
    ...validateExecutionParams(demoId, code, config),
    ...validateDemoCode(code, dependencies),
    ...validateExecutionEnvironment()
  ];

  return {
    isValid: allErrors.length === 0,
    errors: allErrors
  };
};

// Enhanced validator with additional security checks
export class SecureDemoValidator {
  static async validateExecution(
    demoId: string,
    code: string,
    dependencies: string[],
    config: Record<string, any> = {}
  ): Promise<{ isValid: boolean; errors: string[] }> {
    // Perform standard validation first
    const standardValidation = validateDemoExecution(demoId, code, dependencies, config);
    if (!standardValidation.isValid) {
      return standardValidation;
    }

    // In a real implementation, we might perform additional checks like:
    // 1. Check execution time limits
    // 2. Validate code syntax without executing
    // 3. Perform sandboxed execution in an iframe for additional security
    // 4. Check for resource usage (memory, CPU)

    // For now, return the standard validation result
    return standardValidation;
  }
}