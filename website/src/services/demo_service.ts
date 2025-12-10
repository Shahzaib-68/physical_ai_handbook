import { Demo, createDemo } from '../models/Demo';
import { chapterApi } from '../api/chapters';
import { validateDemo } from '../utils/validation';
import { DemoExecutionError, ValidationError } from '../utils/validation';

export class DemoService {
  // Execute a demo in the browser
  async executeDemo(demoId: string, code: string, config: Record<string, any> = {}) {
    try {
      // Validate the code first
      const validationErrors = this.validateCode(code, []);
      if (validationErrors.length > 0) {
        throw new ValidationError(`Invalid demo code: ${validationErrors.join(', ')}`);
      }

      // Execute via API
      const result = await chapterApi.executeDemo(demoId, code, config);
      return result;
    } catch (error) {
      console.error(`Error executing demo with id ${demoId}:`, error);
      if (error instanceof ValidationError) {
        throw error;
      }
      throw new DemoExecutionError(`Failed to execute demo: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  // Validate demo code
  validateCode(code: string, dependencies: string[]): string[] {
    return validateDemo({ code, dependencies } as Demo);
  }

  // Validate a complete demo object
  validateDemo(demo: Demo): string[] {
    return validateDemo(demo);
  }

  // Create a new demo (for content creation tools)
  createDemo(data: Partial<Demo>): Demo {
    return createDemo(data);
  }

  // Validate demo before execution
  async validateForExecution(demo: Demo): Promise<boolean> {
    const validationErrors = this.validateDemo(demo);
    if (validationErrors.length > 0) {
      console.error('Demo validation failed:', validationErrors);
      return false;
    }
    
    // Additional checks specific to execution environment
    if (!demo.dependencies.every(dep => this.isSupportedDependency(dep))) {
      return false;
    }
    
    return true;
  }

  // Check if dependency is supported
  private isSupportedDependency(dependency: string): boolean {
    const supported = ['p5.js', 'Three.js', 'Pyodide', 'TensorFlow.js', 'WebSerial', 'WebUSB'];
    return supported.includes(dependency);
  }

  // Get available dependencies
  getAvailableDependencies(): string[] {
    return ['p5.js', 'Three.js', 'Pyodide', 'TensorFlow.js', 'WebSerial', 'WebUSB'];
  }

  // Prepare demo for execution environment
  async prepareExecutionEnvironment(demo: Demo) {
    // Load required libraries based on dependencies
    for (const dep of demo.dependencies) {
      await this.loadDependency(dep);
    }
  }

  // Load a specific dependency
  private async loadDependency(dependency: string): Promise<void> {
    switch(dependency) {
      case 'p5.js':
        // Load p5.js
        break;
      case 'Three.js':
        // Load Three.js
        break;
      case 'Pyodide':
        // Initialize Pyodide
        break;
      case 'TensorFlow.js':
        // Load TensorFlow.js
        break;
      case 'WebSerial':
      case 'WebUSB':
        // These are browser APIs, no need to load
        break;
      default:
        throw new Error(`Unsupported dependency: ${dependency}`);
    }
  }
}