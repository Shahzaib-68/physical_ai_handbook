// Demo model based on the data model specification
export interface Demo {
  id: string;
  title: string;
  description: string;
  code: string; // Initial code for the demo
  config: Record<string, any>; // Configuration options for the demo
  dependencies: string[]; // List of tech dependencies like Pyodide, TensorFlow.js, etc.
  challengePrompt?: string; // Optional prompt for "make it your own" challenge
  solutionCode?: string; // Optional solution code for the challenge
}

// Validation function for Demo
export const validateDemo = (demo: Partial<Demo>): string[] => {
  const errors: string[] = [];

  if (!demo.id) errors.push('Demo ID is required');
  if (!demo.title) errors.push('Demo title is required');
  if (!demo.description) errors.push('Demo description is required');
  if (!demo.code) errors.push('Demo code is required');
  if (!demo.dependencies) errors.push('Demo dependencies are required');
  if (demo.challengePrompt && !demo.solutionCode) {
    errors.push('Solution code is required when challenge prompt is provided');
  }

  return errors;
};

// Factory function to create a new Demo
export const createDemo = (data: Partial<Demo>): Demo => {
  return {
    id: data.id || '',
    title: data.title || '',
    description: data.description || '',
    code: data.code || '',
    config: data.config || {},
    dependencies: data.dependencies || [],
    challengePrompt: data.challengePrompt,
    solutionCode: data.solutionCode
  };
};