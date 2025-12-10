// TensorFlow.js integration for AI demos in browser
import * as tf from '@tensorflow/tfjs';
import { getAppConfig } from '../utils/config';
import { DemoExecutionError } from '../utils/validation';

// Interface for TensorFlow.js execution result
interface TFJSExecutionResult {
  output: any;
  executionTime: number;
  modelInfo?: {
    size: number;
    inputShape: number[];
    outputShape: number[];
  };
}

// Initialize TensorFlow.js
export const initializeTensorFlow = async (): Promise<void> => {
  try {
    await tf.ready();
    console.log('TensorFlow.js initialized successfully');
  } catch (error) {
    console.error('Failed to initialize TensorFlow.js:', error);
    throw new DemoExecutionError(`Failed to initialize TensorFlow.js: ${error instanceof Error ? error.message : 'Unknown error'}`);
  }
};

// Create a simple model for demonstration
export const createDemoModel = (): tf.LayersModel => {
  const model = tf.sequential();
  
  // Add layers for a simple example
  model.add(tf.layers.dense({
    units: 10,
    activation: 'relu',
    inputShape: [1]
  }));
  
  model.add(tf.layers.dense({
    units: 1,
    activation: 'linear'
  }));
  
  model.compile({
    optimizer: 'sgd',
    loss: 'meanSquaredError'
  });
  
  return model;
};

// Execute TensorFlow.js code (in a safe manner)
export const executeTFJS = async (
  data: number[] | number[][] | tf.Tensor,
  model?: tf.LayersModel
): Promise<TFJSExecutionResult> => {
  try {
    const startTime = Date.now();
    
    // Ensure TensorFlow is initialized
    await initializeTensorFlow();
    
    // Convert data to tensor if needed
    let inputTensor: tf.Tensor;
    if (data instanceof tf.Tensor) {
      inputTensor = data;
    } else {
      inputTensor = tf.tensor(data);
    }
    
    // Use provided model or create a demo model
    const executionModel = model || createDemoModel();
    
    // Perform the prediction
    const output = executionModel.predict(inputTensor) as tf.Tensor;
    const outputData = await output.data();
    
    const endTime = Date.now();
    
    // Get model info
    const modelInfo = {
      size: executionModel.countParams(),
      inputShape: executionModel.inputShape as number[],
      outputShape: executionModel.outputShape as number[]
    };
    
    // Clean up tensors to prevent memory leaks
    inputTensor.dispose();
    output.dispose();
    
    return {
      output: Array.from(outputData),
      executionTime: endTime - startTime,
      modelInfo
    };
  } catch (error) {
    console.error('Error executing TensorFlow.js code:', error);
    throw new DemoExecutionError(`TensorFlow.js execution error: ${error instanceof Error ? error.message : 'Unknown error'}`);
  }
};

// Train a simple model
export const trainSimpleModel = async (
  xs: number[],
  ys: number[],
  epochs: number = 10
): Promise<TFJSExecutionResult> => {
  try {
    const startTime = Date.now();
    
    // Initialize TensorFlow
    await initializeTensorFlow();
    
    // Create tensors from data
    const xsTensor = tf.tensor2d(xs, [xs.length, 1]);
    const ysTensor = tf.tensor2d(ys, [ys.length, 1]);
    
    // Create model
    const model = createDemoModel();
    
    // Train the model
    const history = await model.fit(xsTensor, ysTensor, {
      epochs: epochs,
      batchSize: 32,
      verbose: 1
    });
    
    const endTime = Date.now();
    
    // Clean up tensors
    xsTensor.dispose();
    ysTensor.dispose();
    
    return {
      output: {
        loss: history.history.loss[history.history.loss.length - 1],
        modelInfo: {
          size: model.countParams(),
          inputShape: model.inputShape as number[],
          outputShape: model.outputShape as number[]
        }
      },
      executionTime: endTime - startTime
    };
  } catch (error) {
    console.error('Error training TensorFlow.js model:', error);
    throw new DemoExecutionError(`TensorFlow.js training error: ${error instanceof Error ? error.message : 'Unknown error'}`);
  }
};

// Load a pre-trained model from URL
export const loadModelFromURL = async (url: string): Promise<tf.LayersModel> => {
  try {
    // In a real implementation, this would load a model from a URL
    // For this demo, we'll just return the demo model
    console.log(`Loading model from: ${url}`);
    const model = await tf.loadLayersModel(url);
    return model;
  } catch (error) {
    console.error('Error loading model from URL:', error);
    throw new DemoExecutionError(`Failed to load model: ${error instanceof Error ? error.message : 'Unknown error'}`);
  }
};

// Get TensorFlow.js environment info
export const getTFJSInfo = () => {
  return {
    backend: tf.getBackend(),
    version: tf.version.tfjs,
    isGPUSupported: tf.engine().backendName === 'webgl'
  };
};

// Validate TensorFlow.js code (basic validation)
export const validateTFJS = (code: string): string[] => {
  const errors: string[] = [];

  // Check for potentially problematic TensorFlow operations
  const suspiciousPatterns = [
    /tf\.memory\(\)\.numBytes\s*>\s*\d+/,  // Memory checks
  ];

  for (const pattern of suspiciousPatterns) {
    if (pattern.test(code)) {
      errors.push(`Code contains pattern that may cause performance issues: ${pattern}`);
    }
  }

  return errors;
};

// Cleanup function to dispose of tensors and models
export const cleanupTFJS = () => {
  // Dispose all variables and clear the backend
  tf.disposeVariables();
  tf.disposeVariables(); // Call twice to ensure all are disposed
};