import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { chapterApi } from '../api/chapters';
import { DemoService } from '../services/demo_service';
import { useErrorHandler } from '../utils/ErrorHandler';

interface DemoProps {
  id: string;
  title: string;
  description: string;
  code: string;
  dependencies: string[];
  challengePrompt?: string;
}

const InteractiveDemo: React.FC<DemoProps> = ({
  id,
  title,
  description,
  code,
  dependencies,
  challengePrompt
}) => {
  const [demoCode, setDemoCode] = useState(code);
  const [executionResult, setExecutionResult] = useState<string | null>(null);
  const [isExecuting, setIsExecuting] = useState(false);
  const [executionTime, setExecutionTime] = useState<number | null>(null);
  const { setError } = useErrorHandler();

  // Demo service instance
  const demoService = new DemoService();

  // Initialize the demo environment when component mounts
  useEffect(() => {
    // Load required libraries based on dependencies
    const loadDependencies = async () => {
      try {
        for (const dep of dependencies) {
          switch(dep) {
            case 'p5.js':
              // In a real implementation, we would load p5.js
              break;
            case 'Three.js':
              // In a real implementation, we would load Three.js
              break;
            case 'Pyodide':
              // In a real implementation, we would initialize Pyodide environment
              break;
            case 'TensorFlow.js':
              // In a real implementation, we would initialize TensorFlow.js environment
              break;
            default:
              console.warn(`Unknown dependency: ${dep}`);
          }
        }
      } catch (err) {
        setError(err as Error);
      }
    };

    loadDependencies();
  }, [dependencies, setError]);

  const handleExecute = async () => {
    setIsExecuting(true);
    setExecutionResult(null);
    setExecutionTime(null);

    try {
      // Validate the code before execution
      await demoService.validateForExecution({
        id,
        title,
        description,
        code: demoCode,
        dependencies,
        challengePrompt
      });

      // Execute via API
      const startTime = Date.now();
      const result = await chapterApi.executeDemo(id, demoCode, {});
      const endTime = Date.now();

      setExecutionResult(result.result as string);
      setExecutionTime(endTime - startTime);
    } catch (error) {
      setError(error as Error);
      setExecutionResult(`Error: ${(error as Error).message}`);
    } finally {
      setIsExecuting(false);
    }
  };

  const handleReset = () => {
    setDemoCode(code);
    setExecutionResult(null);
    setExecutionTime(null);
  };

  return (
    <div className="interactive-demo">
      <div className="demo-header">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>

      <div className="demo-editor">
        <div className="editor-header">
          <span>Code Editor</span>
          <div className="editor-controls">
            <button onClick={handleExecute} disabled={isExecuting}>
              {isExecuting ? 'Executing...' : 'Run Demo'}
            </button>
            <button onClick={handleReset}>Reset</button>
          </div>
        </div>
        <textarea
          value={demoCode}
          onChange={(e) => setDemoCode(e.target.value)}
          rows={15}
          className="code-editor"
          spellCheck={false}
        />
      </div>

      <div className="demo-output">
        <div className="output-header">
          <h4>Output:</h4>
          {executionTime !== null && <span className="execution-time">Execution time: {executionTime}ms</span>}
        </div>
        {executionResult && <div className="execution-result">{executionResult}</div>}
      </div>

      {challengePrompt && (
        <div className="demo-challenge">
          <h4>Make it your own 🎨</h4>
          <p>{challengePrompt}</p>
        </div>
      )}
    </div>
  );
};

// Wrapper component to ensure it only runs on browser
const InteractiveDemoWrapper: React.FC<DemoProps> = (props) => (
  <BrowserOnly fallback={<div>Loading interactive demo...</div>}>
    {() => <InteractiveDemo {...props} />}
  </BrowserOnly>
);

export default InteractiveDemoWrapper;