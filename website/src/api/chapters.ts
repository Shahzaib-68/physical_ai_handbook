// API service for chapter and demo management
// This would typically connect to a backend API, but for this implementation
// we'll use mock data to simulate API calls

// Types based on the data model
export interface Chapter {
  id: string;
  title: string;
  description: string;
  content: string; // MDX content
  demos: Demo[];
  learningObjectives: string[];
  prerequisites: string[];
  estimatedTime: number;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  tags: string[];
}

export interface Demo {
  id: string;
  title: string;
  description: string;
  code: string;
  config: Record<string, any>;
  dependencies: string[];
  challengePrompt?: string;
  solutionCode?: string;
}

// Mock data for chapters
const mockChapters: Chapter[] = [
  {
    id: 'ch1-intro',
    title: 'Introduction to Physical AI',
    description: 'Learn the basics of Physical AI',
    content: '# Introduction to Physical AI\n\nPhysical AI is the intersection of artificial intelligence and physical systems...',
    demos: [
      {
        id: 'demo1',
        title: 'Simple Physics Simulation',
        description: 'A basic ball physics demo',
        code: '// p5.js code would go here\nfunction setup() {\n  createCanvas(400, 400);\n}\n\nfunction draw() {\n  background(220);\n  circle(mouseX, mouseY, 50);\n}',
        config: {},
        dependencies: ['p5.js'],
        challengePrompt: 'Modify the gravity value and see what happens'
      }
    ],
    learningObjectives: [
      'Understand basic physics concepts',
      'Run your first simulation in browser'
    ],
    prerequisites: [],
    estimatedTime: 8,
    difficulty: 'beginner',
    tags: ['intro', 'physical-ai', 'simulation']
  },
  {
    id: 'ch2-kinematics',
    title: 'Kinematics Fundamentals',
    description: 'Understanding motion in Physical AI',
    content: '# Kinematics in Physical AI\n\nKinematics is the study of motion without considering the forces that cause it...',
    demos: [
      {
        id: 'demo2',
        title: 'Kinematics Visualization',
        description: 'Visualize motion parameters',
        code: '// Three.js code would go here\n// Visualization of motion parameters',
        config: {},
        dependencies: ['Three.js'],
        challengePrompt: 'Add velocity vectors to the visualization'
      }
    ],
    learningObjectives: [
      'Understand kinematics equations',
      'Visualize motion in 3D'
    ],
    prerequisites: ['ch1-intro'],
    estimatedTime: 10,
    difficulty: 'intermediate',
    tags: ['kinematics', 'motion', '3D']
  }
];

// API service functions
export const chapterApi = {
  // Get all chapters
  getChapters: async (): Promise<Chapter[]> => {
    // Simulate API delay
    await new Promise(resolve => setTimeout(resolve, 300));
    return mockChapters;
  },

  // Get a specific chapter by ID
  getChapter: async (id: string): Promise<Chapter | undefined> => {
    // Simulate API delay
    await new Promise(resolve => setTimeout(resolve, 300));
    return mockChapters.find(chapter => chapter.id === id);
  },

  // Execute a demo in the browser (simulated)
  executeDemo: async (demoId: string, code: string, config: Record<string, any>) => {
    // In a real implementation, this would send code to a browser execution environment
    // For this demo, we'll just simulate execution
    await new Promise(resolve => setTimeout(resolve, 500));
    return {
      result: `Demo ${demoId} executed successfully`,
      visualizationData: 'Sample visualization data',
      executionTime: 500
    };
  },

  // Validate demo code (simulated)
  validateDemo: async (demoId: string, code: string) => {
    // In a real implementation, this would validate code syntax and semantics
    // For this demo, we'll just simulate validation
    await new Promise(resolve => setTimeout(resolve, 200));
    return {
      valid: true,
      errors: [],
      warnings: []
    };
  }
};