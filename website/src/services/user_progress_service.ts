import { Chapter } from '../models/Chapter';

export interface UserProgress {
  userId: string;
  completedChapters: string[]; // list of completed chapter IDs
  demoProgress: Record<string, 'not-started' | 'in-progress' | 'completed'>; // mapping of demo IDs to progress
  customProjects: CustomProject[]; // projects created by user in "make it your own" mode
  timestamp: Date; // when progress was last updated
}

export interface CustomProject {
  id: string;
  userId: string;
  baseDemoId: string; // the original demo this project was based on
  title: string;
  code: string;
  description: string;
  shared: boolean; // whether the project is shared publicly
  createdAt: Date;
  updatedAt: Date;
}

export class UserProgressService {
  private progress: UserProgress | null = null;
  
  // Initialize user progress
  async initialize(userId: string, initialChapters: string[] = []): Promise<UserProgress> {
    // Check if progress exists in storage
    const stored = this.getStoredProgress(userId);
    
    if (stored) {
      this.progress = stored;
      return stored;
    }
    
    // Create new progress
    this.progress = {
      userId,
      completedChapters: initialChapters,
      demoProgress: {},
      customProjects: [],
      timestamp: new Date()
    };
    
    this.saveProgress();
    return this.progress;
  }
  
  // Get current user progress
  getProgress(): UserProgress | null {
    return this.progress;
  }
  
  // Mark a chapter as completed
  async markChapterCompleted(chapterId: string): Promise<void> {
    if (!this.progress) {
      throw new Error('User progress not initialized');
    }
    
    if (!this.progress.completedChapters.includes(chapterId)) {
      this.progress.completedChapters.push(chapterId);
      this.progress.timestamp = new Date();
      this.saveProgress();
    }
  }
  
  // Check if a chapter is completed
  isChapterCompleted(chapterId: string): boolean {
    if (!this.progress) return false;
    return this.progress.completedChapters.includes(chapterId);
  }
  
  // Update demo progress
  async updateDemoProgress(demoId: string, status: 'not-started' | 'in-progress' | 'completed'): Promise<void> {
    if (!this.progress) {
      throw new Error('User progress not initialized');
    }
    
    this.progress.demoProgress[demoId] = status;
    this.progress.timestamp = new Date();
    this.saveProgress();
  }
  
  // Save progress to local storage
  private saveProgress(): void {
    if (!this.progress) return;
    
    try {
      localStorage.setItem(
        `progress-${this.progress.userId}`, 
        JSON.stringify({
          ...this.progress,
          timestamp: this.progress.timestamp.toISOString(),
          customProjects: this.progress.customProjects.map(p => ({
            ...p,
            createdAt: p.createdAt.toISOString(),
            updatedAt: p.updatedAt.toISOString()
          }))
        })
      );
    } catch (error) {
      console.error('Failed to save progress to localStorage:', error);
    }
  }
  
  // Load progress from local storage
  private getStoredProgress(userId: string): UserProgress | null {
    try {
      const stored = localStorage.getItem(`progress-${userId}`);
      if (!stored) return null;
      
      const parsed = JSON.parse(stored);
      return {
        ...parsed,
        timestamp: new Date(parsed.timestamp),
        customProjects: parsed.customProjects.map((p: any) => ({
          ...p,
          createdAt: new Date(p.createdAt),
          updatedAt: new Date(p.updatedAt)
        }))
      };
    } catch (error) {
      console.error('Failed to load progress from localStorage:', error);
      return null;
    }
  }
  
  // Create a custom project
  async createCustomProject(
    baseDemoId: string,
    title: string,
    code: string,
    description: string
  ): Promise<CustomProject> {
    if (!this.progress) {
      throw new Error('User progress not initialized');
    }
    
    const project: CustomProject = {
      id: `project-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      userId: this.progress.userId,
      baseDemoId,
      title,
      code,
      description,
      shared: false,
      createdAt: new Date(),
      updatedAt: new Date()
    };
    
    this.progress.customProjects.push(project);
    this.progress.timestamp = new Date();
    this.saveProgress();
    
    return project;
  }
  
  // Update a custom project
  async updateCustomProject(
    projectId: string,
    updates: Partial<CustomProject>
  ): Promise<void> {
    if (!this.progress) {
      throw new Error('User progress not initialized');
    }
    
    const projectIndex = this.progress.customProjects.findIndex(p => p.id === projectId);
    if (projectIndex === -1) {
      throw new Error(`Project with id ${projectId} not found`);
    }
    
    // Update project
    this.progress.customProjects[projectIndex] = {
      ...this.progress.customProjects[projectIndex],
      ...updates,
      updatedAt: new Date()
    };
    
    this.progress.timestamp = new Date();
    this.saveProgress();
  }
  
  // Check if user can access a chapter based on prerequisites
  async canAccessChapter(chapter: Chapter): Promise<boolean> {
    if (!this.progress) {
      throw new Error('User progress not initialized');
    }
    
    // If no prerequisites, user can access
    if (!chapter.prerequisites || chapter.prerequisites.length === 0) {
      return true;
    }
    
    // Check if user has completed all prerequisites
    return chapter.prerequisites.every(prereq => 
      this.progress!.completedChapters.includes(prereq)
    );
  }
  
  // Get user's current learning path
  async getLearningPath(allChapters: Chapter[]): Promise<Chapter[]> {
    if (!this.progress) {
      throw new Error('User progress not initialized');
    }
    
    // Filter chapters based on prerequisites and completion status
    return allChapters.filter(chapter => {
      // If already completed, might still be relevant for review
      if (this.isChapterCompleted(chapter.id)) return true;
      
      // If not completed, check if prerequisites are met
      return this.canAccessChapter(chapter);
    });
  }
}