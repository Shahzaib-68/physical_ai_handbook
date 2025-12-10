// Chapter model based on the data model specification
export interface Chapter {
  id: string;
  title: string;
  description: string;
  content: string; // MDX content of the chapter
  demos: Demo[]; // List of interactive demos in the chapter
  learningObjectives: string[]; // List of learning objectives
  prerequisites: string[]; // Prerequisites from previous chapters
  estimatedTime: number; // Estimated time to complete in minutes
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  tags: string[]; // List of tags for search/filtering
}

// Validation function for Chapter
export const validateChapter = (chapter: Partial<Chapter>): string[] => {
  const errors: string[] = [];

  if (!chapter.id) errors.push('Chapter ID is required');
  if (!chapter.title) errors.push('Chapter title is required');
  if (!chapter.description) errors.push('Chapter description is required');
  if (!chapter.content) errors.push('Chapter content is required');
  if (!chapter.demos) errors.push('At least one demo is required');
  if (chapter.estimatedTime && chapter.estimatedTime > 10) {
    errors.push('Estimated time must be ≤ 10 minutes');
  }

  return errors;
};

// Factory function to create a new Chapter
export const createChapter = (data: Partial<Chapter>): Chapter => {
  return {
    id: data.id || '',
    title: data.title || '',
    description: data.description || '',
    content: data.content || '',
    demos: data.demos || [],
    learningObjectives: data.learningObjectives || [],
    prerequisites: data.prerequisites || [],
    estimatedTime: data.estimatedTime || 8, // Default to 8 minutes
    difficulty: data.difficulty || 'beginner',
    tags: data.tags || []
  };
};