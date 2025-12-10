import { Chapter, createChapter } from '../models/Chapter';
import { Demo, createDemo } from '../models/Demo';
import { chapterApi } from '../api/chapters';
import { validateChapter, validateDemo } from '../utils/validation';

export class ChapterService {
  // Get all chapters
  async getAllChapters(): Promise<Chapter[]> {
    try {
      const chapters = await chapterApi.getChapters();
      return chapters;
    } catch (error) {
      console.error('Error fetching chapters:', error);
      throw error;
    }
  }

  // Get a specific chapter by ID
  async getChapterById(id: string): Promise<Chapter | null> {
    try {
      const chapter = await chapterApi.getChapter(id);
      return chapter || null;
    } catch (error) {
      console.error(`Error fetching chapter with id ${id}:`, error);
      throw error;
    }
  }

  // Validate a chapter
  validateChapter(chapter: Chapter): string[] {
    return validateChapter(chapter);
  }

  // Create a new chapter (for content creation tools)
  createChapter(data: Partial<Chapter>): Chapter {
    return createChapter(data);
  }

  // Check if a user can access a chapter based on prerequisites
  async canAccessChapter(chapterId: string, userProgress: string[]): Promise<boolean> {
    const chapter = await this.getChapterById(chapterId);
    if (!chapter) return false;

    // If no prerequisites, user can access
    if (!chapter.prerequisites || chapter.prerequisites.length === 0) {
      return true;
    }

    // Check if user has completed all prerequisites
    return chapter.prerequisites.every(prereq => userProgress.includes(prereq));
  }

  // Get next chapter in sequence
  async getNextChapter(currentChapterId: string, allChapters: Chapter[]): Promise<Chapter | null> {
    const currentIndex = allChapters.findIndex(ch => ch.id === currentChapterId);
    if (currentIndex === -1 || currentIndex >= allChapters.length - 1) {
      return null; // No next chapter
    }
    return allChapters[currentIndex + 1];
  }
}