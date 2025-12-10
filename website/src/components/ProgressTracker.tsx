import React, { useEffect, useState } from 'react';
import { Chapter } from '../models/Chapter';
import { UserProgressService } from '../services/user_progress_service';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface ProgressTrackerProps {
  userId: string;
  currentChapterId: string;
  allChapters: Chapter[];
}

const ProgressTracker: React.FC<ProgressTrackerProps> = ({ 
  userId, 
  currentChapterId, 
  allChapters 
}) => {
  const [progress, setProgress] = useState<number>(0);
  const [completedChapters, setCompletedChapters] = useState<string[]>([]);
  const [loading, setLoading] = useState(true);
  
  useEffect(() => {
    const loadProgress = async () => {
      try {
        const service = new UserProgressService();
        await service.initialize(userId);
        
        const userProgress = service.getProgress();
        if (userProgress) {
          setCompletedChapters(userProgress.completedChapters);
          
          // Calculate progress percentage
          const progressPercent = Math.round(
            (userProgress.completedChapters.length / allChapters.length) * 100
          );
          setProgress(progressPercent);
        }
      } catch (error) {
        console.error('Error loading user progress:', error);
      } finally {
        setLoading(false);
      }
    };
    
    loadProgress();
  }, [userId, allChapters.length]);
  
  // Mark current chapter as completed
  const markCurrentChapterCompleted = async () => {
    const service = new UserProgressService();
    await service.initialize(userId);
    await service.markChapterCompleted(currentChapterId);
    
    // Update local state
    if (!completedChapters.includes(currentChapterId)) {
      setCompletedChapters([...completedChapters, currentChapterId]);
      const newProgress = Math.round(
        ((completedChapters.length + 1) / allChapters.length) * 100
      );
      setProgress(newProgress);
    }
  };
  
  if (loading) {
    return <div>Loading progress...</div>;
  }
  
  return (
    <div className="progress-tracker">
      <div className="progress-bar-container">
        <div className="progress-label">
          Your Learning Progress: {progress}%
        </div>
        <div className="progress-bar">
          <div 
            className="progress-fill" 
            style={{ width: `${progress}%` }}
          />
        </div>
        <div className="progress-stats">
          {completedChapters.length} of {allChapters.length} chapters completed
        </div>
      </div>
      
      <div className="chapter-list">
        <h3>All Chapters:</h3>
        <ul>
          {allChapters.map((chapter, index) => {
            const isCompleted = completedChapters.includes(chapter.id);
            const isCurrent = chapter.id === currentChapterId;
            
            return (
              <li 
                key={chapter.id} 
                className={`chapter-item ${isCompleted ? 'completed' : ''} ${isCurrent ? 'current' : ''}`}
              >
                <span className="chapter-index">{index + 1}.</span>
                <span className="chapter-title">{chapter.title}</span>
                {isCompleted && <span className="status completed">✓ Completed</span>}
                {isCurrent && !isCompleted && (
                  <button 
                    onClick={markCurrentChapterCompleted}
                    className="complete-btn"
                  >
                    Mark as Completed
                  </button>
                )}
              </li>
            );
          })}
        </ul>
      </div>
    </div>
  );
};

// Wrapper component to ensure it only runs on browser
const ProgressTrackerWrapper: React.FC<ProgressTrackerProps> = (props) => (
  <BrowserOnly fallback={<div>Loading progress tracker...</div>}>
    {() => <ProgressTracker {...props} />}
  </BrowserOnly>
);

export default ProgressTrackerWrapper;