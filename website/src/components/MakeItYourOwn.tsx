import React, { useState } from 'react';
import { CustomProject, UserProgressService } from '../services/user_progress_service';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface MakeItYourOwnProps {
  userId: string;
  baseDemoId: string;
  initialCode: string;
  challengePrompt: string;
}

const MakeItYourOwn: React.FC<MakeItYourOwnProps> = ({ 
  userId, 
  baseDemoId, 
  initialCode, 
  challengePrompt 
}) => {
  const [code, setCode] = useState(initialCode);
  const [title, setTitle] = useState('');
  const [description, setDescription] = useState('');
  const [isShared, setIsShared] = useState(false);
  const [saving, setSaving] = useState(false);
  const [savedProject, setSavedProject] = useState<CustomProject | null>(null);
  const [error, setError] = useState<string | null>(null);
  
  const handleSave = async () => {
    if (!title.trim()) {
      setError('Please enter a title for your project');
      return;
    }
    
    setSaving(true);
    setError(null);
    
    try {
      const service = new UserProgressService();
      await service.initialize(userId);
      
      const project = await service.createCustomProject(
        baseDemoId,
        title,
        code,
        description
      );
      
      // Update sharing status
      await service.updateCustomProject(project.id, { shared: isShared });
      
      setSavedProject(project);
      setError(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to save project');
    } finally {
      setSaving(false);
    }
  };
  
  const handleLoadProject = (projectId: string) => {
    // In a real implementation, this would load the project code
    console.log(`Loading project: ${projectId}`);
  };
  
  return (
    <div className="make-it-your-own">
      <div className="challenge-section">
        <h3>Make it Your Own 🎨</h3>
        <p>{challengePrompt}</p>
      </div>
      
      <div className="project-form">
        <div className="input-group">
          <label htmlFor="project-title">Project Title:</label>
          <input
            id="project-title"
            type="text"
            value={title}
            onChange={(e) => setTitle(e.target.value)}
            placeholder="Give your project a creative name"
          />
        </div>
        
        <div className="input-group">
          <label htmlFor="project-description">Description:</label>
          <textarea
            id="project-description"
            value={description}
            onChange={(e) => setDescription(e.target.value)}
            placeholder="Describe what you've created..."
          />
        </div>
        
        <div className="input-group">
          <label>
            <input
              type="checkbox"
              checked={isShared}
              onChange={(e) => setIsShared(e.target.checked)}
            />
            Share this project with the community
          </label>
        </div>
        
        <div className="editor-section">
          <h4>Your Code:</h4>
          <textarea
            value={code}
            onChange={(e) => setCode(e.target.value)}
            rows={15}
            className="code-editor"
            spellCheck={false}
          />
        </div>
        
        <div className="actions">
          <button 
            onClick={handleSave} 
            disabled={saving || !title.trim()}
            className="save-btn"
          >
            {saving ? 'Saving...' : 'Save Project'}
          </button>
          
          {error && <div className="error">{error}</div>}
          {savedProject && (
            <div className="success">
              Project saved successfully! 
              <button onClick={() => handleLoadProject(savedProject.id)}>
                Load this project
              </button>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

// Wrapper component to ensure it only runs on browser
const MakeItYourOwnWrapper: React.FC<MakeItYourOwnProps> = (props) => (
  <BrowserOnly fallback={<div>Loading project editor...</div>}>
    {() => <MakeItYourOwn {...props} />}
  </BrowserOnly>
);

export default MakeItYourOwnWrapper;