# Data Model: Physical AI – Hands-On Book for Everyone

**Feature**: 1-physical-ai-book
**Date**: 2024-12-09

## Entities

### Chapter
- **Description**: A self-contained learning unit in the Physical AI book
- **Fields**:
  - id: string (unique identifier like "ch1-intro" or "ch5-control")
  - title: string (chapter title)
  - description: string (brief description of the chapter content)
  - content: string (MDX content of the chapter)
  - demos: Demo[] (list of interactive demos in the chapter)
  - learningObjectives: string[] (list of learning objectives)
  - prerequisites: string[] (prerequisites from previous chapters)
  - estimatedTime: number (estimated time to complete in minutes)
  - difficulty: "beginner" | "intermediate" | "advanced"
  - tags: string[] (list of tags for search/filtering)

### Demo
- **Description**: An interactive browser-based simulation in a chapter
- **Fields**:
  - id: string (unique identifier within the chapter)
  - title: string (demo title)
  - description: string (brief description of the demo)
  - code: string (initial code for the demo)
  - config: object (configuration options for the demo)
  - dependencies: string[] (list of tech dependencies like Pyodide, TensorFlow.js, etc.)
  - challengePrompt: string (optional prompt for "make it your own" challenge)
  - solutionCode: string (optional solution code for the challenge)

### UserProgress
- **Description**: Tracks user progress through the book
- **Fields**:
  - userId: string (unique user identifier)
  - completedChapters: string[] (list of completed chapter IDs)
  - demoProgress: object (mapping of demo IDs to progress/completion status)
  - customProjects: CustomProject[] (projects created by user in "make it your own" mode)
  - timestamp: Date (when progress was last updated)

### CustomProject
- **Description**: A project created by a user during "make it your own" challenges
- **Fields**:
  - id: string (unique identifier)
  - userId: string (user who created it)
  - baseDemoId: string (the original demo this project was based on)
  - title: string (user-defined title)
  - code: string (customized code)
  - description: string (optional user description)
  - shared: boolean (whether the project is shared publicly)
  - createdAt: Date (timestamp of creation)
  - updatedAt: Date (timestamp of last update)

### Comment
- **Description**: Community discussion around a chapter or demo
- **Fields**:
  - id: string (unique identifier)
  - userId: string (user who made the comment)
  - targetId: string (ID of chapter or demo being commented on)
  - targetType: "chapter" | "demo" (type of target being commented on)
  - content: string (content of the comment)
  - timestamp: Date (when comment was made)
  - replies: Comment[] (nested replies to this comment)

## State Transitions

### UserProgress State Transitions
- New User → Chapter In Progress: When user starts a new chapter
- Chapter In Progress → Chapter Completed: When user completes all requirements for a chapter
- Chapter Completed → Custom Project Created: When user completes "make it your own" challenge

### Demo State Transitions
- Created → Running: When user starts the interactive demo
- Running → Modified: When user changes code in the demo
- Modified → Saved: When user saves their custom version

## Validation Rules

1. **Chapter**:
   - Title and description are required
   - Estimated time must be ≤ 10 minutes
   - Must have at least one demo

2. **Demo**:
   - Code must be valid for the specified dependencies
   - Challenge prompt is optional but if present, should have a solution
   - Must run in under 2 minutes

3. **UserProgress**:
   - userId is required
   - Cannot mark chapters as complete without prerequisites

4. **CustomProject**:
   - Must have a valid baseDemoId
   - Title and code are required
   - Cannot be shared if it contains invalid code

## Relationships

- Chapter (1) → Demo (0..n): One chapter contains zero or more demos
- User (1) → UserProgress (1): One user has one progress record
- UserProgress (1) → CustomProject (0..n): One user progress record contains zero or more custom projects
- Chapter/Demo (1) → Comment (0..n): One chapter or demo can have zero or more comments