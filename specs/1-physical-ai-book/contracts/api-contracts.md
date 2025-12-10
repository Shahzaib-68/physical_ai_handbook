# API Contracts: Physical AI – Hands-On Book for Everyone

**Feature**: 1-physical-ai-book
**Date**: 2024-12-09

## Overview

This document specifies the API contracts for the Physical AI interactive textbook. These are browser-based APIs that enable the interactive functionality of the book.

## Endpoints

### Chapter Management

#### `GET /api/chapters`
Get a list of all available chapters

**Request**:
- Method: GET
- Path: /api/chapters
- Headers: None required

**Response**:
- Status: 200
- Content-Type: application/json
- Body:
```json
{
  "chapters": [
    {
      "id": "ch1-intro",
      "title": "Introduction to Physical AI",
      "description": "Learn the basics of Physical AI",
      "estimatedTime": 8,
      "difficulty": "beginner",
      "tags": ["intro", "physical-ai", "simulation"]
    }
  ]
}
```

#### `GET /api/chapters/{id}`
Get a specific chapter's content

**Request**:
- Method: GET
- Path: /api/chapters/{id}
- Headers: None required

**Response**:
- Status: 200
- Content-Type: application/json
- Body:
```json
{
  "id": "ch1-intro",
  "title": "Introduction to Physical AI",
  "description": "Learn the basics of Physical AI",
  "content": "MDX content of the chapter",
  "demos": [
    {
      "id": "demo1",
      "title": "Simple Physics Simulation",
      "description": "A basic ball physics demo",
      "code": "initial demo code",
      "config": {},
      "dependencies": ["p5.js"],
      "challengePrompt": "Modify the gravity value and see what happens"
    }
  ],
  "learningObjectives": [
    "Understand basic physics concepts",
    "Run your first simulation in browser"
  ],
  "prerequisites": [],
  "estimatedTime": 8,
  "difficulty": "beginner",
  "tags": ["intro", "physical-ai", "simulation"]
}
```

### Demo Execution

#### `POST /api/demos/execute`
Execute a demo in the browser environment

**Request**:
- Method: POST
- Path: /api/demos/execute
- Headers: 
  - Content-Type: application/json
- Body:
```json
{
  "demoId": "demo1",
  "code": "user's modified code",
  "config": {}
}
```

**Response**:
- Status: 200
- Content-Type: application/json
- Body:
```json
{
  "result": "execution result",
  "visualizationData": "data for 3D visualization",
  "executionTime": 1200
}
```

#### `POST /api/demos/validate`
Validate user code before execution

**Request**:
- Method: POST
- Path: /api/demos/validate
- Headers: 
  - Content-Type: application/json
- Body:
```json
{
  "demoId": "demo1",
  "code": "user's modified code"
}
```

**Response**:
- Status: 200
- Content-Type: application/json
- Body:
```json
{
  "valid": true,
  "errors": [],
  "warnings": []
}
```

### User Progress

#### `GET /api/user/progress`
Get user's progress through the book

**Request**:
- Method: GET
- Path: /api/user/progress
- Headers: Authorization: Bearer {token}

**Response**:
- Status: 200
- Content-Type: application/json
- Body:
```json
{
  "userId": "user-123",
  "completedChapters": ["ch1-intro"],
  "demoProgress": {
    "demo1": "completed",
    "demo2": "in-progress"
  },
  "customProjects": [
    {
      "id": "proj-1",
      "baseDemoId": "demo1",
      "title": "My Custom Physics",
      "shared": false,
      "createdAt": "2024-12-09T10:00:00Z"
    }
  ]
}
```

#### `PUT /api/user/progress`
Update user's progress

**Request**:
- Method: PUT
- Path: /api/user/progress
- Headers: 
  - Content-Type: application/json
  - Authorization: Bearer {token}
- Body:
```json
{
  "chapterId": "ch1-intro",
  "status": "completed"
}
```

**Response**:
- Status: 200
- Content-Type: application/json
- Body:
```json
{
  "message": "Progress updated successfully"
}
```

### Custom Projects

#### `POST /api/projects`
Save a custom project created by the user

**Request**:
- Method: POST
- Path: /api/projects
- Headers: 
  - Content-Type: application/json
  - Authorization: Bearer {token}
- Body:
```json
{
  "baseDemoId": "demo1",
  "title": "My Custom Physics",
  "code": "user's custom code",
  "description": "A modified version of the original demo"
}
```

**Response**:
- Status: 201
- Content-Type: application/json
- Body:
```json
{
  "id": "proj-123",
  "message": "Project saved successfully"
}
```

#### `GET /api/projects/{id}`
Get a specific custom project

**Request**:
- Method: GET
- Path: /api/projects/{id}
- Headers: Authorization: Bearer {token}

**Response**:
- Status: 200
- Content-Type: application/json
- Body:
```json
{
  "id": "proj-123",
  "userId": "user-123",
  "baseDemoId": "demo1",
  "title": "My Custom Physics",
  "code": "user's custom code",
  "description": "A modified version of the original demo",
  "shared": false,
  "createdAt": "2024-12-09T10:00:00Z",
  "updatedAt": "2024-12-09T10:30:00Z"
}
```

#### `PUT /api/projects/{id}/share`
Update sharing status of a project

**Request**:
- Method: PUT
- Path: /api/projects/{id}/share
- Headers: 
  - Content-Type: application/json
  - Authorization: Bearer {token}
- Body:
```json
{
  "shared": true
}
```

**Response**:
- Status: 200
- Content-Type: application/json
- Body:
```json
{
  "message": "Project sharing updated successfully"
}
```