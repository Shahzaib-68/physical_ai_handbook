---
id: 1
title: Tasks Physical AI Book
stage: tasks
date: 2024-12-09
surface: agent
model: Qwen
feature: physical-ai-book
branch: 1-physical-ai-book
user: user
command: /sp.tasks
labels: [tasks,physical-ai,book,education]
links:
  spec: specs/1-physical-ai-book/spec.md
  plan: specs/1-physical-ai-book/plan.md
  adr: null
  pr: null
files:
 - specs/1-physical-ai-book/tasks.md
tests:
 - null
---

## Prompt

```
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load design documents**: Read from FEATURE_DIR:
   - **Required**: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
   - **Optional**: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
   - Note: Not all projects have all documents. Generate tasks based on what's available.

3. **Execute task generation workflow**:
   - Load plan.md and extract tech stack, libraries, project structure
   - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
   - If data-model.md exists: Extract entities and map to user stories
   - If contracts/ exists: Map endpoints to user stories
   - If research.md exists: Extract decisions for setup tasks
   - Generate tasks organized by user story (see Task Generation Rules below)
   - Generate dependency graph showing user story completion order
   - Create parallel execution examples per user story
   - Validate task completeness (each user story has all needed tasks, independently testable)

4. **Generate tasks.md**: Use `.specify.specify/templates/tasks-template.md` as structure, fill with:
   - Correct feature name from plan.md
   - Phase 1: Setup tasks (project initialization)
   - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
   - Phase 3+: One phase per user story (in priority order from spec.md)
   - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
   - Final Phase: Polish & cross-cutting concerns
   - All tasks must follow the strict checklist format (see Task Generation Rules below)
   - Clear file paths for each task
   - Dependencies section showing story completion order
   - Parallel execution examples per story
   - Implementation strategy section (MVP first, incremental delivery)

5. **Report**: Output path to generated tasks.md and summary:
   - Total task count
   - Task count per user story
   - Parallel opportunities identified
   - Independent test criteria for each story
   - Suggested MVP scope (typically just User Story 1)
   - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)

Context for task generation: Project: Physical AI & Humanoid Robotics Textbook Goal: Build the official hackathon-winning interactive textbook using Docusaurus that fully follows the given course modules (ROS 2, Gazebo, NVIDIA Isaac Sim, VLA, humanoid simulation, voice-to-action capstone) Create a clean, prioritized task list in Markdown with exactly this format: ## Backlog → To Do → In Progress → Review → Done (Use emoji flags: To Do, In Progress, Review, Done) Tasks must cover: - Docusaurus site setup + GitHub Pages deployment - 12–14 chapters following the exact weekly breakdown in the hackathon doc - Embedded runnable code + videos + diagrams for ROS 2, Gazebo, Isaac Sim - Simulated humanoid robot demos (at least 3 working examples) - Voice-to-Action VLA integration demo - Optional: simple RAG chatbot - Final README + submission video prep Rules: - Every task ≤ 5 days max - Mark hardware-related tasks clearly (Jetson, RealSense, etc.) - Add realistic time estimates and priority (P0 = must-have, P1 = nice-to-have) - Total tasks ≤ 50 - Tone: professional, excited, judge-friendly

The tasks.md should be immediately executable - each task must be specific enough that an LLM can complete it without additional context.

## Response snapshot

I'll execute the task generation workflow for the Physical AI book project. Let me start by checking the prerequisites and getting the feature directory information:

...

## Outcome

- ✅ Impact: Created comprehensive task list for Physical AI book project
- 🧪 Tests: none
- 📁 Files: Created tasks.md following the correct format and requirements
- 🔁 Next prompts: Ready for implementation
- 🧠 Reflection: Successfully generated a task list that aligns with browser-based, accessible Physical AI book requirements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All tasks follow the required format
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Not applicable