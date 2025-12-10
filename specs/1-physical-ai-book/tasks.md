# Tasks: Physical AI – Hands-On Book for Everyone

**Input**: Design documents from `/specs/1-physical-ai-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

**Constitution Alignment**: All tasks must ensure browser-first experience (<2 mins setup), hands-on interactivity, and inclusive accessibility (Hinglish + emojis).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions
- Tasks must support browser-based, hands-on experience as per constitution

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- For Physical AI book: Focus on client-side browser implementations
- Paths shown below assume single project - adjust based on plan.md structure

<!--
  ============================================================================
  IMPORTANT: The tasks below are SAMPLE TASKS for illustration purposes only.

  The /sp.tasks command MUST replace these with actual tasks based on:
  - User stories from spec.md (with their priorities P1, P2, P3...)
  - Feature requirements from plan.md
  - Entities from data-model.md
  - Endpoints from contracts/

  Tasks MUST be organized by user story so each story can be:
  - Implemented independently
  - Tested independently
  - Delivered as an MVP increment

  Tasks MUST align with Physical AI book constitution:
  - Browser-First Experience: All examples run in browser in <2 mins
  - Hands-On Mastery: Each concept has interactive examples
  - Inclusive Accessibility: Casual Hinglish English + emojis
  - Progressive Learning: Build from simple to complex
  - Real-World Relevance: Practical applications
  - Community-Driven Growth: Encourage experimentation

  DO NOT keep these sample tasks in the generated tasks.md file.
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project structure with Docusaurus per implementation plan
- [X] T002 Initialize JavaScript/TypeScript project with MDX dependencies
- [X] T003 [P] Configure linting and formatting tools for MDX files

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [X] T004 Setup Docusaurus configuration with proper MDX support
- [X] T005 [P] Create base components for InteractiveDemo, PhysicsSimulator, RobotController
- [X] T006 [P] Setup API endpoints structure for chapter management
- [X] T007 Create base Chapter and Demo models based on data-model.md
- [X] T008 Configure error handling and validation for browser-based execution
- [X] T009 Setup environment configuration management for browser execution

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Interactive Chapter Experience (Priority: P1) 🎯 MVP

**Goal**: Enable young learners (ages 14-25) to dive into Physical AI concepts by immediately interacting with browser-based demos in Chrome/Edge, running live examples with zero setup time.

**Independent Test**: Can be fully tested by loading a chapter in the browser, running the live demo, and verifying it works within 2 minutes without any installation.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T010 [P] [US1] Contract test for chapter API endpoint in tests/contract/test_chapters.py
- [ ] T011 [P] [US1] Integration test for demo execution in tests/integration/test_demo_execution.py

### Implementation for User Story 1

- [X] T012 [P] [US1] Create Chapter model in src/models/chapter.ts following data-model.md
- [X] T013 [P] [US1] Create Demo model in src/models/demo.ts following data-model.md
- [X] T014 [US1] Implement ChapterService in src/services/chapter_service.ts (depends on T012)
- [X] T015 [US1] Implement DemoService in src/services/demo_service.ts (depends on T013)
- [X] T016 [US1] Create API endpoint GET /api/chapters in src/api/chapters.ts
- [X] T017 [US1] Create API endpoint GET /api/chapters/{id} in src/api/chapters.ts
- [X] T018 [US1] Build InteractiveDemo component in src/components/InteractiveDemo.tsx
- [X] T019 [US1] Integrate demo execution API in src/components/InteractiveDemo.tsx
- [X] T020 [US1] Create first chapter content in docs/01-intro/index.md
- [X] T021 [US1] Add first interactive demo using p5.js in docs/01-intro/index.md
- [X] T022 [US1] Implement validation for demo execution in src/api/demos.ts
- [X] T023 [US1] Add Pyodide integration for Python execution in browser
- [X] T024 [US1] Add TensorFlow.js integration for AI demos in browser
- [X] T025 [US1] Add Three.js integration for 3D visualization demos
- [X] T026 [US1] Add WebSerial/WebUSB integration for hardware connection demos

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Progressive Learning Path (Priority: P2)

**Goal**: Ensure learners can build on concepts progressively, with each chapter building naturally on previous ones, creating a coherent learning journey without gaps in understanding.

**Independent Test**: Can be tested by following the book from start to finish and verifying that each chapter logically builds on the preceding content.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T027 [P] [US2] Contract test for user progress API in tests/contract/test_progress.py
- [ ] T028 [P] [US2] Integration test for chapter dependencies in tests/integration/test_chapter_dependencies.py

### Implementation for User Story 2

- [ ] T029 [P] [US2] Create UserProgress model in src/models/user_progress.js following data-model.md
- [ ] T030 [US2] Implement UserProgressService in src/services/user_progress_service.js
- [ ] T031 [US2] Create API endpoint GET /api/user/progress in src/api/user_progress.js
- [ ] T032 [US2] Create API endpoint PUT /api/user/progress in src/api/user_progress.js
- [ ] T033 [US2] Implement prerequisite validation in ChapterService
- [ ] T034 [US2] Create progress tracking UI in src/components/ProgressTracker.js
- [ ] T035 [US2] Create second chapter content in docs/02-kinematics/index.md
- [ ] T036 [US2] Add progressive demo content linking to first chapter in docs/02-kinematics/index.md
- [ ] T037 [US2] Create third chapter content in docs/03-dynamics/index.md
- [ ] T038 [US2] Add progressive demo content linking to second chapter in docs/03-dynamics/index.md
- [ ] T039 [US2] Add navigation between related chapters in src/theme/Doc.js

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Accessible Content Format (Priority: P3)

**Goal**: Ensure students with no hardware access or advanced technical backgrounds can experience Physical AI concepts through approachable, culturally relevant content with Hinglish explanations and emojis.

**Independent Test**: Can be tested by having a target user (age 14-25 with no hardware) read and understand content without jargon or complex terminology.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T040 [P] [US3] Contract test for content accessibility API in tests/contract/test_accessibility.py
- [ ] T041 [P] [US3] Integration test for Hinglish content rendering in tests/integration/test_content_rendering.py

### Implementation for User Story 3

- [ ] T042 [P] [US3] Create content styling to support Hinglish and emojis in src/css/content-styles.css
- [ ] T043 [US3] Implement Hinglish translation layer for technical terms in src/components/HinglishGlossary.js
- [ ] T044 [US3] Add emoji integration for enhanced engagement in src/components/EmojiSupport.js
- [ ] T045 [US3] Create fourth chapter content with Hinglish + emojis in docs/04-sensors/index.md
- [ ] T046 [US3] Create fifth chapter content with Hinglish + emojis in docs/05-control/index.md
- [ ] T047 [US3] Add accessibility features for screen readers in src/theme/Layout.js
- [ ] T048 [US3] Create CustomProject model in src/models/custom_project.js following data-model.md
- [ ] T049 [US3] Implement CustomProjectService in src/services/custom_project_service.js
- [ ] T050 [US3] Add "make it your own" challenge feature to InteractiveDemo component
- [ ] T051 [US3] Create API endpoint POST /api/projects for saving custom projects
- [ ] T052 [US3] Create API endpoint GET /api/projects/{id} for loading custom projects
- [ ] T053 [US3] Add project sharing functionality with privacy controls

**Checkpoint**: All user stories should now be independently functional

---

[Add more user story phases as needed, following the same pattern]

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T054 [P] Documentation updates in docs/ ensuring Hinglish + emojis as per constitution
- [ ] T055 Code cleanup and refactoring
- [ ] T056 Performance optimization ensuring <2 mins browser load time
- [ ] T057 [P] Additional unit tests (if requested) in tests/unit/
- [ ] T058 Security hardening
- [ ] T059 Accessibility improvements following inclusive design principles
- [ ] T060 Run quickstart.md validation ensuring browser-first experience
- [ ] T061 Verify all examples work in major browsers (Chrome, Firefox, Safari, Edge)
- [ ] T062 Confirm all content aligns with progressive learning approach
- [ ] T063 Create remaining chapters 6-10 following established patterns
- [ ] T064 Add Comment model and functionality for community interaction
- [ ] T065 Implement search functionality across all chapters
- [ ] T066 Create mobile-responsive layout improvements
- [ ] T067 Add offline capability for basic functionality using service workers
- [ ] T068 Deploy to GitHub Pages or Vercel as specified in requirements

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for chapter API endpoint in tests/contract/test_chapters.py"
Task: "Integration test for demo execution in tests/integration/test_demo_execution.py"

# Launch all models for User Story 1 together:
Task: "Create Chapter model in src/models/chapter.js following data-model.md"
Task: "Create Demo model in src/models/demo.js following data-model.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence