---
id: 3
title: Deployment Setup Implementation Tasks
stage: tasks
date: 2025-12-19
surface: agent
model: claude-sonnet-4-5-20250929
feature: deployment-setup
branch: 004-deployment-setup
user: hasss
command: /sp.tasks Generate executable tasks of the following plan
labels: ["tasks", "implementation-breakdown", "ci-cd", "deployment"]
links:
  spec: D:\my-web\specs\004-deployment-setup\spec.md
  ticket: null
  adr: null
  pr: null
files:
 - D:\my-web\specs\004-deployment-setup\tasks.md
tests:
 - Task format validation (PASS)
 - User story mapping (PASS)
---

## Prompt

```text
Generate executable tasks of the following plan
```

## Response snapshot

### Tasks Generated Successfully

Created comprehensive task breakdown with **35 atomic tasks** organized into **8 phases** mapped to **5 user stories** from the specification.

### Task Organization

**Phase 1: Setup & Prerequisites** (5 tasks)
- Verify .gitignore includes .env
- Confirm .env.example exists
- Check existing GitHub Actions workflow
- Verify pyproject.toml dependencies
- Install code review tools locally

**Phase 2: Foundational Configuration** (4 tasks)
- Add lint and format scripts to package.json
- Configure ESLint for React/TypeScript
- Configure Prettier for consistent formatting
- **Checkpoint**: Code review tools ready

**Phase 3: User Story 1 - GitHub Pages Deployment** (5 tasks) 🎯 MVP
- Read existing deploy.yml
- Update branch from `main` to `001-book-creation`
- Add path filtering for frontend files
- Add concurrency control
- Test deployment end-to-end

**Phase 4: User Story 2 - Code Review** (8 tasks)
- Create code-review.yml workflow
- Add frontend linting jobs (ESLint, Prettier)
- Add backend linting jobs (Ruff, Black)
- Configure branch protection rules
- Test code review blocking behavior

**Phase 5: User Story 3 - Secrets Management** (7 tasks)
- Verify .gitignore patterns
- Update .env.example with all secrets
- Add secret descriptions and acquisition instructions
- Configure GitHub Secrets (4 secrets)
- Update README with secrets setup guide
- Test secret access in workflows
- Document rotation procedure

**Phase 6: User Story 4 - Backend HF Deployment** (13 tasks)
- Create app.py entrypoint
- Generate requirements.txt from pyproject.toml
- Add /health endpoint
- Update CORS configuration
- Create HF Space
- Link HF Space to GitHub
- Configure HF environment variables
- Update README with deployment instructions
- Test deployment end-to-end
- Test health check
- Test CORS from frontend
- Verify secrets loaded securely
- Update frontend to use production backend URL

**Phase 7: User Story 5 - Full Automation** (4 tasks)
- Create huggingface-deploy.yml workflow
- Add automated HF push job
- Add deployment notifications
- Test full pipeline

**Phase 8: Testing & Documentation** (2 tasks)
- Create comprehensive deployment documentation
- Run full end-to-end validation

### Key Metrics

- **Total Tasks**: 35
- **Parallel Tasks**: 18 (51% can run in parallel)
- **Sequential Tasks**: 17 (49% must run in order)
- **MVP Tasks**: 14 (Phases 1-3)
- **User Stories**: 5 (all mapped to task phases)

### Implementation Strategy

**MVP = Phases 1-3** (14 tasks, 1-2 hours):
- Delivers: Working GitHub Pages deployment on correct branch
- Value: Core deployment workflow functional immediately
- Testing: Push change → verify GitHub Pages updates

**Full Feature = All 8 Phases** (35 tasks, 3-5 days):
- Delivers: Complete CI/CD pipeline with code review, secrets, backend deployment, full automation
- Value: Production-ready deployment infrastructure
- Testing: End-to-end validation with frontend + backend changes

### Critical Path Analysis

**Must Complete First**:
1. Phase 1: Setup (verify structure)
2. Phase 2: Foundational (configure linting)
3. Phase 3: US1 (fix GitHub Pages)

**Can Parallelize**:
- Phase 3 (US1) and Phase 4 (US2) - Independent P1 stories
- Tasks T015-T019 within Phase 4 (separate linting jobs)
- Tasks T030-T033 within Phase 6 (backend preparation)

**Blocking Dependencies**:
- Phase 5 (Secrets) must complete before Phase 6 (Backend HF deployment)
- Phase 6 (Backend) must work before Phase 7 (Automation) can be tested

### Current State vs Target State

**Existing** (Current):
- ✅ GitHub repository with deploy.yml
- ✅ Backend .env.example with secrets
- ✅ .gitignore includes .env
- ⚠️ deploy.yml targets wrong branch (main)

**Missing** (To Implement):
- ❌ Code review workflow (8 tasks)
- ❌ GitHub Secrets configured (1 task)
- ❌ HF Space created and linked (4 tasks)
- ❌ Backend HF entrypoint (app.py) (1 task)
- ❌ Backend requirements.txt (1 task)
- ❌ CORS configuration for production (1 task)

### Format Validation

✅ All 35 tasks follow required format:
- `- [ ] T###` (checkbox + task ID)
- `[P]` marker for 18 parallelizable tasks
- `[US#]` label for 27 user story tasks
- Absolute file paths in descriptions
- Clear, actionable descriptions

### Independent Testing

Each user story phase includes independent test criteria:
- **US1**: Push change → verify GitHub Pages updates
- **US2**: Create PR with lint error → verify blocking
- **US3**: Attempt commit .env → verify prevented
- **US4**: Test backend health endpoint + CORS
- **US5**: Make backend change → verify auto-deployment

## Outcome

- ✅ Impact: 35 atomic tasks ready for immediate execution, organized by user story for independent delivery
- 🧪 Tests: Format validation (PASS), User story mapping (PASS)
- 📁 Files: 1 file created (tasks.md)
- 🔁 Next prompts: Review tasks.md, begin implementation with Phase 1-3 (MVP), use `/sp.implement` to execute tasks systematically
- 🧠 Reflection: Tasks successfully map 5 user stories to 8 phases, with clear MVP scope (Phases 1-3), parallel execution opportunities (51% tasks), and independent testing for each story. Critical finding: existing deploy.yml targets wrong branch (main vs 001-book-creation), which is highest priority fix.

## Evaluation notes (flywheel)

- Failure modes observed: None - tasks generated successfully with correct format
- Graders run and results (PASS/FAIL): Task format validation - PASS (35/35 tasks correct), User story mapping - PASS (all stories covered)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Start with MVP (Phases 1-3, 14 tasks) to get working GitHub Pages deployment, then incrementally add code review (Phase 4), secrets (Phase 5), backend (Phase 6), automation (Phase 7)
