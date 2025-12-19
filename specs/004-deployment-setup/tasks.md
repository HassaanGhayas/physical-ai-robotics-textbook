# Implementation Tasks: Deployment Setup & Code Review

**Feature**: Deployment Setup & Code Review
**Branch**: `004-deployment-setup`
**Generated**: 2025-12-19
**Tech Stack**: GitHub Actions, Hugging Face Spaces, ESLint, Prettier, Ruff, Black

## Overview

This task list breaks down the deployment setup implementation into **independently testable user story phases**. Each phase corresponds to a user story from spec.md and can be implemented, tested, and deployed independently.

**Total Tasks**: 35
**User Stories**: 5 (P1, P1, P2, P2, P3)
**Parallel Opportunities**: 18 tasks marked [P]

## Implementation Strategy

### MVP Approach (Recommended)
**MVP = Phase 3 (User Story 1: GitHub Pages Deployment)**
- Delivers: Fix existing deploy.yml to target correct branch, trigger automatic deployment
- Testable: Push small change to 001-book-creation, verify GitHub Pages updates
- Value: Core deployment workflow functional
- Estimated Time: 1-2 hours

### Incremental Delivery
1. **Day 1**: Phase 1-3 (Setup + US1: GitHub Pages Fix) → Working frontend deployment
2. **Day 2**: Phase 4 (US2: Code Review) → Quality gate before deployment
3. **Day 3**: Phase 5 (US3: Secrets Management) → Secure configuration
4. **Day 4**: Phase 6 (US4: Backend Deployment) → Full stack deployed
5. **Day 5**: Phase 7 (US5: Full Automation) → Complete CI/CD pipeline

Each phase is independently testable and deployable, allowing for continuous value delivery.

---

## Phase 1: Setup & Prerequisites (5 tasks)

**Goal**: Verify project structure and prepare for deployment configuration

**Independent Test**: Run `git status`, verify `.env` in `.gitignore`, confirm backend/.env.example exists

### Tasks

- [X] T001 Verify `.env` is in D:\my-web\.gitignore to prevent secrets from being committed
- [X] T002 Verify D:\my-web\backend\.env.example exists and documents all required secrets
- [X] T003 [P] Check existing GitHub Actions workflow at D:\my-web\.github\workflows\deploy.yml
- [X] T004 [P] Verify D:\my-web\backend\pyproject.toml exists with all dependencies listed
- [X] T005 [P] Install code review tools locally: `npm install -D eslint prettier` and `pip install ruff black`

**Validation**: All prerequisites confirmed, tools installed successfully

---

## Phase 2: Foundational Configuration (4 tasks)

**Goal**: Configure foundational code review and linting infrastructure that ALL user stories depend on

**Independent Test**: Run linters manually (`npm run lint`, `ruff check backend/`), verify they execute without errors

⚠️ **CRITICAL**: No user story work can begin until this phase is complete

### Tasks

- [X] T006 Add lint script to D:\my-web\package.json: `"lint": "eslint src/ --ext .js,.jsx,.ts,.tsx"`
- [X] T007 Add format check script to D:\my-web\package.json: `"format:check": "prettier --check ."`
- [X] T008 [P] Create or update D:\my-web\.eslintrc.json with React, TypeScript, JSX-a11y rules
- [X] T009 [P] Create or update D:\my-web\.prettierrc with consistent formatting rules

**Checkpoint**: Code review tools configured - user story implementation can now begin

---

## Phase 3: User Story 1 - Fix GitHub Pages Deployment (Priority: P1) 🎯 MVP

**Story Goal**: Fix existing deployment workflow to target correct branch (001-book-creation instead of main) and ensure automatic deployment to GitHub Pages

**Independent Test**:
1. Push a small change (update any .md file in docs/) to 001-book-creation branch
2. Verify GitHub Actions workflow triggers automatically
3. Check workflow completes successfully
4. Confirm changes appear on https://hassaanghayas.github.io/physical-ai-robotics-textbook within 5 minutes

**Acceptance Criteria** (from spec.md):
- ✓ GitHub Actions triggers on push to 001-book-creation
- ✓ Workflow builds Docusaurus successfully
- ✓ Site deploys to GitHub Pages
- ✓ Changes visible within 5 minutes

### Tasks

- [X] T010 [US1] Read existing workflow at D:\my-web\.github\workflows\deploy.yml to understand current configuration
- [X] T011 [US1] Update D:\my-web\.github\workflows\deploy.yml: Change `branches: [main]` to `branches: [001-book-creation]` in push trigger
- [X] T012 [US1] Add path filtering to D:\my-web\.github\workflows\deploy.yml to only trigger on frontend changes (src/, docs/, static/, docusaurus.config.ts, package.json)
- [X] T013 [US1] Add concurrency group to D:\my-web\.github\workflows\deploy.yml to prevent overlapping deployments: `concurrency: { group: "pages", cancel-in-progress: false }`
- [X] T014 [US1] Test deployment by committing change to 001-book-creation and verifying GitHub Actions runs successfully

**Story Complete When**: Push to 001-book-creation triggers workflow, builds successfully, deploys to GitHub Pages

---

## Phase 4: User Story 2 - Automated Code Review (Priority: P1) (8 tasks)

**Story Goal**: Implement automated code review workflow that runs on pull requests and blocks deployment if critical issues are found

**Independent Test**:
1. Create a test PR with intentional lint error (e.g., missing semicolon)
2. Verify code-review.yml workflow runs automatically
3. Confirm workflow fails and blocks PR merge
4. Fix lint error, push again
5. Verify workflow passes and allows merge

**Acceptance Criteria** (from spec.md):
- ✓ Code review runs on all PRs to 001-book-creation
- ✓ Review completes within 3 minutes
- ✓ Blocks merge if critical issues found
- ✓ Generates detailed report with file paths and line numbers

### Tasks

- [X] T015 [P] [US2] Create D:\my-web\.github\workflows\code-review.yml with pull_request trigger for 001-book-creation branch
- [X] T016 [P] [US2] Add lint-frontend job to code-review.yml: checkout, setup Node.js 20, run `npm ci`, run `npm run lint`
- [X] T017 [P] [US2] Add format-check-frontend job to code-review.yml: run `npm run format:check`
- [X] T018 [P] [US2] Add lint-backend job to code-review.yml: checkout, setup Python 3.11, install ruff/black, run `ruff check backend/`
- [X] T019 [P] [US2] Add format-check-backend job to code-review.yml: run `black --check backend/`
- [ ] T020 [US2] Configure GitHub branch protection rules: Settings → Branches → 001-book-creation → Require status checks (lint-frontend, format-check-frontend, lint-backend, format-check-backend)
- [ ] T021 [US2] Test code review by creating PR with lint error, verify workflow blocks merge
- [X] T022 [US2] Document code review workflow in D:\my-web\README.md or docs/ (optional)

**Story Complete When**: PRs to 001-book-creation run automated code review, blocking merge if issues found

---

## Phase 5: User Story 3 - Secrets Management (Priority: P2) (7 tasks)

**Story Goal**: Configure secure secrets management for GitHub Actions and Hugging Face Spaces

**Independent Test**:
1. Verify `.env` exists in `.gitignore` and cannot be committed
2. Verify `.env.example` in backend/ documents all secrets
3. Configure GitHub Secrets in repository settings
4. Verify GitHub Actions can access secrets (test with echo of first 4 chars)
5. Configure HF Spaces secrets and verify backend loads them

**Acceptance Criteria** (from spec.md):
- ✓ `.env.example` committed with placeholder values
- ✓ `.env` cannot be committed (gitignored)
- ✓ GitHub Secrets accessible in workflows
- ✓ HF Spaces secrets load in backend

### Tasks

- [X] T023 [US3] Verify D:\my-web\.gitignore contains `.env` pattern (if not, add it)
- [X] T024 [P] [US3] Update D:\my-web\backend\.env.example to include all required secrets: COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL, NEON_DATABASE_URL (if used)
- [X] T025 [P] [US3] Add comments to .env.example explaining what each secret is for and how to obtain it
- [ ] T026 [US3] Configure GitHub Secrets: Go to repository Settings → Secrets and variables → Actions → New repository secret:
  - Add COHERE_API_KEY
  - Add QDRANT_API_KEY
  - Add QDRANT_URL
  - Add HF_TOKEN (optional, for automated deployment)
- [X] T027 [US3] Update D:\my-web\README.md with instructions for configuring secrets (GitHub + HF Spaces)
- [ ] T028 [US3] Test secret access by adding temporary step to deploy.yml: `echo "COHERE_API_KEY: ${{ secrets.COHERE_API_KEY != '' }}"` (verify true, then remove)
- [X] T029 [US3] Document secret rotation procedure in README.md or docs/deployment.md

**Story Complete When**: Secrets configured in GitHub and HF, `.env` cannot be committed, `.env.example` documents all secrets

---

## Phase 6: User Story 4 - Backend Deployment to Hugging Face Spaces (Priority: P2) (13 tasks)

**Story Goal**: Deploy FastAPI backend to Hugging Face Spaces with health check, CORS, and secure secrets loading

**Independent Test**:
1. Create HF Space at huggingface.co/spaces
2. Link HF Space to GitHub repository (backend/ directory)
3. Configure HF Spaces secrets
4. Push to 001-book-creation, verify HF auto-deploys
5. Test health endpoint: `curl https://[space-name].hf.space/health`
6. Test CORS: Verify frontend can make requests to backend
7. Verify secrets loaded correctly (check app logs, no secret values exposed)

**Acceptance Criteria** (from spec.md):
- ✓ Backend deploys to HF Spaces within 10 minutes
- ✓ Secrets loaded from HF configuration
- ✓ CORS allows GitHub Pages domain
- ✓ Health check returns 200 OK
- ✓ No secrets exposed in logs

### Tasks

#### Backend Preparation
- [X] T030 [P] [US4] Create D:\my-web\backend\app.py as HF Spaces entrypoint (wrapper that imports and runs api.py)
- [X] T031 [P] [US4] Generate D:\my-web\backend\requirements.txt from pyproject.toml: `poetry export -f requirements.txt --output requirements.txt --without-hashes`
- [X] T032 [P] [US4] Add /health endpoint to D:\my-web\backend\api.py that returns {"status": "healthy", "timestamp": "..."}
- [X] T033 [US4] Update CORS configuration in D:\my-web\backend\api.py to allow origin: `https://hassaanghayas.github.io`

#### Hugging Face Spaces Setup
- [ ] T034 [US4] Create Hugging Face Space: Go to huggingface.co/new-space, name: "physical-ai-robotics-backend", SDK: Docker, Visibility: Public
- [ ] T035 [US4] Link HF Space to GitHub repository: Settings → Link to GitHub → Select HassaanGhayas/physical-ai-robotics-textbook → Sync directory: backend/
- [ ] T036 [US4] Configure HF Spaces environment variables: Settings → Variables:
  - COHERE_API_KEY = [your_key]
  - QDRANT_API_KEY = [your_key]
  - QDRANT_URL = [your_cluster_url]
  - NEON_DATABASE_URL = [your_db_url] (if used)
- [X] T037 [US4] Update D:\my-web\backend\README.md with HF deployment instructions and Space URL

#### Testing & Validation
- [ ] T038 [US4] Test backend deployment: Push to 001-book-creation, wait for HF build (5-10 min), verify Space shows "Running" status
- [ ] T039 [US4] Test health endpoint: `curl https://[space-name].hf.space/health` (expect 200 OK with JSON)
- [ ] T040 [US4] Test CORS: Create test HTML file that makes fetch request from GitHub Pages domain to backend, verify request succeeds
- [ ] T041 [US4] Verify secrets loaded: Check HF Space logs, confirm no secret values are exposed, verify backend can connect to Cohere and Qdrant
- [ ] T042 [US4] Update D:\my-web\src\lib\api\chatbot.ts (or equivalent) to use production backend URL when deployed to GitHub Pages

**Story Complete When**: Backend deployed to HF Spaces, health check returns 200, CORS configured, frontend can call backend API

---

## Phase 7: User Story 5 - Automated Deployment Pipeline (Priority: P3) (4 tasks)

**Story Goal**: Optionally automate backend deployment via GitHub Actions when backend/ directory changes are detected

**Independent Test**:
1. Make change to backend/ directory
2. Push to 001-book-creation
3. Verify GitHub Actions triggers both frontend and backend workflows
4. Confirm HF Space rebuilds automatically
5. Verify deployment completes successfully end-to-end

**Acceptance Criteria** (from spec.md):
- ✓ Backend deployment triggers automatically on backend/ changes
- ✓ Deployment pipeline handles failures gracefully
- ✓ Notifications sent on success/failure

### Tasks

- [X] T043 [P] [US5] Create D:\my-web\.github\workflows\huggingface-deploy.yml with push trigger for 001-book-creation and path filter: backend/**
- [X] T044 [P] [US5] Add job to huggingface-deploy.yml that pushes backend/ to HF Space repository using HF_TOKEN (optional - can rely on HF GitHub sync instead)
- [X] T045 [US5] Add notification step to all workflows: use actions/github-script to comment on commit with deployment status
- [ ] T046 [US5] Test full pipeline: Make change affecting both frontend and backend, push, verify both deployments succeed

**Story Complete When**: Backend changes trigger automatic HF deployment, notifications sent on success/failure

---

## Phase 8: Testing & Documentation (2 tasks)

**Goal**: Final validation and comprehensive documentation

### Tasks

- [X] T047 [P] Create D:\my-web\docs\deployment.md documenting complete deployment process, secrets configuration, troubleshooting guide
- [ ] T048 Run full end-to-end deployment test: Create PR with frontend + backend changes, verify code review passes, merge, verify both deployments succeed

**Phase Complete When**: Complete deployment documentation exists, end-to-end test passes

---

## Dependencies & Execution Order

### User Story Dependency Graph

```
Phase 1 (Setup) → Phase 2 (Foundational)
                        ↓
        ┌───────────────┴─────────────────┐
        ↓                                 ↓
    Phase 3 (US1: GitHub Pages)   Phase 4 (US2: Code Review)
        ↓                                 ↓
        └───────────────┬─────────────────┘
                        ↓
                  Phase 5 (US3: Secrets)
                        ↓
                  Phase 6 (US4: Backend HF)
                        ↓
                  Phase 7 (US5: Automation)
                        ↓
                  Phase 8 (Testing & Docs)
```

**Critical Path**: Phase 1 → Phase 2 → Phase 3 (US1) → Phase 5 (US3) → Phase 6 (US4) → Phase 8

**Parallel Opportunities**:
- Phase 3 (US1) and Phase 4 (US2) can be developed in parallel after Phase 2
- Tasks within each phase marked [P] can run in parallel

### Blocking Dependencies

- **Phase 2 blocks all user stories**: Linting tools must be configured before code review workflow can run
- **Phase 3 and Phase 4 are independent**: Can be implemented in parallel (both P1 priority)
- **Phase 5 blocks Phase 6**: Secrets must be configured before backend can deploy successfully
- **Phase 6 blocks Phase 7**: Backend deployment must work before automation can be tested

---

## Parallel Execution Examples

### Day 1 Parallelization (Phase 3: US1)
Developer 1:
- T010-T011 (Read existing workflow, update branch)
- T012-T013 (Path filtering, concurrency)

Developer 2:
- T008-T009 (ESLint, Prettier config)
- T014 (Test deployment)

### Day 2 Parallelization (Phase 4: US2)
Developer 1:
- T015-T017 (Frontend code review jobs)

Developer 2:
- T018-T019 (Backend code review jobs)

Both developers:
- T020 (Configure branch protection together)

### Day 4 Parallelization (Phase 6: US4)
Developer 1:
- T030-T033 (Backend code preparation)

Developer 2:
- T034-T037 (HF Space setup)

Both developers:
- T038-T042 (Testing and validation)

---

## Success Criteria Validation Checklist

After completing all phases, validate against spec.md success criteria:

- [ ] **SC-001**: Push to GitHub → Changes on GitHub Pages within 5 minutes (measure: actual deployment)
- [ ] **SC-002**: Code review analyzes codebase within 3 minutes (measure: GitHub Actions duration)
- [ ] **SC-003**: Zero secrets in Git history (measure: `git log -p | grep -i "api_key\|secret\|token"`)
- [ ] **SC-004**: Backend shows "Running" on HF Spaces (measure: check HF Space status)
- [ ] **SC-005**: Frontend → Backend API response < 2 seconds (measure: browser DevTools Network tab)
- [ ] **SC-006**: Deployment success rate ≥ 95% (measure: track 20 deployments)
- [ ] **SC-007**: Failure notifications within 2 minutes (measure: GitHub Actions email/comment)
- [ ] **SC-008**: Code review finds ≥ 80% of lint issues (measure: manual lint + automated lint comparison)
- [ ] **SC-009**: Backend uptime ≥ 99% (measure: UptimeRobot or manual tracking)
- [ ] **SC-010**: All secrets documented in .env.example (measure: manual review)
- [ ] **SC-011**: Rollback within 5 minutes (measure: test rollback via Git revert + redeploy)
- [ ] **SC-012**: Zero manual steps for frontend deployment after push (measure: deployment checklist)

---

## Notes

- **[P] Tasks**: Can be executed in parallel (different files, no dependencies on incomplete tasks)
- **[US#] Labels**: Map to user stories from spec.md (US1 = Story 1, etc.)
- **File Paths**: All paths are absolute from project root (D:\my-web\...)
- **GitHub Actions**: Free for public repositories (2000 minutes/month)
- **Hugging Face Spaces**: Free tier provides 2 CPU cores, 16 GB RAM
- **Deployment Time**: Frontend 3-5 min, Backend 6-10 min

---

## Quick Reference

**Most Critical Tasks** (cannot skip):
- T001-T005 (Setup & Prerequisites)
- T006-T009 (Foundational linting config)
- T010-T014 (US1: Fix GitHub Pages deployment)
- T023-T029 (US3: Secrets management)

**Recommended MVP** (Day 1):
- Phases 1-3 = Tasks T001-T014 (Setup + Foundational + US1)
- Deploy after T014 for working GitHub Pages deployment

**Quality Gates**:
- After Phase 3: Verify GitHub Pages deployment works on correct branch
- After Phase 4: Verify code review blocks PRs with lint errors
- After Phase 6: Verify backend is accessible from frontend
- After Phase 8: Verify all success criteria from spec.md

---

## Current State Analysis

**Existing Infrastructure** (verified by research):
- ✅ GitHub repository exists with deploy.yml workflow
- ✅ Backend .env.example exists with documented secrets
- ✅ .gitignore includes .env pattern
- ⚠️ deploy.yml targets wrong branch (main instead of 001-book-creation)
- ❌ Code review workflow missing
- ❌ Hugging Face Space not yet created
- ❌ Backend requirements.txt not generated
- ❌ Backend app.py entrypoint missing

**Required Changes**:
1. Fix deploy.yml branch (T011)
2. Create code-review.yml (T015-T019)
3. Create HF Space (T034)
4. Generate requirements.txt (T031)
5. Create app.py (T030)
6. Configure secrets (T026, T036)
