---
id: 2
title: Deployment Setup Implementation Plan
stage: plan
date: 2025-12-19
surface: agent
model: claude-sonnet-4-5-20250929
feature: deployment-setup
branch: 004-deployment-setup
user: hasss
command: /sp.plan
labels: ["planning", "devops", "ci-cd", "architecture", "deployment"]
links:
  spec: D:\my-web\specs\004-deployment-setup\spec.md
  ticket: null
  adr: null
  pr: null
files:
 - D:\my-web\specs\004-deployment-setup\plan.md
 - D:\my-web\specs\004-deployment-setup\research.md
 - D:\my-web\specs\004-deployment-setup\data-model.md
 - D:\my-web\specs\004-deployment-setup\quickstart.md
 - D:\my-web\specs\004-deployment-setup\contracts\github-actions-workflow-schema.yaml
 - D:\my-web\specs\004-deployment-setup\contracts\code-review-report-schema.json
 - D:\my-web\specs\004-deployment-setup\contracts\deployment-log-schema.json
 - D:\my-web\CLAUDE.md
tests:
 - Constitution compliance check (PASS)
 - Technical context validation (PASS)
---

## Prompt

```text
/sp.plan
```

## Response snapshot

### Planning Complete

Successfully generated comprehensive implementation plan for deployment setup feature using DevOps specialist agent (ID: a9a0681).

### Artifacts Created (7 files)

**Core Planning Documents**:
1. **plan.md** (17.4 KB) - Complete implementation plan with technical context, constitution check, project structure, risk analysis
2. **research.md** (26.3 KB) - 5 technology decisions researched with rationale, alternatives, and cost analysis
3. **data-model.md** (25.8 KB) - 7 core entities with schemas, relationships, state transitions, validation rules
4. **quickstart.md** (18.1 KB) - Step-by-step setup guide (30-45 min), prerequisites, troubleshooting

**Contract Schemas**:
5. **github-actions-workflow-schema.yaml** (8.2 KB) - Workflow structure for deploy.yml, code-review.yml, huggingface-deploy.yml
6. **code-review-report-schema.json** (9.2 KB) - Review report format with severity levels, file paths, line numbers
7. **deployment-log-schema.json** (10.1 KB) - Deployment event logging schema with timestamps, status, errors

### Key Technical Decisions

**1. Code Review Tooling** (Research Decision #1):
- **Selected**: GitHub Actions + ESLint/Prettier (frontend) + Ruff/Black (backend)
- **Rationale**: Free, fast (<2 min), native integration, no external dependencies
- **Rejected**: SonarCloud (costs $10/mo for private repos), external AI services (slower, costs $$$)

**2. GitHub Actions Workflow Structure** (Research Decision #2):
- **Selected**: Separate workflows per deployment target
  - deploy.yml: Frontend to GitHub Pages (updated to branch 001-book-creation)
  - code-review.yml: Linting and quality checks (blocking)
  - huggingface-deploy.yml: Backend deployment (optional automation)
- **Rationale**: Independent deployment, easier debugging, selective triggering

**3. Hugging Face Deployment Method** (Research Decision #3):
- **Selected**: Git-based deployment with app.py entrypoint
- **Rationale**: Simple, free, automatic rebuilds, native HF Spaces integration
- **Rejected**: Docker (more complex, slower builds), API-based (requires token management)

**4. Secrets Management** (Research Decision #4):
- **Selected**: GitHub Secrets for CI/CD + HF Spaces environment variables for runtime
- **Rationale**: Platform-native, secure, no additional tools needed
- **Required Secrets**: COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL, HF_TOKEN (optional)

**5. Code Review Integration** (Research Decision #5):
- **Selected**: PR blocking + optional pre-commit hooks
- **Rationale**: Prevents bad code from merging, fast feedback (<2 min), developer flexibility

### Current State Analysis

**Existing Infrastructure**:
- Frontend: Docusaurus 3.9.2, React 19.0.0, deployed to GitHub Pages
- Backend: Python 3.11+, FastAPI, Cohere, Qdrant, in `backend/` directory
- Existing workflow: `.github/workflows/deploy.yml` (targets wrong branch: main instead of 001-book-creation)
- Secrets documented: `backend/.env.example` with all required variables

**Required Changes**:
1. Update deploy.yml branch from `main` to `001-book-creation`
2. Create code-review.yml workflow for linting (ESLint, Prettier, Ruff, Black)
3. Create backend/app.py as Hugging Face Spaces entrypoint
4. Generate backend/requirements.txt from pyproject.toml (HF Spaces requirement)
5. Configure 4 GitHub Secrets in repository settings
6. Create Hugging Face Space and configure environment variables
7. Update backend CORS to allow GitHub Pages domain (hassaanghayas.github.io)

### Constitution Compliance

**All 8 Constitution Principles**: ✅ PASS
- Spec-Driven Development ✓ (Following Spec-Kit Plus)
- Modular Architecture ✓ (Separate frontend/backend deployments)
- AI Integration First ✓ (Preserves existing RAG chatbot)
- Authentication & Personalization ✓ (No impact on Better Auth)
- Performance & Scalability ✓ (Serverless platforms scale automatically)
- Multi-language Support ✓ (No impact on Urdu translation)
- Technology Stack Requirements ✓ (Uses existing stack, adds only CI/CD)
- Development Workflow ✓ (Fits "GitHub Push → Deployment" phase)

**Zero Constitution Violations** - Feature aligns perfectly with project principles.

### Data Model Highlights

**7 Core Entities**:
1. **GitHubRepository** - Source code with frontend/backend separation
2. **GitHubActionsWorkflow** - CI/CD automation with status tracking
3. **CodeReviewReport** - Review results with severity levels (critical/high/medium/low)
4. **GitHubPagesDeployment** - Frontend deployment with URL and build time
5. **HuggingFaceSpace** - Backend deployment with runtime status
6. **DeploymentLog** - Event tracking with timestamps and error context
7. **EnvironmentSecret** - Secure credential storage with platform mapping

**State Transitions**:
- Workflow: idle → queued → running → completed/failed
- Deployment: pending → building → deploying → deployed/failed
- Code Review: pending → analyzing → completed (with pass/fail/error status)

### Cost Analysis

**Total Monthly Cost**: $0 (Free Tier)
- GitHub Actions: Free (public repo, 2000 min/mo)
- GitHub Pages: Free (public repo, 1 GB storage)
- Hugging Face Spaces: Free (CPU-basic, 16 GB disk, 2 CPU cores, community support)

## Outcome

- ✅ Impact: Complete implementation plan with all research, data models, contracts, and quickstart guide ready for task generation
- 🧪 Tests: Constitution compliance (PASS), Technical feasibility (PASS)
- 📁 Files: 7 files created (plan, research, data model, quickstart, 3 contract schemas), 1 updated (CLAUDE.md)
- 🔁 Next prompts: Review planning artifacts, then run `/sp.tasks` to generate atomic implementation tasks, consider `/sp.adr` for documenting deployment architecture decisions
- 🧠 Reflection: DevOps agent successfully analyzed existing infrastructure (found deploy.yml targets wrong branch), researched all technology decisions, and created comprehensive planning artifacts. All constitution principles validated with zero violations.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully with all required artifacts
- Graders run and results (PASS/FAIL): Constitution check - PASS (8/8 principles)
- Prompt variant (if applicable): Used DevOps specialist agent (a9a0681) for deployment expertise
- Next experiment (smallest change to try): Run `/sp.tasks` to break down into atomic implementation tasks, starting with highest priority (fix deploy.yml branch, create code-review.yml)
