# Implementation Plan: Deployment Setup & Code Review

**Branch**: `004-deployment-setup` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `D:\my-web\specs\004-deployment-setup\spec.md`

## Summary

This feature establishes automated deployment infrastructure for the Physical AI & Humanoid Robotics textbook project. It enables:
1. Automated frontend (Docusaurus) deployment to GitHub Pages on push to main branch (001-book-creation)
2. Backend (FastAPI RAG agent) deployment to Hugging Face Spaces
3. Automated code review before deployment using GitHub Actions native reviewers with ESLint/Prettier for frontend and Ruff/Black for backend
4. Secure secrets management using GitHub Actions Secrets and Hugging Face Spaces environment variables

The existing GitHub Pages deployment workflow needs to be updated to push to the correct main branch (001-book-creation instead of main) and include code review checks. The backend needs new Hugging Face deployment configuration.

## Technical Context

**Language/Version**:
- Frontend: Node.js 20+, TypeScript 5.6.2, React 19.0.0
- Backend: Python 3.11+, FastAPI, Uvicorn

**Primary Dependencies**:
- Frontend: Docusaurus 3.9.2, @docusaurus/preset-classic, React 19, Playwright (testing), Lighthouse (performance)
- Backend: FastAPI, Uvicorn, Cohere (embeddings/generation), Qdrant-client (vector storage), Pydantic (validation), Tenacity (retries), PyBreaker (circuit breakers)

**Storage**:
- Frontend: Static files (GitHub Pages)
- Backend: Qdrant Cloud (vector storage), Neon Serverless Postgres (database via Better Auth)

**Testing**:
- Frontend: Vitest (unit tests), Playwright (E2E tests), Lighthouse (performance/a11y)
- Backend: Pytest (unit/integration tests)

**Target Platform**:
- Frontend: GitHub Pages (static hosting)
- Backend: Hugging Face Spaces (containerized Python app)

**Project Type**: Web application (separate frontend and backend deployments)

**Performance Goals**:
- Frontend: Lighthouse score >90, < 3s page load
- Backend: < 5s response time (95th percentile), 50 concurrent users
- Deployment: < 5 minutes for frontend, < 10 minutes for backend

**Constraints**:
- GitHub Actions: Free tier limits (2000 minutes/month for private repos)
- Hugging Face Spaces: Free tier with 2 CPU cores, 16GB RAM
- Secrets must never be exposed in logs or committed to Git
- CORS must be configured for cross-origin requests (GitHub Pages to HF Spaces)

**Scale/Scope**:
- Single production deployment (no staging environment in Phase 1)
- ~100 pages of documentation content
- Backend serves RAG queries with Cohere and Qdrant integration
- Expected traffic: < 1000 users/month initially

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Spec-Driven Development ✓
- All development follows Spec-Kit Plus methodology
- Feature spec exists at `D:\my-web\specs\004-deployment-setup\spec.md`
- Plan, research, data-model, contracts, quickstart will be created

### Modular Architecture ✓
- Frontend (Docusaurus) and backend (FastAPI) are separate modules
- Deployment pipelines are independent (GitHub Actions for frontend, HF for backend)
- Code review integrated as separate validation step before deployment

### AI Integration First ✓
- Backend RAG agent uses Cohere Models for embeddings and generation
- OpenAI Agents SDK available for future enhancement
- Code review can leverage AI-powered tools (GitHub Copilot integration optional)

### Authentication & Personalization ✓
- Better Auth configured for user authentication
- Secrets for auth stored securely in environment variables
- Production secrets managed via GitHub Actions Secrets and HF Spaces

### Performance & Scalability ✓
- Frontend optimized with Lighthouse CI validation
- Backend uses circuit breakers and retry logic for resilience
- Deployment automation reduces manual overhead

### Multi-language Support ✓
- Urdu translation feature enabled for authenticated users
- Deployment does not affect translation capabilities

### Technology Stack Alignment ✓
- Docusaurus for documentation ✓
- GitHub Pages for deployment ✓
- Cohere Models for RAG ✓
- FastAPI for backend services ✓
- Neon Serverless Postgres for database ✓
- Qdrant Cloud for vector storage ✓
- Better Auth for authentication ✓

### Development Workflow Alignment ✓
- Follows Book Content → UI/UX Customization → GitHub Push → Deployment → Chatbot → Authentication → Testing
- This feature focuses on the "GitHub Push → Deployment" phase
- Testing integration (Lighthouse, Playwright, Pytest) included

## Project Structure

### Documentation (this feature)

```text
specs/004-deployment-setup/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - Technology research decisions
├── data-model.md        # Phase 1 output - Deployment entities and relationships
├── quickstart.md        # Phase 1 output - Setup instructions for developers
├── contracts/           # Phase 1 output - API contracts and schemas
│   ├── github-actions-workflow-schema.yaml
│   ├── code-review-report-schema.json
│   └── deployment-log-schema.json
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Frontend (Docusaurus)
D:\my-web\
├── src/                     # React components, pages, theme customization
│   ├── components/          # ChatBot, SkipLink, UI components
│   ├── pages/               # index.tsx (homepage)
│   ├── css/                 # Custom styles, theme colors, typography
│   └── theme/               # Docusaurus theme swizzled components
├── docs/                    # Markdown documentation content
│   └── book/                # Physical AI & Robotics textbook chapters
├── static/                  # Static assets (images, animations)
├── tests/                   # Playwright E2E tests
├── docusaurus.config.ts     # Docusaurus configuration
├── sidebars.ts              # Sidebar navigation configuration
└── package.json             # Dependencies and scripts

# Backend (Python FastAPI)
D:\my-web\backend\
├── api.py                   # FastAPI application with CORS, error handling
├── rag_agent.py             # RAG query processing logic
├── main.py                  # Embedding pipeline and Qdrant operations
├── retrieval.py             # Retrieval functionality
├── config.py                # Pydantic settings for env var loading
├── models.py                # Pydantic models for requests/responses
├── .env.example             # Example environment variables (committed)
├── .env                     # Actual secrets (gitignored)
├── pyproject.toml           # Python project dependencies
├── tests/                   # Pytest unit and integration tests
└── README.md                # Backend documentation

# GitHub Actions Workflows
D:\my-web\.github\workflows\
├── deploy.yml               # EXISTING: GitHub Pages deployment (needs update)
└── code-review.yml          # NEW: Code review workflow (to be created)

# Hugging Face Deployment
D:\my-web\backend\
├── app.py                   # NEW: HF Spaces entrypoint (wrapper for api.py)
├── requirements.txt         # NEW: Generated from pyproject.toml for HF
└── README.md                # Update with HF deployment instructions
```

**Structure Decision**:
The project uses a web application structure with separate frontend and backend directories. Frontend is deployed as static files to GitHub Pages, while backend is deployed as a containerized Python application to Hugging Face Spaces. Both deployments are automated via GitHub Actions, with secrets managed through platform-native secret stores (GitHub Actions Secrets, HF Spaces environment variables).

## Complexity Tracking

> **No Constitution violations detected**

This feature aligns with all constitution principles:
- Follows Spec-Driven Development with complete documentation
- Maintains modular architecture with independent deployment pipelines
- Preserves existing AI integration (RAG agent, Cohere, authentication)
- Enhances performance through automated validation (Lighthouse, code review)
- Uses approved technology stack (Docusaurus, GitHub Pages, FastAPI, HF Spaces)

---

## Phase 0: Research & Technology Decisions

**Goal**: Research and document technology choices for deployment infrastructure, code review tools, and secrets management.

**Deliverable**: `research.md` with justified technology decisions.

### Research Questions

1. **Code Review Tool Selection**
   - **Options**:
     - GitHub Actions built-in reviewers (ESLint, Prettier, Ruff, Black)
     - Third-party SaaS (SonarCloud, CodeClimate, Codacy)
     - AI-powered (GitHub Copilot, custom Claude Code integration)
   - **Decision Criteria**: Cost (free tier), integration effort, language support (JS/TS + Python), actionable feedback
   - **Recommended**: GitHub Actions with ESLint/Prettier (frontend) + Ruff/Black (backend)
   - **Rationale**: Free, native GitHub integration, fast execution, language-specific best practices

2. **GitHub Actions Workflow Structure**
   - **Options**:
     - Single monolithic workflow
     - Separate workflows per deployment target
     - Matrix jobs for parallel execution
   - **Decision Criteria**: Maintainability, speed, failure isolation
   - **Recommended**: Separate workflows (deploy.yml for frontend, huggingface-deploy.yml for backend)
   - **Rationale**: Clear separation of concerns, independent failure handling, easier debugging

3. **Hugging Face Spaces Deployment Method**
   - **Options**:
     - Git-based (push to HF repo triggers rebuild)
     - GitHub Actions with HF API
     - Manual deployment via HF UI
   - **Decision Criteria**: Automation level, maintenance overhead, secret handling
   - **Recommended**: Git-based deployment with GitHub Actions automation
   - **Rationale**: Simple, reliable, version-controlled, native HF integration

4. **Secrets Management Pattern**
   - **Options**:
     - GitHub Actions Secrets only
     - External secret manager (AWS Secrets Manager, HashiCorp Vault)
     - Encrypted .env files in repository
   - **Decision Criteria**: Security, cost, complexity, platform support
   - **Recommended**: GitHub Actions Secrets + HF Spaces environment variables
   - **Rationale**: Native platform support, free, secure, no external dependencies

5. **Code Review Integration Points**
   - **Options**:
     - Pre-commit hooks (local)
     - GitHub Actions on PR (blocking)
     - Post-merge analysis (non-blocking)
   - **Decision Criteria**: Developer experience, deployment blocking, feedback timing
   - **Recommended**: GitHub Actions on PR (blocking) + optional pre-commit hooks
   - **Rationale**: Prevents bad code from reaching main, provides CI feedback, optional local validation

### Research Outputs

See `research.md` for detailed analysis, trade-offs, and final recommendations.

---

## Phase 1: Design & Contracts

**Goal**: Define deployment entities, API contracts, and developer quickstart guide.

**Deliverables**:
- `data-model.md` - Deployment entities and state transitions
- `contracts/` - Schema definitions for workflows, reports, logs
- `quickstart.md` - Step-by-step setup instructions

### Data Model

**Entities**:
1. **GitHubRepository** - Source of truth for code
2. **GitHubActionsWorkflow** - CI/CD automation runner
3. **CodeReviewReport** - Analysis results from linters/formatters
4. **GitHubPagesDeployment** - Frontend hosting instance
5. **HuggingFaceSpace** - Backend hosting instance
6. **DeploymentLog** - Event record with timestamps and status
7. **EnvironmentSecret** - Secure configuration value

**Relationships**:
- Repository triggers Workflows on push
- Workflows generate ReviewReports (blocking gate)
- Workflows create Deployments (GitHub Pages, HF Space)
- Deployments emit DeploymentLogs
- Deployments consume EnvironmentSecrets

See `data-model.md` for entity attributes, state diagrams, and lifecycle flows.

### Contracts

**GitHub Actions Workflow Schema** (`github-actions-workflow-schema.yaml`):
- Workflow structure for `deploy.yml` and `code-review.yml`
- Job definitions, steps, inputs, outputs
- Secret references and environment variables

**Code Review Report Schema** (`code-review-report-schema.json`):
- Review result format (pass/fail, issues found)
- Issue structure (file, line, severity, message, rule)
- Metadata (tool name, execution time, commit SHA)

**Deployment Log Schema** (`deployment-log-schema.json`):
- Log entry format (timestamp, level, message, context)
- Deployment event types (started, building, deployed, failed, rolled_back)
- Metadata (deployment ID, environment, commit SHA, duration)

### Quickstart

Developer setup guide covering:
1. Prerequisites (Node.js, Python, Git, GitHub/HF accounts)
2. Local development setup (install deps, configure .env)
3. GitHub Secrets configuration (COHERE_API_KEY, QDRANT_API_KEY, HF_TOKEN)
4. HF Spaces setup (create Space, configure secrets, link repo)
5. Triggering deployments (push to 001-book-creation branch)
6. Monitoring deployments (GitHub Actions UI, HF Spaces logs)
7. Troubleshooting common issues (build failures, secret errors, CORS)

See `quickstart.md` for complete step-by-step instructions.

---

## Phase 2: Task Breakdown

**Goal**: Generate atomic, testable tasks for implementation.

**Deliverable**: `tasks.md` (created by `/sp.tasks` command, NOT by `/sp.plan`)

**Task Categories** (preview):
1. Update `.github/workflows/deploy.yml` to target `001-book-creation` branch
2. Create `.github/workflows/code-review.yml` for linting/formatting checks
3. Add ESLint and Prettier to frontend CI checks
4. Add Ruff and Black to backend CI checks
5. Create `backend/app.py` as HF Spaces entrypoint
6. Generate `backend/requirements.txt` from `pyproject.toml`
7. Update `backend/README.md` with HF deployment instructions
8. Configure GitHub Actions Secrets (COHERE_API_KEY, QDRANT_API_KEY, HF_TOKEN)
9. Create HF Space and link to GitHub repository
10. Configure HF Spaces environment variables
11. Update `api.py` CORS configuration for production domain
12. Test end-to-end deployment flow
13. Document rollback procedures

**Note**: Detailed task breakdown with acceptance criteria will be generated by `/sp.tasks`.

---

## Implementation Phases Summary

| Phase | Deliverable | Status | Dependencies |
|-------|-------------|--------|--------------|
| **Phase 0** | `research.md` | ✓ Ready | None |
| **Phase 1** | `data-model.md`, `contracts/`, `quickstart.md` | ✓ Ready | Phase 0 |
| **Phase 2** | `tasks.md` | Pending | Phase 1, `/sp.tasks` command |
| **Phase 3** | Implementation | Pending | Phase 2, task-by-task execution |
| **Phase 4** | Testing & Validation | Pending | Phase 3, all tasks complete |

---

## Risk Analysis

| Risk | Impact | Mitigation |
|------|--------|------------|
| GitHub Actions quota exceeded | High - Deployment blocked | Monitor usage, optimize workflow efficiency, use caching |
| HF Spaces free tier limits | Medium - Performance degradation | Monitor resource usage, optimize backend, upgrade if needed |
| Secrets exposed in logs | Critical - Security breach | Validate secret masking, never log secret values, audit workflows |
| CORS misconfiguration | High - Frontend can't call backend | Test CORS in staging, use explicit origin whitelist, add health checks |
| Code review false positives | Low - Developer friction | Tune linter rules, allow override with justification |
| Deployment pipeline failure | High - Site unavailable | Implement rollback automation, keep previous deployment active |
| Concurrent deployments | Medium - Race conditions | Use GitHub Actions concurrency groups, queue deployments |

---

## Success Metrics

**Deployment Speed**:
- Frontend deployment: < 5 minutes from push to live
- Backend deployment: < 10 minutes from push to running

**Deployment Reliability**:
- Success rate: > 95% (excluding code errors)
- Rollback time: < 5 minutes

**Code Quality**:
- Code review finds issues: > 80% detection rate
- False positive rate: < 10%

**Security**:
- Zero secrets in Git history (validated by git-secrets scan)
- Zero secrets in logs (validated by log analysis)

**Developer Experience**:
- Setup time: < 30 minutes for new developer
- Deployment trigger: Single command (git push)
- Error feedback: < 2 minutes after failure

---

## Next Steps

1. ✓ Review and approve this plan
2. ✓ Create `research.md` with detailed technology analysis
3. ✓ Create `data-model.md` with entity definitions
4. ✓ Create `contracts/` with schema files
5. ✓ Create `quickstart.md` with setup instructions
6. Run `/sp.tasks` to generate atomic task breakdown
7. Begin implementation task-by-task with TDD approach
8. Validate deployment flow end-to-end
9. Document lessons learned and update plan if needed

---

**Plan Status**: ✓ Complete - Ready for Phase 1 artifact generation
**Last Updated**: 2025-12-19
**Next Milestone**: Create research.md, data-model.md, contracts/, quickstart.md
