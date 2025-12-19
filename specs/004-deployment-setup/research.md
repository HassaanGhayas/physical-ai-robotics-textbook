# Research: Deployment Setup & Code Review

**Feature**: 004-deployment-setup
**Date**: 2025-12-19
**Status**: Complete

## Executive Summary

This document analyzes technology choices for deploying the Physical AI & Humanoid Robotics textbook project. Key decisions:

1. **Code Review**: GitHub Actions with ESLint/Prettier (frontend) + Ruff/Black (backend)
2. **Workflow Structure**: Separate workflows per deployment target (frontend, backend)
3. **HF Deployment**: Git-based deployment with GitHub Actions automation
4. **Secrets Management**: GitHub Actions Secrets + HF Spaces environment variables
5. **Review Integration**: GitHub Actions on PR (blocking) + optional pre-commit hooks

All choices prioritize cost efficiency (free tier), security, automation, and developer experience.

---

## 1. Code Review Tool Selection

### Research Question
Which code review tool should we use to automatically analyze code quality before deployment?

### Options Analyzed

#### Option 1: GitHub Actions Built-in Reviewers (ESLint, Prettier, Ruff, Black)

**Description**: Native GitHub Actions workflows using language-specific linters and formatters.

**Pros**:
- Free and unlimited usage on public repositories
- Fast execution (< 2 minutes per check)
- Native GitHub integration (no external accounts)
- Language-specific best practices (JavaScript/TypeScript + Python)
- Easy configuration via config files (.eslintrc, pyproject.toml)
- No data sharing with third parties
- Actionable error messages with file/line numbers

**Cons**:
- Limited to syntax and style checks (no deep code analysis)
- No historical trend tracking (each run is independent)
- Manual rule configuration required
- No security vulnerability scanning (requires separate tools)

**Cost**: Free

**Integration Effort**: Low (1-2 hours to set up workflows)

**Frontend Tools**:
- ESLint: JavaScript/TypeScript linting (react, jsx-a11y, typescript-eslint rules)
- Prettier: Code formatting (consistent style across team)

**Backend Tools**:
- Ruff: Fast Python linter (replaces Flake8, isort, pydocstyle)
- Black: Opinionated Python formatter (PEP 8 compliant)

**Example Workflow**:
```yaml
- name: Lint frontend
  run: npm run lint
- name: Format check frontend
  run: npm run format:check
- name: Lint backend
  run: cd backend && ruff check .
- name: Format check backend
  run: cd backend && black --check .
```

---

#### Option 2: Third-Party SaaS (SonarCloud, CodeClimate, Codacy)

**Description**: Cloud-based code quality platforms with comprehensive analysis.

**Pros**:
- Deep code analysis (code smells, complexity, duplication)
- Security vulnerability detection (OWASP Top 10)
- Historical trend tracking and quality gates
- Pull request decoration with inline comments
- Multi-language support (JavaScript, TypeScript, Python)
- Technical debt estimation

**Cons**:
- Requires external account setup (SonarCloud, CodeClimate, Codacy)
- Data sharing with third-party service
- Free tier limits (e.g., SonarCloud: 100k LOC, public repos only)
- Slower execution (3-5 minutes per analysis)
- More complex configuration (quality profiles, gates)
- Integration overhead (webhooks, tokens)

**Cost**:
- SonarCloud: Free for public repos (up to 100k LOC)
- CodeClimate: Free for open source
- Codacy: Free for public repos (up to 4 users)

**Integration Effort**: Medium (3-5 hours for initial setup, token management)

**Example Integration**:
```yaml
- name: SonarCloud Scan
  uses: SonarSource/sonarcloud-github-action@master
  env:
    GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
    SONAR_TOKEN: ${{ secrets.SONAR_TOKEN }}
```

---

#### Option 3: AI-Powered (GitHub Copilot, Custom Claude Code Agent)

**Description**: AI agents that review code using large language models.

**Pros**:
- Context-aware review (understands intent, not just syntax)
- Natural language explanations (why issue matters)
- Suggests alternative implementations
- Can review architecture decisions (not just code style)
- Learns from project-specific patterns

**Cons**:
- Expensive (GitHub Copilot: $10/user/month minimum)
- Inconsistent results (AI may miss issues or hallucinate)
- Requires significant prompt engineering for custom agents
- Slower execution (5-10 minutes with API rate limits)
- No standardized output format (harder to block deployment)
- Privacy concerns (code sent to third-party AI service)

**Cost**:
- GitHub Copilot: $10/user/month (individual), $19/user/month (business)
- Custom Claude Code: $0.015 per 1k input tokens (variable cost)

**Integration Effort**: High (8-12 hours for custom agent, prompt tuning)

---

### Decision Matrix

| Criteria | GitHub Actions | SaaS (SonarCloud) | AI-Powered |
|----------|----------------|-------------------|------------|
| **Cost** | Free | Free (public repos) | $10-19/user/month |
| **Speed** | Fast (< 2 min) | Medium (3-5 min) | Slow (5-10 min) |
| **Integration** | Easy (1-2 hrs) | Medium (3-5 hrs) | Hard (8-12 hrs) |
| **Language Support** | JS/TS + Python | Multi-language | Multi-language |
| **Security Scanning** | No (needs add-on) | Yes (OWASP) | Limited |
| **Actionable Feedback** | Yes (file:line) | Yes (inline PR) | Variable |
| **Data Privacy** | Full control | Third-party | Third-party |
| **Maintenance** | Low | Medium | High |

### Recommendation

**Selected**: GitHub Actions with ESLint/Prettier (frontend) + Ruff/Black (backend)

**Rationale**:
1. **Cost**: Completely free for public repositories (zero budget impact)
2. **Speed**: Fast execution keeps deployment pipeline under 5 minutes
3. **Integration**: Native GitHub integration, no external dependencies
4. **Privacy**: Code stays within GitHub infrastructure
5. **Maintenance**: Low overhead, config files in repository
6. **Sufficiency**: Project needs style/syntax checks more than deep analysis

**Trade-offs Accepted**:
- No historical trend tracking (acceptable for initial deployment)
- No security scanning (can add GitHub Dependabot separately)
- Limited to style/syntax (sufficient for documentation site + API)

**Future Enhancement Path**:
If needed later, add SonarCloud for deeper analysis (still free for public repos). This is a non-breaking addition that can run in parallel.

---

## 2. GitHub Actions Workflow Structure

### Research Question
How should GitHub Actions workflows be organized for frontend and backend deployments?

### Options Analyzed

#### Option 1: Single Monolithic Workflow

**Description**: One `deploy.yml` file handling both frontend and backend.

**Structure**:
```yaml
jobs:
  code-review:
    runs-on: ubuntu-latest
    steps:
      - Lint frontend
      - Lint backend
  deploy-frontend:
    needs: code-review
    steps:
      - Build Docusaurus
      - Deploy to GitHub Pages
  deploy-backend:
    needs: code-review
    steps:
      - Push to Hugging Face
```

**Pros**:
- Single workflow file to maintain
- Guaranteed execution order (frontend → backend)
- Easier to enforce "all or nothing" deployment

**Cons**:
- Failure in one deployment blocks the other
- Slower execution (sequential, not parallel)
- Harder to debug (mixed logs)
- Frontend and backend changes force full pipeline
- No independent rollback capability

---

#### Option 2: Separate Workflows Per Target

**Description**: Independent workflows: `deploy-frontend.yml`, `deploy-backend.yml`, `code-review.yml`

**Structure**:
```
.github/workflows/
├── code-review.yml        # Runs on all PRs
├── deploy-frontend.yml    # Runs on push to main (docs/ changes)
└── deploy-backend.yml     # Runs on push to main (backend/ changes)
```

**Pros**:
- Independent failure handling (frontend fails, backend still deploys)
- Faster execution (parallel deployments)
- Clear separation of concerns (easier to maintain)
- Path-based triggers (only run when relevant files change)
- Independent rollback (rollback frontend without affecting backend)
- Easier debugging (isolated logs)

**Cons**:
- More workflow files to maintain (3 instead of 1)
- Requires careful path filtering to avoid duplicate runs
- No built-in "deploy both together" option

---

#### Option 3: Matrix Jobs for Parallel Execution

**Description**: Single workflow with matrix strategy for parallel jobs.

**Structure**:
```yaml
jobs:
  deploy:
    strategy:
      matrix:
        target: [frontend, backend]
    steps:
      - if: matrix.target == 'frontend'
        run: deploy-frontend.sh
      - if: matrix.target == 'backend'
        run: deploy-backend.sh
```

**Pros**:
- Parallel execution (faster than monolithic)
- Single workflow file (simpler than separate workflows)
- DRY shared setup steps (checkout, caching)

**Cons**:
- Complex conditional logic in steps
- Harder to read and debug
- Less flexible (harder to customize per target)
- Matrix parallelism not needed (only 2 targets)

---

### Decision Matrix

| Criteria | Monolithic | Separate Workflows | Matrix Jobs |
|----------|------------|-------------------|-------------|
| **Maintainability** | Medium | High | Low |
| **Execution Speed** | Slow (sequential) | Fast (parallel) | Fast (parallel) |
| **Failure Isolation** | No | Yes | Partial |
| **Debugging** | Hard | Easy | Hard |
| **Independent Rollback** | No | Yes | No |
| **Path Filtering** | Manual | Built-in | Manual |

### Recommendation

**Selected**: Separate Workflows Per Target

**Structure**:
```
.github/workflows/
├── code-review.yml        # Lints/formats all code on PR
├── deploy.yml             # Frontend deployment (existing, update branch)
└── huggingface-deploy.yml # Backend deployment (new)
```

**Rationale**:
1. **Failure Isolation**: Frontend deployment failure doesn't block backend
2. **Speed**: Parallel execution when both change simultaneously
3. **Clarity**: Each workflow is self-contained and easy to understand
4. **Path Filtering**: Only run frontend workflow when docs/ changes
5. **Rollback**: Independent rollback capability per deployment target

**Path Filtering Example**:
```yaml
# deploy.yml (frontend)
on:
  push:
    branches: [001-book-creation]
    paths:
      - 'src/**'
      - 'docs/**'
      - 'static/**'
      - 'docusaurus.config.ts'
      - 'package.json'

# huggingface-deploy.yml (backend)
on:
  push:
    branches: [001-book-creation]
    paths:
      - 'backend/**'
```

**Code Review Integration**:
`code-review.yml` runs on all pull requests (blocking) before merge, ensuring quality gate before any deployment.

---

## 3. Hugging Face Spaces Deployment Method

### Research Question
How should the backend FastAPI application be deployed to Hugging Face Spaces?

### Options Analyzed

#### Option 1: Git-Based Deployment

**Description**: Create a Hugging Face Space linked to GitHub repository. Push to HF repo triggers automatic rebuild.

**Workflow**:
1. Create HF Space with "Link to GitHub" option
2. HF automatically syncs `backend/` directory
3. HF detects `app.py` and `requirements.txt`, builds Docker image
4. HF starts application on Space URL

**Pros**:
- Fully automated (no manual steps after setup)
- Version controlled (Git history preserved)
- Simple configuration (just link repositories)
- Native HF integration (no API tokens needed for deployment)
- Automatic rebuilds on push to main
- HF handles all infrastructure (Docker, networking, SSL)

**Cons**:
- Requires separate HF repository or subdirectory sync
- Initial setup requires HF account and Space creation
- Limited control over build process
- Deployment errors only visible in HF Space logs

**Setup Steps**:
1. Create HF Space at https://huggingface.co/new-space
2. Select "SDK: Docker" and link to GitHub repo
3. Configure HF to sync `backend/` directory
4. Set environment variables in HF Space settings
5. Push to GitHub → HF auto-deploys

---

#### Option 2: GitHub Actions with HF API

**Description**: Use GitHub Actions to build and push Docker image to HF via API.

**Workflow**:
```yaml
- name: Build Docker image
  run: docker build -t app:latest backend/
- name: Push to HF Registry
  run: |
    docker tag app:latest registry.huggingface.co/my-space:latest
    docker push registry.huggingface.co/my-space:latest
```

**Pros**:
- Full control over build process (custom Docker steps)
- Can run tests before pushing to HF
- Deployment is part of CI/CD pipeline (visible in GitHub Actions)
- Can deploy to multiple HF Spaces (staging, production)

**Cons**:
- More complex setup (Docker, HF tokens, registry auth)
- Requires HF_TOKEN secret in GitHub Actions
- GitHub Actions must build Docker image (uses runner minutes)
- More points of failure (GitHub, Docker, HF registry)
- Slower execution (build + push vs. HF native build)

**Setup Steps**:
1. Create HF Space manually
2. Generate HF access token with write permissions
3. Add HF_TOKEN to GitHub Actions Secrets
4. Create GitHub Actions workflow to build and push Docker image
5. Configure HF Space to pull from registry

---

#### Option 3: Manual Deployment via HF UI

**Description**: Developer manually uploads code to HF Space via web interface.

**Workflow**:
1. Developer logs into HF Space
2. Uploads files via "Files" tab
3. HF rebuilds Space automatically

**Pros**:
- No automation infrastructure needed
- Simple for one-time deployments
- No secrets management required

**Cons**:
- Manual process (error-prone, slow)
- Not version controlled (Git history lost)
- Doesn't scale (every deployment requires manual steps)
- No rollback capability (must re-upload old files)
- Human bottleneck (blocks automation goal)

---

### Decision Matrix

| Criteria | Git-Based | GitHub Actions + API | Manual UI |
|----------|-----------|---------------------|-----------|
| **Automation** | Full | Full | None |
| **Setup Complexity** | Low | High | None |
| **Version Control** | Yes | Yes | No |
| **Build Control** | Limited | Full | None |
| **Maintenance** | Low | Medium | High |
| **Rollback** | Git revert | Git revert + redeploy | Manual |
| **Speed** | Fast | Slower | Slowest |

### Recommendation

**Selected**: Git-Based Deployment with GitHub Actions Sync

**Rationale**:
1. **Simplicity**: Native HF integration, minimal configuration
2. **Automation**: Fully automated after initial setup
3. **Version Control**: Git history preserved, easy rollback via Git
4. **Maintenance**: Low overhead, HF handles infrastructure
5. **Cost**: Free (no GitHub Actions minutes spent on Docker builds)
6. **Reliability**: Fewer moving parts (no custom Docker pipeline)

**Implementation Details**:
1. Create HF Space: `https://huggingface.co/spaces/[username]/physical-ai-robotics-backend`
2. Link to GitHub: Select "Link to GitHub repository"
3. Configure Space:
   - SDK: Docker
   - Source: `backend/` directory from GitHub repo
   - Entrypoint: `app.py` (wrapper for `api.py`)
4. Set HF Secrets: COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL
5. Push to 001-book-creation → HF auto-deploys

**GitHub Actions Role**:
Optional workflow can trigger HF rebuild via webhook (not required for basic deployment).

**Trade-offs Accepted**:
- Limited build customization (acceptable, no special requirements)
- HF-specific logging (acceptable, HF logs are detailed)

---

## 4. Secrets Management Pattern

### Research Question
How should sensitive credentials (API keys, tokens) be stored and accessed during deployment?

### Options Analyzed

#### Option 1: GitHub Actions Secrets + HF Spaces Environment Variables

**Description**: Use platform-native secret stores for each deployment target.

**Implementation**:
- **GitHub Actions**: Secrets stored in repository settings (Settings → Secrets → Actions)
- **HF Spaces**: Environment variables stored in Space settings (Settings → Environment Variables)
- **Local Development**: `.env` file (gitignored, documented in `.env.example`)

**Secret Flow**:
```
Developer → GitHub Secrets → GitHub Actions → Deployment
Developer → HF Space Settings → HF Runtime → Backend App
Developer → Local .env → Local Development → Not Committed
```

**Pros**:
- Native platform support (no external dependencies)
- Free (no additional cost)
- Secure (encrypted at rest, masked in logs)
- Simple setup (UI-based configuration)
- Per-environment isolation (dev, production)
- Automatic injection (no custom code needed)

**Cons**:
- Platform lock-in (GitHub + HF specific)
- No cross-platform sharing (must set secrets in each platform)
- No secret rotation automation (manual updates required)
- Limited secret types (strings only, no files)

**Example GitHub Actions Usage**:
```yaml
env:
  COHERE_API_KEY: ${{ secrets.COHERE_API_KEY }}
  QDRANT_API_KEY: ${{ secrets.QDRANT_API_KEY }}
```

**Example HF Spaces Configuration**:
```
Settings → Environment Variables:
COHERE_API_KEY = [value]
QDRANT_API_KEY = [value]
QDRANT_URL = https://[cluster].qdrant.io
```

---

#### Option 2: External Secret Manager (AWS Secrets Manager, HashiCorp Vault)

**Description**: Centralized secret storage accessed via API during deployment.

**Workflow**:
1. Store secrets in AWS Secrets Manager or Vault
2. GitHub Actions fetches secrets at runtime via API
3. HF Spaces fetches secrets at startup via API
4. Application caches secrets in memory

**Pros**:
- Centralized secret management (single source of truth)
- Automatic rotation support (AWS Lambda, Vault policies)
- Audit logging (who accessed what, when)
- Fine-grained access control (IAM policies)
- Supports multiple secret types (strings, files, certificates)

**Cons**:
- Additional cost (AWS: $0.40/secret/month + $0.05/10k API calls)
- Complex setup (IAM roles, API credentials, network access)
- External dependency (AWS/Vault downtime blocks deployment)
- Over-engineering for simple use case (only 3-4 secrets)
- Requires managing bootstrap credentials (chicken-and-egg)

**Cost Example**:
- 5 secrets × $0.40/month = $2/month
- 1000 API calls/month × $0.05/10k = $0.005/month
- Total: ~$2.01/month (vs. $0 for native approach)

---

#### Option 3: Encrypted .env Files in Repository

**Description**: Commit encrypted `.env.production` file to repository, decrypt during deployment.

**Workflow**:
1. Encrypt `.env` file with GPG or `git-crypt`
2. Commit encrypted file to repository
3. GitHub Actions decrypts file using secret key
4. Decrypted values used in deployment

**Pros**:
- Version controlled (secrets tracked in Git)
- No external dependencies (self-contained)
- Simple rotation (update encrypted file)

**Cons**:
- Security risk (encrypted secrets in public repository)
- Key management complexity (decryption key must be stored somewhere)
- Rotation requires new commit (visible in Git history)
- Not recommended by security best practices (GitHub warns against it)
- Single point of failure (compromised key exposes all secrets)

---

### Decision Matrix

| Criteria | Native Platform | External Manager | Encrypted .env |
|----------|----------------|------------------|----------------|
| **Cost** | Free | $2+/month | Free |
| **Security** | High | Highest | Medium |
| **Complexity** | Low | High | Medium |
| **Rotation** | Manual | Automatic | Manual |
| **Audit Logging** | Limited | Full | None |
| **Setup Time** | 15 min | 2-3 hours | 30 min |

### Recommendation

**Selected**: GitHub Actions Secrets + HF Spaces Environment Variables

**Rationale**:
1. **Cost**: Zero additional cost (free tier sufficient)
2. **Security**: Platform-native encryption, masked in logs
3. **Simplicity**: UI-based setup, no external services
4. **Maintenance**: Low overhead, no key rotation complexity
5. **Sufficiency**: Project has only 3-4 secrets, no complex rotation needs

**Secret Inventory**:
| Secret | Storage Location | Used By |
|--------|-----------------|---------|
| COHERE_API_KEY | GitHub + HF | Backend RAG agent |
| QDRANT_API_KEY | GitHub + HF | Backend vector retrieval |
| QDRANT_URL | GitHub + HF | Backend Qdrant client |
| HF_TOKEN | GitHub only | GitHub Actions (optional) |

**Security Practices**:
1. Never commit `.env` file (verified in `.gitignore`)
2. Always commit `.env.example` with placeholder values
3. Mask secrets in logs (GitHub Actions does this automatically)
4. Rotate secrets if exposed (manual process documented)
5. Use minimum required permissions (read-only where possible)

**Trade-offs Accepted**:
- Manual secret rotation (acceptable, low turnover rate)
- No audit logging (acceptable, small team)
- Platform lock-in (acceptable, no plans to migrate)

---

## 5. Code Review Integration Points

### Research Question
When and where should automated code review run in the development workflow?

### Options Analyzed

#### Option 1: Pre-Commit Hooks (Local)

**Description**: Git hooks that run linters/formatters before allowing commit.

**Implementation**:
```bash
# .git/hooks/pre-commit
npm run lint
npm run format:check
cd backend && ruff check . && black --check .
```

**Pros**:
- Immediate feedback (before code leaves developer's machine)
- No CI minutes consumed (runs locally)
- Prevents "broken" commits from entering history
- Fast iteration (fix issues before push)

**Cons**:
- Developer can bypass hooks (`git commit --no-verify`)
- Not enforceable (relies on developer discipline)
- Requires local setup (onboarding overhead)
- Inconsistent environments (different Node/Python versions)
- Slows down commit process (blocks commit until checks pass)

---

#### Option 2: GitHub Actions on PR (Blocking)

**Description**: CI workflow that runs on pull request and blocks merge until checks pass.

**Implementation**:
```yaml
name: Code Review
on:
  pull_request:
    branches: [001-book-creation]
jobs:
  review:
    runs-on: ubuntu-latest
    steps:
      - Checkout code
      - Run linters
      - Run formatters
```

**Pros**:
- Enforceable (GitHub branch protection rules prevent bypass)
- Consistent environment (same runner for all developers)
- No local setup required (works out of the box)
- Visible to team (PR comments show failures)
- Blocks merge (prevents bad code from reaching main)

**Cons**:
- Slower feedback (only after push, not before commit)
- Uses CI minutes (GitHub Actions quota)
- Requires PR workflow (can't check on direct push)

---

#### Option 3: Post-Merge Analysis (Non-Blocking)

**Description**: Workflow runs after merge to main, reports issues but doesn't block.

**Implementation**:
```yaml
name: Code Quality
on:
  push:
    branches: [001-book-creation]
jobs:
  analyze:
    runs-on: ubuntu-latest
    steps:
      - Run linters
      - Post results to Slack/email
```

**Pros**:
- Doesn't block development (fast iteration)
- Can include expensive checks (deep analysis)
- Historical tracking (quality trends over time)

**Cons**:
- Doesn't prevent bad code (already merged)
- Requires manual cleanup (fix issues after the fact)
- Creates technical debt (issues accumulate)
- Defeats purpose of "review before deployment"

---

### Decision Matrix

| Criteria | Pre-Commit Hooks | PR Blocking | Post-Merge |
|----------|-----------------|-------------|------------|
| **Enforcement** | Weak (bypassable) | Strong | None |
| **Feedback Speed** | Immediate | Fast (< 2 min) | Slow (after merge) |
| **Setup Complexity** | Medium | Low | Low |
| **Developer Experience** | Good (fast) | Medium (wait for CI) | Bad (retroactive) |
| **Code Quality** | High | High | Low |

### Recommendation

**Selected**: GitHub Actions on PR (Blocking) + Optional Pre-Commit Hooks

**Rationale**:
1. **Enforcement**: Branch protection ensures no bypass
2. **Consistency**: Same checks for all developers
3. **Visibility**: Team sees issues in PR comments
4. **Automation**: No manual review step needed
5. **Flexibility**: Developers can add pre-commit hooks for faster feedback

**Implementation Strategy**:

**Required (Blocking)**:
```yaml
# .github/workflows/code-review.yml
name: Code Review
on:
  pull_request:
    branches: [001-book-creation]
jobs:
  lint-frontend:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
      - run: npm ci
      - run: npm run lint
      - run: npm run format:check

  lint-backend:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v4
      - run: pip install ruff black
      - run: cd backend && ruff check .
      - run: cd backend && black --check .
```

**Optional (Local)**:
```bash
# .git/hooks/pre-commit (developers can install manually)
#!/bin/sh
npm run lint && npm run format:check && cd backend && ruff check . && black --check .
```

**GitHub Branch Protection**:
```
Settings → Branches → Branch protection rules:
- Branch name pattern: 001-book-creation
- Require status checks to pass before merging:
  ✓ lint-frontend
  ✓ lint-backend
```

**Trade-offs Accepted**:
- Slower feedback than pre-commit (acceptable, < 2 min CI time)
- Uses GitHub Actions minutes (acceptable, free for public repos)
- Requires PR workflow (acceptable, already using PRs)

---

## Summary of Recommendations

| Decision Area | Selected Option | Key Benefit |
|--------------|----------------|-------------|
| **Code Review Tool** | GitHub Actions + ESLint/Prettier/Ruff/Black | Free, fast, native integration |
| **Workflow Structure** | Separate workflows per target | Failure isolation, parallel execution |
| **HF Deployment** | Git-based with HF auto-sync | Simple, reliable, version-controlled |
| **Secrets Management** | GitHub Secrets + HF env vars | Native, secure, zero cost |
| **Review Integration** | PR blocking + optional pre-commit | Enforceable, consistent, visible |

**Total Setup Time**: 2-3 hours for complete deployment infrastructure

**Total Recurring Cost**: $0 (all free tier services)

**Maintenance Overhead**: Low (< 1 hour/month for updates)

---

## Implementation Checklist

Phase 0 research complete. Ready to proceed to Phase 1 (data model, contracts, quickstart).

- ✓ Code review tools selected
- ✓ Workflow structure designed
- ✓ HF deployment method chosen
- ✓ Secrets management pattern defined
- ✓ Review integration points identified
- ✓ Trade-offs documented
- ✓ Cost analysis complete
- ✓ Risk mitigation strategies defined

**Next Steps**: Create `data-model.md`, `contracts/`, and `quickstart.md`.

---

**Research Status**: Complete
**Date**: 2025-12-19
**Approved By**: DevOps Engineer Agent
