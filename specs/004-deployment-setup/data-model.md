# Data Model: Deployment Setup & Code Review

**Feature**: 004-deployment-setup
**Date**: 2025-12-19
**Status**: Complete

## Overview

This document defines the entities, relationships, and state transitions for the deployment infrastructure. The system manages automated deployments of a Docusaurus frontend to GitHub Pages and a FastAPI backend to Hugging Face Spaces, with automated code review gates and secure secrets management.

---

## Entity Definitions

### 1. GitHubRepository

**Description**: Source code repository hosted on GitHub, containing both frontend and backend code.

**Attributes**:
| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| owner | string | Yes | GitHub username or organization (e.g., "HassaanGhayas") |
| name | string | Yes | Repository name (e.g., "physical-ai-robotics-textbook") |
| main_branch | string | Yes | Main deployment branch (e.g., "001-book-creation") |
| url | string | Yes | Repository URL (e.g., "https://github.com/HassaanGhayas/...") |
| visibility | enum | Yes | "public" or "private" |
| last_commit_sha | string | Yes | SHA of latest commit on main branch |
| last_commit_timestamp | datetime | Yes | ISO 8601 timestamp of latest commit |

**Constraints**:
- URL must match pattern: `https://github.com/{owner}/{name}`
- main_branch must exist in repository
- Repository must have GitHub Actions enabled

**Example**:
```json
{
  "owner": "HassaanGhayas",
  "name": "physical-ai-robotics-textbook",
  "main_branch": "001-book-creation",
  "url": "https://github.com/HassaanGhayas/physical-ai-robotics-textbook",
  "visibility": "public",
  "last_commit_sha": "088573c4a5d...",
  "last_commit_timestamp": "2025-12-19T10:30:00Z"
}
```

---

### 2. GitHubActionsWorkflow

**Description**: CI/CD automation workflow that triggers on repository events (push, pull request).

**Attributes**:
| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| workflow_id | string | Yes | Unique identifier (e.g., "deploy-frontend") |
| workflow_name | string | Yes | Human-readable name (e.g., "Deploy Frontend") |
| file_path | string | Yes | Path to workflow YAML (e.g., ".github/workflows/deploy.yml") |
| trigger_event | enum | Yes | "push", "pull_request", "workflow_dispatch" |
| trigger_branches | array[string] | Yes | Branches that trigger workflow (e.g., ["001-book-creation"]) |
| trigger_paths | array[string] | No | File paths that trigger workflow (e.g., ["src/**", "docs/**"]) |
| jobs | array[Job] | Yes | List of jobs in workflow |
| status | enum | Yes | "idle", "queued", "running", "success", "failure", "cancelled" |
| run_id | string | No | GitHub Actions run ID (present when running) |
| run_url | string | No | URL to workflow run (e.g., "https://github.com/.../actions/runs/...") |
| started_at | datetime | No | ISO 8601 timestamp when workflow started |
| completed_at | datetime | No | ISO 8601 timestamp when workflow completed |
| duration_seconds | integer | No | Total execution time in seconds |

**Constraints**:
- file_path must exist in repository under `.github/workflows/`
- trigger_branches must match repository branches
- status transitions must follow: idle → queued → running → (success|failure|cancelled)
- duration_seconds = completed_at - started_at (if both present)

**Job Structure**:
```json
{
  "job_id": "lint-frontend",
  "job_name": "Lint Frontend Code",
  "runs_on": "ubuntu-latest",
  "steps": [
    {"name": "Checkout code", "uses": "actions/checkout@v4"},
    {"name": "Setup Node", "uses": "actions/setup-node@v4"},
    {"name": "Install dependencies", "run": "npm ci"},
    {"name": "Run ESLint", "run": "npm run lint"}
  ],
  "status": "success",
  "conclusion": "success"
}
```

**Example**:
```json
{
  "workflow_id": "deploy-frontend",
  "workflow_name": "Deploy to GitHub Pages",
  "file_path": ".github/workflows/deploy.yml",
  "trigger_event": "push",
  "trigger_branches": ["001-book-creation"],
  "trigger_paths": ["src/**", "docs/**", "docusaurus.config.ts"],
  "jobs": [
    {
      "job_id": "deploy",
      "job_name": "Deploy to GitHub Pages",
      "runs_on": "ubuntu-latest",
      "steps": [...],
      "status": "success"
    }
  ],
  "status": "success",
  "run_id": "7654321098",
  "run_url": "https://github.com/HassaanGhayas/physical-ai-robotics-textbook/actions/runs/7654321098",
  "started_at": "2025-12-19T10:31:00Z",
  "completed_at": "2025-12-19T10:35:00Z",
  "duration_seconds": 240
}
```

---

### 3. CodeReviewReport

**Description**: Analysis results from automated code review (linters, formatters).

**Attributes**:
| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| report_id | string | Yes | Unique identifier (UUID) |
| commit_sha | string | Yes | Commit SHA that was reviewed |
| tool_name | string | Yes | Review tool used (e.g., "ESLint", "Ruff") |
| tool_version | string | Yes | Tool version (e.g., "8.52.0") |
| status | enum | Yes | "pass", "fail", "error" |
| issue_count | integer | Yes | Total number of issues found |
| issues | array[Issue] | Yes | List of specific issues (empty if none) |
| execution_time_ms | integer | Yes | Time taken to run review in milliseconds |
| created_at | datetime | Yes | ISO 8601 timestamp when review completed |

**Constraints**:
- status = "pass" if issue_count = 0
- status = "fail" if issue_count > 0 and all issues are "error" or "warning"
- status = "error" if tool execution failed
- issue_count = len(issues)

**Issue Structure**:
```json
{
  "file_path": "src/components/ChatBot/ChatBot.tsx",
  "line": 42,
  "column": 12,
  "severity": "error",
  "message": "React Hook 'useState' is called conditionally",
  "rule": "react-hooks/rules-of-hooks",
  "suggested_fix": null
}
```

**Severity Levels**:
- `error`: Must be fixed (blocks deployment)
- `warning`: Should be fixed (doesn't block deployment)
- `info`: Informational (FYI only)

**Example**:
```json
{
  "report_id": "a7b8c9d0-1234-5678-90ab-cdef12345678",
  "commit_sha": "088573c4a5d...",
  "tool_name": "ESLint",
  "tool_version": "8.52.0",
  "status": "fail",
  "issue_count": 2,
  "issues": [
    {
      "file_path": "src/pages/index.tsx",
      "line": 15,
      "column": 8,
      "severity": "error",
      "message": "Missing semicolon",
      "rule": "semi",
      "suggested_fix": "Add semicolon at end of line"
    },
    {
      "file_path": "src/components/ChatBot/ChatBot.tsx",
      "line": 42,
      "column": 12,
      "severity": "warning",
      "message": "Unused variable 'count'",
      "rule": "no-unused-vars",
      "suggested_fix": "Remove unused variable"
    }
  ],
  "execution_time_ms": 1250,
  "created_at": "2025-12-19T10:31:30Z"
}
```

---

### 4. GitHubPagesDeployment

**Description**: Frontend hosting instance on GitHub Pages (static site).

**Attributes**:
| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| deployment_id | string | Yes | Unique identifier (GitHub deployment ID) |
| site_url | string | Yes | Public URL (e.g., "https://hassaanghayas.github.io/...") |
| commit_sha | string | Yes | Commit SHA deployed |
| branch | string | Yes | Source branch (e.g., "gh-pages") |
| status | enum | Yes | "pending", "building", "deployed", "failed" |
| build_logs_url | string | Yes | URL to GitHub Actions logs |
| deployed_at | datetime | No | ISO 8601 timestamp when deployed |
| build_duration_seconds | integer | No | Time taken to build and deploy |
| error_message | string | No | Error details (if status = "failed") |

**Constraints**:
- site_url must be accessible via HTTPS
- status transitions: pending → building → (deployed|failed)
- deployed_at required if status = "deployed"
- error_message required if status = "failed"

**Example**:
```json
{
  "deployment_id": "gh-pages-123456789",
  "site_url": "https://hassaanghayas.github.io/physical-ai-robotics-textbook",
  "commit_sha": "088573c4a5d...",
  "branch": "gh-pages",
  "status": "deployed",
  "build_logs_url": "https://github.com/HassaanGhayas/physical-ai-robotics-textbook/actions/runs/7654321098",
  "deployed_at": "2025-12-19T10:35:00Z",
  "build_duration_seconds": 180,
  "error_message": null
}
```

---

### 5. HuggingFaceSpace

**Description**: Backend hosting instance on Hugging Face Spaces (containerized Python app).

**Attributes**:
| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| space_id | string | Yes | HF Space identifier (e.g., "username/space-name") |
| space_url | string | Yes | Public URL (e.g., "https://username-space-name.hf.space") |
| commit_sha | string | Yes | Commit SHA deployed |
| status | enum | Yes | "pending", "building", "running", "failed", "sleeping" |
| build_logs_url | string | Yes | URL to HF build logs |
| runtime_status | enum | Yes | "starting", "running", "error", "stopped" |
| deployed_at | datetime | No | ISO 8601 timestamp when deployed |
| build_duration_seconds | integer | No | Time taken to build Docker image |
| health_check_url | string | Yes | URL to health endpoint (e.g., "/health") |
| last_health_check_at | datetime | No | Last successful health check timestamp |
| error_message | string | No | Error details (if status = "failed") |

**Constraints**:
- space_url must be accessible via HTTPS
- status transitions: pending → building → (running|failed) → sleeping
- runtime_status = "running" only if status = "running"
- health_check_url must return 200 OK when status = "running"
- deployed_at required if status = "running"
- error_message required if status = "failed"

**Example**:
```json
{
  "space_id": "hassaanghayas/physical-ai-robotics-backend",
  "space_url": "https://hassaanghayas-physical-ai-robotics-backend.hf.space",
  "commit_sha": "088573c4a5d...",
  "status": "running",
  "build_logs_url": "https://huggingface.co/spaces/hassaanghayas/physical-ai-robotics-backend/logs",
  "runtime_status": "running",
  "deployed_at": "2025-12-19T10:40:00Z",
  "build_duration_seconds": 420,
  "health_check_url": "https://hassaanghayas-physical-ai-robotics-backend.hf.space/health",
  "last_health_check_at": "2025-12-19T10:45:00Z",
  "error_message": null
}
```

---

### 6. DeploymentLog

**Description**: Event record with timestamps, status, and context for auditing.

**Attributes**:
| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| log_id | string | Yes | Unique identifier (UUID) |
| deployment_id | string | Yes | Reference to GitHubPagesDeployment or HuggingFaceSpace |
| deployment_type | enum | Yes | "frontend" or "backend" |
| event_type | enum | Yes | "started", "building", "deployed", "failed", "rolled_back" |
| timestamp | datetime | Yes | ISO 8601 timestamp of event |
| level | enum | Yes | "info", "warning", "error" |
| message | string | Yes | Human-readable event description |
| context | object | No | Additional metadata (commit_sha, user, etc.) |

**Constraints**:
- timestamp must be in ISO 8601 format with timezone (UTC)
- level = "error" if event_type = "failed"
- level = "info" for all other event_types (except rollback = "warning")

**Event Types**:
- `started`: Deployment process initiated
- `building`: Build/compilation in progress
- `deployed`: Deployment completed successfully
- `failed`: Deployment failed with error
- `rolled_back`: Previous version restored after failure

**Example**:
```json
{
  "log_id": "f1e2d3c4-5678-90ab-cdef-1234567890ab",
  "deployment_id": "gh-pages-123456789",
  "deployment_type": "frontend",
  "event_type": "deployed",
  "timestamp": "2025-12-19T10:35:00Z",
  "level": "info",
  "message": "Frontend deployed successfully to GitHub Pages",
  "context": {
    "commit_sha": "088573c4a5d...",
    "commit_message": "Update documentation content",
    "deployed_by": "github-actions[bot]",
    "build_duration_seconds": 180,
    "deployment_url": "https://hassaanghayas.github.io/physical-ai-robotics-textbook"
  }
}
```

---

### 7. EnvironmentSecret

**Description**: Secure configuration value stored in platform-native secret stores.

**Attributes**:
| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| secret_name | string | Yes | Variable name (e.g., "COHERE_API_KEY") |
| secret_type | enum | Yes | "api_key", "token", "url", "password" |
| storage_location | enum | Yes | "github_actions", "huggingface_spaces", "local_env" |
| is_required | boolean | Yes | Whether secret is mandatory for deployment |
| used_by | array[string] | Yes | Services using this secret (e.g., ["backend-api"]) |
| description | string | Yes | What this secret is for |
| example_value | string | No | Placeholder value (for .env.example) |
| created_at | datetime | Yes | When secret was first configured |
| updated_at | datetime | Yes | When secret was last modified |

**Constraints**:
- secret_name must be uppercase with underscores (e.g., "COHERE_API_KEY")
- storage_location = "local_env" is for development only (never deployed)
- example_value must be placeholder (e.g., "your_api_key_here"), never real value
- Actual secret value is NEVER stored in this entity (only metadata)

**Example**:
```json
{
  "secret_name": "COHERE_API_KEY",
  "secret_type": "api_key",
  "storage_location": "huggingface_spaces",
  "is_required": true,
  "used_by": ["backend-api", "rag-agent"],
  "description": "Cohere API key for embeddings and text generation",
  "example_value": "your_cohere_api_key_here",
  "created_at": "2025-12-17T20:00:00Z",
  "updated_at": "2025-12-17T20:00:00Z"
}
```

---

## Entity Relationships

### Relationship Diagram

```
GitHubRepository
├── triggers → GitHubActionsWorkflow (on push/PR)
│   ├── generates → CodeReviewReport (blocking gate)
│   └── creates → GitHubPagesDeployment | HuggingFaceSpace
│       └── emits → DeploymentLog (audit trail)
│
└── consumes → EnvironmentSecret (from GitHub Secrets / HF Spaces)
```

### Relationship Details

#### 1. Repository → Workflow (1:N)
- **Type**: One-to-many
- **Description**: One repository can have multiple workflows (frontend, backend, code review)
- **Cardinality**: 1 repository : N workflows
- **Trigger**: Push event, pull request event, manual dispatch

#### 2. Workflow → CodeReviewReport (1:N)
- **Type**: One-to-many
- **Description**: Each workflow run can generate multiple review reports (one per tool)
- **Cardinality**: 1 workflow run : N review reports
- **Dependency**: Deployment workflows depend on review reports passing

#### 3. Workflow → Deployment (1:1)
- **Type**: One-to-one
- **Description**: Each workflow run creates one deployment (frontend OR backend)
- **Cardinality**: 1 workflow run : 1 deployment
- **Conditional**: Only created if code review passes

#### 4. Deployment → DeploymentLog (1:N)
- **Type**: One-to-many
- **Description**: Each deployment emits multiple log entries (started, building, deployed, etc.)
- **Cardinality**: 1 deployment : N log entries
- **Temporal**: Logs capture deployment lifecycle events

#### 5. Deployment → EnvironmentSecret (N:M)
- **Type**: Many-to-many
- **Description**: Deployments consume multiple secrets, secrets are used by multiple deployments
- **Cardinality**: N deployments : M secrets
- **Access**: Secrets injected at runtime via platform-native mechanisms

---

## State Transitions

### GitHubActionsWorkflow State Diagram

```
       ┌──────────────┐
       │     idle     │ (waiting for trigger)
       └──────┬───────┘
              │ push/PR event
              ▼
       ┌──────────────┐
       │    queued    │ (waiting for runner)
       └──────┬───────┘
              │ runner available
              ▼
       ┌──────────────┐
       │   running    │ (executing jobs)
       └──┬─────┬─────┘
          │     │
          │     │ cancelled (manual)
          │     └────────────────────┐
          │                          ▼
          │ all jobs complete   ┌───────────┐
          ├──────────────────►  │ cancelled │ (terminal)
          │                     └───────────┘
          ▼
    ┌──────────┐
    │ success  │ (all jobs passed)
    └──────────┘
         OR
    ┌──────────┐
    │ failure  │ (at least one job failed)
    └──────────┘
```

**Valid Transitions**:
- idle → queued (trigger event occurs)
- queued → running (runner becomes available)
- running → success (all jobs pass)
- running → failure (at least one job fails)
- running → cancelled (manual cancellation)

**Terminal States**: success, failure, cancelled

---

### CodeReviewReport State Diagram

```
    ┌──────────────┐
    │   pending    │ (review not started)
    └──────┬───────┘
           │ workflow starts review
           ▼
    ┌──────────────┐
    │   running    │ (linters/formatters executing)
    └───┬──────┬───┘
        │      │
        │      │ tool execution fails
        │      └──────────────┐
        │                     ▼
        │              ┌──────────┐
        │              │  error   │ (terminal - tool crashed)
        │              └──────────┘
        ▼
  ┌──────────┐
  │   pass   │ (no issues found)
  └──────────┘
       OR
  ┌──────────┐
  │   fail   │ (issues found)
  └──────────┘
```

**Valid Transitions**:
- pending → running (review starts)
- running → pass (no issues found)
- running → fail (issues found)
- running → error (tool execution failed)

**Terminal States**: pass, fail, error

**Blocking Logic**:
- If status = "pass" → deployment proceeds
- If status = "fail" or "error" → deployment blocked

---

### GitHubPagesDeployment State Diagram

```
    ┌──────────────┐
    │   pending    │ (waiting for code review)
    └──────┬───────┘
           │ code review passes
           ▼
    ┌──────────────┐
    │   building   │ (npm run build in progress)
    └───┬──────┬───┘
        │      │
        │      │ build fails
        │      └──────────────┐
        │                     ▼
        │              ┌──────────┐
        │              │  failed  │ (terminal - build error)
        │              └──────────┘
        ▼
  ┌──────────┐
  │ deployed │ (site live on GitHub Pages)
  └──────────┘
```

**Valid Transitions**:
- pending → building (code review passes, build starts)
- building → deployed (build succeeds, deployed to GitHub Pages)
- building → failed (build fails)

**Terminal States**: deployed, failed

---

### HuggingFaceSpace State Diagram

```
    ┌──────────────┐
    │   pending    │ (waiting for code review)
    └──────┬───────┘
           │ code review passes
           ▼
    ┌──────────────┐
    │   building   │ (Docker image building)
    └───┬──────┬───┘
        │      │
        │      │ build fails
        │      └──────────────┐
        │                     ▼
        │              ┌──────────┐
        │              │  failed  │ (terminal - build error)
        │              └──────────┘
        ▼
  ┌──────────────┐
  │   running    │ (container started, app responding)
  └──────┬───────┘
         │
         │ inactivity timeout (30 min)
         ▼
  ┌──────────────┐
  │  sleeping    │ (container stopped to save resources)
  └──────┬───────┘
         │
         │ new request received
         └──────────► (back to building → running)
```

**Valid Transitions**:
- pending → building (code review passes, build starts)
- building → running (build succeeds, container starts)
- building → failed (build fails)
- running → sleeping (inactivity timeout)
- sleeping → building (wakeup request, rebuilds container)

**Terminal States**: failed (all others are transient)

---

## Deployment Lifecycle Flow

### Frontend Deployment (GitHub Pages)

```
1. Developer pushes to 001-book-creation
   ↓
2. GitHub triggers deploy.yml workflow
   ↓
3. Workflow runs code-review.yml (blocking)
   ├─ ESLint checks JavaScript/TypeScript
   ├─ Prettier checks formatting
   └─ If pass → continue, else → block
   ↓
4. Workflow runs npm run build
   ├─ Docusaurus builds static site
   └─ Output: ./build directory
   ↓
5. Workflow deploys to gh-pages branch
   ├─ Uses peaceiris/actions-gh-pages@v4
   └─ Pushes ./build to gh-pages
   ↓
6. GitHub Pages serves site
   └─ Site live at https://hassaanghayas.github.io/...
```

**Duration**: 3-5 minutes (code review: 1-2 min, build: 1-2 min, deploy: 1 min)

---

### Backend Deployment (Hugging Face Spaces)

```
1. Developer pushes to 001-book-creation
   ↓
2. GitHub triggers huggingface-deploy.yml workflow
   ↓
3. Workflow runs code-review.yml (blocking)
   ├─ Ruff checks Python linting
   ├─ Black checks formatting
   └─ If pass → continue, else → block
   ↓
4. GitHub pushes to HF Space repository
   ├─ HF detects changes in backend/
   └─ Triggers automatic rebuild
   ↓
5. HF builds Docker image
   ├─ Installs dependencies from requirements.txt
   ├─ Loads environment variables from HF Secrets
   └─ Starts app.py (wrapper for api.py)
   ↓
6. HF Space becomes "Running"
   └─ API live at https://[space-name].hf.space
```

**Duration**: 6-10 minutes (code review: 1-2 min, push: 1 min, HF build: 4-7 min)

---

## Data Access Patterns

### Query: Get Latest Deployment Status

```sql
SELECT
  d.deployment_id,
  d.status,
  d.deployed_at,
  w.workflow_name,
  r.commit_sha
FROM GitHubPagesDeployment d
JOIN GitHubActionsWorkflow w ON d.run_id = w.run_id
JOIN GitHubRepository r ON w.repository_id = r.repository_id
WHERE r.name = 'physical-ai-robotics-textbook'
ORDER BY d.deployed_at DESC
LIMIT 1;
```

### Query: Get Code Review Reports for Commit

```sql
SELECT
  tool_name,
  status,
  issue_count,
  execution_time_ms
FROM CodeReviewReport
WHERE commit_sha = '088573c4a5d...'
ORDER BY created_at ASC;
```

### Query: Get Deployment Logs for Failed Deployment

```sql
SELECT
  timestamp,
  event_type,
  level,
  message,
  context
FROM DeploymentLog
WHERE deployment_id = 'gh-pages-123456789'
  AND level = 'error'
ORDER BY timestamp ASC;
```

---

## Validation Rules

### Pre-Deployment Validation

**Before Frontend Deployment**:
1. Code review reports for commit must all have status = "pass"
2. Branch must be "001-book-creation"
3. No active deployments for same commit (prevent duplicates)

**Before Backend Deployment**:
1. Code review reports for commit must all have status = "pass"
2. All required secrets must be configured in HF Spaces
3. Health check endpoint must be accessible after deployment

### Post-Deployment Validation

**After Frontend Deployment**:
1. site_url must return HTTP 200
2. HTML must contain expected content (title tag, meta tags)
3. No console errors in browser

**After Backend Deployment**:
1. health_check_url must return HTTP 200 with JSON response
2. /docs endpoint must return OpenAPI documentation
3. CORS headers must allow GitHub Pages origin

---

## Audit Requirements

**Deployment Logs Must Capture**:
- All state transitions (started, building, deployed, failed)
- User/actor who triggered deployment (github-actions[bot] or developer)
- Commit SHA and commit message
- Build duration and timestamps
- Error messages (if deployment failed)
- Rollback events (if deployment was reverted)

**Code Review Reports Must Capture**:
- Tool name and version (for reproducibility)
- All issues found (file path, line number, severity)
- Execution time (for performance monitoring)
- Commit SHA (for historical tracking)

**Retention Policy**:
- Deployment logs: 90 days
- Code review reports: 90 days
- Workflow runs: 90 days (GitHub default)

---

## Summary

This data model defines 7 core entities with clear relationships, state transitions, and validation rules. The model supports:

1. Automated deployments triggered by Git push
2. Code review as blocking gate before deployment
3. Separate frontend (GitHub Pages) and backend (HF Spaces) deployments
4. Comprehensive audit trail via DeploymentLog
5. Secure secrets management via EnvironmentSecret metadata

**Next Steps**: Create contracts (schemas) and quickstart guide based on this data model.

---

**Data Model Status**: Complete
**Date**: 2025-12-19
**Approved By**: DevOps Engineer Agent
