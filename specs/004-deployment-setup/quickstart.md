# Quickstart Guide: Deployment Setup

**Feature**: 004-deployment-setup
**Date**: 2025-12-19
**Audience**: Developers setting up deployment infrastructure

## Overview

This guide walks you through setting up automated deployment for the Physical AI & Humanoid Robotics textbook project. After completing this guide, you'll have:

- Automated frontend deployment to GitHub Pages (triggered on push)
- Automated backend deployment to Hugging Face Spaces (triggered on push)
- Code review checks that run before deployment (blocking gate)
- Secure secrets management for API keys and credentials

**Time Required**: 30-45 minutes (one-time setup)

---

## Prerequisites

Before starting, ensure you have:

### Required Accounts
- [ ] GitHub account with access to repository: https://github.com/HassaanGhayas/physical-ai-robotics-textbook
- [ ] Hugging Face account (free tier): https://huggingface.co/join
- [ ] Cohere API key: https://dashboard.cohere.com/api-keys
- [ ] Qdrant Cloud account: https://cloud.qdrant.io/

### Required Software (Local Development)
- [ ] Git 2.30+
- [ ] Node.js 20.x or later
- [ ] Python 3.11 or later
- [ ] npm 10.x or later (comes with Node.js)
- [ ] pip 23.x or later (comes with Python)

### Repository Access
- [ ] Clone the repository locally:
  ```bash
  git clone https://github.com/HassaanGhayas/physical-ai-robotics-textbook.git
  cd physical-ai-robotics-textbook
  ```

---

## Step 1: Local Development Setup

### 1.1 Frontend Setup

```bash
# From repository root
npm install

# Copy environment example (frontend doesn't need secrets for local dev)
# Frontend secrets (if any) will be added later

# Start local development server
npm start

# Open browser to http://localhost:3000
# Verify site loads correctly
```

**Expected Output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at:
  http://localhost:3000/
```

### 1.2 Backend Setup

```bash
# Navigate to backend directory
cd backend

# Create Python virtual environment
python -m venv .venv

# Activate virtual environment
# Windows:
.venv\Scripts\activate
# Linux/Mac:
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt
# OR use UV (faster):
uv pip install -r requirements.txt

# Copy environment variables template
cp .env.example .env
```

### 1.3 Configure Backend Secrets (Local Development)

Edit `backend/.env` and add your API keys:

```bash
# backend/.env (DO NOT COMMIT THIS FILE)

# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here
COHERE_MODEL=command-r-08-2024

# Qdrant Configuration
QDRANT_URL=https://your-cluster-url.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=rag_embedding

# Application Configuration
TARGET_DOCS_URL=https://hassaanghayas.github.io/physical-ai-robotics-textbook/
TOP_K=5
LOG_LEVEL=INFO

# RAG Agent Configuration
RAG_AGENT_TIMEOUT=30.0
MAX_RETRIES=3
CIRCUIT_BREAKER_THRESHOLD=5
```

**Where to Get API Keys**:
- **Cohere API Key**: https://dashboard.cohere.com/api-keys (free tier: 1000 API calls/month)
- **Qdrant URL & API Key**: https://cloud.qdrant.io/ → Create cluster → Copy URL and API key

### 1.4 Start Backend Server

```bash
# From backend/ directory (with .venv activated)
python api.py

# OR use uvicorn directly:
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

**Expected Output**:
```
INFO:     Started server process [12345]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

**Test Backend**:
```bash
# Health check
curl http://localhost:8000/health

# Expected response:
{
  "status": "healthy",
  "services": {
    "cohere": "available",
    "qdrant": "available"
  },
  "timestamp": "2025-12-19T10:00:00Z",
  "version": "1.0.0"
}
```

---

## Step 2: GitHub Repository Configuration

### 2.1 Verify .gitignore

Ensure `.gitignore` includes these patterns (already configured):

```gitignore
# Environment variables
.env
.env.local
.env.*.local
backend/.env

# Allow examples
!.env.example
!backend/.env.example
```

**Verify**:
```bash
# This should show only .env.example (never .env)
git status backend/
```

### 2.2 Configure GitHub Secrets

1. Go to repository settings: https://github.com/HassaanGhayas/physical-ai-robotics-textbook/settings/secrets/actions

2. Click "New repository secret" and add the following secrets:

| Secret Name | Value | Used By |
|-------------|-------|---------|
| `COHERE_API_KEY` | Your Cohere API key | Backend deployment |
| `QDRANT_API_KEY` | Your Qdrant API key | Backend deployment |
| `QDRANT_URL` | Your Qdrant cluster URL | Backend deployment |
| `HF_TOKEN` | Hugging Face access token (optional) | Backend deployment (manual push) |

**How to Get HF_TOKEN** (Optional):
1. Go to https://huggingface.co/settings/tokens
2. Click "New token"
3. Name: "GitHub Actions Deployment"
4. Type: "Write"
5. Copy token and add to GitHub Secrets

**Note**: `GITHUB_TOKEN` is auto-provided by GitHub Actions (no need to add manually).

### 2.3 Enable GitHub Pages

1. Go to repository settings: https://github.com/HassaanGhayas/physical-ai-robotics-textbook/settings/pages

2. Configure:
   - **Source**: Deploy from a branch
   - **Branch**: `gh-pages` (will be created automatically by workflow)
   - **Folder**: `/ (root)`

3. Click "Save"

**Expected**: Site will be published at https://hassaanghayas.github.io/physical-ai-robotics-textbook/

---

## Step 3: Hugging Face Spaces Setup

### 3.1 Create Hugging Face Space

1. Go to https://huggingface.co/new-space

2. Configure Space:
   - **Owner**: Your username (e.g., `hassaanghayas`)
   - **Space name**: `physical-ai-robotics-backend`
   - **License**: MIT
   - **Visibility**: Public (or Private if preferred)
   - **SDK**: Docker
   - **Hardware**: CPU basic (free tier)

3. Click "Create Space"

### 3.2 Link Space to GitHub Repository

**Option 1: Automatic Sync (Recommended)**

1. In your HF Space, click "Settings" tab
2. Scroll to "Repository"
3. Click "Link to GitHub repository"
4. Select: `HassaanGhayas/physical-ai-robotics-textbook`
5. Path: `backend/` (only sync backend directory)
6. Branch: `001-book-creation`
7. Click "Link repository"

**Option 2: Manual Push (Alternative)**

```bash
# Add Hugging Face remote
git remote add hf https://huggingface.co/spaces/hassaanghayas/physical-ai-robotics-backend

# Push backend to HF (create subtree)
git subtree push --prefix=backend hf main
```

### 3.3 Configure Hugging Face Secrets

1. In your HF Space, click "Settings" tab
2. Scroll to "Repository secrets"
3. Click "Add a secret" for each:

| Secret Name | Value |
|-------------|-------|
| `COHERE_API_KEY` | Your Cohere API key |
| `QDRANT_API_KEY` | Your Qdrant API key |
| `QDRANT_URL` | Your Qdrant cluster URL |

**Important**: Use the same values as in GitHub Secrets for consistency.

### 3.4 Create HF Space Entrypoint

Create `backend/app.py` (if it doesn't exist):

```python
"""
Hugging Face Spaces entrypoint.
Wrapper for api.py to work with HF's container environment.
"""
import os
from api import app

if __name__ == "__main__":
    import uvicorn

    # HF Spaces expects app to run on port 7860
    port = int(os.getenv("PORT", 7860))

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=port,
        log_level="info"
    )
```

### 3.5 Create requirements.txt

Generate `backend/requirements.txt` from `pyproject.toml`:

```bash
cd backend

# Option 1: Manual extraction (if you have pyproject.toml)
# Copy dependencies list from pyproject.toml [project.dependencies]

# Create requirements.txt
cat > requirements.txt << 'EOF'
fastapi>=0.104.0
uvicorn>=0.24.0
cohere>=5.0.0
qdrant-client>=1.10.0
pydantic>=2.0.0
pydantic-settings>=2.0.0
python-dotenv>=1.0.0
httpx>=0.25.0
tenacity>=8.0.0
pybreaker>=1.0.0
EOF
```

**Verify**:
```bash
pip install -r requirements.txt
# Should install all dependencies without errors
```

---

## Step 4: Update GitHub Actions Workflows

### 4.1 Update Frontend Deployment Workflow

Edit `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [001-book-creation]  # Changed from 'main' to '001-book-creation'
    paths:
      - 'src/**'
      - 'docs/**'
      - 'static/**'
      - 'docusaurus.config.ts'
      - 'package.json'
      - 'package-lock.json'
  workflow_dispatch:

permissions:
  contents: write

jobs:
  deploy:
    name: Build and Deploy Frontend
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-node@v4
        with:
          node-version: 22
          cache: npm

      - name: Install dependencies
        run: npm ci

      - name: Build website
        run: npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v4
        if: github.ref == 'refs/heads/001-book-creation'
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
          user_name: github-actions[bot]
          user_email: 41898282+github-actions[bot]@users.noreply.github.com
```

### 4.2 Create Code Review Workflow

Create `.github/workflows/code-review.yml`:

```yaml
name: Code Review

on:
  pull_request:
    branches: [001-book-creation]
  push:
    branches: [001-book-creation]

jobs:
  lint-frontend:
    name: Lint Frontend Code
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-node@v4
        with:
          node-version: 20
          cache: npm

      - name: Install dependencies
        run: npm ci

      - name: Run ESLint
        run: npm run lint
        continue-on-error: false

      - name: Check formatting with Prettier
        run: npm run format:check
        continue-on-error: false

  lint-backend:
    name: Lint Backend Code
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-python@v4
        with:
          python-version: '3.11'

      - name: Install linting tools
        run: pip install ruff black

      - name: Run Ruff
        run: |
          cd backend
          ruff check . --output-format=github
        continue-on-error: false

      - name: Check formatting with Black
        run: |
          cd backend
          black --check --verbose .
        continue-on-error: false
```

### 4.3 Create Backend Deployment Workflow (Optional)

Create `.github/workflows/huggingface-deploy.yml`:

```yaml
name: Deploy to Hugging Face Spaces

on:
  push:
    branches: [001-book-creation]
    paths:
      - 'backend/**'
  workflow_dispatch:

jobs:
  deploy:
    name: Deploy Backend to HF
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      - name: Notify HF Space
        run: |
          echo "Hugging Face Space will auto-sync from GitHub"
          echo "No manual push needed with automatic sync enabled"

      # Alternative: Manual push to HF (if not using auto-sync)
      # - name: Push to HF Space
      #   run: |
      #     git remote add hf https://huggingface.co/spaces/${{ secrets.HF_USERNAME }}/${{ secrets.HF_SPACE_NAME }}
      #     git subtree push --prefix=backend hf main
```

---

## Step 5: Test Deployment Flow

### 5.1 Test Frontend Deployment

```bash
# Make a small change to documentation
echo "# Test" >> docs/test.md

# Commit and push
git add docs/test.md
git commit -m "test: verify deployment workflow"
git push origin 001-book-creation

# Monitor GitHub Actions
# Go to: https://github.com/HassaanGhayas/physical-ai-robotics-textbook/actions

# Expected:
# - "Code Review" workflow runs and passes
# - "Deploy to GitHub Pages" workflow runs and deploys
# - Site updates at https://hassaanghayas.github.io/physical-ai-robotics-textbook/
```

**Verification**:
1. GitHub Actions shows green checkmark
2. Site is accessible at GitHub Pages URL
3. Changes appear in live site (may take 1-2 minutes)

### 5.2 Test Backend Deployment

```bash
# Make a small change to backend
cd backend
echo "# Version 1.0.1" >> README.md

# Commit and push
git add README.md
git commit -m "test: verify backend deployment"
git push origin 001-book-creation

# Monitor Hugging Face Space
# Go to: https://huggingface.co/spaces/hassaanghayas/physical-ai-robotics-backend

# Expected:
# - HF detects GitHub push (if auto-sync enabled)
# - HF rebuilds Docker image (takes 5-7 minutes)
# - Space status changes: Building → Running
```

**Verification**:
```bash
# Test backend health endpoint
curl https://hassaanghayas-physical-ai-robotics-backend.hf.space/health

# Expected response:
{
  "status": "healthy",
  "services": {
    "cohere": "available",
    "qdrant": "available"
  },
  "timestamp": "2025-12-19T12:00:00Z",
  "version": "1.0.0"
}
```

### 5.3 Test Code Review (Failure Case)

```bash
# Introduce a linting error
cd src/pages
echo "const test = 'no semicolon'" >> index.tsx

# Commit and push
git add index.tsx
git commit -m "test: verify code review blocks deployment"
git push origin 001-book-creation

# Expected:
# - "Code Review" workflow runs
# - ESLint detects missing semicolon
# - Workflow fails (red X)
# - Deployment does NOT proceed

# Fix the error
# Remove the test line from index.tsx
git add index.tsx
git commit -m "fix: remove test line"
git push origin 001-book-creation

# Expected:
# - Code review passes
# - Deployment proceeds
```

---

## Step 6: Configure CORS (Backend)

Update `backend/api.py` to allow GitHub Pages origin:

```python
# backend/api.py

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local development
        "https://hassaanghayas.github.io"  # Production GitHub Pages
    ],
    allow_credentials=False,
    allow_methods=["POST", "GET", "OPTIONS"],
    allow_headers=["Content-Type"],
    max_age=600,
)
```

Commit and push:
```bash
cd backend
git add api.py
git commit -m "fix: update CORS for production domain"
git push origin 001-book-creation
```

---

## Step 7: Monitoring and Maintenance

### 7.1 Monitor GitHub Actions

**View Workflow Runs**:
- https://github.com/HassaanGhayas/physical-ai-robotics-textbook/actions

**View Specific Run**:
- Click on a workflow run → View logs for each job

**Enable Email Notifications**:
- GitHub automatically sends emails on workflow failure
- Settings → Notifications → Actions (enable)

### 7.2 Monitor Hugging Face Space

**View Space Logs**:
- https://huggingface.co/spaces/hassaanghayas/physical-ai-robotics-backend
- Click "Logs" tab → View build and runtime logs

**Check Space Status**:
- "Running" = healthy (green indicator)
- "Building" = deployment in progress (yellow indicator)
- "Paused" = sleeping due to inactivity (gray indicator)
- "Error" = deployment failed (red indicator)

**Wake Sleeping Space**:
- Visit Space URL: https://hassaanghayas-physical-ai-robotics-backend.hf.space
- Space automatically wakes up (takes 30-60 seconds)

### 7.3 View Deployment Logs

**Frontend Deployment Logs**:
```bash
# View latest deployment
gh run list --workflow=deploy.yml --limit 1

# View specific run logs
gh run view <run-id> --log
```

**Backend Deployment Logs**:
- HF Spaces UI → Logs tab → View build and runtime logs

---

## Troubleshooting

### Issue: GitHub Pages deployment fails with "Build failed"

**Symptoms**: Workflow fails at "Build website" step

**Solution**:
```bash
# Run build locally to see errors
npm run build

# Common issues:
# - Missing dependencies: npm install
# - TypeScript errors: npm run typecheck
# - Broken links: Check console output
```

### Issue: Backend fails with "Missing secret COHERE_API_KEY"

**Symptoms**: HF Space shows "Error" status, logs show "field required"

**Solution**:
1. Go to HF Space → Settings → Repository secrets
2. Verify `COHERE_API_KEY` is set
3. Check spelling (case-sensitive)
4. Restart Space: Settings → Factory reboot

### Issue: CORS error in browser console

**Symptoms**: Frontend can't call backend, "blocked by CORS policy"

**Solution**:
1. Check `backend/api.py` CORS configuration
2. Ensure GitHub Pages domain is in `allow_origins`
3. Use exact URL (no trailing slash mismatch)
4. Redeploy backend after fixing

### Issue: Code review fails on valid code

**Symptoms**: ESLint/Ruff reports false positives

**Solution**:
```bash
# Frontend: Check ESLint config
cat .eslintrc.json

# Backend: Check Ruff config
cat backend/pyproject.toml

# Disable specific rules if needed:
# .eslintrc.json: "rules": { "rule-name": "off" }
# pyproject.toml: [tool.ruff] ignore = ["E501"]
```

### Issue: Deployment is slow (> 10 minutes)

**Symptoms**: Workflow takes longer than expected

**Solution**:
1. Check GitHub Actions status: https://www.githubstatus.com/
2. Optimize build:
   - Enable npm caching (already enabled)
   - Reduce dependencies (if possible)
3. HF Space: Upgrade to paid tier for faster builds

---

## Next Steps

Now that deployment is set up:

1. Run `/sp.tasks` to generate atomic implementation tasks
2. Implement tasks one-by-one with testing
3. Create rollback procedures documentation
4. Set up monitoring dashboards (optional)
5. Document deployment best practices for team

---

## Summary Checklist

Verify all steps are complete:

- [ ] Local frontend runs: `npm start`
- [ ] Local backend runs: `python api.py`
- [ ] GitHub Secrets configured (COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL)
- [ ] GitHub Pages enabled (gh-pages branch)
- [ ] HF Space created and linked to GitHub
- [ ] HF Secrets configured (same as GitHub Secrets)
- [ ] GitHub Actions workflows updated (.github/workflows/)
- [ ] Code review workflow created (code-review.yml)
- [ ] CORS configured for production domain
- [ ] Test deployment successful (frontend + backend)
- [ ] Code review blocking tested (intentional failure)
- [ ] Monitoring dashboards bookmarked

**Total Setup Time**: 30-45 minutes

**Deployment Frequency**: Every push to 001-book-creation branch

**Deployment Duration**:
- Frontend: 3-5 minutes
- Backend: 6-10 minutes

---

**Quickstart Status**: Complete
**Date**: 2025-12-19
**Next**: Run `/sp.tasks` to generate implementation tasks
