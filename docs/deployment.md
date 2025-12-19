# Deployment Guide

Complete guide for deploying the Physical AI & Humanoid Robotics textbook project.

## Overview

This project uses a two-part deployment strategy:
- **Frontend**: Docusaurus site deployed to GitHub Pages
- **Backend**: FastAPI RAG agent deployed to Hugging Face Spaces

## Prerequisites

- GitHub account with repository access
- Hugging Face account (for backend deployment)
- API keys:
  - Cohere API key (for embeddings)
  - Qdrant API key and URL (for vector storage)

## Frontend Deployment (GitHub Pages)

### Automatic Deployment

Changes pushed to the `001-book-creation` branch automatically deploy to GitHub Pages.

**Deployment Workflow**:
1. Push changes to `001-book-creation` branch
2. GitHub Actions builds the Docusaurus site
3. Site deploys to `gh-pages` branch
4. Live within 5 minutes at: https://hassaanghayas.github.io/physical-ai-robotics-textbook/

### Workflow Configuration

The deployment workflow (`.github/workflows/deploy.yml`) includes:
- **Trigger**: Push to `001-book-creation` branch
- **Path filtering**: Only triggers on frontend file changes
  - `src/**`
  - `docs/**`
  - `static/**`
  - `docusaurus.config.ts`
  - `package.json`
- **Concurrency control**: Prevents overlapping deployments
- **Build time**: 3-5 minutes

### Manual Deployment

If needed, you can deploy manually:

```bash
npm run build
GIT_USER=<Your GitHub username> npm run deploy
```

## Backend Deployment (Hugging Face Spaces)

### Step 1: Create Hugging Face Space

1. Go to https://huggingface.co/new-space
2. Configure Space:
   - **Name**: `physical-ai-robotics-backend`
   - **SDK**: Docker (or Gradio if using app.py)
   - **Visibility**: Public
   - **Hardware**: CPU Basic (free tier)

### Step 2: Link to GitHub Repository

1. In Space Settings → Repository
2. Click "Link to GitHub"
3. Select repository: `HassaanGhayas/physical-ai-robotics-textbook`
4. Set sync directory: `backend/`
5. Enable automatic sync

### Step 3: Configure Environment Variables

In Space Settings → Variables, add:

| Variable | Description | Example |
|----------|-------------|---------|
| `COHERE_API_KEY` | Your Cohere API key | `co-xxxxx...` |
| `QDRANT_API_KEY` | Your Qdrant API key | `xxxxx...` |
| `QDRANT_URL` | Your Qdrant cluster URL | `https://xxx.cloud.qdrant.io` |
| `COHERE_MODEL` | Model name | `command-r-08-2024` |
| `QDRANT_COLLECTION_NAME` | Collection name | `rag_embedding` |
| `TARGET_DOCS_URL` | Docs URL | `https://hassaanghayas.github.io/physical-ai-robotics-textbook/` |
| `TOP_K` | Number of results | `5` |
| `LOG_LEVEL` | Logging level | `INFO` |

### Step 4: Deploy

1. Push changes to `001-book-creation` branch
2. Hugging Face will automatically build the Space
3. Build time: 5-10 minutes
4. Monitor build logs in Space UI

### Step 5: Verify Deployment

Test the health endpoint:

```bash
curl https://physical-ai-robotics-backend.hf.space/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-19T10:30:00Z",
  "services": {
    "cohere": "operational",
    "qdrant": "operational"
  }
}
```

Test a query:

```bash
curl -X POST "https://physical-ai-robotics-backend.hf.space/ask" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is physical AI?",
    "top_k": 5,
    "include_sources": true
  }'
```

## Code Review Workflow

All pull requests to `001-book-creation` automatically run code quality checks.

### Quality Checks

**Frontend**:
- ESLint: JavaScript/TypeScript linting
- Prettier: Code formatting

**Backend**:
- Ruff: Python linting
- Black: Python formatting

### Running Checks Locally

Before creating a PR:

```bash
# Frontend
npm run lint
npm run format:check

# Backend
ruff check backend/
black --check backend/
```

### Branch Protection

The `001-book-creation` branch requires all checks to pass before merge:
- ✓ lint-frontend
- ✓ format-check-frontend
- ✓ lint-backend
- ✓ format-check-backend

## Secrets Management

### Local Development

1. Copy `.env.example` files:
   ```bash
   cp .env.example .env
   cp backend/.env.example backend/.env
   ```

2. Fill in your API keys (never commit `.env` files)

### GitHub Actions

Secrets are configured in repository settings:
1. Go to Settings → Secrets and variables → Actions
2. Add secrets:
   - `COHERE_API_KEY`
   - `QDRANT_API_KEY`
   - `QDRANT_URL`
   - `HF_TOKEN` (optional, for automated HF deployment)

### Hugging Face Spaces

Secrets are configured in Space settings (see Backend Deployment above).

## Monitoring & Troubleshooting

### Frontend Issues

**Deployment Failed**:
- Check GitHub Actions logs
- Verify Node.js version (should be 20)
- Ensure all dependencies are in package.json

**Site Not Updating**:
- Check if workflow triggered (Actions tab)
- Wait 5 minutes after push
- Clear browser cache

### Backend Issues

**Space Not Running**:
- Check HF Space logs
- Verify all environment variables are set
- Check requirements.txt for missing dependencies

**API Errors**:
- Test health endpoint first
- Check HF Space logs for errors
- Verify secrets are correct
- Ensure Qdrant collection has data

**Slow Response Times**:
- Check individual timing metrics in response
- Monitor Cohere and Qdrant API latency
- Consider upgrading HF Space hardware

### Health Check Endpoints

**Frontend**: https://hassaanghayas.github.io/physical-ai-robotics-textbook/

**Backend**: https://physical-ai-robotics-backend.hf.space/health

## Rollback Procedures

### Frontend Rollback

1. Find last working commit:
   ```bash
   git log --oneline 001-book-creation
   ```

2. Revert to that commit:
   ```bash
   git revert <commit-hash>
   git push origin 001-book-creation
   ```

3. Deployment triggers automatically

### Backend Rollback

1. In HF Space Settings → Repository
2. Select specific commit or branch
3. Force rebuild
4. Wait for deployment (5-10 minutes)

## Performance Targets

| Metric | Target | Current |
|--------|--------|---------|
| Frontend deployment time | < 5 min | 3-4 min |
| Backend deployment time | < 10 min | 6-8 min |
| Backend response time (p95) | < 5 sec | 3-4 sec |
| Frontend uptime | 99.9% | Monitored by GitHub |
| Backend uptime | 99% | Monitored by HF |

## CI/CD Pipeline Overview

```
Push to 001-book-creation
         │
         ├─→ Frontend Changes
         │   ├─→ Code Review (ESLint, Prettier)
         │   ├─→ Build Docusaurus
         │   └─→ Deploy to GitHub Pages
         │
         └─→ Backend Changes
             ├─→ Code Review (Ruff, Black)
             ├─→ Sync to HF Space
             ├─→ Build Docker/Python
             └─→ Deploy to HF Spaces
```

## Related Documentation

- **Backend API**: See `backend/README.md`
- **Code Quality**: See main `README.md`
- **Specifications**: See `specs/004-deployment-setup/`
- **Architecture Decisions**: See `history/adr/`

## Support

For deployment issues:
1. Check GitHub Actions logs
2. Check Hugging Face Space logs
3. Review this troubleshooting guide
4. Create GitHub issue with logs
