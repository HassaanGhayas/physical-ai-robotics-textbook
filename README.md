# Website

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## Code Review & Quality

This project uses automated code review workflows to maintain code quality. All pull requests to the `001-book-creation` branch are automatically checked for:

### Frontend Quality Checks
- **ESLint**: Lints JavaScript/TypeScript code for errors and style issues
- **Prettier**: Checks code formatting consistency

### Backend Quality Checks
- **Ruff**: Fast Python linter for the FastAPI backend
- **Black**: Python code formatter

### Running Checks Locally

Before creating a pull request, run these commands to ensure your code passes all checks:

```bash
# Frontend checks
npm run lint        # Run ESLint
npm run format:check  # Check Prettier formatting

# Backend checks (requires Python 3.11+)
ruff check backend/    # Run Ruff linter
black --check backend/ # Check Black formatting
```

### Branch Protection

The `001-book-creation` branch is protected with required status checks. Pull requests cannot be merged unless all code review checks pass:
- ✓ lint-frontend
- ✓ format-check-frontend
- ✓ lint-backend
- ✓ format-check-backend

This ensures all merged code meets our quality standards.

## Secrets Management

This project uses environment variables to manage sensitive API keys and configuration. Secrets are required for both local development and production deployment.

### Local Development Setup

1. **Copy the environment file template**:
   ```bash
   # Backend secrets
   cp backend/.env.example backend/.env
   ```

2. **Fill in your API keys** in `backend/.env`:
   - `COHERE_API_KEY`: Get from https://cohere.com/ (Dashboard → API Keys)
   - `QDRANT_API_KEY` and `QDRANT_URL`: Get from https://cloud.qdrant.io/ (Create cluster → Settings → API Keys)

3. **Verify `.env` is gitignored**:
   - The `.env` file is already in `.gitignore` and will never be committed
   - Only `.env.example` with placeholder values should be committed

### GitHub Actions Secrets

For automated deployments and CI/CD workflows, configure secrets in GitHub:

1. **Navigate to repository settings**:
   ```
   Repository → Settings → Secrets and variables → Actions
   ```

2. **Add the following repository secrets**:
   - `COHERE_API_KEY`: Your Cohere API key for RAG agent
   - `QDRANT_API_KEY`: Your Qdrant API key for vector storage
   - `QDRANT_URL`: Your Qdrant cluster URL (format: https://xxx.cloud.qdrant.io)
   - `HF_TOKEN`: (Optional) Hugging Face token for automated backend deployment

3. **Verify secrets are accessible**:
   - GitHub Actions workflows can access these via `${{ secrets.SECRET_NAME }}`
   - Secrets are never exposed in logs or pull request comments

### Hugging Face Spaces Secrets

For backend deployment on Hugging Face Spaces:

1. **Navigate to your Space settings**:
   ```
   Hugging Face Space → Settings → Variables
   ```

2. **Add environment variables**:
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_API_KEY`: Your Qdrant API key
   - `QDRANT_URL`: Your Qdrant cluster URL
   - `COHERE_MODEL`: command-r-08-2024
   - `QDRANT_COLLECTION_NAME`: rag_embedding
   - `TARGET_DOCS_URL`: https://hassaanghayas.github.io/physical-ai-robotics-textbook/
   - `TOP_K`: 5
   - `LOG_LEVEL`: INFO

3. **Variables are automatically loaded** by the backend when the Space starts

### Secret Rotation

For security best practices, rotate your API keys regularly:

**Recommended Rotation Schedule:**
- **Development keys**: Every 90 days
- **Production keys**: Every 60 days
- **After security incident**: Immediately

**Rotation Procedure:**

1. **Generate new API key** in the respective service (Cohere/Qdrant)
2. **Update all locations**:
   - Local `.env` file (development)
   - GitHub Secrets (CI/CD)
   - Hugging Face Space Variables (production backend)
3. **Test the new key** in development first
4. **Deploy to production** after verification
5. **Revoke the old key** in the service dashboard
6. **Document the rotation** in team changelog

**Emergency Rotation (if key is compromised):**
1. Immediately revoke compromised key in service dashboard
2. Generate new key and update all locations simultaneously
3. Monitor service logs for unauthorized access attempts
4. Review recent API usage for anomalies

### Security Best Practices

- ✅ Never commit `.env` files or actual secrets to git
- ✅ Use different API keys for development and production
- ✅ Regularly audit who has access to secrets
- ✅ Monitor API usage for unexpected patterns
- ✅ Enable API key restrictions when available (IP allowlists, usage limits)
- ✅ Use read-only keys when write access is not needed
- ❌ Never share API keys in Slack, email, or screenshots
- ❌ Never log secret values, even partially
- ❌ Never hardcode secrets in source code
