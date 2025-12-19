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
