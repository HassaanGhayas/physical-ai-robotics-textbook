"""
Hugging Face Spaces entrypoint for the Physical AI & Robotics RAG backend.

This file serves as the main entry point when deployed to Hugging Face Spaces.
It imports and runs the FastAPI application from api.py.
"""

import uvicorn
from api import app

if __name__ == "__main__":
    # Run the FastAPI application
    # Host 0.0.0.0 allows external connections (required for HF Spaces)
    # Port 7860 is the default for Hugging Face Spaces
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=7860,
        log_level="info"
    )
