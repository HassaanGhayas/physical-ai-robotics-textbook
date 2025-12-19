"""
Configuration management for RAG Agent.
Loads and validates environment variables using Pydantic Settings.
"""
from pydantic_settings import BaseSettings
from pydantic import Field, validator
from typing import Optional
import logging

logger = logging.getLogger(__name__)


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Cohere Configuration
    cohere_api_key: str = Field(..., env="COHERE_API_KEY")
    cohere_model: str = Field(default="command-r-08-2024", env="COHERE_MODEL")

    # Qdrant Configuration
    qdrant_url: str = Field(..., env="QDRANT_URL")
    qdrant_api_key: str = Field(..., env="QDRANT_API_KEY")
    qdrant_collection_name: str = Field(default="rag_embedding", env="QDRANT_COLLECTION_NAME")

    # RAG Agent Configuration
    rag_agent_timeout: float = Field(default=5.0, env="RAG_AGENT_TIMEOUT")

    # Application Configuration
    top_k: int = Field(default=5, env="TOP_K")
    log_level: str = Field(default="INFO", env="LOG_LEVEL")

    # Retry and Circuit Breaker Configuration
    max_retries: int = Field(default=3, env="MAX_RETRIES")
    circuit_breaker_threshold: int = Field(default=5, env="CIRCUIT_BREAKER_THRESHOLD")

    class Config:
        env_file = ".env"
        case_sensitive = False
        extra = "ignore"  # Ignore extra fields from .env

    @validator("log_level")
    def validate_log_level(cls, v):
        """Validate log level is a valid logging level."""
        valid_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
        if v.upper() not in valid_levels:
            raise ValueError(f"log_level must be one of {valid_levels}")
        return v.upper()

    @validator("top_k")
    def validate_top_k(cls, v):
        """Validate top_k is within acceptable range."""
        if v < 1 or v > 10:
            raise ValueError("top_k must be between 1 and 10")
        return v

    @validator("rag_agent_timeout")
    def validate_timeout(cls, v):
        """Validate timeout is positive."""
        if v <= 0:
            raise ValueError("rag_agent_timeout must be positive")
        return v

    def configure_logging(self):
        """Configure logging based on log_level setting."""
        logging.basicConfig(
            level=getattr(logging, self.log_level),
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        logger.info(f"Logging configured at {self.log_level} level")

    def validate_rag_agent_config(self):
        """Validate that Cohere API key is set for RAG agent functionality."""
        if not self.cohere_api_key:
            logger.warning("COHERE_API_KEY not set - RAG agent functionality will not work")
            return False
        return True


# Global settings instance
settings = Settings()
settings.configure_logging()

logger.info("Configuration loaded successfully")

# Validate RAG agent configuration
if settings.validate_rag_agent_config():
    logger.info("RAG agent configuration validated - Cohere client ready")
else:
    logger.error("RAG agent cannot function without COHERE_API_KEY")
