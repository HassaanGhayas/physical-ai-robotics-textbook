/**
 * Validation utilities for API requests and responses
 */

import type { AskRequest, AskResponse, BackendSource } from './types';

/**
 * Validation result
 */
export interface ValidationResult {
  isValid: boolean;
  errors: string[];
}

/**
 * Validate an AskRequest before sending
 */
export function validateAskRequest(request: AskRequest): ValidationResult {
  const errors: string[] = [];

  if (!request.query || request.query.trim().length === 0) {
    errors.push('Query cannot be empty');
  }

  if (request.query && request.query.length > 1000) {
    errors.push('Query exceeds maximum length of 1000 characters');
  }

  if (request.top_k !== undefined) {
    if (!Number.isInteger(request.top_k) || request.top_k < 1 || request.top_k > 20) {
      errors.push('top_k must be an integer between 1 and 20');
    }
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Validate an AskResponse from the backend
 */
export function validateAskResponse(response: AskResponse): ValidationResult {
  const errors: string[] = [];

  if (!response.answer || response.answer.trim().length === 0) {
    errors.push('Answer cannot be empty');
  }

  if (!Array.isArray(response.sources)) {
    errors.push('Sources must be an array');
  }

  if (response.chunk_count !== response.sources.length) {
    errors.push('chunk_count does not match sources array length');
  }

  response.sources.forEach((source, index) => {
    if (source.similarity_score < 0 || source.similarity_score > 1) {
      errors.push(`Source ${index}: similarity_score must be between 0 and 1`);
    }

    if (source.position < 1) {
      errors.push(`Source ${index}: position must be a positive integer`);
    }

    if (!source.url || !isValidUrl(source.url)) {
      errors.push(`Source ${index}: invalid URL`);
    }
  });

  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Validate a URL format
 */
export function isValidUrl(url: string): boolean {
  try {
    new URL(url);
    return true;
  } catch {
    return false;
  }
}

/**
 * Validate a source object
 */
export function validateSource(source: BackendSource): ValidationResult {
  const errors: string[] = [];

  if (!source.id) {
    errors.push('Source ID cannot be empty');
  }

  if (!source.content) {
    errors.push('Source content cannot be empty');
  }

  if (!source.url || !isValidUrl(source.url)) {
    errors.push('Invalid source URL');
  }

  if (!source.chunk_id) {
    errors.push('Chunk ID cannot be empty');
  }

  if (!source.document_id) {
    errors.push('Document ID cannot be empty');
  }

  if (source.similarity_score < 0 || source.similarity_score > 1) {
    errors.push('Similarity score must be between 0.0 and 1.0');
  }

  if (source.position < 1) {
    errors.push('Position must be a positive integer');
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}
