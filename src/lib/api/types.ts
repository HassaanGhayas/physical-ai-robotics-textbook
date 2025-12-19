/**
 * API client types for RAG chatbot backend
 * Based on contracts/api-contract.yaml
 */

/**
 * Request payload for /ask endpoint
 */
export interface AskRequest {
  /** User's natural language question */
  query: string;

  /** Optional: number of sources (default: 5) */
  top_k?: number;

  /** Optional: client-generated request ID */
  request_id?: string;
}

/**
 * Backend source structure
 */
export interface BackendSource {
  /** Unique identifier for this source */
  id: string;

  /** Full text content of the document chunk */
  content: string;

  /** URL to the original document */
  url: string;

  /** Identifier for this specific chunk */
  chunk_id: string;

  /** Identifier for the parent document */
  document_id: string;

  /** Relevance score from vector similarity search (0.0 - 1.0) */
  similarity_score: number;

  /** Ranking position in the result set (1-based) */
  position: number;
}

/**
 * Response payload from /ask endpoint
 */
export interface AskResponse {
  /** AI-generated answer */
  answer: string;

  /** Retrieved document chunks */
  sources: BackendSource[];

  /** Original query (echoed back) */
  query: string;

  /** Number of sources returned */
  chunk_count: number;

  /** Backend processing time in milliseconds */
  response_time_ms: number;
}

/**
 * Error response from backend
 */
export interface APIError {
  /** Always true for errors */
  error: boolean;

  /** Error message */
  message: string;

  /** HTTP status code */
  status_code: number;

  /** Error type (e.g., 'ValidationError') */
  type: string;

  /** Optional error details */
  details?: any;
}

/**
 * Health status response
 */
export interface HealthStatus {
  /** Overall status */
  status: 'healthy' | 'unhealthy' | 'degraded';

  /** API version */
  version: string;

  /** Server timestamp */
  timestamp: string;

  /** Status of dependent services */
  services?: {
    cohere?: 'available' | 'unavailable';
    qdrant?: 'available' | 'unavailable';
  };
}

/**
 * Structured error type for API failures
 */
export interface ApiError {
  /** Error code (e.g., 'NETWORK_ERROR', 'TIMEOUT') */
  code: string;

  /** User-friendly error message */
  message: string;

  /** HTTP status code if available */
  statusCode?: number;

  /** Error type from backend if available */
  type?: string;

  /** Technical details for debugging */
  details?: any;

  /** Whether the operation can be retried */
  retryable: boolean;

  /** Original error object */
  originalError?: Error;
}

/**
 * Result type for API operations
 */
export type ApiResult<T> =
  | { success: true; data: T }
  | { success: false; error: ApiError };

/**
 * Configuration for the API client
 */
export interface ApiClientConfig {
  /** Base URL of the backend API */
  baseUrl: string;

  /** Request timeout in milliseconds (default: 10000) */
  timeout?: number;

  /** Maximum number of retry attempts (default: 3) */
  maxRetries?: number;

  /** Delay between retries in milliseconds (default: 1000) */
  retryDelay?: number;

  /** Whether to use exponential backoff for retries (default: true) */
  exponentialBackoff?: boolean;

  /** Additional headers to include in requests */
  headers?: Record<string, string>;
}

/**
 * Main API client interface
 */
export interface RagApiClient {
  /**
   * Submit a question to the RAG chatbot
   */
  ask(request: AskRequest): Promise<ApiResult<AskResponse>>;

  /**
   * Check the health status of the backend API
   */
  checkHealth(): Promise<ApiResult<HealthStatus>>;

  /**
   * Cancel an ongoing request
   */
  cancelRequest(requestId: string): void;

  /**
   * Update the client configuration
   */
  updateConfig(config: Partial<ApiClientConfig>): void;
}

/**
 * Error code constants
 */
export const ErrorCodes = {
  // Network errors
  NETWORK_ERROR: 'NETWORK_ERROR',
  TIMEOUT: 'TIMEOUT',
  CONNECTION_REFUSED: 'CONNECTION_REFUSED',

  // Client errors (4xx)
  VALIDATION_ERROR: 'VALIDATION_ERROR',
  BAD_REQUEST: 'BAD_REQUEST',

  // Server errors (5xx)
  SERVER_ERROR: 'SERVER_ERROR',
  SERVICE_UNAVAILABLE: 'SERVICE_UNAVAILABLE',

  // Application errors
  PARSE_ERROR: 'PARSE_ERROR',
  UNKNOWN_ERROR: 'UNKNOWN_ERROR',
} as const;

/**
 * Error messages for user display
 */
export const ErrorMessages = {
  [ErrorCodes.NETWORK_ERROR]: 'Unable to connect. Check your internet connection.',
  [ErrorCodes.TIMEOUT]: 'Request timed out. Try a simpler question.',
  [ErrorCodes.SERVICE_UNAVAILABLE]: 'The chatbot is temporarily unavailable. Try again in a moment.',
  [ErrorCodes.VALIDATION_ERROR]: 'Please enter a valid question.',
  [ErrorCodes.SERVER_ERROR]: 'Something went wrong. Please try again.',
  [ErrorCodes.CONNECTION_REFUSED]: 'Unable to connect to the chatbot service. Please try again in a moment.',
  [ErrorCodes.UNKNOWN_ERROR]: 'An unexpected error occurred. Please try again.',
} as const;
