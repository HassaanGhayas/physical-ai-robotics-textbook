/**
 * Frontend API Client Contract
 *
 * This file defines the TypeScript interface for the API client that
 * the frontend will use to communicate with the RAG chatbot backend.
 *
 * Implementation will use axios for HTTP requests with proper error handling,
 * retries, and timeout configuration.
 */

import type {
  AskRequest,
  AskResponse,
  APIError,
} from './types';

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
 * Result type for API operations
 */
export type ApiResult<T> =
  | { success: true; data: T }
  | { success: false; error: ApiError };

/**
 * Structured error type for API failures
 */
export interface ApiError {
  /** Error code (e.g., 'NETWORK_ERROR', 'TIMEOUT', 'SERVER_ERROR') */
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
 * Main API client interface
 *
 * All methods return ApiResult to provide consistent error handling.
 * Callers can check result.success to determine if the operation succeeded.
 */
export interface RagApiClient {
  /**
   * Submit a question to the RAG chatbot
   *
   * @param request - The question and optional parameters
   * @returns Promise resolving to ApiResult with AskResponse or ApiError
   *
   * @example
   * ```typescript
   * const result = await client.ask({
   *   query: "What is physical AI?",
   *   top_k: 5
   * });
   *
   * if (result.success) {
   *   console.log(result.data.answer);
   *   console.log(result.data.sources);
   * } else {
   *   console.error(result.error.message);
   * }
   * ```
   */
  ask(request: AskRequest): Promise<ApiResult<AskResponse>>;

  /**
   * Check the health status of the backend API
   *
   * @returns Promise resolving to ApiResult with health status
   *
   * @example
   * ```typescript
   * const result = await client.checkHealth();
   * if (result.success) {
   *   console.log('API is healthy:', result.data.status);
   * }
   * ```
   */
  checkHealth(): Promise<ApiResult<HealthStatus>>;

  /**
   * Cancel an ongoing request
   *
   * @param requestId - The ID of the request to cancel
   *
   * @example
   * ```typescript
   * const requestId = crypto.randomUUID();
   * const promise = client.ask({ query: "...", request_id: requestId });
   *
   * // Cancel if user navigates away
   * client.cancelRequest(requestId);
   * ```
   */
  cancelRequest(requestId: string): void;

  /**
   * Update the client configuration
   *
   * @param config - Partial configuration to merge with existing config
   *
   * @example
   * ```typescript
   * client.updateConfig({ timeout: 15000 });
   * ```
   */
  updateConfig(config: Partial<ApiClientConfig>): void;
}

/**
 * Health status response from /health endpoint
 */
export interface HealthStatus {
  /** Overall status: healthy, unhealthy, or degraded */
  status: 'healthy' | 'unhealthy' | 'degraded';

  /** API version */
  version: string;

  /** Server timestamp */
  timestamp: string;

  /** Status of dependent services */
  services?: {
    cohere?: 'available' | 'unavailable';
    qdrant?: 'available' | 'unavailable';
    openai?: 'available' | 'unavailable';
  };
}

/**
 * Factory function to create an API client instance
 *
 * @param config - Configuration for the client
 * @returns Configured API client instance
 *
 * @example
 * ```typescript
 * const client = createApiClient({
 *   baseUrl: 'https://api.example.com',
 *   timeout: 10000,
 *   maxRetries: 3,
 * });
 * ```
 */
export function createApiClient(config: ApiClientConfig): RagApiClient;

/**
 * Error code constants for consistent error handling
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
 * Type guard to check if an error is retryable
 *
 * @param error - The error to check
 * @returns True if the error is retryable
 */
export function isRetryableError(error: ApiError): boolean;

/**
 * Type guard to check if an error is a network error
 *
 * @param error - The error to check
 * @returns True if the error is a network error
 */
export function isNetworkError(error: ApiError): boolean;

/**
 * Type guard to check if an error is a timeout error
 *
 * @param error - The error to check
 * @returns True if the error is a timeout
 */
export function isTimeoutError(error: ApiError): boolean;

/**
 * Type guard to check if an error is a validation error
 *
 * @param error - The error to check
 * @returns True if the error is a validation error
 */
export function isValidationError(error: ApiError): boolean;

/**
 * Utility to create a user-friendly error message from an ApiError
 *
 * @param error - The API error
 * @returns User-friendly error message
 *
 * @example
 * ```typescript
 * const result = await client.ask(request);
 * if (!result.success) {
 *   toast.error(getUserFriendlyMessage(result.error));
 * }
 * ```
 */
export function getUserFriendlyMessage(error: ApiError): string;

/**
 * Request interceptor type for modifying requests before sending
 *
 * Can be used for adding authentication tokens, logging, etc.
 */
export type RequestInterceptor = (
  config: RequestConfig
) => RequestConfig | Promise<RequestConfig>;

/**
 * Response interceptor type for processing responses
 *
 * Can be used for logging, analytics, error transformation, etc.
 */
export type ResponseInterceptor = (
  response: any
) => any | Promise<any>;

/**
 * Internal request configuration (used by interceptors)
 */
export interface RequestConfig {
  url: string;
  method: 'GET' | 'POST' | 'PUT' | 'DELETE';
  headers: Record<string, string>;
  data?: any;
  params?: Record<string, any>;
  timeout: number;
}

/**
 * Extended API client interface with interceptor support
 */
export interface ExtendedRagApiClient extends RagApiClient {
  /**
   * Add a request interceptor
   *
   * @param interceptor - Function to intercept and modify requests
   * @returns Function to remove the interceptor
   */
  addRequestInterceptor(interceptor: RequestInterceptor): () => void;

  /**
   * Add a response interceptor
   *
   * @param interceptor - Function to intercept and process responses
   * @returns Function to remove the interceptor
   */
  addResponseInterceptor(interceptor: ResponseInterceptor): () => void;
}

/**
 * Mock implementation for testing purposes
 *
 * @param responses - Map of request signatures to mock responses
 * @returns Mock API client
 *
 * @example
 * ```typescript
 * const mockClient = createMockClient({
 *   ask: {
 *     success: true,
 *     data: {
 *       answer: "Mock answer",
 *       sources: [],
 *       query: "test",
 *       chunk_count: 0,
 *       response_time_ms: 100,
 *     }
 *   }
 * });
 * ```
 */
export function createMockClient(
  responses: Partial<Record<keyof RagApiClient, ApiResult<any>>>
): RagApiClient;

/**
 * Validation utilities for requests and responses
 */
export const Validators = {
  /**
   * Validate an AskRequest before sending
   *
   * @param request - The request to validate
   * @returns Validation errors, empty array if valid
   */
  validateAskRequest(request: AskRequest): string[];

  /**
   * Validate an AskResponse from the backend
   *
   * @param response - The response to validate
   * @returns Validation errors, empty array if valid
   */
  validateAskResponse(response: AskResponse): string[];

  /**
   * Validate a URL format
   *
   * @param url - The URL to validate
   * @returns True if valid HTTP/HTTPS URL
   */
  isValidUrl(url: string): boolean;
} as const;
