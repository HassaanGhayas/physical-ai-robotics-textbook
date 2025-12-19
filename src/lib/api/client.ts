/**
 * API client for RAG chatbot backend
 * Implements retry logic, error handling, and request cancellation
 */

import axios, { AxiosInstance, AxiosError, CancelTokenSource } from 'axios';
import type {
  AskRequest,
  AskResponse,
  ApiResult,
  ApiError,
  ApiClientConfig,
  RagApiClient,
  HealthStatus,
  BackendSource,
  APIError,
} from './types';
import { ErrorCodes, ErrorMessages } from './types';
import { validateAskRequest, validateAskResponse } from './validators';
import type { SourceReference } from '../../types/chat';
import { extractTitleFromUrl } from '../../types/chat';

/**
 * Map backend source to frontend SourceReference
 */
function mapBackendSourceToFrontend(source: BackendSource): SourceReference {
  return {
    id: source.id,
    content: source.content,
    preview: source.content.substring(0, 150) + (source.content.length > 150 ? '...' : ''),
    url: source.url,
    title: extractTitleFromUrl(source.url),
    chunkId: source.chunk_id,
    documentId: source.document_id,
    similarityScore: source.similarity_score,
    position: source.position,
  };
}

/**
 * Handle API errors and convert to ApiError
 */
function handleApiError(error: unknown): ApiError {
  if (axios.isAxiosError(error)) {
    const axiosError = error as AxiosError<APIError>;

    // Network error (no response)
    if (!axiosError.response) {
      if (axiosError.code === 'ECONNABORTED') {
        return {
          code: ErrorCodes.TIMEOUT,
          message: ErrorMessages[ErrorCodes.TIMEOUT],
          retryable: true,
          originalError: error as Error,
        };
      }

      return {
        code: ErrorCodes.NETWORK_ERROR,
        message: ErrorMessages[ErrorCodes.NETWORK_ERROR],
        retryable: true,
        originalError: error as Error,
      };
    }

    // Server responded with error
    const statusCode = axiosError.response.status;
    const backendError = axiosError.response.data;

    if (statusCode === 400) {
      return {
        code: ErrorCodes.VALIDATION_ERROR,
        message: backendError?.message || ErrorMessages[ErrorCodes.VALIDATION_ERROR],
        statusCode,
        type: backendError?.type,
        details: backendError?.details,
        retryable: false,
        originalError: error as Error,
      };
    }

    if (statusCode === 503) {
      return {
        code: ErrorCodes.SERVICE_UNAVAILABLE,
        message: backendError?.message || ErrorMessages[ErrorCodes.SERVICE_UNAVAILABLE],
        statusCode,
        type: backendError?.type,
        retryable: true,
        originalError: error as Error,
      };
    }

    if (statusCode >= 500) {
      return {
        code: ErrorCodes.SERVER_ERROR,
        message: backendError?.message || ErrorMessages[ErrorCodes.SERVER_ERROR],
        statusCode,
        type: backendError?.type,
        retryable: true,
        originalError: error as Error,
      };
    }

    return {
      code: ErrorCodes.UNKNOWN_ERROR,
      message: ErrorMessages[ErrorCodes.UNKNOWN_ERROR],
      statusCode,
      retryable: false,
      originalError: error as Error,
    };
  }

  // Non-axios error
  return {
    code: ErrorCodes.UNKNOWN_ERROR,
    message: ErrorMessages[ErrorCodes.UNKNOWN_ERROR],
    retryable: false,
    originalError: error as Error,
  };
}

/**
 * Sleep utility for retry delays
 */
function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

/**
 * Create an API client instance
 */
export function createApiClient(config: ApiClientConfig): RagApiClient {
  const axiosInstance: AxiosInstance = axios.create({
    baseURL: config.baseUrl,
    timeout: config.timeout || 10000,
    headers: {
      'Content-Type': 'application/json',
      ...config.headers,
    },
  });

  // Track cancel tokens for request cancellation
  const cancelTokens = new Map<string, CancelTokenSource>();

  // Request interceptor for logging
  axiosInstance.interceptors.request.use(
    (requestConfig) => {
      console.log(`[API] ${requestConfig.method?.toUpperCase()} ${requestConfig.url}`);
      return requestConfig;
    },
    (error) => {
      console.error('[API] Request error:', error.message);
      return Promise.reject(error);
    }
  );

  // Response interceptor for logging
  axiosInstance.interceptors.response.use(
    (response) => {
      console.log(`[API] Response: ${response.status}`);
      return response;
    },
    (error) => {
      console.error('[API] Response error:', error.message);
      return Promise.reject(error);
    }
  );

  /**
   * Implement ask method with retry logic
   */
  async function ask(request: AskRequest): Promise<ApiResult<AskResponse>> {
    // Validate request
    const validation = validateAskRequest(request);
    if (!validation.isValid) {
      return {
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: validation.errors.join(', '),
          retryable: false,
        },
      };
    }

    const maxRetries = config.maxRetries || 3;
    const retryDelay = config.retryDelay || 1000;
    const exponentialBackoff = config.exponentialBackoff !== false;

    let lastError: ApiError | null = null;

    // Retry loop
    for (let attempt = 0; attempt <= maxRetries; attempt++) {
      try {
        // Create cancel token for this request
        const cancelToken = axios.CancelToken.source();
        if (request.request_id) {
          cancelTokens.set(request.request_id, cancelToken);
        }

        console.log(`[API] Attempt ${attempt + 1}/${maxRetries + 1} for query: "${request.query.substring(0, 50)}..."`);

        const response = await axiosInstance.post<AskResponse>(
          '/ask',
          request,
          { cancelToken: cancelToken.token }
        );

        // Cleanup cancel token
        if (request.request_id) {
          cancelTokens.delete(request.request_id);
        }

        // Validate response
        const responseValidation = validateAskResponse(response.data);
        if (!responseValidation.isValid) {
          console.warn('[API] Response validation failed:', responseValidation.errors);
          // Continue anyway, but log the issue
        }

        console.log(`[API] Success: ${response.data.sources.length} sources, ${response.data.response_time_ms}ms`);

        return { success: true, data: response.data };
      } catch (error) {
        lastError = handleApiError(error);

        // Cleanup cancel token
        if (request.request_id) {
          cancelTokens.delete(request.request_id);
        }

        // Check if error is retryable
        if (!lastError.retryable || attempt >= maxRetries) {
          console.error(`[API] Failed after ${attempt + 1} attempts:`, lastError.message);
          return { success: false, error: lastError };
        }

        // Calculate retry delay with exponential backoff
        const delay = exponentialBackoff
          ? retryDelay * Math.pow(2, attempt)
          : retryDelay;

        console.warn(`[API] Attempt ${attempt + 1} failed, retrying in ${delay}ms:`, lastError.message);
        await sleep(delay);
      }
    }

    // Should never reach here, but just in case
    return {
      success: false,
      error: lastError || {
        code: 'UNKNOWN_ERROR',
        message: 'Request failed after all retries',
        retryable: false,
      },
    };
  }

  /**
   * Check health status
   */
  async function checkHealth(): Promise<ApiResult<HealthStatus>> {
    try {
      const response = await axiosInstance.get<HealthStatus>('/health');
      return { success: true, data: response.data };
    } catch (error) {
      return { success: false, error: handleApiError(error) };
    }
  }

  /**
   * Cancel an ongoing request
   */
  function cancelRequest(requestId: string): void {
    const cancelToken = cancelTokens.get(requestId);
    if (cancelToken) {
      cancelToken.cancel('Request cancelled by user');
      cancelTokens.delete(requestId);
      console.log(`[API] Cancelled request: ${requestId}`);
    }
  }

  /**
   * Update client configuration
   */
  function updateConfig(newConfig: Partial<ApiClientConfig>): void {
    Object.assign(config, newConfig);

    // Update axios instance if baseURL or timeout changed
    if (newConfig.baseUrl) {
      axiosInstance.defaults.baseURL = newConfig.baseUrl;
    }
    if (newConfig.timeout) {
      axiosInstance.defaults.timeout = newConfig.timeout;
    }

    console.log('[API] Configuration updated:', newConfig);
  }

  return {
    ask,
    checkHealth,
    cancelRequest,
    updateConfig,
  };
}

/**
 * Type guard: Check if error is retryable
 */
export function isRetryableError(error: ApiError): boolean {
  return error.retryable;
}

/**
 * Type guard: Check if error is a network error
 */
export function isNetworkError(error: ApiError): boolean {
  return error.code === 'NETWORK_ERROR' || error.code === 'CONNECTION_REFUSED';
}

/**
 * Type guard: Check if error is a timeout error
 */
export function isTimeoutError(error: ApiError): boolean {
  return error.code === 'TIMEOUT';
}

/**
 * Type guard: Check if error is a validation error
 */
export function isValidationError(error: ApiError): boolean {
  return error.code === 'VALIDATION_ERROR';
}

/**
 * Get user-friendly error message
 */
export function getUserFriendlyMessage(error: ApiError): string {
  return error.message;
}
