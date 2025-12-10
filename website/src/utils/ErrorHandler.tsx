import React from 'react';

interface ErrorHandlerProps {
  error?: Error | string | null;
  children?: React.ReactNode;
  fallback?: React.ComponentType<{ error?: Error | string | null }>;
}

export const withErrorHandler = <P extends object>(
  Component: React.ComponentType<P>,
  fallback?: React.ComponentType<{ error?: Error | string | null }>
) => {
  return (props: P) => {
    return (
      <ErrorHandler fallback={fallback}>
        <Component {...props} />
      </ErrorHandler>
    );
  };
};

export const ErrorHandler: React.FC<ErrorHandlerProps> = ({ 
  error, 
  children, 
  fallback: Fallback 
}) => {
  if (error) {
    if (Fallback) {
      return <Fallback error={error} />;
    }
    return (
      <div className="error-container">
        <h3>⚠️ Something went wrong</h3>
        <p>{error instanceof Error ? error.message : String(error)}</p>
        <button onClick={() => window.location.reload()}>Reload Page</button>
      </div>
    );
  }

  return <>{children}</>;
};

// Context for global error handling
import React, { createContext, useContext, useState } from 'react';

interface ErrorContextType {
  error: Error | null;
  setError: (error: Error | null) => void;
  clearError: () => void;
}

const ErrorContext = createContext<ErrorContextType | undefined>(undefined);

export const ErrorProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [error, setError] = useState<Error | null>(null);

  const clearError = () => setError(null);

  return (
    <ErrorContext.Provider value={{ error, setError, clearError }}>
      <ErrorHandler error={error}>
        {children}
      </ErrorHandler>
    </ErrorContext.Provider>
  );
};

export const useErrorHandler = () => {
  const context = useContext(ErrorContext);
  if (context === undefined) {
    throw new Error('useErrorHandler must be used within an ErrorProvider');
  }
  return context;
};