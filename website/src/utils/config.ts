// Environment configuration for the Physical AI book
// This handles configuration that can vary between development, testing, and production

// Default configuration values
const defaultConfig = {
  apiBaseUrl: '',
  enableLogging: true,
  maxExecutionTime: 120000, // 2 minutes in milliseconds
  maxOutputLength: 10000,
  supportedLanguages: ['javascript', 'python'], // via Pyodide
  allowedOrigins: ['localhost', 'your-username.github.io'],
  enableDemoSharing: true,
  enableUserProgress: true,
  enableComments: true,
  enableRAGChatbot: false // Optional feature
};

// Configuration based on environment
const envConfig = {
  development: {
    ...defaultConfig,
    apiBaseUrl: 'http://localhost:3000/api',
    enableLogging: true,
  },
  production: {
    ...defaultConfig,
    apiBaseUrl: 'https://your-username.github.io/the-physical-ai-handbook/api',
    enableLogging: false,
  },
  test: {
    ...defaultConfig,
    apiBaseUrl: 'http://localhost:3001/api',
    enableLogging: false,
  }
};

// Get configuration based on current environment
const getEnvironment = (): keyof typeof envConfig => {
  // In browser environments, we might determine this based on window.location
  if (typeof window !== 'undefined') {
    const hostname = window.location.hostname;
    if (hostname === 'localhost' || hostname.startsWith('127.0.0.1')) {
      return 'development';
    } else if (hostname.endsWith('.github.io')) {
      return 'production';
    }
  }
  
  // Default to development if not in browser
  return 'development';
};

// Current configuration
const currentEnv = getEnvironment();
const config = envConfig[currentEnv];

export interface AppConfig {
  apiBaseUrl: string;
  enableLogging: boolean;
  maxExecutionTime: number;
  maxOutputLength: number;
  supportedLanguages: string[];
  allowedOrigins: string[];
  enableDemoSharing: boolean;
  enableUserProgress: boolean;
  enableComments: boolean;
  enableRAGChatbot: boolean;
}

// Get the current configuration
export const getAppConfig = (): AppConfig => {
  return { ...config };
};

// Validate and normalize configuration values
export const validateConfig = (cfg: Partial<AppConfig>): string[] => {
  const errors: string[] = [];

  if (cfg.maxExecutionTime && cfg.maxExecutionTime <= 0) {
    errors.push('maxExecutionTime must be positive');
  }

  if (cfg.maxOutputLength && cfg.maxOutputLength <= 0) {
    errors.push('maxOutputLength must be positive');
  }

  return errors;
};

// Initialize configuration and validate
export const initConfig = (): AppConfig => {
  const errors = validateConfig(config);
  
  if (errors.length > 0) {
    console.error('Configuration validation errors:', errors);
    throw new Error(`Configuration validation failed: ${errors.join(', ')}`);
  }

  if (config.enableLogging) {
    console.log(`Environment: ${currentEnv}`);
    console.log('Configuration loaded successfully');
  }

  return config;
};

// Configuration provider for React components
import React, { createContext, useContext } from 'react';

interface ConfigContextType {
  config: AppConfig;
  isProduction: boolean;
  isDevelopment: boolean;
}

const ConfigContext = createContext<ConfigContextType | undefined>(undefined);

export const ConfigProvider: React.FC<{ 
  children: React.ReactNode,
  config?: Partial<AppConfig> 
}> = ({ children, config: overrideConfig }) => {
  // Merge override config with current config
  const finalConfig: AppConfig = {
    ...getAppConfig(),
    ...overrideConfig
  };

  const contextValue: ConfigContextType = {
    config: finalConfig,
    isProduction: currentEnv === 'production',
    isDevelopment: currentEnv === 'development'
  };

  return (
    <ConfigContext.Provider value={contextValue}>
      {children}
    </ConfigContext.Provider>
  );
};

export const useConfig = () => {
  const context = useContext(ConfigContext);
  if (context === undefined) {
    throw new Error('useConfig must be used within a ConfigProvider');
  }
  return context;
};