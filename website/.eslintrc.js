module.exports = {
  root: true,
  extends: [
    '@docusaurus',
    'prettier',
    'plugin:prettier/recommended',
  ],
  plugins: [
    '@typescript-eslint',
    'prettier'
  ],
  rules: {
    'prettier/prettier': 'error',
    'prefer-const': 'error',
    '@typescript-eslint/no-unused-vars': 'error'
  }
};