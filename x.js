module.exports = {
  root: true,
  env: {
    browser: true,
    commonjs: true,
    es6: true
  },
  extends: ['plugin:react/recommended', 'prettier'],
  plugins: [ 'prettier', 'preact', 'react-hooks'],
  parser: 'babel-eslint',
  parserOptions: {
    sourceType: 'module',
    ecmaVersion: 2018,
    ecmaFeatures: {
      jsx: true
    }
  },
  settings: {
    react: {
      createClass: 'createReactClass',
      pragma: 'React', // Pragma to use, default to "React"
      version: '16.8'
    },
    propWrapperFunctions: [
      'forbidExtraProps',
      { property: 'freeze', object: 'Object' },
      { property: 'myFavoriteWrapper' }
    ],
    linkComponents: ['Hyperlink', { name: 'Link', linkAttribute: 'to' }]
  },
  rules: {
    // react-config -> https://github.com/yannickcr/eslint-plugin-react/
    'react/jsx-uses-react': 'error',
    'react/jsx-uses-vars': 'error',
    'react/no-deprecated': 'error',
    'react/display-name': [0],
    'react/prop-types': [0],
    'react-hooks/rules-of-hooks': 'error',
    'react-hooks/exhaustive-deps': 'warn',
    strict: 'error',
    'check-constructor': [0],
    'no-comma-dangle': [0],
    'no-empty-interface': [0],
    'no-unused-vars': [0],
    'prettier/prettier': [
      'error',
      {
        semi: false,
        singleQuote: "true",
        printWidth: 80,
        tabWidth: 2
      }
    ]
  }
}
