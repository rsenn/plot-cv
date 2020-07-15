module.exports = {
  env: {
    browser: true,
    es6: true,
    node: true
  },
  extends: ["eslint:recommended", "plugin:preact/recommended"],
  globals: {
    Atomics: "readonly",
    SharedArrayBuffer: "readonly"
  },
  parserOptions: {
    ecmaFeatures: {
      modules: true,
      jsx: true
    },
    ecmaVersion: 2020,
    sourceType: "module"
  },
  settings: {
    preact: {
      pragma: "h"
    }
  },
  plugins: [
    "class-property",
    "prettier",
    "preact" //,'no-unused-code'
  ],
  rules: {
    "no-unused-vars": ["error", { varsIgnorePattern: "h" }],
    "no-mixed-spaces-and-tabs": "off",
    "no-console": "warn",
    indent: ["error", 2]
  }
};
