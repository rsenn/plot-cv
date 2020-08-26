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
    react: {
      pragma: "h",
      version: "16.3" // React version. "detect" automatically picks the version you have installed.
      // You can also use `16.0`, `16.3`, etc, if you want to override the detected value.
      // default to latest and warns if missing
      // It will default to "detect" in the future
    }
  },
  plugins: [
    "class-property",
    "prettier",
    "preact" //,"no-unused-code"
  ],
  rules: {
    "no-unused-vars": ["error", { varsIgnorePattern: "h" }],
    "no-mixed-spaces-and-tabs": "off",
    "no-console": "warn",
    indent: ["error", 2],
    "react/react-in-jsx-scope": "off",
    "react/prop-types": "off"
  }
};
