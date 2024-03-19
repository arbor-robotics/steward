---
layout: default
title: Code style
---

# Code style

## Overview

Nobody likes burdensome, ridiculous style guidelines. Let's avoid that. However, basic style guidelines help make our project feel unified, and it keeps our codebase organized.

## Naming conventions for C++ AND Python

- Variable names should use `snake_case`.
- Class names should use `PascalCase` (the first letter should be capitalized).
- Functions should use `camelCase` (the first letter should be lowercase).

## Python

### The Black formatter

The good news is that the vast majority of code formatting is done for you by the Black formatter. All committed code is automatically formatted by GitHub using a precommit hook. If this sounds complicated, just know that most code formatting is handled for you. You can also use Black in VS Code using [this extension](https://marketplace.visualstudio.com/items?itemName=ms-python.black-formatter).

## Git branch names

- Branch names should be `separated-by-hyphens`.
- Every branch name should begin with one of the following:
  - `feature/` (for new features)
  - `milestone/` (combines multiple new features)
  - `bugfix/` (fixes bugs found in code from feature branches)

Examples:

- `milestone/progress-review-2`
- `feature/gnss-filter-package`
