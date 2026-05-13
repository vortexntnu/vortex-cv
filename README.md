[![Industrial CI](https://github.com/vortexntnu/vortex-cv/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/vortex-cv/actions/workflows/industrial-ci.yml)
[![pre-commit](https://github.com/vortexntnu/vortex-cv/actions/workflows/pre-commit.yml/badge.svg)](https://github.com/vortexntnu/vortex-cv/actions/workflows/pre-commit.yml)
[![codecov](https://codecov.io/gh/vortexntnu/vortex-cv/graph/badge.svg?token=GoXxR2nR0y)](https://codecov.io/gh/vortexntnu/vortex-cv)

# Introduction

```
TODO: Write a simple description / introduction to the repository
```

# Setup

```
TODO: Write a setup guide
```

## Using pre-commit

This project uses [pre-commit](https://github.com/pre-commit/pre-commit) to manage automated checks.

### Install pre-commit
```bash
pip install pre-commit
```

### Run all hooks manually
```bash
pre-commit run --all-files
```

### Install the git hook
This will make the checks run automatically every time you `git commit`:
```bash
pre-commit install
```

### Update pre-commit hooks
```bash
pre-commit autoupdate
```

## GitHub Actions CI
- This project uses GitHub Actions for CI.
- Workflows are located in [`.github/workflows`](.github/workflows/).
- For more information, see [vortex-ci](https://github.com/vortexntnu/vortex-ci).
