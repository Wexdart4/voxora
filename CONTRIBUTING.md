# Contributing to Voxora

Thank you for your interest in contributing to **Voxora** — the deterministic 2D-to-spatial vision library in pure Rust!

## Development Guidelines

### 1. Pure Rust & Zero Unnecessary Dependencies
- Core algorithms must remain implemented in pure Rust with minimal external dependencies.
- No pretrained neural networks or heavy ML frameworks are permitted in core reconstruction algorithms.

### 2. Code Quality & Standards
Before submitting a Pull Request, ensure that all verification checks pass locally:

```bash
# Check compilation across all targets
cargo check --workspace --all-targets

# Run complete unit & integration test suite
cargo test --workspace

# Run Clippy lints
cargo clippy --workspace --all-targets -- -D warnings

# Verify code formatting
cargo fmt --all --check
```

### 3. Pull Request Process
1. Fork the repository and create your feature branch (`git checkout -b feature/my-feature`).
2. Implement your changes along with comprehensive unit tests in the relevant workspace crate.
3. Commit your changes with clear, descriptive commit messages.
4. Push to the branch (`git push origin feature/my-feature`) and open a Pull Request against `main`.

## License
By contributing to Voxora, you agree that your contributions will be licensed under the MIT License.
