# CONTRIBUTING

Thanks for contributing.

This file covers collaboration rules and best practices. Before working on this repository, complete [Setup](docs/SETUP.md). For daily workflows and architecture notes, see [docs/](docs/README.md).

## 1) Branching rules

- Do not push directly to `main`.
- Create a topic branch for every change.
- Use branch names in this format: `type/topic-short-description`.
  - `type` should be one of:
    - `feature`: new functionality or meaningful behavior expansion
    - `fix`: bug fix or regression fix
    - `docs`: documentation-only changes
    - `chore`: maintenance work (tooling, dependencies, CI, cleanup)
    - `spike`: short exploratory work to validate an approach
  - Example: `fix/log-decode-timeout`, `docs/setup-b2-clarification`

## 2) Commit and history hygiene

- Keep commits small and focused.
- Prefer clear, descriptive commit messages.
- Once a branch is pushed, avoid history rewrites.
- Do not force-push shared branch history.

## 3) Keep your branch current

Merge `main` into your branch regularly:

```bash
git fetch origin
git merge origin/main
```

After resolving conflicts, rerun local checks before pushing.

## 4) Pull request expectations

- Open a PR as soon as the change is reviewable.
- Keep PRs scoped to one logical change.
- Link the related issue/task when available.
- Describe:
  - what changed
  - why it changed
  - how you validated it
- Request at least one reviewer.
- Do not merge with failing required checks.
- Use **Squash and merge**.

## 5) Required local checks before push

From repo root:

```bash
pre-commit run --all-files
just fmt --check
just ci-checks
just test
```

Note that these checks will also be enforced by the CI.

## 6) Review quality guidelines

- Prefer small PRs over large batches.
- Call out breaking changes explicitly.
- Include tests when adding/changing behavior.
- If something cannot be tested locally, state that clearly in the PR.

## 7) After merge

- Delete the remote branch.
- Delete your local branch.

## 8) Licensing of contributions

By opening a PR you agree that your contribution is licensed under MIT OR Apache-2.0, the same terms as the rest of the repository. See [License](README.md#license).

## 9) Source headers

All crate-root `lib.rs` and `main.rs` files should begin with:

```rust
// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0
```
