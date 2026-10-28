---
name: merge-pr
description: Merge a stable-flight PR once the maintainer says to merge: confirm CI on the head commit, squash or rebase, and merge the bottom of a PR stack without closing the PRs above it.
---

# Merge a PR

Merge only when told to. Squash vs rebase, and the `main` ruleset, are in AGENTS.md
(**Commits, PRs, and releases**).

## One PR

1. **Confirm a completed CI run on the PR's head commit:**
   `gh run list --commit $(gh pr view <n> --json headRefOid -q .headRefOid)`.
   `gh pr checks --watch` returns at once if GitHub hasn't registered the checks yet, so don't
   rely on it right after a push.
2. Merge: `gh pr merge <n> --squash --delete-branch` for a single change (subject
   `<PR title> (#id)`), `--rebase` when the PR has follow-up commits.
3. Pull `main` and report the merge commit.

## A PR stack

CI runs only on PRs based on `main`, so only the bottom of a stack gets checks. To merge the
bottom PR (the parent):

1. Record the parent's tip SHA.
2. **Retarget the child to `main` first:** `gh pr edit <child> --base main`. Merging with
   `--delete-branch` closes PRs still based on the parent, for good once their head is
   force-pushed.
3. Merge the parent (**One PR** above).
4. From the top of the stack: `git rebase --update-refs --onto main <old-parent-sha>`, then
   `git push --force-with-lease` each branch.
