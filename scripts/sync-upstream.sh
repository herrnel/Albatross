#!/usr/bin/env bash
# -----------------------------------------------------------------------
# sync-upstream.sh
#
# Rebase a fork-tracking branch (`ai-grand-prix`) onto upstream's main.
#
# Usage:
#   scripts/sync-upstream.sh <submodule-path> [upstream-branch]
#
# Example:
#   scripts/sync-upstream.sh external/PX4-Autopilot main
#   scripts/sync-upstream.sh external/PX4-Autopilot/Tools/simulation/gz main
#
# Conventions assumed (set up by the initial fork+wire pass):
#   - remote 'origin'  = upstream project        (read-only for us)
#   - remote 'myfork'  = your fork on github.com (push target)
#   - working branch   = 'ai-grand-prix'
#
# After a successful rebase + push, run scripts/bump-submodule.sh to
# update the parent repo's submodule pointer.
# -----------------------------------------------------------------------
set -euo pipefail

SUBPATH="${1:?usage: $0 <submodule-path> [upstream-branch]}"
UPSTREAM_BRANCH="${2:-main}"
BRANCH="ai-grand-prix"

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "${REPO_ROOT}/${SUBPATH}"

if ! git remote get-url origin >/dev/null 2>&1; then
	echo "error: no 'origin' remote in ${SUBPATH}" >&2; exit 1
fi
if ! git remote get-url myfork >/dev/null 2>&1; then
	echo "error: no 'myfork' remote in ${SUBPATH}" >&2
	echo "       Add it with:  git remote add myfork <your-fork-url>" >&2
	exit 1
fi

if ! git diff --quiet || ! git diff --cached --quiet; then
	echo "error: ${SUBPATH} has uncommitted changes; commit or stash first." >&2
	exit 1
fi

echo "[sync] Fetching origin (upstream)..."
git fetch origin "${UPSTREAM_BRANCH}"

echo "[sync] Checking out ${BRANCH}..."
git checkout "${BRANCH}"

echo "[sync] Rebasing ${BRANCH} onto origin/${UPSTREAM_BRANCH}..."
if ! git rebase "origin/${UPSTREAM_BRANCH}"; then
	echo
	echo "[sync] Rebase hit conflicts. Resolve them, then:"
	echo "         git add <files> && git rebase --continue"
	echo "         scripts/sync-upstream.sh ${SUBPATH} ${UPSTREAM_BRANCH}  # to finish push"
	exit 1
fi

echo "[sync] Force-pushing to myfork (with lease)..."
git push --force-with-lease myfork "${BRANCH}"

echo
echo "[sync] ${SUBPATH} is up to date with origin/${UPSTREAM_BRANCH}."
echo "       Now bump the parent repo:"
echo "         scripts/bump-submodule.sh ${SUBPATH} 'Sync with upstream'"
