#!/usr/bin/env bash
# -----------------------------------------------------------------------
# bump-submodule.sh
#
# After you edit files inside a submodule under external/, run:
#   scripts/bump-submodule.sh <submodule-path> "<commit message>"
#
# Example:
#   scripts/bump-submodule.sh external/PX4-Autopilot "Tweak gz launch args"
#
# What it does, in order:
#   1. Verifies you're on a real (non-detached) branch in the submodule.
#   2. Commits everything currently `git diff`'d in the submodule.
#   3. Pushes the submodule branch to its tracked remote.
#   4. Stages the submodule pointer bump in the parent repo and commits.
#
# What it does NOT do:
#   - Push the parent repo. Do that manually after reviewing:
#       git log -1 && git push
#   - Add new (untracked) files inside the submodule. Add those by hand
#     first, then re-run this script.
# -----------------------------------------------------------------------
set -euo pipefail

if [ $# -lt 2 ]; then
	echo "usage: $0 <submodule-path> <commit-message>" >&2
	exit 2
fi

SUBPATH="$1"
MSG="$2"

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "${REPO_ROOT}"

if [ ! -d "${SUBPATH}/.git" ] && [ ! -f "${SUBPATH}/.git" ]; then
	echo "error: ${SUBPATH} is not a git submodule" >&2
	exit 1
fi

pushd "${SUBPATH}" >/dev/null

BRANCH="$(git symbolic-ref --quiet --short HEAD || true)"
if [ -z "${BRANCH}" ]; then
	echo "error: submodule ${SUBPATH} is in detached HEAD." >&2
	echo "       Switch to your working branch first, e.g.:" >&2
	echo "         (cd ${SUBPATH} && git checkout ai-grand-prix)" >&2
	exit 1
fi

if git diff --quiet && git diff --cached --quiet; then
	echo "[bump] ${SUBPATH}: no staged or unstaged changes; skipping commit."
else
	git add -u
	git -c user.name="Nelson Herrera" -c user.email="ndanielherrera@icloud.com" \
		commit -m "${MSG}"
fi

echo "[bump] ${SUBPATH}: pushing ${BRANCH}..."
git push

NEW_SHA="$(git rev-parse HEAD)"
popd >/dev/null

# Now bump the pointer in the parent repo.
git add "${SUBPATH}"
if git diff --cached --quiet -- "${SUBPATH}"; then
	echo "[bump] parent repo: submodule pointer already at ${NEW_SHA:0:10}; nothing to commit."
	exit 0
fi

git -c user.name="Nelson Herrera" -c user.email="ndanielherrera@icloud.com" \
	commit -m "Bump ${SUBPATH} -> ${NEW_SHA:0:10}

${MSG}"

echo
echo "[bump] Done. Review and push the parent repo when ready:"
echo "         git log -1 && git push"
