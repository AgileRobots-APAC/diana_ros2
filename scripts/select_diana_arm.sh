#!/usr/bin/env bash
# Select the Diana arm that libfranka is compiled for.
#
# libfranka's rate-limiting parameters are chosen at COMPILE time through
# diana_libfranka_overlay. This script checks out the overlay tag for the given
# arm, resets the libfranka submodule to its pinned commit and applies that
# arm's patch. Rebuild the workspace afterwards (the script prints the command).
#
# Usage: see `select_diana_arm.sh --help`.

set -euo pipefail

readonly ARMS=(diana3 diana7 diana7g2)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
readonly DEPENDENCY_REPOS="${REPO_ROOT}/dependency.repos"

usage() {
  cat <<EOF
Usage: $(basename "$0") <diana3|diana7|diana7g2> [options]
       $(basename "$0") --status [options]

Switch diana_libfranka_overlay to the given Diana arm and apply its libfranka
patch. Run it before the first build and every time you switch to a different
real robot, then rebuild libfranka and all packages that depend on it.

Options:
  --overlay <path>   Path to diana_libfranka_overlay
                     (default: \$DIANA_LIBFRANKA_OVERLAY or
                     ${REPO_ROOT}/diana_libfranka_overlay)
  --release <prefix> Overlay tag prefix (default: ${DEFAULT_RELEASE},
                     read from dependency.repos); the tag
                     <prefix>-<arm> is checked out
  --no-fetch         Do not run 'git fetch --tags' (offline use)
  --force            Discard local changes in the libfranka submodule even if
                     they are not exactly a previously applied arm patch
  --status           Print which arm libfranka is currently patched for
  -h, --help         Show this help
EOF
}

info() { echo -e "\033[1;34m[select_diana_arm]\033[0m $*"; }
warn() { echo -e "\033[1;33m[select_diana_arm] WARNING:\033[0m $*" >&2; }
die() {
  echo -e "\033[1;31m[select_diana_arm] ERROR:\033[0m $*" >&2
  exit 1
}

# Prints the diana_libfranka_overlay tag pinned by dependency.repos.
dependency_overlay_tag() {
  [[ -f "${DEPENDENCY_REPOS}" ]] ||
    die "Dependency file not found at '${DEPENDENCY_REPOS}'."

  local tag
  tag="$(awk '
    $1 == "diana_libfranka_overlay:" { in_overlay = 1; next }
    in_overlay && $1 == "version:" { print $2; exit }
    in_overlay && /^  [^ ]/ { exit }
  ' "${DEPENDENCY_REPOS}")"
  tag="${tag%\"}"
  tag="${tag#\"}"
  [[ -n "${tag}" ]] ||
    die "Could not read diana_libfranka_overlay.version from '${DEPENDENCY_REPOS}'."
  printf '%s\n' "${tag}"
}

# Prints an overlay tag prefix by removing its supported arm suffix.
overlay_release_prefix() {
  local tag="$1"
  local arm
  for arm in "${ARMS[@]}"; do
    if [[ "${tag}" == *-"${arm}" ]]; then
      printf '%s\n' "${tag%-"${arm}"}"
      return 0
    fi
  done
  return 1
}

DEPENDENCY_OVERLAY_TAG="$(dependency_overlay_tag)"
DEFAULT_RELEASE="$(overlay_release_prefix "${DEPENDENCY_OVERLAY_TAG}")" ||
  die "Overlay tag '${DEPENDENCY_OVERLAY_TAG}' from '${DEPENDENCY_REPOS}' does not end in a supported arm."
readonly DEPENDENCY_OVERLAY_TAG DEFAULT_RELEASE

# Prints the arm that rate_limiting.h is patched for, based on kMaxTranslationalJerk.
detect_arm() {
  local header="$1/include/franka/rate_limiting.h"
  [[ -f "${header}" ]] || {
    echo "unknown (libfranka submodule not initialized)"
    return
  }
  local value
  value="$(sed -n 's/^constexpr double kMaxTranslationalJerk = \([0-9.]*\).*/\1/p' "${header}")"
  case "${value}" in
    2500) echo "diana3" ;;
    3000.0) echo "diana7" ;;
    5000.0) echo "diana7g2" ;;
    4500.0) echo "none (unpatched upstream libfranka, Franka FR3 limits)" ;;
    *) echo "unknown (kMaxTranslationalJerk = '${value}')" ;;
  esac
}

ARM=""
OVERLAY="${DIANA_LIBFRANKA_OVERLAY:-${REPO_ROOT}/diana_libfranka_overlay}"
RELEASE="${DEFAULT_RELEASE}"
FETCH=1
FORCE=0
STATUS_ONLY=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h | --help)
      usage
      exit 0
      ;;
    --overlay)
      [[ $# -ge 2 ]] || die "--overlay needs a path"
      OVERLAY="$2"
      shift 2
      ;;
    --release)
      [[ $# -ge 2 ]] || die "--release needs a tag prefix"
      RELEASE="$2"
      shift 2
      ;;
    --no-fetch)
      FETCH=0
      shift
      ;;
    --force)
      FORCE=1
      shift
      ;;
    --status)
      STATUS_ONLY=1
      shift
      ;;
    -*) die "Unknown option '$1'. See --help." ;;
    *)
      [[ -z "${ARM}" ]] || die "Only one arm can be given."
      ARM="$(echo "$1" | tr '[:upper:]' '[:lower:]')"
      shift
      ;;
  esac
done

[[ -d "${OVERLAY}/.git" || -f "${OVERLAY}/.git" ]] ||
  die "diana_libfranka_overlay not found at '${OVERLAY}'.
       Run 'vcs import src < src/dependency.repos --recursive' first, or pass --overlay <path>."
OVERLAY="$(cd "${OVERLAY}" && pwd)"
LIBFRANKA="${OVERLAY}/libfranka"

if [[ ${STATUS_ONLY} -eq 1 ]]; then
  info "Overlay:   ${OVERLAY} ($(git -C "${OVERLAY}" describe --tags --always 2>/dev/null))"
  info "libfranka: patched for $(detect_arm "${LIBFRANKA}")"
  exit 0
fi

[[ -n "${ARM}" ]] || {
  usage >&2
  exit 1
}
[[ " ${ARMS[*]} " == *" ${ARM} "* ]] || die "Unknown arm '${ARM}'. Use one of: ${ARMS[*]}."

TAG="${RELEASE}-${ARM}"
PATCH_REL="patches/${ARM}.patch"

# The overlay itself must be clean; only the libfranka submodule may carry a patch.
if [[ -n "$(git -C "${OVERLAY}" status --porcelain --ignore-submodules=all)" ]]; then
  git -C "${OVERLAY}" status --short --ignore-submodules=all >&2
  die "diana_libfranka_overlay has local changes (listed above). Commit or discard them first."
fi

# Prints every path a patch touches (old and new side of each 'diff --git').
patch_paths() {
  sed -n 's|^diff --git a/\(.*\) b/\(.*\)$|\1\n\2|p' "$1" | sort -u
}

# Prints every changed or untracked file in the libfranka submodule.
dirty_paths() {
  git -C "${LIBFRANKA}" status --porcelain --untracked-files=all | cut -c4- | sort -u
}

# 1. Remove the previously applied patch from the libfranka submodule.
#    The patch is discarded only if every changed file belongs to it and its
#    content still matches it, so unrelated local work is never deleted.
if [[ -e "${LIBFRANKA}/.git" ]] && [[ -n "$(dirty_paths)" ]]; then
  CURRENT_PATCH="$(ls "${OVERLAY}"/patches/*.patch 2>/dev/null | head -n 1 || true)"
  if [[ -n "${CURRENT_PATCH}" ]] &&
    [[ -z "$(comm -23 <(dirty_paths) <(patch_paths "${CURRENT_PATCH}"))" ]] &&
    git -C "${LIBFRANKA}" apply --reverse --check --whitespace=nowarn "${CURRENT_PATCH}" \
      2>/dev/null; then
    info "Removing previously applied patch $(basename "${CURRENT_PATCH}")"
    mapfile -t PATCHED_FILES < <(dirty_paths)
    git -C "${LIBFRANKA}" reset --hard --quiet
    git -C "${LIBFRANKA}" clean -fdq -- "${PATCHED_FILES[@]}"
  elif [[ ${FORCE} -eq 1 ]]; then
    warn "Discarding all local changes in ${LIBFRANKA} (--force)."
    git -C "${LIBFRANKA}" reset --hard --quiet
    git -C "${LIBFRANKA}" clean -fdq
  else
    git -C "${LIBFRANKA}" status --short >&2
    die "The libfranka submodule has changes that are not a Diana arm patch (listed above).
       Save them, then re-run with --force to discard them."
  fi
  [[ -z "$(dirty_paths)" ]] || die "Could not restore a clean libfranka submodule."
fi

# 2. Check out the overlay tag for the requested arm and its pinned libfranka.
if [[ ${FETCH} -eq 1 ]]; then
  info "Fetching overlay tags"
  git -C "${OVERLAY}" fetch --tags --quiet origin ||
    warn "git fetch failed; continuing with locally available tags."
fi
git -C "${OVERLAY}" rev-parse --verify --quiet "refs/tags/${TAG}" >/dev/null ||
  die "Tag '${TAG}' not found in ${OVERLAY}."

info "Checking out ${TAG}"
git -C "${OVERLAY}" -c advice.detachedHead=false checkout --quiet --detach "refs/tags/${TAG}"
git -C "${OVERLAY}" submodule update --init --recursive --quiet

# 3. Apply the arm patch.
PATCH="${OVERLAY}/${PATCH_REL}"
[[ -f "${PATCH}" ]] || die "Patch '${PATCH_REL}' not found in ${TAG}."
git -C "${LIBFRANKA}" apply --check --whitespace=nowarn "${PATCH}" ||
  die "Patch '${PATCH_REL}' does not apply to libfranka at $(git -C "${LIBFRANKA}" rev-parse --short HEAD)."
info "Applying ${PATCH_REL}"
git -C "${LIBFRANKA}" apply --whitespace=nowarn "${PATCH}"

# 4. Verify.
DETECTED="$(detect_arm "${LIBFRANKA}")"
[[ "${DETECTED}" == "${ARM}" ]] ||
  die "Verification failed: rate_limiting.h is patched for '${DETECTED}', expected '${ARM}'."
info "libfranka is now patched for: ${ARM}"

if [[ -e "${REPO_ROOT}/libfranka" && "$(cd "${REPO_ROOT}/libfranka" && pwd -P)" != "${LIBFRANKA}" ]]; then
  warn "'${REPO_ROOT}/libfranka' also exists. colcon will find two 'libfranka' packages;
         remove it before building."
fi

cat <<EOF

Next steps (from the workspace root):

  # Rebuild libfranka and every package that depends on it:
  colcon build --symlink-install --packages-above libfranka --cmake-clean-first \\
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
  source install/setup.bash

  On a fresh workspace, run the normal full 'colcon build' instead.
  Launch with robot_type:=${ARM}. libfranka does not check that robot_type
  matches the arm it was built for.
EOF
