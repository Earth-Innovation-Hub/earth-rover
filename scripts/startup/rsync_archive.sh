#!/bin/bash
# Earth Rover  --  archive rsync of "settled" files (yesterday and earlier)
# from the Extreme SSD to the 22TB NAS.
#
# Why this shape?
#   - Running on every boot copies new "yesterday or earlier" data to the NAS
#     without ever racing with files actively being written today.
#   - Files modified today are deliberately skipped; they'll be picked up on
#     the next boot (or by a daily timer pass).
#   - rsync --partial + --append-verify resumes interrupted files cleanly.
#   - Low ionice/nice priority so it never starves the rover stack.
#
# Inputs (all overridable via environment variables):
#   RSYNC_ARCHIVE_SRC    -- source mountpoint (default: /media/jdas/Extreme_SSD)
#   RSYNC_ARCHIVE_USER   -- ssh user on the NAS  (default: jdas)
#   RSYNC_ARCHIVE_HOST   -- destination host     (default: 192.168.0.232)
#   RSYNC_ARCHIVE_PATH   -- destination path     (default: /mnt/22tb-hdd/ssd-xtreme-2024-2025)
#   RSYNC_ARCHIVE_LOG    -- log file             (default: ~/ssd-xtreme-transfer/rsync-archive.log)
#
# Exit codes:
#   0  success or graceful no-op (mount missing / NAS unreachable / already running)
#   non-zero  rsync hard failure (preserved from rsync)

set -uo pipefail

SRC="${RSYNC_ARCHIVE_SRC:-/media/jdas/Extreme_SSD}"
DEST_HOST="${RSYNC_ARCHIVE_HOST:-192.168.0.232}"
DEST_USER="${RSYNC_ARCHIVE_USER:-jdas}"
DEST_PATH="${RSYNC_ARCHIVE_PATH:-/mnt/22tb-hdd/ssd-xtreme-2024-2025}"
LOG="${RSYNC_ARCHIVE_LOG:-$HOME/ssd-xtreme-transfer/rsync-archive.log}"
LOCK="$HOME/ssd-xtreme-transfer/rsync-archive.lock"

mkdir -p "$(dirname "$LOG")"

stamp() { date '+%Y-%m-%dT%H:%M:%S%z'; }
log() {
    printf '[%s] %s\n' "$(stamp)" "$*" | tee -a "$LOG"
}

# ---- single-instance lock ----------------------------------------------------
exec 9>"$LOCK"
if ! flock -n 9; then
    log "another rsync_archive instance is already running; exiting cleanly."
    exit 0
fi

# ---- preflight ---------------------------------------------------------------
if ! mountpoint -q "$SRC"; then
    log "preflight: source $SRC is not a mountpoint -- nothing to archive."
    exit 0
fi

if ! ssh -o BatchMode=yes -o ConnectTimeout=5 \
        "${DEST_USER}@${DEST_HOST}" \
        "test -d ${DEST_PATH} && echo ok" >/dev/null 2>&1; then
    log "preflight: ${DEST_USER}@${DEST_HOST}:${DEST_PATH} unreachable; skipping run."
    exit 0
fi

# ---- compute "yesterday and earlier" cutoff ---------------------------------
# We want files whose mtime is BEFORE 00:00 today (local time). find's
# `! -newermt 'today 00:00'` matches exactly that set.
CUTOFF="$(date '+%Y-%m-%d') 00:00:00"

# ---- enumerate eligible files into a temp list ------------------------------
LIST="$(mktemp -t rsync-archive.XXXXXX)"
trap 'rm -f "$LIST"' EXIT

log "starting archive scan"
log "  src     = $SRC"
log "  dst     = ${DEST_USER}@${DEST_HOST}:${DEST_PATH}"
log "  cutoff  = files modified strictly before $CUTOFF"

# Generate paths RELATIVE to $SRC so rsync --files-from interprets them
# against the source root.  -print0 + --from0 keeps spaces/specials safe.
( cd "$SRC" && find . \( -type f -o -type l \) ! -newermt "$CUTOFF" -print0 ) > "$LIST"

# Count how many files are eligible (NUL-separated, so `tr | wc`).
N=$(tr -cd '\0' < "$LIST" | wc -c)
log "  eligible files: $N"

if [ "$N" -eq 0 ]; then
    log "nothing to archive (no files older than today). done."
    exit 0
fi

# ---- the actual transfer ----------------------------------------------------
log "rsync starting"
ionice -c2 -n7 nice -n 19 \
    rsync \
        -avh \
        --from0 \
        --files-from="$LIST" \
        --partial \
        --append-verify \
        --info=stats2 \
        --human-readable \
        "$SRC/" \
        "${DEST_USER}@${DEST_HOST}:${DEST_PATH}/" \
        >> "$LOG" 2>&1
rc=$?
log "rsync exited rc=$rc"
exit "$rc"
