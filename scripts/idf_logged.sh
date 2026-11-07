#!/usr/bin/env bash

set -uo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/idf_logged.sh <idf.py arguments...>

Examples:
  ./scripts/idf_logged.sh build
  ./scripts/idf_logged.sh -p /dev/ttyACM0 flash monitor
  ./scripts/idf_logged.sh -p /dev/ttyACM0 monitor

Build logs are saved under run-logs/builds/ and monitor logs under
run-logs/hardware/. A sibling .evidence directory records the
exact Git state, tracked patch, untracked files, configuration, and artifact
hashes. Press Ctrl+] to leave the IDF monitor.
EOF
}

while (($# > 0)); do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done

if (($# == 0)); then
    usage >&2
    exit 2
fi

if ! command -v idf.py >/dev/null 2>&1; then
    echo "error: idf.py is not available; source the ESP-IDF export.sh first" >&2
    exit 127
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
timestamp="$(date '+%Y-%m-%d_%H-%M-%S')"

has_build=false
has_monitor=false
has_flash=false
for argument in "$@"; do
    case "$argument" in
        build) has_build=true ;;
        monitor) has_monitor=true ;;
        flash) has_flash=true ;;
    esac
done

if [[ "$has_monitor" == true ]]; then
    log_subdir="hardware"
    if [[ "$has_flash" == true ]]; then
        operation="flash-monitor"
    else
        operation="monitor"
    fi
elif [[ "$has_build" == true ]]; then
    log_subdir="builds"
    operation="build"
else
    log_subdir="idf-commands"
    operation="idf-command"
fi

log_dir="$project_dir/run-logs/$log_subdir"
log_file="$log_dir/${timestamp}-${operation}.log"
evidence_dir="$log_dir/${timestamp}-${operation}.evidence"
mkdir -p -- "$log_dir"
mkdir -p -- "$evidence_dir"
chmod 700 "$evidence_dir"

# Resolve the selected build/config paths for isolated experiment evidence.
artifact_dir="$project_dir/build"
active_config="$project_dir/sdkconfig"
previous_argument=""
for argument in "$@"; do
    if [[ "$previous_argument" == "-B" || "$previous_argument" == "--build-dir" ]]; then
        artifact_dir="$argument"
    elif [[ "$argument" == --build-dir=* ]]; then
        artifact_dir="${argument#*=}"
    elif [[ "$argument" == SDKCONFIG=* || "$argument" == -DSDKCONFIG=* ]]; then
        active_config="${argument#*=}"
    fi
    previous_argument="$argument"
done
[[ "$artifact_dir" == /* ]] || artifact_dir="$project_dir/$artifact_dir"
[[ "$active_config" == /* ]] || active_config="$project_dir/$active_config"

git_revision="$(git -C "$project_dir" rev-parse --short HEAD 2>/dev/null || printf 'unavailable')"
git_revision_full="$(git -C "$project_dir" rev-parse HEAD 2>/dev/null || printf 'unavailable')"
git_state="clean"
if [[ -n "$(git -C "$project_dir" status --porcelain 2>/dev/null)" ]]; then
    git_state="dirty"
fi

git -C "$project_dir" status --short >"$evidence_dir/git-status.txt"
git -C "$project_dir" diff --binary HEAD >"$evidence_dir/working-tree.patch"
git -C "$project_dir" ls-files --others --exclude-standard \
    >"$evidence_dir/untracked-files.txt"

mapfile -t untracked_files <"$evidence_dir/untracked-files.txt"
if ((${#untracked_files[@]} > 0)); then
    tar -C "$project_dir" -czf "$evidence_dir/untracked-files.tar.gz" \
        -- "${untracked_files[@]}"
fi

if [[ -f "$active_config" ]]; then
    install -m 600 "$active_config" "$evidence_dir/sdkconfig"
fi
for config_file in sdkconfig.defaults dependencies.lock partitions.csv; do
    if [[ -f "$project_dir/$config_file" ]]; then
        install -m 600 "$project_dir/$config_file" \
            "$evidence_dir/$config_file"
    fi
done

{
    printf 'UREX reproducibility snapshot\n'
    printf 'Captured: %s\n' "$(date --iso-8601=seconds)"
    printf 'Git revision: %s\n' "$git_revision_full"
    printf 'Git state: %s\n' "$git_state"
    printf 'Working directory: %s\n' "$project_dir"
    printf 'ESP-IDF: '
    idf.py --version 2>&1
    printf 'Command:'
    printf ' %q' idf.py "$@"
    printf '\n'
} >"$evidence_dir/manifest.txt"

(
    cd -- "$evidence_dir"
    sha256sum manifest.txt git-status.txt working-tree.patch \
        untracked-files.txt >snapshot-files.sha256
    for snapshot_file in sdkconfig sdkconfig.defaults dependencies.lock \
        partitions.csv untracked-files.tar.gz; do
        if [[ -f "$snapshot_file" ]]; then
            sha256sum "$snapshot_file" >>snapshot-files.sha256
        fi
    done
)

{
    printf 'UREX ESP-IDF terminal log\n'
    printf 'Started: %s\n' "$(date --iso-8601=seconds)"
    printf 'Git revision: %s (%s)\n' "$git_revision" "$git_state"
    printf 'Evidence: %s\n' "$evidence_dir"
    printf 'Working directory: %s\n' "$project_dir"
    printf 'Command:'
    printf ' %q' idf.py "$@"
    printf '\n\n'
} | tee "$log_file"

cd -- "$project_dir"
if idf.py "$@" 2>&1 | tee -a "$log_file"; then
    command_status=0
else
    command_status=${PIPESTATUS[0]}
fi

artifact_hash_file="$evidence_dir/build-artifacts.sha256"
: >"$artifact_hash_file"
for artifact in \
    esp-everything.elf \
    esp-everything.bin \
    bootloader/bootloader.bin \
    partition_table/partition-table.bin; do
    if [[ -f "$artifact_dir/$artifact" ]]; then
        sha256sum "$artifact_dir/$artifact" >>"$artifact_hash_file"
    fi
done

{
    printf '\nFinished: %s\n' "$(date --iso-8601=seconds)"
    printf 'Exit status: %d\n' "$command_status"
    printf 'Log file: %s\n' "$log_file"
    printf 'Evidence directory: %s\n' "$evidence_dir"
} | tee -a "$log_file"

exit "$command_status"
