"""Run directory names shared by run_experiment.py and nanoslam_receive.py.

Format: <profile>_run<N>_<YYYYmmdd>_<HHMMSS>, e.g. 8x8_15hz_run3_20261108_154646.
<profile> is the ToF profile (8x8_15hz, 4x4_30hz, ... or tofoff). N counts runs of
that profile on that day, across results/experiments and results/slam, starting
at 1. A SLAM upload reuses the name of the experiment recording it belongs to, so
one flight has the same name in both folders.
"""
from __future__ import annotations

import json
import re
from datetime import datetime
from pathlib import Path

NAME_RE = re.compile(r'^(?P<label>[0-9a-z]+(?:_[0-9]+hz)?)_run(?P<n>\d+)_(?P<date>\d{8})_(?P<time>\d{6})')
PAIR_WINDOW_S = 15 * 60  # an upload belongs to an experiment started this recently


def profile_label(profile: str | None = None, *, columns: int | None = None, hz: int | None = None) -> str:
    """'8x8_15' -> '8x8_15hz', 'off' -> 'tofoff'; or from columns/hz."""
    if profile is not None:
        if profile == 'off':
            return 'tofoff'
        side, rate = profile.split('_')
        return f'{side}_{rate}hz'
    columns = columns or 8
    hz = hz or (15 if columns == 8 else 30)
    return f'{columns}x{columns}_{hz}hz'


def _existing(roots) -> list[tuple[str, str, int, Path]]:
    found = []
    for root in roots:
        root = Path(root)
        if not root.is_dir():
            continue
        for entry in root.iterdir():
            m = NAME_RE.match(entry.name)
            if m:
                found.append((m['label'], m['date'], int(m['n']), entry))
    return found


def next_run_name(label: str, when: datetime, roots) -> str:
    date = when.strftime('%Y%m%d')
    numbers = [n for lab, d, n, _ in _existing(roots) if lab == label and d == date]
    return f'{label}_run{max(numbers, default=0) + 1}_{when:%Y%m%d_%H%M%S}'


def experiment_start(directory: Path) -> datetime | None:
    try:
        run = json.loads((directory / 'run.json').read_text())
        return datetime.fromtimestamp(run['started_unix_ns'] / 1e9)
    except (OSError, ValueError, KeyError):
        return None


def name_for_upload(label: str, received: datetime, experiments_root, slam_root) -> str:
    """Name of the newest experiment of the same profile that started within
    PAIR_WINDOW_S before the upload and has no SLAM result yet; otherwise a new
    run name."""
    slam_root, experiments_root = Path(slam_root), Path(experiments_root)
    best = None
    for lab, _, _, path in _existing([experiments_root]):
        if lab != label or (slam_root / path.name).exists():
            continue
        started = experiment_start(path)
        if started is None:
            continue
        age = (received - started).total_seconds()
        if 0 <= age <= PAIR_WINDOW_S and (best is None or started > best[0]):
            best = (started, path.name)
    if best:
        return best[1]
    return next_run_name(label, received, [experiments_root, slam_root])
