"""Run directory naming: <profile>_run<N>_<date>_<time>, shared by a flight's
experiment recording and its SLAM upload."""
import json
import sys
import time
from datetime import datetime, timedelta
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_naming import name_for_upload, next_run_name, profile_label  # noqa: E402


def test_profile_labels():
    assert profile_label('8x8_15') == '8x8_15hz'
    assert profile_label('4x4_30') == '4x4_30hz'
    assert profile_label('off') == 'tofoff'
    assert profile_label(columns=4, hz=30) == '4x4_30hz'
    assert profile_label(columns=8, hz=None) == '8x8_15hz'   # v1 uploads carry no rate


def test_numbering_is_per_profile_per_day(tmp_path):
    exp, slam = tmp_path / 'experiments', tmp_path / 'slam'
    exp.mkdir(); slam.mkdir()
    when = datetime(2026, 11, 8, 16, 30, 5)
    assert next_run_name('8x8_15hz', when, [exp, slam]) == '8x8_15hz_run1_20261108_163005'
    (exp / '8x8_15hz_run1_20261108_150000').mkdir()
    (slam / '8x8_15hz_run2_20261108_151000').mkdir()
    (exp / '4x4_30hz_run7_20261108_152000').mkdir()
    (exp / '8x8_15hz_run9_20261107_120000').mkdir()   # other day
    assert next_run_name('8x8_15hz', when, [exp, slam]) == '8x8_15hz_run3_20261108_163005'
    assert next_run_name('4x4_30hz', when, [exp, slam]) == '4x4_30hz_run8_20261108_163005'


def test_upload_takes_its_experiments_name(tmp_path):
    exp, slam = tmp_path / 'experiments', tmp_path / 'slam'
    slam.mkdir()
    started = datetime.now() - timedelta(minutes=3)
    flight = exp / f'4x4_30hz_run2_{started:%Y%m%d_%H%M%S}'
    flight.mkdir(parents=True)
    (flight / 'run.json').write_text(json.dumps({'started_unix_ns': int(started.timestamp() * 1e9)}))
    assert name_for_upload('4x4_30hz', datetime.now(), exp, slam) == flight.name
    # an 8x8 upload does not take the 4x4 flight's name
    assert name_for_upload('8x8_15hz', datetime.now(), exp, slam).startswith('8x8_15hz_run1_')
    # once that flight has its SLAM result, the next upload gets a new number
    (slam / flight.name).mkdir()
    assert name_for_upload('4x4_30hz', datetime.now(), exp, slam).startswith('4x4_30hz_run3_')
