#!/usr/bin/env python3
"""
Read spectrometer spectra from a ROS 2 rosbag and save a time–wavelength heatmap.

Expects `std_msgs/msg/Float64MultiArray` payloads in the same layout as
`spectrometery_ros2` publishes: [integration_time_us, intensities..., wavelengths...].

Time axis uses the bag recorder timestamp (message has no header).

Usage:
  source /opt/ros/jazzy/setup.bash
  python3 ~/earth-rover/scripts/analyze_spectrometer_rosbag.py \\
    --bag ~/earth-rover-bags/rosbag2_2026_05_02-06_56_48

  python3 ... --bag /path/to/bag --topic /spectrometer --output spectra.png
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:
    print(f'Error: {exc}', file=sys.stderr)
    print(
        'Source a ROS 2 environment that provides rosbag2_py and rclpy, e.g.\n'
        '  source /opt/ros/jazzy/setup.bash',
        file=sys.stderr,
    )
    sys.exit(1)


def _storage_id_from_metadata(bag_dir: Path) -> Optional[str]:
    meta = bag_dir / 'metadata.yaml'
    if not meta.is_file():
        return None
    text = meta.read_text(encoding='utf-8', errors='replace')
    m = re.search(
        r'storage_identifier:\s*\n\s*name:\s*(\S+)',
        text,
    )
    if m:
        return m.group(1).strip('"').strip("'")
    return None


def _open_reader(bag_path: Path) -> Tuple[rosbag2_py.SequentialReader, str, str]:
    """Match ``deepgis_rosbag_injector`` storage heuristics; return (reader, id, uri)."""
    reader = rosbag2_py.SequentialReader()
    if bag_path.is_file():
        if str(bag_path).endswith('.mcap'):
            storage_id = 'mcap'
        else:
            storage_id = 'sqlite3'
        storage_uri = str(bag_path.parent)
    else:
        mcap_files = list(bag_path.glob('*.mcap'))
        db3_files = list(bag_path.glob('*.db3'))
        meta_sid = _storage_id_from_metadata(bag_path)
        if meta_sid:
            storage_id = meta_sid
        elif mcap_files:
            storage_id = 'mcap'
        elif db3_files:
            storage_id = 'sqlite3'
        else:
            storage_id = 'mcap'
        storage_uri = str(bag_path)

    storage_options = rosbag2_py.StorageOptions(uri=storage_uri, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)
    return reader, storage_id, storage_uri


# Public re-exports for sibling scripts (e.g. geoplot_rosbag.py) that just want
# the rosbag opener / topic resolver without depending on the spectrometer
# message layout.
def open_bag_reader(bag_path: Path) -> Tuple[rosbag2_py.SequentialReader, str, str]:
    """Open a rosbag2 directory or single mcap/db3 file. Returns (reader, storage_id, uri)."""
    return _open_reader_fallback(bag_path)


def resolve_topic_name(type_map: Dict[str, str], requested: str) -> Optional[str]:
    """Best-effort topic resolution against a {topic_name: type_str} map."""
    return _resolve_topic(type_map, requested)


def _open_reader_fallback(bag_path: Path) -> Tuple[rosbag2_py.SequentialReader, str, str]:
    """Try primary open; on failure flip mcap/sqlite3 for directory bags."""
    try:
        return _open_reader(bag_path)
    except Exception:
        if not bag_path.is_dir():
            raise
        for storage_id in ('sqlite3', 'mcap'):
            try:
                reader = rosbag2_py.SequentialReader()
                reader.open(
                    rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
                    rosbag2_py.ConverterOptions(
                        input_serialization_format='cdr',
                        output_serialization_format='cdr',
                    ),
                )
                return reader, storage_id, str(bag_path)
            except Exception:
                continue
        raise


def _normalize_topic_name(name: str) -> str:
    return '/' + name.lstrip('/')


def _resolve_topic(type_map: Dict[str, str], requested: str) -> Optional[str]:
    candidates = [
        requested,
        _normalize_topic_name(requested),
        requested.lstrip('/'),
    ]
    for c in candidates:
        if c in type_map:
            return c
    for name in type_map:
        if name.rstrip('/').split('/')[-1] == 'spectrometer':
            return name
    return None


def _parse_spectrum_row(
    data: List[float],
) -> Optional[Tuple[float, np.ndarray, np.ndarray]]:
    """Match spectrometery_ros2 Intensity_Plot layout."""
    if len(data) < 3:
        return None
    half = (len(data) - 1) // 2
    if half < 1:
        return None
    integration_us = float(data[0])
    intensities = np.asarray(data[1 : half + 1], dtype=np.float64)
    wavelengths = np.asarray(data[half + 1 :], dtype=np.float64)
    if intensities.shape != wavelengths.shape or intensities.size == 0:
        return None
    return integration_us, intensities, wavelengths


class TopicNotFoundError(RuntimeError):
    """Raised when the requested spectrometer topic isn't in the bag."""


def load_spectra_from_bag(
    bag_path: Path,
    topic: str = '/spectrometer',
    time_stride: int = 1,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, str, Dict[str, object]]:
    """Read spectrometer ``Float64MultiArray`` messages from a rosbag2.

    Returns
    -------
    times_s : (N,) float64
        Seconds since the first kept message (recorder timestamp; the message
        itself has no header).
    wavelengths : (W,) float64
        Wavelength axis taken from the first valid message, in nm.
    intensities : (N, W) float64
        Per-sample spectra in the same row order as ``times_s``.
    resolved_topic : str
        Topic name actually used in the bag (after fuzzy resolution).
    stats : dict
        ``storage_id``, ``storage_uri``, ``n_read``, ``n_skipped``.
    """
    reader, storage_id, storage_uri = _open_reader_fallback(bag_path)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    resolved = _resolve_topic(type_map, topic)
    if resolved is None:
        topics = '\n  '.join(f'{n}  ({type_map[n]})' for n in sorted(type_map))
        raise TopicNotFoundError(
            f'Topic {topic!r} not found. Topics in bag:\n  {topics}'
        )

    msg_type_name = type_map[resolved]
    if 'Float64MultiArray' not in msg_type_name:
        print(
            f'Warning: topic type is {msg_type_name!r} (expected std_msgs/Float64MultiArray).',
            file=sys.stderr,
        )

    filt = rosbag2_py.StorageFilter(topics=[resolved])
    reader.set_filter(filt)

    msg_cls = get_message(msg_type_name)

    times_ns: List[int] = []
    intensities_rows: List[np.ndarray] = []
    wavelengths_ref: Optional[np.ndarray] = None
    skipped = 0
    n_read = 0

    while reader.has_next():
        _tname, data, t_rec = reader.read_next()
        n_read += 1
        if time_stride > 1 and (n_read - 1) % time_stride != 0:
            continue
        msg = deserialize_message(data, msg_cls)
        row = _parse_spectrum_row(list(msg.data))
        if row is None:
            skipped += 1
            continue
        _it, intensities, wavelengths = row
        if wavelengths_ref is None:
            wavelengths_ref = wavelengths.copy()
        elif not np.allclose(wavelengths, wavelengths_ref, rtol=0, atol=1e-3):
            skipped += 1
            continue
        if intensities.shape != wavelengths_ref.shape:
            skipped += 1
            continue
        times_ns.append(int(t_rec))
        intensities_rows.append(intensities)

    if not times_ns:
        raise RuntimeError(
            f'No usable spectrometer samples (messages scanned={n_read}, skipped={skipped}).'
        )

    t_ns = np.asarray(times_ns, dtype=np.int64)
    t0 = int(t_ns.min())
    times_s = (t_ns - t0).astype(np.float64) * 1e-9
    wl = np.asarray(wavelengths_ref, dtype=np.float64)
    i_mat = np.stack(intensities_rows, axis=0)
    stats = {
        'storage_id': storage_id,
        'storage_uri': storage_uri,
        'n_read': n_read,
        'n_skipped': skipped,
        'times_ns': t_ns,
        't0_ns': t0,
    }
    return times_s, wl, i_mat, resolved, stats


def analyze_bag(
    bag_path: Path,
    topic: str,
    output: Path,
    time_stride: int,
    percentile_clip: Tuple[float, float],
    cmap: str,
    figsize: Tuple[float, float],
    dpi: float,
    title: Optional[str],
) -> None:
    try:
        times_s, wl, i_mat, resolved, stats = load_spectra_from_bag(
            bag_path=bag_path, topic=topic, time_stride=time_stride,
        )
    except TopicNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(2)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(3)

    print(
        f'Opened bag: uri={stats["storage_uri"]!r} storage_id={stats["storage_id"]!r}'
    )
    print(f'Using topic: {resolved}')

    skipped = int(stats['n_skipped'])

    lo_p, hi_p = percentile_clip
    vmin = float(np.percentile(i_mat, lo_p))
    vmax = float(np.percentile(i_mat, hi_p))
    if vmax <= vmin:
        vmax = vmin + 1.0

    time_grid, wl_grid = np.meshgrid(times_s, wl, indexing='ij')

    fig, ax = plt.subplots(figsize=figsize, dpi=dpi)
    pcm = ax.pcolormesh(
        wl_grid,
        time_grid,
        i_mat,
        shading='nearest',
        cmap=cmap,
        vmin=vmin,
        vmax=vmax,
    )
    cbar = fig.colorbar(pcm, ax=ax, label='Intensity (counts)')
    cbar.ax.tick_params(labelsize=9)
    ax.set_xlabel('Wavelength (nm)')
    ax.set_ylabel('Time (s from first sample)')
    ax.set_title(
        title
        or f'Spectrometer spectra — {bag_path.name} ({len(times_s)} samples, topic {resolved})',
    )
    fig.tight_layout()
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)
    print(
        f'Wrote {output}  ({len(times_s)} spectra × {wl.size} channels, '
        f'skipped={skipped}, stride={time_stride})',
    )


def main() -> None:
    p = argparse.ArgumentParser(
        description='Plot spectrometer Float64MultiArray spectra from a rosbag as a time–wavelength image.',
    )
    p.add_argument(
        '--bag',
        '-b',
        type=Path,
        required=True,
        help='Rosbag directory (e.g. rosbag2_...) or parent path used by rosbag2',
    )
    p.add_argument(
        '--topic',
        '-t',
        default='/spectrometer',
        help='Topic name (default: /spectrometer)',
    )
    p.add_argument(
        '--output',
        '-o',
        type=Path,
        default=None,
        help='Output image path (default: <bag_dir>_spectra_timeseries.png next to bag)',
    )
    p.add_argument(
        '--stride',
        type=int,
        default=1,
        help='Keep every N-th message for lighter plots (default: 1)',
    )
    p.add_argument(
        '--clip-lo',
        type=float,
        default=2.0,
        help='Lower percentile for color scale (default: 2)',
    )
    p.add_argument(
        '--clip-hi',
        type=float,
        default=98.0,
        help='Upper percentile for color scale (default: 98)',
    )
    p.add_argument('--cmap', default='viridis', help='Matplotlib colormap name')
    p.add_argument(
        '--figsize',
        nargs=2,
        type=float,
        default=(12.0, 7.0),
        metavar=('W', 'H'),
        help='Figure size in inches',
    )
    p.add_argument('--dpi', type=float, default=150.0, help='Figure DPI')
    p.add_argument('--title', default=None, help='Override plot title')
    args = p.parse_args()

    bag_path = args.bag.expanduser()
    if not bag_path.exists():
        print(f'Path does not exist: {bag_path}', file=sys.stderr)
        sys.exit(1)

    if args.output is None:
        stem = bag_path.name if bag_path.is_dir() else bag_path.parent.name
        out = bag_path.parent / f'{stem}_spectra_timeseries.png'
    else:
        out = args.output

    if args.stride < 1:
        print('--stride must be >= 1', file=sys.stderr)
        sys.exit(1)

    analyze_bag(
        bag_path=bag_path,
        topic=args.topic,
        output=out,
        time_stride=args.stride,
        percentile_clip=(args.clip_lo, args.clip_hi),
        cmap=args.cmap,
        figsize=(args.figsize[0], args.figsize[1]),
        dpi=args.dpi,
        title=args.title,
    )


if __name__ == '__main__':
    main()
