#!/usr/bin/env python3
"""
Spectral product: one spectrometer reading reduced to what mission control needs.

Pure numpy, no ROS. build() takes one raw spectrum (wavelengths, counts, integration time)
and the laser-ranger samples from the same half second, and returns a small JSON-ready
dict: the spectrum on a 1 nm grid, which bands clipped, a few band ratios, the range to
the target and the size of the patch the spectrometer averaged over. The keyframe
publisher posts it to DeepGIS with the camera frame taken at the same moment.

What the numbers are, and are not:

- Counts are raw detector counts above the baseline, not radiance and not reflectance.
  The instrument's own response (grating and detector efficiency) is still in them, so
  the ratios are only comparable with other readings from this instrument.
- The baseline is the median of 200-280 nm. No sunlight reaches the ground below about
  290 nm, so that band holds only the electronic offset and stray light.
- A band that has a clipped pixel yields no ratio: a clipped band has no shape left.
- The spectrometer has no fore-optic. It averages over a cone (fov_deg), so the reading
  belongs to a patch whose width grows with range, not to a point.
"""

import math

import numpy as np

CLIP_COUNTS = 64500.0          # the ADC tops out at 65535; the response bends just below
GRID_START_NM = 300.0
GRID_STEP_NM = 1.0
MIN_BAND_COUNTS = 150.0        # a band dimmer than this above the baseline is noise
LASER_MIN_M = 0.05
LASER_NO_RETURN_M = 129.9      # the ranger reports 130.0 when nothing comes back

# What the ratios usually are on this instrument, in raw counts, from the unclipped
# readings of its four recorded drives (2026-05-05 twice, 2026-10-07, 2026-10-08).
# NIR/red index, 1074 readings: median -0.38, quartiles -0.49 and -0.17, 5% below -0.66
# and 5% above +0.14. Parked with a leafy tree filling the middle of the view it read
# +0.05. Blue/red, 702 readings: median 1.09, quartiles 0.89 and 1.28, 5% below 0.54 and
# 5% above 1.71. The two thresholds on each are judgement, not calibration: nobody has
# yet pointed the instrument at labelled targets.
NIR_RED_MEDIAN = -0.4
NIR_RED_TOP = 0.14             # 5% of readings are above this
NIR_RED_ABOVE = -0.30
NIR_RED_HIGH = -0.10
BLUE_RED_MEDIAN = 1.1
BLUE_RED_LOW = 0.8
BLUE_RED_HIGH = 1.6

BANDS = {
    'blue': (450.0, 480.0),
    'green': (540.0, 570.0),
    'red': (650.0, 680.0),
    'red_edge_low': (670.0, 685.0),
    'red_edge_high': (740.0, 750.0),
    'nir': (780.0, 800.0),
    'o2a': (759.5, 762.5),
    'o2a_left': (750.0, 756.0),
    'o2a_right': (771.0, 777.0),
}


def _band(wl, d, clipped, lo, hi):
    """Mean counts above baseline in [lo, hi] nm, or None if clipped, missing or too dim."""
    m = (wl >= lo) & (wl <= hi)
    if not m.any() or clipped[m].any():
        return None
    v = float(d[m].mean())
    return v if v >= MIN_BAND_COUNTS else None


def _spans(wl_grid, flags, step):
    """[[lo, hi], ...] in nm for runs of True in flags."""
    out = []
    start = None
    for i, f in enumerate(flags):
        if f and start is None:
            start = i
        if start is not None and (not f or i == len(flags) - 1):
            end = i if f else i - 1
            out.append([round(float(wl_grid[start]), 1), round(float(wl_grid[end] + step), 1)])
            start = None
    return out


def laser_summary(samples):
    """samples: ranges in metres taken during the spectrum's integration. None if there are none."""
    a = np.asarray(list(samples), dtype=float)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return None
    good = a[(a > LASER_MIN_M) & (a < LASER_NO_RETURN_M)]
    out = {'n': int(a.size), 'valid': round(float(good.size / a.size), 2)}
    if good.size:
        out.update(range_m=round(float(np.median(good)), 2), min_m=round(float(good.min()), 2),
                   max_m=round(float(good.max()), 2))
    return out


def build(wl, counts, integration_us, laser_samples=(), fov_deg=25.0, boresight=(0.5, 0.5),
          camera_hfov_deg=0.0):
    """The product dict for one spectrum, or None if the spectrum is unusable."""
    wl = np.asarray(wl, dtype=float)
    counts = np.asarray(counts, dtype=float)
    if wl.size < 64 or wl.size != counts.size or not np.isfinite(counts).all():
        return None
    clipped = counts >= CLIP_COUNTS
    dark = (wl >= 200.0) & (wl <= 280.0)
    baseline = float(np.median(counts[dark])) if dark.sum() >= 20 else float(np.percentile(counts, 2))
    d = counts - baseline
    full_scale = CLIP_COUNTS - baseline

    # 1 nm grid from 300 nm: below that there is no daylight to plot
    start = max(GRID_START_NM, math.ceil(wl[0]))
    n = int((wl[-1] - start) // GRID_STEP_NM)
    if n < 32:
        return None
    edges = start + GRID_STEP_NM * np.arange(n + 1)
    idx = np.digitize(wl, edges) - 1
    ok = (idx >= 0) & (idx < n)
    sums = np.bincount(idx[ok], weights=d[ok], minlength=n)
    hits = np.bincount(idx[ok], minlength=n)
    grid = np.where(hits > 0, sums / np.maximum(hits, 1), np.nan)
    if np.isnan(grid).any():       # a bin wider than the pixel pitch never happens on the Flame; be safe
        good = ~np.isnan(grid)
        grid = np.interp(np.arange(n), np.flatnonzero(good), grid[good])
    grid_clipped = np.bincount(idx[ok], weights=clipped[ok].astype(float), minlength=n) > 0

    b = {name: _band(wl, d, clipped, lo, hi) for name, (lo, hi) in BANDS.items()}
    indices = {}
    if b['nir'] is not None and b['red'] is not None:
        indices['nir_red'] = round((b['nir'] - b['red']) / (b['nir'] + b['red']), 3)
    if b['red_edge_high'] is not None and b['red_edge_low'] is not None:
        indices['red_edge'] = round(b['red_edge_high'] / b['red_edge_low'], 3)
    if b['blue'] is not None and b['red'] is not None:
        indices['blue_red'] = round(b['blue'] / b['red'], 3)
    if b['green'] is not None and b['red'] is not None:
        indices['green_red'] = round(b['green'] / b['red'], 3)
    if None not in (b['o2a'], b['o2a_left'], b['o2a_right']):
        indices['o2a'] = round(b['o2a'] / (0.5 * (b['o2a_left'] + b['o2a_right'])), 3)

    visible = (wl >= 400.0) & (wl <= 700.0)
    product = {
        'wl0': float(start),
        'dwl': GRID_STEP_NM,
        'counts': [int(round(v)) for v in grid],
        'full_scale': int(round(full_scale)),
        'baseline': int(round(baseline)),
        'integration_ms': round(float(integration_us) / 1000.0, 2),
        'clipped': _spans(edges[:-1], grid_clipped, GRID_STEP_NM),
        'clipped_pct': round(100.0 * float(clipped[visible].mean()), 1) if visible.any() else 0.0,
        'peak_frac': round(float(np.clip(d.max() / full_scale, 0.0, 1.0)), 3),
        'indices': indices,
    }
    laser = laser_summary(laser_samples)
    footprint = {'u': round(float(boresight[0]), 3), 'v': round(float(boresight[1]), 3), 'fov_deg': float(fov_deg)}
    if camera_hfov_deg and camera_hfov_deg > 0:
        # how wide the cone is in the picture, as a share of the image width
        footprint['frac_w'] = round(math.tan(math.radians(fov_deg) / 2) / math.tan(math.radians(camera_hfov_deg) / 2), 3)
    if laser:
        product['laser'] = laser
        if 'range_m' in laser:
            footprint['diameter_m'] = round(2.0 * laser['range_m'] * math.tan(math.radians(fov_deg) / 2), 2)
    product['footprint'] = footprint
    product['notes'] = notes(product)
    return product


def notes(p):
    """Short plain sentences on what the numbers mean for this instrument, most useful first."""
    out = []
    ind = p.get('indices', {})
    if p['clipped_pct'] >= 1.0:
        spans = ', '.join('%d-%d nm' % (lo, hi) for lo, hi in p['clipped'][:3])
        out.append('Clipped at %s (%.0f%% of the visible range): %g ms integration is too long for this scene, '
                   'and ratios from clipped bands are left out.' % (spans, p['clipped_pct'], p['integration_ms']))
    elif p['peak_frac'] < 0.1:
        out.append('Dim: the peak is %.0f%% of full scale, so the ratios are noisy.' % (100 * p['peak_frac']))
    v = ind.get('nir_red')
    if v is not None:
        v = round(v, 2) + 0.0      # so that a tiny negative is not printed as -0.00
        if v >= NIR_RED_HIGH:
            what = 'high: a strong red-edge step, so green vegetation is a large part of the patch'
        elif v >= NIR_RED_ABOVE:
            what = 'above usual: a moderate red-edge step, so some green vegetation is probably in the patch'
        else:
            what = 'usual or below: no clear red-edge step, so no sign of much green vegetation in the patch'
        out.append('NIR/red index %+.2f, %s (this instrument reads about %.1f on a typical drive; 5%% of its '
                   'readings are above %+.2f).' % (v, what, NIR_RED_MEDIAN, NIR_RED_TOP))
    v = ind.get('blue_red')
    if v is not None:
        if v >= BLUE_RED_HIGH:
            out.append('Blue/red %.2f, well above the usual %.1f: more blue than usual, as sky or sky-lit shade '
                       'in the patch gives.' % (v, BLUE_RED_MEDIAN))
        elif v <= BLUE_RED_LOW:
            out.append('Blue/red %.2f, below the usual %.1f: less blue than usual, as soil, bark, brick and '
                       'leaves give.' % (v, BLUE_RED_MEDIAN))
    v = ind.get('o2a')
    if v is not None and v > 0.85:
        out.append('The oxygen band at 761 nm is nearly absent (%.2f): this light is not daylight.' % v)
    laser = p.get('laser')
    fp = p.get('footprint', {})
    if laser and 'range_m' in laser:
        text = 'Laser range %.1f m' % laser['range_m']
        if laser['max_m'] - laser['min_m'] > 0.25 * laser['range_m']:
            text += ' (it moved between %.1f and %.1f m during the reading, so the patch mixes near and far surfaces)' % (
                laser['min_m'], laser['max_m'])
        if 'diameter_m' in fp:
            text += '; with no fore-optic the spectrometer averaged a patch about %.1f m across there' % fp['diameter_m']
        out.append(text + '.')
    elif laser:
        out.append('The laser got no return: the target is beyond its range or is open sky.')
    return out[:5]
