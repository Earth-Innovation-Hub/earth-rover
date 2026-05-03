#!/usr/bin/env python3
"""
End-to-end unsupervised report for a spectrometer rosbag.

Pipeline (matches steps 1–4 of the spectroscopy methods discussion):
  1. Wavelength mask + SNV + Savitzky–Golay 1st derivative
  2. PCA (keep components covering ``--pca-variance``, capped at ``--max-pcs``)
  3. UMAP (2D) → HDBSCAN clustering on the embedding
  4. NMF on (clipped, non-negative) raw spectra → endmembers + abundances

Saves individual figures, a 6-panel summary, and a small text report into
``<bag_parent>/<bag_name>_cluster_report/`` (or ``--output-dir``).

Reuses ``load_spectra_from_bag`` from the sibling
``analyze_spectrometer_rosbag.py`` for rosbag2 IO.

Usage:
  source /opt/ros/jazzy/setup.bash
  python3 ~/earth-rover/scripts/cluster_spectrometer_rosbag.py \\
      --bag ~/earth-rover-bags/rosbag2_2026_05_02-06_56_48
"""

from __future__ import annotations

import argparse
import sys
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# --- sibling import (scripts/ is not a package) ------------------------------
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

try:
    from analyze_spectrometer_rosbag import (  # noqa: E402
        TopicNotFoundError,
        load_spectra_from_bag,
    )
except ImportError as exc:
    print(f'Error importing sibling helper: {exc}', file=sys.stderr)
    print(
        'Expected analyze_spectrometer_rosbag.py next to this script.',
        file=sys.stderr,
    )
    sys.exit(1)

# --- science stack -----------------------------------------------------------
try:
    from scipy.signal import savgol_filter
    from sklearn.decomposition import NMF, PCA
except ImportError as exc:
    print(f'Missing required dependency: {exc}', file=sys.stderr)
    print('Install with: pip install --user scikit-learn scipy', file=sys.stderr)
    sys.exit(1)

# Optional reducers / clusterers (graceful fallbacks below).
try:
    import umap as _umap

    HAS_UMAP = True
except ImportError:
    HAS_UMAP = False

_HDBSCAN_BACKEND: Optional[str] = None
_hdbscan_pkg = None
_SkHDBSCAN = None
try:
    import hdbscan as _hdbscan_pkg  # type: ignore

    _HDBSCAN_BACKEND = 'hdbscan'
except ImportError:
    try:
        from sklearn.cluster import HDBSCAN as _SkHDBSCAN  # sklearn >= 1.3

        _HDBSCAN_BACKEND = 'sklearn'
    except ImportError:
        _HDBSCAN_BACKEND = None


# ---- preprocessing ----------------------------------------------------------


def edge_mask(
    wl: np.ndarray, lo: Optional[float], hi: Optional[float]
) -> np.ndarray:
    keep = np.ones_like(wl, dtype=bool)
    if lo is not None:
        keep &= wl >= lo
    if hi is not None:
        keep &= wl <= hi
    return keep


def snv(X: np.ndarray) -> np.ndarray:
    """Standard Normal Variate (per-row z-score)."""
    mu = X.mean(axis=1, keepdims=True)
    sd = X.std(axis=1, keepdims=True)
    sd = np.where(sd < 1e-12, 1.0, sd)
    return (X - mu) / sd


def safe_savgol(
    X: np.ndarray, window: int, poly: int, deriv: int
) -> np.ndarray:
    """Savitzky–Golay along axis=1, with sanity-clamped window/poly."""
    w = max(5, window | 1)  # must be odd and >= 5
    if X.shape[1] < w:
        w = max(5, (X.shape[1] // 2) * 2 + 1)
    p = max(1, min(poly, w - 2))
    return savgol_filter(
        X, window_length=w, polyorder=p, deriv=deriv, axis=1, mode='interp'
    )


# ---- model fitting ----------------------------------------------------------


@dataclass
class PCAResult:
    model: PCA
    scores_full: np.ndarray  # (N, n_max)
    scores_kept: np.ndarray  # (N, n_kept)
    n_kept: int


def fit_pca(
    X: np.ndarray, target_var: float, max_components: int, random_state: int
) -> PCAResult:
    n_max = max(2, min(max_components, X.shape[0] - 1, X.shape[1]))
    pca = PCA(n_components=n_max, random_state=random_state)
    scores = pca.fit_transform(X)
    cum = np.cumsum(pca.explained_variance_ratio_)
    n_kept = int(np.searchsorted(cum, target_var) + 1)
    n_kept = min(max(n_kept, 2), n_max)
    return PCAResult(
        model=pca, scores_full=scores, scores_kept=scores[:, :n_kept], n_kept=n_kept
    )


def project_2d(
    scores: np.ndarray, n_neighbors: int, min_dist: float, random_state: int
) -> Tuple[np.ndarray, str]:
    if HAS_UMAP and scores.shape[0] >= 4:
        nn = max(2, min(n_neighbors, scores.shape[0] - 1))
        reducer = _umap.UMAP(
            n_components=2,
            n_neighbors=nn,
            min_dist=min_dist,
            random_state=random_state,
            metric='euclidean',
        )
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            emb = reducer.fit_transform(scores)
        return emb, 'umap'
    return scores[:, :2], 'pca2d'


def cluster_density(
    emb: np.ndarray, min_cluster_size: int, random_state: int
) -> Tuple[np.ndarray, str]:
    if _HDBSCAN_BACKEND == 'hdbscan':
        clusterer = _hdbscan_pkg.HDBSCAN(min_cluster_size=int(min_cluster_size))
        labels = clusterer.fit_predict(emb)
        return labels, 'hdbscan'
    if _HDBSCAN_BACKEND == 'sklearn':
        clusterer = _SkHDBSCAN(min_cluster_size=int(min_cluster_size))
        labels = clusterer.fit_predict(emb)
        return labels, 'sklearn-hdbscan'

    from sklearn.cluster import KMeans
    from sklearn.metrics import silhouette_score

    best_labels: Optional[np.ndarray] = None
    best_k = 3
    best_s = -1.0
    for k in range(3, 9):
        if emb.shape[0] <= k + 1:
            break
        km = KMeans(n_clusters=k, n_init=10, random_state=random_state)
        labels = km.fit_predict(emb)
        try:
            s = float(silhouette_score(emb, labels))
        except Exception:
            s = -1.0
        if s > best_s:
            best_s, best_k, best_labels = s, k, labels
    if best_labels is None:
        best_labels = np.zeros(emb.shape[0], dtype=int)
    return best_labels, f'kmeans-k{best_k}'


def fit_nmf(
    X_pos: np.ndarray, n_components: int, random_state: int
) -> Tuple[NMF, np.ndarray, np.ndarray]:
    """NMF on non-negative spectra: returns (model, W abundances, H endmembers)."""
    n_components = max(2, min(n_components, min(X_pos.shape) - 1))
    model = NMF(
        n_components=n_components,
        init='nndsvda',
        max_iter=400,
        random_state=random_state,
    )
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        W = model.fit_transform(X_pos)
    H = model.components_
    return model, W, H


@dataclass
class ClusterPipelineResult:
    """Bundle of intermediates produced by ``compute_clusters_and_nmf``."""

    wl_masked: np.ndarray
    X_masked: np.ndarray  # (N, W_kept)
    X_derivative: np.ndarray  # SNV + Savitzky-Golay
    pca: PCAResult
    embedding: np.ndarray  # (N, 2)
    embedding_name: str
    labels: np.ndarray  # (N,) cluster ids; -1 = noise
    cluster_backend: str
    cluster_min_size: int
    nmf_model: NMF
    nmf_abundances: np.ndarray  # W (N, K)
    nmf_endmembers: np.ndarray  # H (K, W_kept)


def compute_clusters_and_nmf(
    X: np.ndarray,
    wl: np.ndarray,
    *,
    mask_lo: Optional[float] = None,
    mask_hi: Optional[float] = None,
    savgol_window: int = 15,
    savgol_poly: int = 3,
    savgol_deriv: int = 1,
    pca_variance: float = 0.99,
    max_pcs: int = 30,
    umap_neighbors: int = 30,
    umap_min_dist: float = 0.05,
    hdbscan_min_cluster: int = 0,
    nmf_components: int = 5,
    random_seed: int = 0,
) -> ClusterPipelineResult:
    """Run the steps 1–4 unsupervised pipeline and return intermediates.

    Mirrors what ``run_report`` does, but without any IO/plotting so other
    scripts (e.g. geoplot overlays) can consume the labels and abundances.
    """
    keep = edge_mask(wl, mask_lo, mask_hi)
    if int(keep.sum()) < 32:
        keep = np.ones_like(wl, dtype=bool)
    wl_m = wl[keep]
    X_m = X[:, keep]
    X_snv = snv(X_m)
    X_d = safe_savgol(X_snv, savgol_window, savgol_poly, savgol_deriv)

    pca_res = fit_pca(X_d, pca_variance, max_pcs, random_seed)
    emb, embed_name = project_2d(
        pca_res.scores_kept, umap_neighbors, umap_min_dist, random_seed,
    )
    if hdbscan_min_cluster <= 0:
        mcs = max(20, X.shape[0] // 100)
    else:
        mcs = int(hdbscan_min_cluster)
    labels, cluster_backend = cluster_density(emb, mcs, random_seed)

    X_pos = np.clip(X_m, 0.0, None)
    nmf_model, W_abund, H_endm = fit_nmf(X_pos, nmf_components, random_seed)

    return ClusterPipelineResult(
        wl_masked=wl_m,
        X_masked=X_m,
        X_derivative=X_d,
        pca=pca_res,
        embedding=emb,
        embedding_name=embed_name,
        labels=labels,
        cluster_backend=cluster_backend,
        cluster_min_size=mcs,
        nmf_model=nmf_model,
        nmf_abundances=W_abund,
        nmf_endmembers=H_endm,
    )


# ---- plotting helpers -------------------------------------------------------


def _cluster_palette() -> "plt.cm.colors.Colormap":  # type: ignore[name-defined]
    return plt.get_cmap('tab20')


def plot_pca_scree(ax, var_ratio: np.ndarray, n_kept: int) -> None:
    idx = np.arange(1, len(var_ratio) + 1)
    ax.bar(idx, var_ratio * 100.0, color='#7a8ab8', alpha=0.8, label='per-PC')
    ax.set_ylabel('Variance (%)')
    ax.set_xlabel('Principal component')
    ax.axvline(n_kept + 0.5, ls='--', color='k', alpha=0.6)
    ax2 = ax.twinx()
    ax2.plot(idx, np.cumsum(var_ratio) * 100.0, 'r-o', ms=3, lw=1.2, label='cumulative')
    ax2.set_ylabel('Cumulative (%)', color='r')
    ax2.tick_params(axis='y', labelcolor='r')
    ax2.set_ylim(0, 105)
    ax.set_title(f'PCA scree (kept {n_kept} PCs)')


def plot_embedding(
    ax, emb: np.ndarray, labels: np.ndarray, embed_name: str
) -> None:
    cmap = _cluster_palette()
    uniq = np.unique(labels)
    for i, lab in enumerate(uniq):
        m = labels == lab
        if lab == -1:
            ax.scatter(
                emb[m, 0], emb[m, 1], s=4, c='lightgray', alpha=0.5, label='noise'
            )
        else:
            ax.scatter(
                emb[m, 0],
                emb[m, 1],
                s=8,
                c=[cmap(int(lab) % 20)],
                alpha=0.85,
                label=f'c{int(lab)} (n={int(m.sum())})',
            )
    ax.set_xlabel(f'{embed_name}-1')
    ax.set_ylabel(f'{embed_name}-2')
    ax.set_title(f'{embed_name.upper()} + clusters')
    ax.legend(fontsize=7, loc='best', markerscale=1.4, ncol=2, framealpha=0.85)
    ax.grid(alpha=0.2)


def plot_cluster_means(
    ax, wl: np.ndarray, X_raw: np.ndarray, labels: np.ndarray
) -> None:
    cmap = _cluster_palette()
    uniq = [int(lab) for lab in np.unique(labels) if lab != -1]
    for i, lab in enumerate(uniq):
        m = labels == lab
        if not np.any(m):
            continue
        mu = X_raw[m].mean(axis=0)
        ax.plot(
            wl,
            mu,
            lw=1.3,
            color=cmap(lab % 20),
            label=f'c{lab} (n={int(m.sum())})',
        )
    if (labels == -1).any():
        mu = X_raw[labels == -1].mean(axis=0)
        ax.plot(
            wl,
            mu,
            lw=1.0,
            color='lightgray',
            ls='--',
            label=f'noise (n={int((labels == -1).sum())})',
        )
    ax.set_xlabel('Wavelength (nm)')
    ax.set_ylabel('Mean intensity (counts)')
    ax.set_title('Cluster mean spectra (raw)')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(alpha=0.3)


def plot_nmf_endmembers(ax, wl: np.ndarray, H: np.ndarray) -> None:
    cmap = plt.get_cmap('Set1')
    for i, h in enumerate(H):
        ax.plot(wl, h, lw=1.4, color=cmap(i), label=f'EM{i}')
    ax.set_xlabel('Wavelength (nm)')
    ax.set_ylabel('Endmember intensity (a.u.)')
    ax.set_title(f'NMF endmembers (K={H.shape[0]})')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)


def plot_nmf_abundances(
    ax, times_s: np.ndarray, W: np.ndarray
) -> None:
    row_sum = W.sum(axis=1, keepdims=True)
    row_sum = np.where(row_sum < 1e-12, 1.0, row_sum)
    A = W / row_sum
    cmap = plt.get_cmap('Set1')
    colors = [cmap(i) for i in range(A.shape[1])]
    ax.stackplot(
        times_s,
        A.T,
        colors=colors,
        labels=[f'EM{i}' for i in range(A.shape[1])],
    )
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Abundance fraction')
    ax.set_title('NMF abundances over time')
    ax.set_ylim(0.0, 1.0)
    if times_s.size > 1:
        ax.set_xlim(float(times_s.min()), float(times_s.max()))
    ax.legend(fontsize=7, loc='upper right', ncol=min(W.shape[1], 3))


def plot_cluster_timeline(
    ax, times_s: np.ndarray, labels: np.ndarray
) -> None:
    cmap = _cluster_palette()
    for lab in np.unique(labels):
        m = labels == lab
        if not np.any(m):
            continue
        c = 'lightgray' if lab == -1 else cmap(int(lab) % 20)
        ax.scatter(
            times_s[m], np.full(int(m.sum()), int(lab)), c=[c], s=8, alpha=0.85
        )
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cluster id (-1 = noise)')
    ax.set_title('Cluster vs time')
    ax.grid(alpha=0.3)


# ---- main pipeline ----------------------------------------------------------


def run_report(args: argparse.Namespace) -> None:
    bag_path = args.bag.expanduser()
    if not bag_path.exists():
        print(f'Path does not exist: {bag_path}', file=sys.stderr)
        sys.exit(1)

    try:
        times_s, wl, X, resolved, stats = load_spectra_from_bag(
            bag_path=bag_path, topic=args.topic, time_stride=args.stride,
        )
    except TopicNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(2)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(3)

    print(
        f'Opened bag: uri={stats["storage_uri"]!r} '
        f'storage_id={stats["storage_id"]!r}'
    )
    print(
        f'Loaded {X.shape[0]} spectra × {X.shape[1]} channels '
        f'from {resolved} (skipped={stats["n_skipped"]}, stride={args.stride})'
    )

    out_stem = bag_path.name if bag_path.is_dir() else bag_path.parent.name
    out_dir = (
        args.output_dir.expanduser()
        if args.output_dir is not None
        else bag_path.parent / f'{out_stem}_cluster_report'
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f'Writing report to: {out_dir}')

    result = compute_clusters_and_nmf(
        X, wl,
        mask_lo=args.mask_lo, mask_hi=args.mask_hi,
        savgol_window=args.savgol_window,
        savgol_poly=args.savgol_poly,
        savgol_deriv=args.savgol_deriv,
        pca_variance=args.pca_variance,
        max_pcs=args.max_pcs,
        umap_neighbors=args.umap_neighbors,
        umap_min_dist=args.umap_min_dist,
        hdbscan_min_cluster=args.hdbscan_min_cluster,
        nmf_components=args.nmf_components,
        random_seed=args.random_seed,
    )
    wl_m = result.wl_masked
    X_m = result.X_masked
    pca_res = result.pca
    var_ratio = pca_res.model.explained_variance_ratio_
    emb = result.embedding
    embed_name = result.embedding_name
    labels = result.labels
    cluster_backend = result.cluster_backend
    mcs = result.cluster_min_size
    nmf_model = result.nmf_model
    W_abund = result.nmf_abundances
    H_endm = result.nmf_endmembers
    n_clusters = int(np.unique(labels[labels >= 0]).size)
    n_noise = int((labels == -1).sum())

    print(
        f'Preprocessed: {X_m.shape[1]} channels in '
        f'[{wl_m.min():.1f}, {wl_m.max():.1f}] nm, '
        f'SNV + savgol(window={args.savgol_window}, poly={args.savgol_poly}, '
        f'deriv={args.savgol_deriv}).'
    )
    print(
        f'PCA kept {pca_res.n_kept} components '
        f'({float(var_ratio[: pca_res.n_kept].sum()) * 100.0:.2f}% variance).'
    )
    print(
        f'Embedding: {embed_name}  Clustering: {cluster_backend}  '
        f'min_cluster_size={mcs}  clusters={n_clusters}  '
        f'noise={n_noise}/{labels.size}'
    )
    print(
        f'NMF: K={H_endm.shape[0]}  '
        f'reconstruction_err={nmf_model.reconstruction_err_:.4g}'
    )

    # ---- figures ------------------------------------------------------------
    dpi = float(args.dpi)

    fig, ax = plt.subplots(figsize=(8, 5), dpi=dpi)
    plot_pca_scree(ax, var_ratio, pca_res.n_kept)
    fig.tight_layout()
    fig.savefig(out_dir / f'{out_stem}_pca_scree.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 6), dpi=dpi)
    plot_embedding(ax, emb, labels, embed_name)
    fig.tight_layout()
    fig.savefig(out_dir / f'{out_stem}_umap_hdbscan.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 5), dpi=dpi)
    plot_cluster_means(ax, wl_m, X_m, labels)
    fig.tight_layout()
    fig.savefig(out_dir / f'{out_stem}_cluster_means.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 5), dpi=dpi)
    plot_nmf_endmembers(ax, wl_m, H_endm)
    fig.tight_layout()
    fig.savefig(out_dir / f'{out_stem}_nmf_endmembers.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4.5), dpi=dpi)
    plot_nmf_abundances(ax, times_s, W_abund)
    fig.tight_layout()
    fig.savefig(out_dir / f'{out_stem}_nmf_abundances.png')
    plt.close(fig)

    fig, axes = plt.subplots(2, 3, figsize=(20, 11), dpi=dpi)
    plot_pca_scree(axes[0, 0], var_ratio, pca_res.n_kept)
    plot_embedding(axes[0, 1], emb, labels, embed_name)
    plot_cluster_means(axes[0, 2], wl_m, X_m, labels)
    plot_nmf_endmembers(axes[1, 0], wl_m, H_endm)
    plot_nmf_abundances(axes[1, 1], times_s, W_abund)
    plot_cluster_timeline(axes[1, 2], times_s, labels)
    fig.suptitle(
        f'{bag_path.name} — {X.shape[0]} spectra, {X_m.shape[1]} channels, '
        f'{n_clusters} clusters, NMF K={H_endm.shape[0]}',
        fontsize=14,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_dir / f'{out_stem}_summary.png')
    plt.close(fig)

    # ---- text report --------------------------------------------------------
    lines: List[str] = []
    lines.append(f'bag: {bag_path}')
    lines.append(f'topic: {resolved}')
    lines.append(f'storage_id: {stats["storage_id"]}')
    lines.append(
        f'samples (after stride={args.stride}): {X.shape[0]}'
    )
    lines.append(
        f'channels (after mask): {X_m.shape[1]} of {X.shape[1]}'
    )
    lines.append(f'wavelength range used: {wl_m.min():.1f}–{wl_m.max():.1f} nm')
    lines.append(
        f'duration covered: {float(times_s[-1] - times_s[0]):.2f} s'
    )
    lines.append('')
    lines.append(
        f'preprocessing: SNV + Savitzky-Golay '
        f'(window={args.savgol_window}, poly={args.savgol_poly}, '
        f'deriv={args.savgol_deriv})'
    )
    lines.append(
        f'PCA: kept {pca_res.n_kept} components, '
        f'cumulative variance '
        f'{float(var_ratio[: pca_res.n_kept].sum()) * 100.0:.2f}%'
    )
    lines.append(f'embedding: {embed_name}')
    lines.append(
        f'clustering backend: {cluster_backend}  '
        f'min_cluster_size={mcs}'
    )
    lines.append(
        f'clusters: {n_clusters}  noise: {n_noise}/{labels.size}'
    )
    for lab in sorted(int(x) for x in np.unique(labels)):
        n = int((labels == lab).sum())
        lines.append(f'  cluster {lab:>2d}: n={n}')
    lines.append('')
    lines.append(
        f'NMF: K={H_endm.shape[0]}  '
        f'reconstruction_err={nmf_model.reconstruction_err_:.4g}'
    )
    abund_share = W_abund / np.clip(
        W_abund.sum(axis=1, keepdims=True), 1e-12, None
    )
    for i in range(H_endm.shape[0]):
        peak_nm = float(wl_m[int(np.argmax(H_endm[i]))])
        mean_share = float(abund_share[:, i].mean())
        lines.append(
            f'  EM{i}: peak ≈ {peak_nm:.1f} nm, '
            f'mean abundance share = {mean_share:.3f}'
        )

    (out_dir / f'{out_stem}_report.txt').write_text('\n'.join(lines) + '\n')
    print(f'Wrote {out_dir / f"{out_stem}_summary.png"} (+ 5 panels + report.txt).')


def main() -> None:
    p = argparse.ArgumentParser(
        description=(
            'Unsupervised spectroscopy report (SNV + Savitzky-Golay → PCA → '
            'UMAP/HDBSCAN → NMF) over a spectrometer rosbag.'
        ),
    )
    p.add_argument('--bag', '-b', type=Path, required=True)
    p.add_argument('--topic', '-t', default='/spectrometer')
    p.add_argument(
        '--output-dir',
        type=Path,
        default=None,
        help=(
            'Directory for figures and report '
            '(default: <bag>_cluster_report next to bag).'
        ),
    )
    p.add_argument('--stride', type=int, default=1)

    p.add_argument(
        '--mask-lo',
        type=float,
        default=None,
        help='Drop wavelengths below this (nm).',
    )
    p.add_argument(
        '--mask-hi',
        type=float,
        default=None,
        help='Drop wavelengths above this (nm).',
    )

    p.add_argument('--savgol-window', type=int, default=15)
    p.add_argument('--savgol-poly', type=int, default=3)
    p.add_argument('--savgol-deriv', type=int, default=1)

    p.add_argument('--pca-variance', type=float, default=0.99)
    p.add_argument('--max-pcs', type=int, default=30)

    p.add_argument('--umap-neighbors', type=int, default=30)
    p.add_argument('--umap-min-dist', type=float, default=0.05)

    p.add_argument(
        '--hdbscan-min-cluster',
        type=int,
        default=0,
        help='HDBSCAN min_cluster_size (0 = auto = max(20, N/100)).',
    )

    p.add_argument('--nmf-components', type=int, default=5)

    p.add_argument('--random-seed', type=int, default=0)
    p.add_argument('--dpi', type=float, default=140.0)

    args = p.parse_args()

    if args.stride < 1:
        print('--stride must be >= 1', file=sys.stderr)
        sys.exit(1)

    run_report(args)


if __name__ == '__main__':
    main()
