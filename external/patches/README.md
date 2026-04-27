# external/patches

Local-machine patches we want to keep applied on top of the pinned upstream
SHAs in `external/` and `earth-rover.repos`.  Without these, the rover behaves
slightly differently from how it does today on `santacruz`.

| File | Applies to | What it does |
|------|------------|--------------|
| `metavision_driver.local.patch` | `external/metavision_driver` (`darknight-007/metavision_driver` @ master / `ea628b52`) | Disables the `LD_PRELOAD=libasan.so` injection that was used during a debug session. |
| `orbslam3_ros2.local.patch` | `~/ros2_ws/src/orbslam3_ros2` after `vcs import` (`darknight-007/orbslam3_ros2` @ main / `eb2534c9`) | Local fixes to `CMakeLists.txt`, `FindORB_SLAM3.cmake`, and the monocular SLAM node (5 files, ~27 lines). |
| `orbslam3_gh_monocular.yaml` | drop into `~/ros2_ws/src/orbslam3_ros2/config/monocular/gh.yaml` after `vcs import` | Grasshopper3 monocular calibration / config. Was untracked in the source tree. |

## Apply

After `setup_workspace.sh` has cloned/imported the upstream packages:

```bash
( cd ~/earth-rover/external/metavision_driver \
    && git apply --3way ../patches/metavision_driver.local.patch )

( cd ~/ros2_ws/src/orbslam3_ros2 \
    && git apply --3way ~/earth-rover/external/patches/orbslam3_ros2.local.patch \
    && cp ~/earth-rover/external/patches/orbslam3_gh_monocular.yaml \
         config/monocular/gh.yaml )
```

`setup_workspace.sh --apply-patches` does this automatically.

## Long-term plan

These patches are a **bridge**, not a destination. The right end-state is to:

1. Push the changes upstream to `darknight-007/metavision_driver` and
   `darknight-007/orbslam3_ros2` so the pinned SHAs already include them; OR
2. Maintain a `earth-rover` branch in each fork that includes the patches and
   pin THAT branch in the submodule / `.repos`.

Either way these patches should disappear once the fork SHAs are bumped.
