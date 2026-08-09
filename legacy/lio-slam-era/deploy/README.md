# LIO-SLAM deployment

Self-contained compose files for running the LIO-SLAM stack from a published
release image. Each compose file targets one dataset / hardware profile.

The compose files mirror `tools/run_fast_lio_pgo_prod.sh` but split each
ROS node into its own service so a container restart only takes seconds
and per-component logs land in separate streams.

## Files

| File                              | Profile                                        |
| --------------------------------- | ---------------------------------------------- |
| `docker-compose.giant-0514.yml`   | Replay the giant `0514/fp-demo` bag end-to-end |
| `config/fast_lio_xsw_hesai.yaml`  | FAST-LIO2 config (Hesai 32-line + FP IMU)      |
| `config/params_fast_lio_pgo.yaml` | PGO backend config (FP IMU + GPS ENU)          |

The configs under `config/` are bind-mounted into every service so on-site
tuning means editing files here, not rebuilding the image.

## Run

```bash
cd deploy/

# Use the release image published by CI on a tag push:
IMAGE=wanderer123/fslam-humble:amd64 \
  BAG_DIR=/home/data/giant/大块头/0514/fp-demo/rosbag/ros_0514_122932 \
  docker compose -f docker-compose.giant-0514.yml up

# Or the locally built image while iterating:
IMAGE=lio-slam-release:local \
  BAG_DIR=/home/data/giant/大块头/0514/fp-demo/rosbag/ros_0514_122932 \
  docker compose -f docker-compose.giant-0514.yml up
```

## Verify

In another shell on the same host (any ROS 2 humble container with
matching `ROS_DOMAIN_ID` works):

```bash
docker exec -it lio-slam-pgo bash -lic \
  'ros2 topic hz /odom /Odometry /fast_lio_pgo/odometry'
```

Expected, ~10 s into the bag:

| Topic                            | Rate        | Source              |
| -------------------------------- | ----------- | ------------------- |
| `/Odometry`                      | ~10 Hz      | FAST-LIO2 raw       |
| `/fast_lio_pgo/odometry`         | ~1 Hz       | PGO keyframes       |
| `/fast_lio_pgo/fused_odometry`   | ~160 Hz     | PGO + IMU preint    |
| `/odom`                          | ~160 Hz     | same, ENU frame     |

Reference accuracy from `memory/giant_0514_2_results.md`:
PGO 2D mean **≈ 0.37 m**, max ≈ 4.2 m. Per-run drift inside that range
means the deploy stack is producing the same numbers as the dev pipeline.

## Live-sensor mode

Drop the `bag-player` service and add driver services (livox or hesai +
fixposition_driver_ros2). The remaining three services (`fp-imu-relay`,
`fast-lio`, `lio-slam-pgo`) are identical between bag-replay and live.
