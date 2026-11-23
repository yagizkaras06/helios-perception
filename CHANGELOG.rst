^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package helios_obstacle_detection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-11-23)
------------------
* Occlusion-aware tracking: TENTATIVE / CONFIRMED / OCCLUDED / LOST lifecycle with explained
  occlusions (occluder, merge, out of range), re-identification of lost tracks, partial-view
  measurement model, merged clusters split between tracks, association cost with size / IoU /
  background terms (Hungarian kept), static-like tracks predicted with zero velocity.
* Track-guided static background map: objects that moved within ``static_time_s`` are not absorbed.
* Motion classification: change-point onset test (earlier MOVING, thresholds unchanged after a
  sensitivity analysis), motion state frozen without measurements, MOVING without evidence times out.
* Uncertainty outputs: ``velocity_status`` (RELIABLE / UNRELIABLE / UNKNOWN), ``predicted``,
  ``occlusion_reason``, ``track_confidence``, ``motion_confidence``, ``relative_motion_only``,
  ``max_displacement``; TTC only with a reliable velocity.
* Collision warning (``CollisionWarning.msg``, ``/<ns>/collision_warning``): CLEAR / CAUTION / WARNING
  from TTC / CPA of approaching tracks; RViz banner (display only, no control).
* RViz: grouped layout, top-down default view, range rings, only dynamic objects after the static
  background is learned, dashed boxes for predicted tracks.
* Event recorder: 5 s before / after CRITICAL risk, data loss or ego loss into rosbag2 + metadata.json;
  ``record_events:=true`` runs detector and recorder in one process (``perception_node``);
  ``replay.launch.py mode:=rerun|recorded``; ``bag_tool.py events``.
* Evaluation: synthetic occlusion / merge-split / wall-walk / start-stop scenarios, IDF1,
  fragmentation, onset latency, pseudo ground truth tools (``tools/evaluation``).
* Tests: 86 (10 gtest suites, 2 launch tests).

0.2.0 (2026-11-20)
------------------
* CZM (concentric-zone) ground segmentation, range-adaptive clustering, L-shape oriented boxes.
* LiDAR-only ego-motion estimation (planar point-to-plane ICP, hypothesis consensus, covariance).
* Constant-velocity Kalman + Hungarian multi-object tracking, MOVING / STATIC / UNKNOWN
  classification, static background map ("still for 10 s -> static").
* TTC / CPA / risk levels with uncertainty, sensor health and data-loss detection, diagnostics,
  performance monitoring, synthetic Helios-32 simulator and benchmark, rosbag2 tools.

0.1.0 (2026-11-19)
------------------
* First obstacle detection node (zone RANSAC ground, Euclidean clustering, boxes, RViz markers).
