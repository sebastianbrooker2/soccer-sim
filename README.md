# soccer-sim: cooperative ball/opponent tracking evaluation

Evaluation of cooperative tracking for a RoboCup-style humanoid team (Booster K1 robots, 3 vs 3 in the Booster MuJoCo sim).
Compared behaviours, all on identical detections and ground truth:

* **NUbots baseline** - re-implementation of the NUbots_K1 per-robot tracker (`eval/baseline`, mode `faithful`; `tuned` also available).
* **Own SRIF** - per-robot square-root information filter, no sharing (`eval/tracker`, `runSharedTracker` with only own packets).
* **Joint SRIF** - the same filter fusing all robots' packets over a lossy link (`linkDropProb` 0.2).
* **Raw measurement** - no filtering at all, each detection is the estimate (`eval/rawmeas`).

Detections come from a trained YOLO (NUbots_K1 ONNX models) run on frames re-rendered from the sim replays, with a
calibrated error model (`eval/sensing/yoloErrorModel.m`). Ground truth comes from the sim replay recorder.
The older MATLAB code in the repo root (`run_5v5_*.m`, `@GaussianInfo`, `@Measurement*`, `a1/`, `a2/`, `tests/`) is the
underlying SRIF / bearing-fusion machinery (see `bearing_fusion_notes.md`); `multirobot-sim-wip/` is an unrelated WIP simulation.

## Directory map

| path | content |
|---|---|
| `eval/` | MATLAB evaluation pipeline. Contract: `eval/SPEC.md`. Entry: `eval/runEvaluation.m`. Subdirs: `config` (`evalParams.m`), `gt` (ground-truth CSV loaders + CSVs), `sensing` (camera, YOLO->packets, error model), `tracker`, `baseline`, `rawmeas`, `metrics`, `export` (CSV/LaTeX), `tests`, `results` (summaries) |
| `tools/replay_to_csv.py` | sim replay (jsonl chunks) -> GT CSV. See `tools/README_groundtruth.md` |
| `tools/record/` | headless match recording in the Booster sim docker image (`record_match.sh`, `record_matches.sh`, detection logger) |
| `tools/render/` | MuJoCo (EGL) frame rendering from replays (`render_frames.py`) |
| `tools/yolo/` | YOLO inference on rendered frames (`detect_replay.py`), calibration scripts, ONNX models (`models/`) - see `tools/README_yolo_detections.md` |
| `tools/compare3/` | NUbots vs own SRIF vs joint SRIF (+raw, tuned) on real01: `run_compare3.m`, `results.csv`, `README.md` |
| `tools/rawmeas/` | raw-measurement behaviour metrics, `README.md` |
| `tools/ballstudy/` | ball reset/ghost-track/teleport study, `README.md` |
| `tools/viz/` | key-frame viewer + broadcast renderer, track export (`export_tracks.m`), tracked viewer data in `data/` |
| `tools/analysis/` | ad-hoc analysis/tuning scripts used during development (not maintained; some reference scratch files/absolute paths; `yt/` = final tuning runs) |
| `recordings/` | recorded matches `real01..real10` (+`det01`); only small derived files are tracked, see below |

## Requirements

* MATLAB R2026a (`/usr/local/MATLAB/R2026a/bin/matlab -batch "..."`). Run from the repo root (scripts `addpath(genpath(root))` themselves).
* Python 3.13. Two separate venvs (not tracked, recreate):
  ```
  python3 -m venv tools/render/venv && tools/render/venv/bin/pip install -r tools/render/requirements.txt
  python3 -m venv tools/yolo/venv   && tools/yolo/venv/bin/pip install -r tools/yolo/requirements.txt
  ```
  The viewer needs `tools/viz/requirements.txt` (matplotlib, numpy, pillow; the original runs used `~/miniconda3/bin/python`).
  `onnxruntime-gpu` needs the pip-installed NVIDIA libs on the loader path: always run YOLO scripts as
  `cd tools/yolo && LD_LIBRARY_PATH=$(./ldpath.sh) venv/bin/python detect_replay.py ...` (`ldpath.sh` prints
  `venv/.../nvidia/cu13/lib:.../nvidia/cudnn/lib`; adjust if your python/CUDA version differs). CPU `onnxruntime` also works, slowly.
* Booster sim docker image `booster-robotics-registry.cn-beijing.cr.aliyuncs.com/virtual-robot/virtual-robot:0.6.5-beta` (+ Booster Studio install providing
  `robocup_sim_src`), only for recording new matches. `tools/record/record_match.sh` stages the sim in `~/.soccer-train` (`STAGE_DIR`).
* Agent repo `~/test3v3` (`AGENT_REPO`) - the team software that plays the recorded matches. Not part of this repo.
* MuJoCo scene `~/.soccer-train/sim/mjcf/football_match_pitch_6_K1.xml` (+ `K1_22dof.xml`), created by the staging step above. Python tools
  read it from `$SOCCER_MODEL_XML` (default that path).
* YOLO models: `tools/yolo/models/{yolo,yolo_hsl_ft,old}.onnx` (35 MB total, tracked). They are copies from
  `~/NUbots_K1/module/vision/Yolo/data`; if ever removed, copy them back from there. Check NUbots licence before publishing the repo publicly.
* `~/NUbots_K1` is only a reference for the baseline re-implementation (read-only; no runtime dependency).

## Data pipeline

1. **Record** matches (docker, headless): `N=10 SPEED=3 DURATION=200 tools/record/record_matches.sh` -> `recordings/realNN/data/{manifest.json,chunks/frames_*.jsonl}`
   and `recordings/realNN.result.json`; it also converts each to `eval/gt/realNN.csv`. The sim must run with `MATCH_REPLAY_ENABLED=1` etc.
   (see `tools/README_groundtruth.md`).
2. **Replay -> GT CSV**: `python3 tools/replay_to_csv.py recordings/real01/data eval/gt/real01.csv --dt 0.05` (test: `tools/test_replay_to_csv.py`).
3. **Render + YOLO**: `cd tools/yolo && LD_LIBRARY_PATH=$(./ldpath.sh) venv/bin/python detect_replay.py --rec ../../recordings/real01 --robots 1 2 3 --hz 5`
   -> `recordings/real01/yolo/{dets,dets_all,dets_frames}.csv, stats.json`. (`tools/render/render_frames.py` just dumps frames.) Detection format and caveats: `tools/README_yolo_detections.md`.
4. **Evaluate**: `matlab -batch "addpath(genpath(pwd)); runEvaluation(struct('gtFiles',{{'eval/gt/real01.csv'}},'runName','real01_yolo','packetSource','yolo'))"`.
   Options and outputs (CSV + LaTeX tables/figures in `eval/results/<runName>`): header of `eval/runEvaluation.m`, `eval/SPEC.md`.
   All 10 matches with the simulated sensor model: `tools/analysis/run_real.m` (`run_real_cam.m` = camera model) -> `eval/results/real_sim`.
5. **Compare**: `matlab -batch "run('tools/compare3/run_compare3.m')"`, `tools/rawmeas/run_rawmeas.m`, `tools/ballstudy/study.m` (env `STUDY_NAME`, `STUDY_TAG`).
6. **View**: `matlab -batch "run('tools/viz/export_tracks.m')"` writes `tools/viz/data/{tracks,gt}.csv`; render image caches once with
   `MUJOCO_GL=egl tools/render/venv/bin/python tools/viz/render_keyframes.py` and `.../render_top.py` (into `tools/viz/cache*`, untracked), then
   `python tools/viz/viewer.py --t 120` (cameras + YOLO detections, per-robot baseline minimaps, joint SRIF minimap).
   Keys: Left/Right +-1 s, PageUp/PageDown +-10 s, Space play/pause, 1/2/3 joint copy, g ghost GT, q quit.
   `--png out.png` or `--mp4 out.mp4 --t0 100 --dur 60` render offline. `tools/viz/render_broadcast.py` renders a broadcast-style view (see its header).

## Reproducing the headline results (real01)

1. Make sure `recordings/real01/yolo/dets*.csv` (tracked) and `eval/gt/real01.csv` (tracked) exist - then steps 1-3 are not needed.
2. `matlab -batch "run('tools/compare3/run_compare3.m')"` -> `tools/compare3/results.csv` (parameters only in `eval/config/evalParams.m`, `rng(1)`).
3. `matlab -batch "run('tools/rawmeas/run_rawmeas.m')"`, then ball study `tools/ballstudy/study.m`.
4. Unit tests: `eval/tests/*.m` (MATLAB `runtests`), `python3 tools/test_replay_to_csv.py`.
To reproduce all 10 matches you need the raw recordings (not in git): re-record (step 1) or restore them from backup, convert to GT CSVs, run `detect_replay.py` per match, then `run_real.m`.

## Key findings

See `tools/compare3/README.md` (NUbots vs own vs joint SRIF vs raw; tables per class/robot) and `tools/ballstudy/README.md` (ghost ball tracks,
teleports, reset options), `tools/rawmeas/README.md`. In short on real01: the joint SRIF has the best ball in-region-95 (0.61 vs 0.18 NUbots, 0.41 own)
and lower RMSE; NUbots keeps high opponent availability only by carrying ~19 live robot tracks (~17 ghosts per sample).

## Git policy / what is not tracked (see `.gitignore`)

Ignored: venvs, `__pycache__`, logs, all `*.mat`, raw replay chunks `recordings/*/data/`, `recordings/det01/`, YOLO sample PNGs, `recordings/real02..10/yolo`,
per-match `eval/results/**/timeseries_match*.csv`, `eval/gt/real02..10.csv`, `eval/gt/det01.csv`, `eval/gt/old/`, large calibration dumps
(`tools/yolo/calib/{before,after,stage1}.csv`; regenerate with `tools/yolo/dumpYoloCalib.m`), viz caches and outputs.
Tracked deliberately: models (35 MB), `eval/gt/real01.csv` (8 MB), `recordings/real01/yolo/*.csv` (7 MB), `tools/viz/data/*.csv` (7 MB), result summaries (csv/tex).
Regenerating `eval/gt/real02..10.csv` needs the raw chunks (`replay_to_csv.py`); if you want them in git, remove the `eval/gt/real*.csv` ignore (72 MB; consider Git LFS).

## Known caveats

* Hardcoded absolute paths remain in `tools/analysis/*` scratch scripts and a few comments/docs; the main pipeline is path-independent apart from `~/.soccer-train` (override `SOCCER_MODEL_XML`) and `AGENT_REPO`, `STAGE_DIR`, `REC_ROOT`, `SIM_IMAGE` env vars in `tools/record`.
* Sim "detections" logged by the simulator are geometric projections (not YOLO, no robot class); the evaluation uses YOLO on re-rendered frames (see `tools/README_yolo_detections.md`).
* Samples before ~t=100 s of real01 have no detections (availability is diluted equally for all trackers). Only real01 has been used for the three-way comparison and the viewer.
* `tools/viz/viewer_v1..v5.py` are earlier iterations of `viewer.py`, kept for history.
