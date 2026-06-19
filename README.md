# LiDAR Obstacle Detection Pipeline (PCL / C++)

A real-time-capable obstacle detection pipeline for raw Velodyne LiDAR point clouds which obtained from KITTI Dataset, built on the Point Cloud Library (PCL). It loads KITTI `.bin` scans, removes the ground plane with RANSAC, clusters remaining points with a KD-Tree, fits oriented bounding boxes via PCA, classifies each object by size, and renders the result — interactively or to PNG frames you can stitch into a video or GIF. All the processes that we previously mentioned in the repo PCL-Practice

```
.bin file → Load → Voxel Downsample → ROI Crop
         → RANSAC Ground Removal
         → Euclidean Clustering (KD-Tree)
         → PCA Oriented Bounding Boxes + Classification
         → PCLVisualizer Render (window and/or PNG)

This is the preffered seq on the PCL Documentation
```

---

## Table of contents

- [Project structure](#project-structure)
- [Pipeline stages](#pipeline-stages)
- [Dependencies](#dependencies)
- [Getting KITTI data](#getting-kitti-data)
- [Build](#build)
- [Run](#run)
- [Configuration reference](#configuration-reference)
- [Classification logic](#classification-logic)
- [Exporting to GIF / video](#exporting-to-gif--video)
- [Troubleshooting](#troubleshooting)
- [Roadmap ideas](#roadmap-ideas)

---

## Project structure

```
lidar_pcl/
├── CMakeLists.txt
├── Makefile                  ← convenience wrapper (deps / build / run / clean)
├── include/
│   ├── config.h               all tunable parameters live here
│   ├── types.h                PointT, CloudPtr, BoundingBox struct
│   ├── loader.h
│   ├── filter.h
│   ├── segmentation.h
│   ├── clustering.h
│   ├── bounding_box.h         BBox namespace (fitBoxes / classify / printDetections)
│   └── renderer.h
├── src/
│   ├── main.cpp               entry point + CLI argument parsing
│   ├── loader.cpp             reads KITTI .bin files into a PCL cloud
│   ├── filter.cpp             voxel grid downsample + ROI crop
│   ├── segmentation.cpp       RANSAC ground plane removal
│   ├── clustering.cpp         Euclidean clustering (KD-Tree)
│   ├── bounding_box.cpp       PCA-based oriented bounding boxes + classification
│   └── renderer.cpp           PCLVisualizer window + PNG screenshot export
├── KITTI/                     raw downloaded KITTI drive (kept for reference)
├── data/                      ← .bin files the pipeline actually reads (DATA_DIR)
└── output/frames/             ← PNGs saved here when run with --save
```

---

## Pipeline stages

| # | Stage | File | What happens |
|---|-------|------|---------------|
| 1 | Load | `loader.cpp` | Reads a `.bin` file as `float32[4]` per point (x, y, z, intensity) into a `pcl::PointXYZI` cloud |
| 2 | Voxel downsample | `filter.cpp` | `pcl::VoxelGrid` collapses points into a 3D grid (`VOXEL_SIZE`), cutting point count drastically while preserving shape |
| 3 | ROI crop | `filter.cpp` | `pcl::CropBox` keeps only points in a box around the car, then removes a small sphere around the sensor (ego-vehicle self-returns) |
| 4 | Ground removal | `segmentation.cpp` | `pcl::SACSegmentation` (RANSAC) fits a plane model and splits the cloud into `ground` and `obstacles` |
| 5 | Clustering | `clustering.cpp` | `pcl::EuclideanClusterExtraction` over a KD-Tree groups obstacle points into discrete objects |
| 6 | Bounding boxes | `bounding_box.cpp` | PCA on each cluster gives principal axes → oriented bounding box (center, dimensions, quaternion); boxes are filtered by realistic size and labeled by `classify()` |
| 7 | Render | `renderer.cpp` | `PCLVisualizer` draws ground (green), unclustered obstacles (gray), colored clusters, wireframe boxes + text labels; optionally saves a PNG and/or blocks for interactive viewing |

Per-frame timing and a detection table (center, W/D/H, class, distance) print to stdout automatically.

---

### Install everything (Ubuntu20)

```bash
sudo apt update
sudo apt install -y cmake build-essential pkg-config \
    libeigen3-dev libflann-dev libboost-all-dev \
    libvtk7-dev libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev \
    libpcl-dev
```


## Getting KITTI dataset used here

Download any KITTI raw "City" drive — here's the sequence used during development (`2011_09_26_drive_0001`, ~390 MB):

```bash
# Download
wget -c "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0001/2011_09_26_drive_0001_sync.zip"

```
Asked AI chatbot to help me select which could be easily downloaded. in the congif.h change the path of the data based on your choice

---

## Build

```bash
cd lidar_pcl
mkdir -p build 
cd build
cmake ..
make -j
```


---

## Run

The parsing i have used it from a repo already found (good idea)
```bash
cd build

./lidar_pipeline                  # frame 0, interactive window
./lidar_pipeline --frame 10       # a specific frame, interactive window
./lidar_pipeline --all            # all frames sequentially (press Q to advance)
./lidar_pipeline --all --save     # all frames, save PNG per frame, no window block
./lidar_pipeline --frame 0 --save # frame 0, show window AND save PNG
```


---

## Configuration reference

Every tunable parameter lives in `include/config.h` — no other file needs editing for normal use.

| Parameter | Default | Controls |
|---|---|---|
| `DATA_DIR` | `"data"` | folder the loader scans for `.bin` files |
| `OUTPUT_DIR` | `"output/frames"` | where `--save` writes PNGs |
| `VOXEL_SIZE` | `0.2f` (m) | downsampling resolution — lower = denser cloud, slower |
| `ROI_MIN/MAX_X` | `-40 / 40` (m) | forward/backward detection range |
| `ROI_MIN/MAX_Y` | `-20 / 20` (m) | left/right detection range |
| `ROI_MIN/MAX_Z` | `-3 / 1.5` (m) | vertical detection range |
| `EGO_RADIUS` | `1.5f` (m) | sphere around the sensor removed (self-returns) |
| `RANSAC_DISTANCE_THRESH` | `0.25` | max distance from the fitted plane to count as ground |
| `RANSAC_MAX_ITERATIONS` | `150` | RANSAC iteration cap |
| `CLUSTER_TOLERANCE` | `0.5` (m) | max gap between points in the same cluster |
| `CLUSTER_MIN_POINTS` | `10` | minimum points to form a cluster |
| `CLUSTER_MAX_POINTS` | `5000` | maximum points per cluster |
| `BOX_MIN/MAX_W/D/H` | `0.5–6.0` (m) | sanity-filters out boxes too small/large to be real objects |
| `WINDOW_NAME` | `"LiDAR Detection Pipeline"` | viewer window title |
| `POINT_SIZE` | `2.0f` | rendered point size |


---

## Exporting to GIF / video

After running `./lidar_pipeline --all --save`, frames land in `output/frames/frame_0000.png`, `frame_0001.png`, etc. you will find all of these images in the build file i won't push it ofcourse but if you have used similar project you will find it after building the running the code there

**GIF for Using in Result section**
```bash
cd ../output/frames
ffmpeg -framerate 10 -i frame_%04d.png -vf "fps=10,scale=960:-1:flags=lanczos,palettegen" -y palette.png
ffmpeg -framerate 10 -i frame_%04d.png -i palette.png -lavfi "fps=10,scale=960:-1:flags=lanczos[x];[x][1:v]paletteuse" -y output.gif
```

---
## Result
<p align="center">
  <img src="output/frames/output.gif" width="600">
</p>