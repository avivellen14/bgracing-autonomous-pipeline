# BGRacing Autonomous Pipeline Challenge

An end-to-end containerized ROS 2 data processing pipeline simulating an autonomous vehicle's perception and planning workflow. This project implements a Mock SLAM environment with dynamic fault injection and a robust trajectory planner.

## Architecture Overview

The system is built using two distinct Ubuntu 24.04 Docker containers running ROS 2 Jazzy, communicating over a dedicated Docker bridge network (`ros_net`).

1. **Track Publisher (`track_publisher` node)**
   - Acts as the vehicle's perception module (Mock SLAM).
   - Ingests ground truth track boundaries from the `BrandsHatchLayout.csv` dataset.
   - **Fault Injection:** Applies Gaussian measurement noise ($\sigma = 0.2m$) to all points to simulate realistic sensor inaccuracies.
   - **Outlier Generation:** Randomly injects extreme outliers ("ghost cones") outside the track boundaries to test system robustness.
   - Publishes the noisy track data via a `geometry_msgs/Polygon` topic.

2. **Track Processor (`track_processor` node)**
   - Acts as the vehicle's trajectory planning module.
   - **Adaptive Outlier Rejection:** Utilizes a `scipy.spatial.KDTree` to perform real-time statistical $k$-NN distance analysis, automatically filtering out the injected ghost cones.
   - **Directional Routing:** Implements a localized sliding-window algorithm to synchronize the left and right track boundaries, preventing cross-track geometric jumps.
   - **Path Planning:** Fits a tight `scipy.interpolate.splprep` B-Spline to recover a smooth, navigable centerline (`per=False` for open-track accuracy).

## Evolution of the Algorithm (Engineering Process)
During development, several geometric and algorithmic challenges were addressed:
* **The "Nearest Neighbor" Trap:** Initial implementations using basic nearest-neighbor routing caused cross-track jumps where paths ran parallel. This was solved by respecting initial SLAM ordering and applying a localized sliding search window.
* **Closed Loop Assumption:** Using periodic splines (`per=True`) forced unnatural hooks at the track boundaries. Re-evaluating the dataset revealed it as an open layout (Sprint), solved by disabling periodic forcing.
* **Corner Cutting:** Heavy spline smoothing created a "rubber band" effect inside sharp turns. Reducing the smoothing parameter (`s=5.0`) ensured the centerline stayed strictly within the safe drivable area.

## Build and Run Instructions
```bash
# Clone the repository
git clone <your-repo-link>
cd bgracing-autonomous-pipeline

# Deploy the pipeline
docker compose up --build