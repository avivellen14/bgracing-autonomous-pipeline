import os
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from scipy.interpolate import splprep, splev
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon


class RobustTrackFilterProcessor(Node):
    def __init__(self):
        super().__init__('track_processor')
        self.sub = self.create_subscription(Polygon, '/track_points_raw', self.filter_callback, 10)
        self.get_logger().info('Ready! Waiting for raw points...')

    def filter_callback(self, msg):
        start_time = time.perf_counter()

        raw_data = np.array([[p.x, p.y, p.z] for p in msg.points])
        coords = raw_data[:, :2]
        sides = raw_data[:, 2]

        # 1. סינון Outliers סטטיסטי אדפטיבי (Robustness)
        tree = KDTree(coords)
        distances, _ = tree.query(coords, k=6)
        mean_dists = np.mean(distances[:, 1:], axis=1)

        threshold = np.mean(mean_dists) + 1.4 * np.std(mean_dists)
        inlier_mask = mean_dists < threshold

        inliers_xy = coords[inlier_mask]
        inliers_side = sides[inlier_mask]
        outliers_xy = coords[~inlier_mask]

        filter_duration_ms = (time.perf_counter() - start_time) * 1000.0

        # 2. הפרדה לשפות (הנקודות שומרות על הסדר המקורי)
        left_pts = inliers_xy[inliers_side == 1.0]
        right_pts = inliers_xy[inliers_side == 2.0]

        # סנכרון כיוון
        if len(left_pts) > 0 and len(right_pts) > 0:
            dist_start_start = np.linalg.norm(left_pts[0] - right_pts[0])
            dist_start_end = np.linalg.norm(left_pts[0] - right_pts[-1])
            if dist_start_end < dist_start_start:
                right_pts = right_pts[::-1]

        # 3. חישוב קו אמצע באמצעות "חלון עוקב" מקומי (מניעת קפיצת נתיבים בצמתים צפופים)
        center_pts = []
        last_matched_idx = 0

        for p_l in left_pts:
            window_start = max(0, last_matched_idx - 10)
            window_end = min(len(right_pts), last_matched_idx + 15)
            search_window = right_pts[window_start:window_end]

            if len(search_window) == 0:
                continue

            dists = np.linalg.norm(search_window - p_l, axis=1)
            local_min = np.argmin(dists)

            if dists[local_min] < 40.0:
                best_idx = window_start + local_min
                p_r = right_pts[best_idx]
                center_pts.append((p_l + p_r) / 2.0)
                last_matched_idx = best_idx

        center_pts = np.array(center_pts)

        # 4. ספליין הדוק (Trajectory Planning)
        tck, _ = splprep([center_pts[:, 0], center_pts[:, 1]], s=5.0, k=3, per=False)
        u_fine = np.linspace(0, 1, 1000)
        smooth_x, smooth_y = splev(u_fine, tck)

        # 5. ציור ושמירה
        os.makedirs('/results', exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        history_path = f'/results/track_outlier_filtering_{timestamp}.png'
        latest_path = '/results/track_outlier_filtering_latest.png'

        plt.figure(figsize=(11, 9))
        plt.scatter(left_pts[:, 0], left_pts[:, 1], c='#1f77b4', s=14, alpha=0.6, label='Left Boundary')
        plt.scatter(right_pts[:, 0], right_pts[:, 1], c='#ff7f0e', s=14, alpha=0.6, label='Right Boundary')
        plt.scatter(outliers_xy[:, 0], outliers_xy[:, 1], c='red', marker='x', s=90, linewidths=2.2,
                    label=f'Outliers ({len(outliers_xy)})')
        plt.plot(smooth_x, smooth_y, c='#2ca02c', linewidth=2.8, label='Recovered Centerline')

        metrics_text = (
            f"Autonomous System Metrics:\n"
            f"• Filter Latency: {filter_duration_ms:.2f} ms\n"
            f"• Detected Outliers: {len(outliers_xy)}\n"
            f"• Pipeline Status: Robust Trajectory Planned"
        )
        plt.gca().text(0.03, 0.05, metrics_text, transform=plt.gca().transAxes,
                       fontsize=10, verticalalignment='bottom',
                       bbox=dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor='gray', alpha=0.9))

        plt.title('Autonomous Pipeline: Outlier Filtering & Centerline Recovery')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.legend(loc='upper right')
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.axis('equal')

        plt.savefig(history_path, dpi=200, bbox_inches='tight')
        plt.savefig(latest_path, dpi=200, bbox_inches='tight')
        plt.close()

        self.get_logger().info('Plot exported successfully.')
        raise SystemExit


def main():
    rclpy.init()
    node = RobustTrackFilterProcessor()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()