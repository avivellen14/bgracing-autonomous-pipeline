import os
import csv
import time
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32


class TrackFaultInjectorPublisher(Node):
    def __init__(self):
        super().__init__('track_publisher')
        self.pub = self.create_publisher(Polygon, '/track_points_raw', 10)
        time.sleep(5.0)
        self.publish_noisy_track()

    def publish_noisy_track(self):
        csv_file = '/data/BrandsHatchLayout.csv'
        raw_left = []
        raw_right = []

        with open(csv_file, 'r', encoding='utf-8-sig', errors='ignore') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 3: continue
                tokens = [t.strip() for t in row if t.strip()]
                if len(tokens) < 3: continue

                side_code = None
                nums = []
                for t in tokens:
                    low = t.lower()
                    if 'left' in low:
                        side_code = 1.0
                    elif 'right' in low:
                        side_code = 2.0
                    else:
                        try:
                            nums.append(float(t))
                        except ValueError:
                            pass

                if side_code is not None and len(nums) >= 2:
                    y_val, x_val = nums[0], nums[1]
                    if side_code == 1.0:
                        raw_left.append([x_val, y_val])
                    else:
                        raw_right.append([x_val, y_val])

        pts_left = np.array(raw_left)
        pts_right = np.array(raw_right)
        if len(pts_left) == 0: return

        # Baseline - נשמר פעם אחת
        os.makedirs('/results', exist_ok=True)
        raw_plot_path = '/results/raw_input_track.png'
        if not os.path.exists(raw_plot_path):
            plt.figure(figsize=(10, 8))
            plt.scatter(pts_left[:, 0], pts_left[:, 1], c='#1f77b4', s=16, label='Left Boundary')
            plt.scatter(pts_right[:, 0], pts_right[:, 1], c='#ff7f0e', s=16, label='Right Boundary')
            plt.title('Brands Hatch: Ground Truth Raw Input Dataset')
            plt.xlabel('X [m]')
            plt.ylabel('Y [m]')
            plt.grid(True, linestyle='--', alpha=0.5)
            plt.legend()
            plt.axis('equal')
            plt.savefig(raw_plot_path, dpi=200, bbox_inches='tight')
            plt.close()

        # הוספת רעש מבלי לפגוע בסדר
        left_with_side = np.column_stack((pts_left + np.random.normal(0, 0.2, pts_left.shape), np.ones(len(pts_left))))
        right_with_side = np.column_stack(
            (pts_right + np.random.normal(0, 0.2, pts_right.shape), np.full(len(pts_right), 2.0)))
        inlier_pts = np.vstack((left_with_side, right_with_side))

        # הזרקת 12 Outliers בסוף המערך (Fault Injection)
        x_min, x_max = np.min(inlier_pts[:, 0]), np.max(inlier_pts[:, 0])
        y_min, y_max = np.min(inlier_pts[:, 1]), np.max(inlier_pts[:, 1])
        outliers_x = np.random.uniform(x_min - 35, x_max + 35, size=12)
        outliers_y = np.random.uniform(y_min - 35, y_max + 35, size=12)
        outliers_side = np.random.choice([1.0, 2.0], size=12)
        outliers = np.column_stack((outliers_x, outliers_y, outliers_side))

        all_points = np.vstack((inlier_pts, outliers))

        msg = Polygon()
        for x, y, side in all_points:
            p = Point32()
            p.x, p.y, p.z = float(x), float(y), float(side)
            msg.points.append(p)

        self.pub.publish(msg)
        self.get_logger().info(f'Published {len(all_points)} track points with Mock SLAM noise')


def main():
    rclpy.init()
    node = TrackFaultInjectorPublisher()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()