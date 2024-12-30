import cv2
import numpy as np
from typing import Optional, List, Tuple, Dict
import time
from dataclasses import dataclass
from enum import Enum
from src.models.position import Position
import sys


class MarkerType(Enum):
    CORNER = "corner"  # 10x10cm markers for robot localization
    FLAG = "flag"  # 2x2cm markers for capture points
    HINT_FLAG = "hint_flag"  # 2x2cm markers with id 0


@dataclass
class MarkerConfig:
    type: MarkerType
    size: float  # Marker size in meters
    id_range: range  # Valid marker IDs for this type
    max_distance: float  # Maximum detection distance in centimeters


class ArucoDetector:
    def __init__(
        self, calibration_file: str = "camera_calibration.npz", test_mode: bool = False
    ):
        self.test_mode = test_mode

        # Initialize ArUco detector with 6x6 dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Configure marker types (all sizes in cm)
        self.marker_configs = {
            MarkerType.CORNER: MarkerConfig(
                type=MarkerType.CORNER,
                size=0.1,  # 10cm
                id_range=range(1, 5),  # IDs 1-4 for corners
                max_distance=float("inf"),  # No distance limit for corners
            ),
            MarkerType.FLAG: MarkerConfig(
                type=MarkerType.FLAG,
                size=0.02,  # 2cm
                id_range=range(5, 17),  # IDs 5-16 for flags
                max_distance=0.5,  # 50cm
            ),
            MarkerType.HINT_FLAG: MarkerConfig(
                type=MarkerType.HINT_FLAG,
                size=0.02,  # 2cm
                id_range=range(0, 1),  # ID 0
                max_distance=0.5,  # 50cm
            ),
        }

        # Known positions of corner markers in world frame (centimeters)
        self.corner_positions = {
            4: (0.0, 0.0),  # Origin (bottom-left)
            3: (3.0, 0.0),  # Bottom-right
            2: (3.0, 3.0),  # Top-right
            1: (0.0, 3.0),  # Top-left
        }

        self.load_calibration(calibration_file)

    def load_calibration(self, calibration_file: str):
        """Load camera calibration parameters from NPZ file"""
        try:
            with np.load(calibration_file) as data:
                self.camera_matrix = data["camera_matrix"]
                self.dist_coeffs = data["dist_coeffs"]
        except Exception as e:
            print(f"Warning: Using default calibration ({e})")
            self.camera_matrix = np.array(
                [[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32
            )
            self.dist_coeffs = np.zeros(5)

    def detect_markers(
        self, frame
    ) -> Tuple[List[int], List[np.ndarray], List[Tuple[float, float]]]:
        """
        Detect ArUco markers in the frame.
        Returns:
            - ids: List of marker IDs
            - corners: List of marker corners
            - centers: List of marker centers relative to image center (in pixels)
        """
        if self.test_mode:
            return [], [], []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return [], [], []

        # Calculate image center
        image_center_x = frame.shape[1] / 2
        image_center_y = frame.shape[0] / 2

        # Calculate relative centers
        centers = []
        for corner in corners:
            center_x = np.mean(corner[0][:, 0])
            center_y = np.mean(corner[0][:, 1])
            relative_x = center_x - image_center_x
            relative_y = image_center_y - center_y
            centers.append((relative_x, relative_y))

        return ids.flatten().tolist(), corners, centers

    def classify_marker(self, marker_id: int) -> Optional[MarkerType]:
        """
        Determine marker type based on its ID.
        Returns: CORNER (1-4), FLAG (5-16), HINT_FLAG (0), or None if invalid
        """
        if marker_id in self.marker_configs[MarkerType.CORNER].id_range:
            return MarkerType.CORNER
        elif marker_id in self.marker_configs[MarkerType.FLAG].id_range:
            return MarkerType.FLAG
        elif marker_id in self.marker_configs[MarkerType.HINT_FLAG].id_range:
            return MarkerType.HINT_FLAG
        return None

    def get_marker_position(
        self, corners: np.ndarray, marker_type: MarkerType
    ) -> Optional[Tuple[float, float]]:
        """
        Calculate marker's distance and global angle relative to camera.
        Args:
            corners: Marker corners in image coordinates
            marker_type: Type of marker for correct size
        Returns:
            Tuple[float, float]: (distance in cm, angle in rad)
            - distance: distance in centimeters from camera to marker center, in the x,y plan
            - angle: global angle in radians from camera's optical axis
              (0 = center, positive = anti-clockwise/left)
            or None if calculation fails
        """
        # Get marker size in cm
        marker_size = self.marker_configs[marker_type].size

        # Define 3D coordinates of marker corners
        object_points = np.array(
            [
                [-marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, -marker_size / 2, 0],
                [-marker_size / 2, -marker_size / 2, 0],
            ],
            dtype=np.float32,
        )

        # Get marker center in image
        marker_corners = corners[0]
        marker_center = np.mean(marker_corners, axis=0)

        # Calculate angles using camera matrix
        principal_point = (self.camera_matrix[0, 2], self.camera_matrix[1, 2])
        focal_length = (self.camera_matrix[0, 0], self.camera_matrix[1, 1])

        # Get displacement from image center
        x_diff = marker_center[0] - principal_point[0]
        y_diff = marker_center[1] - principal_point[1]

        # Calculate global angle using atan2
        angle = np.arctan2(
            x_diff, -y_diff
        )  # Negative y_diff because image coordinates are inverted

        # Calculate distance using PnP
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            marker_corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not success:
            return None

        # Convert distance to centimeters
        distance = np.linalg.norm(tvec) * 100

        return distance, angle

    def calculate_absolute_orientation(self, camera_angle: float) -> float:
        """
        Convert camera angle to absolute orientation.
        Returns: angle in radians [-π, π] where 0 is north, positive is east
        """
        absolute_angle = camera_angle - np.pi / 2
        return np.arctan2(np.sin(absolute_angle), np.cos(absolute_angle))

    def filter_position(self, new_pose: Position) -> Position:
        """Apply exponential filter to position for smoother movement"""
        if self.last_pose is None:
            self.last_pose = new_pose
            return new_pose

        alpha = self.position_filter_alpha
        filtered_pose = Position(
            x=alpha * new_pose.x + (1 - alpha) * self.last_pose.x,
            y=alpha * new_pose.y + (1 - alpha) * self.last_pose.y,
            theta=new_pose.theta,  # Don't filter orientation
            camera_angle=new_pose.camera_angle,
            timestamp=new_pose.timestamp,
            confidence=new_pose.confidence,
        )
        self.last_pose = filtered_pose
        return filtered_pose

    def estimate_robot_pose(self, frame) -> Optional[Position]:
        """
        Estimate robot's position and orientation in the world frame.

        Returns:
            Position object with:
            - x,y: position in cm
            - theta: absolute orientation in rad [-π, π] where 0=north, positive=clockwise
            or None if position cannot be determined
        """
        # Get detected markers
        ids, corners, _ = self.detect_markers(frame)
        if not ids:
            return None

        positions = []  # Will store (x,y) positions computed from each marker
        orientations = []  # Will store orientations computed from each marker

        # Process each detected marker
        for i, marker_id in enumerate(ids):
            # Only use corner markers (IDs 1-4) for localization
            marker_type = self.classify_marker(marker_id)
            if marker_type != MarkerType.CORNER:
                continue

            # Get distance and angle to marker from camera's perspective
            result = self.get_marker_position(corners[i : i + 1], marker_type)
            if not result:
                continue
            distance, angle = (
                result  # distance in cm, angle in rad from camera's optical axis
            )

            # Get marker's known position in world frame
            world_x, world_y = self.corner_positions[marker_id]

            # Calculate marker's orientation in camera frame
            _, rvec, _ = cv2.solvePnP(
                np.array(
                    [[-5, 5, 0], [5, 5, 0], [5, -5, 0], [-5, -5, 0]], dtype=np.float32
                ),
                corners[i][0],
                self.camera_matrix,
                self.dist_coeffs,
            )
            # Convert rotation vector to matrix and extract orientation
            rmat = cv2.Rodrigues(rvec)[0]
            marker_orientation = np.arctan2(rmat[1, 0], rmat[0, 0])

            # Calculate robot's position
            # Using the known marker position and the detected distance/angle,
            # we can determine where the robot must be
            robot_x = world_x - distance * np.cos(angle)
            robot_y = world_y - distance * np.sin(angle)

            positions.append((robot_x, robot_y))
            orientations.append(marker_orientation)

        if not positions:
            return None

        # Average all position and orientation estimates
        avg_x, avg_y = np.mean(positions, axis=0)
        avg_orientation = np.mean(orientations)

        # Convert camera-relative orientation to absolute orientation
        # (where 0 is north, positive is clockwise)
        absolute_orientation = self.calculate_absolute_orientation(avg_orientation)

        # Create and return filtered position
        new_pose = Position(
            x=float(avg_x),
            y=float(avg_y),
            theta=float(absolute_orientation),
            camera_angle=float(avg_orientation),
            timestamp=time.time(),
            confidence=min(
                len(positions) / 4.0, 1.0
            ),  # Higher confidence with more markers
        )

    return self.filter_position(new_pose)

    def get_visible_flags(self, frame) -> List[int]:
        """Return list of visible flag marker IDs within range"""
        ids, corners, _ = self.detect_markers(frame)
        visible_flags = []

        for i, marker_id in enumerate(ids):
            marker_type = self.classify_marker(marker_id)
            if marker_type != MarkerType.FLAG:
                continue

            result = self.get_marker_position(corners[i : i + 1], marker_type)
            if (
                result
                and result[0] <= self.marker_configs[MarkerType.FLAG].max_distance
            ):
                visible_flags.append(marker_id)

        return visible_flags


def main(frame_path):
    # Initialize detector
    detector = ArucoDetector()

    # Read image
    frame = cv2.imread(frame_path)
    if frame is None:
        print(f"Error: Could not read image at {frame_path}")
        return

    # Get markers information
    ids, corners, centers = detector.detect_markers(frame)

    # Process each detected marker
    for i, marker_id in enumerate(ids):
        # Get marker type
        marker_type = detector.classify_marker(marker_id)
        if marker_type is None:
            continue

        # Get marker position relative to image center
        pos_x, pos_y = centers[i]

        # Get distance and angle
        result = detector.get_marker_position(corners[i : i + 1], marker_type)
        if result is None:
            continue
        distance, angle = result

        # Print results
        print(marker_id, pos_x, pos_y, distance, angle)
        # print(f"{marker_id}, {pos_x:.1f}, {pos_y:.1f}, {distance:.1f}, {angle:.3f}")


if __name__ == "__main__":
    if len(sys.argv) == 2:
        main(sys.argv[1])
