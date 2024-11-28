import cv2
import numpy as np
from typing import Optional, List, Tuple, Dict
import time
from dataclasses import dataclass
from enum import Enum
from models.position import Position


class MarkerType(Enum):
    CORNER = "corner"  # 20x20cm markers for robot localization
    FLAG = "flag"  # 5x5cm markers for capture points


@dataclass
class MarkerConfig:
    type: MarkerType
    size: float  # Marker size in meters
    id_range: range  # Valid marker IDs for this type
    max_distance: float  # Maximum detection distance in meters


class ArucoDetector:
    """
    ArUco marker detector handling two marker types:
    - Corner markers (20x20cm): Used for robot localization on the grid
    - Flag markers (5x5cm): Used as capture points, only detected within 70.7cm
    """

    def __init__(
        self, calibration_file: str = "camera_calibration.npz", test_mode: bool = False
    ):
        self.test_mode = test_mode

        # Initialize ArUco detector with 6x6 dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Configure different marker types
        self.marker_configs = {
            MarkerType.CORNER: MarkerConfig(
                type=MarkerType.CORNER,
                size=0.10,  # 10cm
                id_range=range(1, 5),  # IDs 1-4 for corners
                max_distance=float("inf"),  # No distance limit for corners
            ),
            MarkerType.FLAG: MarkerConfig(
                type=MarkerType.FLAG,
                size=0.02,  # 2cm
                id_range=range(5, 17),  # IDs 5-16 for flags
                max_distance=0.707,  # sqrt(2) * 0.5m = ~70.7cm
            ),
        }

        # Known positions of corner markers in world frame (meters)
        self.corner_positions = {
            4: (0.0, 0.0),  # Origin
            3: (3.0, 0.0),  # Right bottom corner
            2: (3.0, 3.0),  # Right top corner
            1: (0.0, 3.0),  # Left top corner
        }

        self.load_calibration(calibration_file)
        self.last_pose = None  # For position filtering
        self.position_filter_alpha = 0.3  # Filter coefficient (higher = less filtering)

    def load_calibration(self, calibration_file: str):
        """Load camera calibration parameters from NPZ file"""
        try:
            with np.load(calibration_file) as data:
                self.camera_matrix = data["camera_matrix"]
                self.dist_coeffs = data["dist_coeffs"]
        except Exception as e:
            print(f"Warning: Using default calibration ({e})")
            # Default calibration for 640x480 camera
            self.camera_matrix = np.array(
                [[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32
            )
            self.dist_coeffs = np.zeros(5)

    def detect_markers(self, frame) -> Tuple[List, Optional[np.ndarray]]:
        """
        Detect ArUco markers in the frame
        Returns: (corners, ids) where corners is list of marker corners
        and ids is array of marker IDs
        """
        if self.test_mode:
            return [], None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        return corners, ids

    def classify_marker(self, marker_id: int) -> Optional[MarkerType]:
        """Determine marker type (CORNER or FLAG) based on its ID"""
        for config in self.marker_configs.values():
            if marker_id in config.id_range:
                return config.type
        return None

    def estimate_marker_pose(
        self, corners: np.ndarray, marker_id: int
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """
        Estimate 3D pose of a marker
        Returns: (rotation_vector, translation_vector, distance) or None if invalid
        Filters out FLAG markers that are too far away
        """
        marker_type = self.classify_marker(marker_id)
        if not marker_type:
            return None

        marker_size = self.marker_configs[marker_type].size
        max_distance = self.marker_configs[marker_type].max_distance

        # Get pose using solvePnP
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, self.camera_matrix, self.dist_coeffs
        )

        if rvec is None or tvec is None:
            return None

        # Check distance limit for flag markers
        distance = np.linalg.norm(tvec[0])
        if marker_type == MarkerType.FLAG and distance > max_distance:
            return None

        return rvec[0], tvec[0], distance

    def calculate_marker_orientation(self, rvec: np.ndarray) -> float:
        """
        Calculate marker's orientation from its rotation vector
        Returns: angle in radians
        """
        # Convert rotation vector to matrix and extract camera's z-axis
        rmat = cv2.Rodrigues(rvec)[0]
        camera_z = rmat[:, 2]
        return np.arctan2(camera_z[1], camera_z[0])

    def estimate_camera_angle(
        self, corner_markers: Dict[int, Tuple[np.ndarray, np.ndarray]]
    ) -> Optional[float]:
        """
        Estimate camera angle using visible corner markers
        Returns: average angle in radians or None if insufficient markers
        """
        if len(corner_markers) < 2:
            return None

        # Calculate angle from each marker and average
        angles = []
        for marker_id, (rvec, tvec) in corner_markers.items():
            angle = self.calculate_marker_orientation(rvec)
            angles.append(angle)

        return np.mean(angles)

    def filter_position(self, new_pose: Position) -> Position:
        """
        Apply exponential filter to position for smoother movement
        Only filters x,y coordinates, not orientation
        """
        if self.last_pose is None:
            self.last_pose = new_pose
            return new_pose

        # Apply exponential filter
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
        Main method to estimate robot's position and orientation
        Uses corner markers for localization and calculates camera angle
        Returns: Position object or None if position cannot be determined
        """
        corners, ids = self.detect_markers(frame)
        if ids is None:
            return None

        # Separate corner and flag markers
        corner_markers = {}
        flag_markers = {}

        for i in range(len(ids)):
            marker_id = ids[i][0]
            pose = self.estimate_marker_pose(corners[i : i + 1], marker_id)
            if not pose:
                continue

            rvec, tvec, distance = pose
            if self.classify_marker(marker_id) == MarkerType.CORNER:
                corner_markers[marker_id] = (rvec, tvec)
            else:
                flag_markers[marker_id] = (rvec, tvec, distance)

        if not corner_markers:
            return None

        # Calculate robot position from each visible corner
        positions = []
        for marker_id, (rvec, tvec) in corner_markers.items():
            world_x, world_y = self.corner_positions[marker_id]
            # Transform marker position to world coordinates
            positions.append((world_x - tvec[0], world_y - tvec[2]))

        # Average all position estimates
        robot_x, robot_y = np.mean(positions, axis=0)
        camera_angle = self.estimate_camera_angle(corner_markers)

        if camera_angle is None:
            return None

        new_pose = Position(
            x=float(robot_x),
            y=float(robot_y),
            theta=float(camera_angle),
            camera_angle=float(camera_angle),
            timestamp=time.time(),
            confidence=min(len(corner_markers) / 4.0, 1.0),
        )

        return self.filter_position(new_pose)

    def get_visible_flags(self, frame) -> List[int]:
        """
        Return list of visible flag marker IDs that are within range
        Used to detect capture points
        """
        corners, ids = self.detect_markers(frame)
        visible_flags = []

        if ids is None:
            return visible_flags

        for i in range(len(ids)):
            marker_id = ids[i][0]
            if self.classify_marker(marker_id) == MarkerType.FLAG:
                if self.estimate_marker_pose(corners[i : i + 1], marker_id):
                    visible_flags.append(marker_id)

        return visible_flags

    def draw_detected_markers(self, frame, draw_axis: bool = True) -> None:
        """
        Draw detected markers and their axes on the frame
        Useful for debugging and visualization
        """
        corners, ids = self.detect_markers(frame)
        if ids is not None:
            # Draw marker boundaries
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Draw coordinate axes for each marker
            if draw_axis:
                for i in range(len(ids)):
                    pose = self.estimate_marker_pose(corners[i : i + 1], ids[i][0])
                    if pose:
                        rvec, tvec, _ = pose
                        cv2.drawFrameAxes(
                            frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1
                        )
