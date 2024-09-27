import cv2
import numpy as np
import pandas as pd
import pyrealsense2 as rs
import os

# Set up RealSense D435i
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)

# Create a directory to save the frames and features
output_dir1 = "features_output/holes"
output_dir2 = "features_output/contours"
output_dir3 = "features_output/features"
output_dir4 = "features_output/data"
output_dir5 = "features_output/matches"

for output_dir in [output_dir1, output_dir2, output_dir3, output_dir4, output_dir5]:
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

frame_count = 0

# Define depth threshold for hole detection
hole_depth_threshold = 100.0  # Adjust as needed

# Create the ORB instance outside the loop
orb = cv2.ORB_create()

def load_previous_data(frame_count):
    """Load previous corner and contour data from CSV files."""
    corners_df = pd.read_csv(f'features_output/data/corners_{frame_count}.csv')
    return corners_df

def compare_frames(frame1, frame2):
    # Calculate MSE
    mse = np.mean((frame1.astype("float") - frame2.astype("float")) ** 2)
    return mse

def feature_matching(previous_image, current_image):
    orb = cv2.ORB_create()
    
    # Detect keypoints and descriptors
    kp1, des1 = orb.detectAndCompute(previous_image, None)
    kp2, des2 = orb.detectAndCompute(current_image, None)

    # Check keypoints count
    if len(kp1) < 10 or len(kp2) < 10:  # Threshold for minimum keypoints
        print("Not enough keypoints for matching.")
        return None, None, None

    # Match descriptors using BFMatcher
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    
    # Sort matches based on their distances
    matches = sorted(matches, key=lambda x: x.distance)
    
    # Apply Lowe's ratio test (optional)
    if len(matches) > 1:
        good_matches = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)
        matches = good_matches

    return kp1, kp2, matches

def draw_matches(img1, kp1, img2, kp2, matches):
    """Draw matches on the images."""
    img_matches = cv2.drawMatches(img1, kp1, img2, kp2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    return img_matches

try:
    while True:
        # Wait for a new frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert color image to grayscale
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Feature extraction: Detect corners using Shi-Tomasi corner detection
        corners = cv2.goodFeaturesToTrack(gray_image, maxCorners=100, qualityLevel=0.01, minDistance=10)
        corners = np.int0(corners)

        # Detect contours for potential holes
        _, thresh = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        holes = []
        other_features = []
        corner_data = []  # List to store hole data for saving

        # Draw corners on the frame
        for corner in corners:
            x, y = corner.ravel()
            cv2.circle(color_image, (x, y), 3, (0, 255, 0), -1)
            corner_data.append((frame_count, x, y))

        # Define circularity and area thresholds
        circularity_threshold = 0.6  # Adjust as needed
        area_threshold = 100  # Minimum area to consider a contour as a potential hole

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < area_threshold:
                continue
            
            # Calculate perimeter and circularity
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * np.pi * (area / (perimeter ** 2)) if perimeter != 0 else 0
            
            # Check for circular shape based on circularity and area
            if circularity >= circularity_threshold:
                holes.append(contour)  # Add to holes if meets criteria
            else:
                other_features.append(contour)

        # Draw contours
        cv2.drawContours(color_image, holes, -1, (255, 0, 0), 2)  # Draw holes in blue
        cv2.drawContours(color_image, other_features, -1, (0, 0, 255), 2)  # Draw other features in red

        # Visualization
        cv2.imshow("Frame", color_image)

        # Save frame and features every 10 frames
        if frame_count % 25 == 0:
            frame_filename = os.path.join(output_dir3, f"frame_{frame_count}.png")
            cv2.imwrite(frame_filename, color_image)

            # Save holes
            if holes:
                holes_filename = os.path.join(output_dir1, f"holes_{frame_count}.png")
                holes_image = color_image.copy()
                cv2.drawContours(holes_image, holes, -1, (0, 0, 0), 2)  # Draw holes in white
                cv2.imwrite(holes_filename, holes_image)

            # Save other features
            if other_features:
                features_filename = os.path.join(output_dir2, f"features_{frame_count}.png")
                features_image = color_image.copy()
                cv2.drawContours(features_image, other_features, -1, (0, 0, 255), 2)  # Draw features in white
                cv2.imwrite(features_filename, features_image)

            # Save corner data to CSV
            if corner_data:
                df = pd.DataFrame(corner_data, columns=['frame', 'x', 'y'])
                csv_filename = os.path.join(output_dir4, f"corners_{frame_count}.csv")
                df.to_csv(csv_filename, index=False)

            # Load previous data for feature matching
            if frame_count > 0:  # Ensure there's a previous frame to compare
                previous_color_image = cv2.imread(os.path.join(output_dir3, f"frame_{frame_count - 25}.png"))
                if previous_color_image is not None:
                    # Calculate frame similarity
                    mse = compare_frames(previous_color_image, color_image)
                    print(f"MSE between frames: {mse}")
                    
                    if mse < 0.75:  # Define a suitable threshold
                        # Perform feature matching
                        kp1, kp2, matches = feature_matching(previous_color_image, color_image)

                        if len(matches) > 0:
                            img_matches = draw_matches(previous_color_image, kp1, color_image, kp2, matches)
                            match_filename = os.path.join(output_dir5, f"matches_{frame_count}.png")
                            cv2.imwrite(match_filename, img_matches)
                            print(f"Detected {len(matches)} matches.")
                        else:
                            print("No matches detected.")
                    else:
                        print("Frames are too different, skipping matching.")

        frame_count += 1

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipeline
    pipeline.stop()
    cv2.destroyAllWindows()
