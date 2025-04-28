import cv2
import numpy as np
import sys

# --- Simulate the image_processing functions ---

def classifier_parameters():
    """
    Simulates getting color classification parameters.
    Returns example lower and upper bounds for green color in HSV.
    These values would need to be tuned based on the specific color
    you want to segment and lighting conditions.
    """
    # Example bounds for a green-ish color in HSV
    # Hue (0-179), Saturation (0-255), Value (0-255)
    lower_bound = np.array([150, 100, 100]) # Adjusted for hot pink
    upper_bound = np.array([170, 255, 255]) # Adjusted for hot pink
    return lower_bound, upper_bound

def image_segment(image, lower_bound, upper_bound):
    """
    Performs color segmentation on an image based on HSV bounds.
    Returns a binary mask where the target color is white (255).
    """
    # Convert the image from BGR (OpenCV default) to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create a mask using the specified color bounds
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    # Optional: Apply morphological operations to clean up the mask
    # kernel = np.ones((5, 5), np.uint8)
    # mask = cv2.erode(mask, kernel, iterations=1)
    # mask = cv2.dilate(mask, kernel, iterations=1)

    return mask

def image_centroid_horizontal(binary_image):
    """
    Calculates the horizontal centroid (x-coordinate) of the white pixels
    in a binary image. Returns -1 if no white pixels are found.
    """
    # Find contours in the binary image
    # Use cv2.RETR_EXTERNAL to find only outer contours
    # Use cv2.CHAIN_APPROX_SIMPLE to compress horizontal, vertical, and diagonal segments
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return -1 # Return -1 if no contours (segmented object) found

    # Combine all contours into a single mask to calculate the overall centroid
    contour_mask = np.zeros_like(binary_image)
    cv2.drawContours(contour_mask, contours, -1, 255, cv2.FILLED)

    # Calculate moments of the contour mask
    M = cv2.moments(contour_mask)

    # Calculate the centroid's x-coordinate
    if M["m00"] != 0:
        x_centroid = int(M["m10"] / M["m00"])
        return x_centroid
    else:
        return -1 # Return -1 if the area is zero (shouldn't happen if contours were found)

def image_one_to_three_channels(single_channel_image):
    """
    Converts a single-channel (grayscale/binary) image to a three-channel image.
    Useful for drawing color lines on a binary mask.
    """
    # Stack the single channel three times to create a BGR image
    three_channel_image = cv2.cvtColor(single_channel_image, cv2.COLOR_GRAY2BGR)
    return three_channel_image

def image_line_vertical(image, x_coordinate):
    """
    Draws a vertical line on the image at the specified x-coordinate.
    Returns the image with the line drawn.
    """
    if x_coordinate == -1:
        return image # Don't draw a line if centroid wasn't found

    # Get image dimensions
    height, width, _ = image.shape

    # Define the start and end points of the line
    start_point = (x_coordinate, 0)
    end_point = (x_coordinate, height)

    # Define the color of the line (e.g., red in BGR)
    line_color = (0, 0, 255) # BGR for Red

    # Define the thickness of the line
    line_thickness = 2

    # Draw the line on the image
    image_with_line = cv2.line(image, start_point, end_point, line_color, line_thickness)

    return image_with_line

# --- Main execution logic ---

def process_image(image_path):
    """
    Loads an image, performs segmentation and centroid calculation,
    draws a line, and displays the result.
    """
    # Load the image
    image = cv2.imread(image_path)

    if image is None:
        print(f"Error: Could not load image from {image_path}")
        return
    
    target_width = 1920
    target_height = 1080
    image = cv2.resize(image, (target_width, target_height))
    print(f"Resized image to {target_width}x{target_height}")

    # Get segmentation parameters
    lower_bound, upper_bound = classifier_parameters()

    # Segment the image
    segmented_mask = image_segment(image, lower_bound, upper_bound)

    # Compute the horizontal centroid
    x_centroid = image_centroid_horizontal(segmented_mask)

    # Prepare the segmented mask for drawing (convert to 3 channels)
    segmented_mask_color = image_one_to_three_channels(segmented_mask)

    # Draw the vertical line on the segmented image
    segmented_image_with_line = image_line_vertical(segmented_mask_color, x_centroid)

    # Display the results
    cv2.imshow("Original Image", image)
    cv2.imshow("Segmented Mask", segmented_mask)
    cv2.imshow("Segmented Image with Centroid Line", segmented_image_with_line)

    if x_centroid != -1:
        print(f"Horizontal Centroid X-coordinate: {x_centroid}")
    else:
        print("No segmented object found.")

    # Wait for a key press and then close the windows
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    """
    Main function to run the image processing.
    """
    image_file_path = 'images/IMG_2127.JPG'
    


    process_image(image_file_path)

if __name__ == "__main__":
    main()
