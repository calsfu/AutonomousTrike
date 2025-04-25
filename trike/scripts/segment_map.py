from PIL import Image

def segment_by_gray_range(image_path, lower_gray_threshold, upper_gray_threshold, output_path):
    try:
        img = Image.open(image_path).convert("RGB")
        pixels = img.load()
        width, height = img.size

        white_rgb = (255, 255, 255)
        black_rgb = (0, 0, 0)

        for x in range(width):
            for y in range(height):
                r, g, b = pixels[x, y]
                # Check if the color is within the gray range
                if (lower_gray_threshold[0] <= r <= upper_gray_threshold[0] and \
                   lower_gray_threshold[1] <= g <= upper_gray_threshold[1] and \
                   lower_gray_threshold[2] <= b <= upper_gray_threshold[2]) or \
                   white_rgb == (r, g, b):
                    
                    pixels[x, y] = white_rgb
                else:
                    pixels[x, y] = black_rgb

        img.save(output_path)
        print(f"Image segmented by gray range and saved to {output_path}")

    except FileNotFoundError:
        print(f"Error: Image not found at {image_path}")
    except Exception as e:
        print(f"An error occurred during segmentation: {e}")

if __name__ == "__main__":
    input_image = "maps/earth_left_turn.png"  # Replace with your file name
    output_image = "segmented_by_gray.png"

    # Define the range for gray values (adjust these as needed) 
    # actual road: 172, 202, 202
    # gray arrow: 124, 135, 147
    lower_threshold = (1, 1, 1)
    upper_threshold = (200, 215, 215)

    segment_by_gray_range(input_image, lower_threshold, upper_threshold, output_image)