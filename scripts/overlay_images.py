from PIL import Image

def create_timelapse(images, output_path):
    # Ensure there's at least one image
    if not images:
        print("No images provided.")
        return

    # Open the first image to start the overlay process
    base_image = Image.open(images[0]).convert("RGBA")

    # Loop through the rest of the images, overlaying them on top of the base image
    for i, image_path in enumerate(images[1:], start=1):
        # Open the image
        img = Image.open(image_path).convert("RGBA")
        
        # Calculate the transparency for the overlay
        # This makes each successive image more transparent
        alpha = int(255 * (i / len(images)))
        print('alpha',alpha)
        img.putalpha(alpha)
        
        # Overlay the image
        base_image = Image.alpha_composite(base_image, img)

    # Convert back to RGB to save in more common file formats
    base_image = base_image.convert("RGB")
    
    # Save the final composite image
    base_image.save(output_path)
    print(f"Time-lapse image saved to {output_path}")

# List of image paths
image_paths = [
    "/home/yif/Documents/KTH/dynamicCage/evaluation/real-world/circle-pushes-concave/0/captured_images/color_image_50.png",
    "/home/yif/Documents/KTH/dynamicCage/evaluation/real-world/circle-pushes-concave/0/captured_images/color_image_149.png",
]

# Path to save the final image
output_image_path = "/home/yif/Pictures/output.png"

create_timelapse(image_paths, output_image_path)