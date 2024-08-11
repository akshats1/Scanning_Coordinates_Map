import pyvips
import os

# Directory containing the TIFF images
input_dir = "/media/akshat/DATA/August/Images_Scan_UI/"
output_dir = "/media/akshat/DATA/August/Images_Scan_UI/SVS_Images"

# Ensure the output directory exists
os.makedirs(output_dir, exist_ok=True)

# Iterate over all files in the directory
for filename in os.listdir(input_dir):
    if filename.endswith(".tiff") or filename.endswith(".tif"):
        input_path = os.path.join(input_dir, filename)
        output_filename = filename.replace(".tiff", ".svs").replace(".tif", ".svs")
        output_path = os.path.join(output_dir, output_filename)

        # Load the TIFF image
        image = pyvips.Image.new_from_file(input_path, access="sequential")

        # Save as a pyramidal TIFF, mimicking the SVS format
        image.tiffsave(output_path, pyramid=True, tile=True, tile_width=4096, tile_height=4096, compression="none", bigtiff=True, Q=95)

        print(f"Converted {filename} to {output_filename}")

print("Conversion completed for all TIFF images.")

