import os
import pymeshlab

# Create the 'simplified' directory if it doesn't exist
os.makedirs('simplified', exist_ok=True)

# Get a list of all STL files in the 'original' folder
# original_folder = 'original'
# simplified_folder = 'simplified'
original_folder = './meshes'
simplified_folder = './simplified'

for filename in os.listdir(original_folder):
    if filename.endswith('.STL'):  # Check if the file is an STL file
        input_path = os.path.join(original_folder, filename)
        output_path = os.path.join(simplified_folder, filename)

        # Create a new MeshLab project
        ms = pymeshlab.MeshSet()

        # Load the STL file
        ms.load_new_mesh(input_path)

        # Apply the quadric edge collapse decimation filter
        ms.apply_filter('meshing_decimation_quadric_edge_collapse', targetfacenum=10000)

        # Save the simplified STL
        ms.save_current_mesh(output_path)

        print(f"Processed and saved: {output_path}")

print("All STL files have been processed and saved.")
