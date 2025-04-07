import os

# Get the current directory
directory = "/its/home/drs25/Documents/GitHub/poppy-humanoid/hardware/URDF/meshes"
# Loop through all files in the directory
for filename in os.listdir(directory):
    # Split filename and extension
    name, ext = os.path.splitext(filename)

    # If the extension is any case variation of .stl
    if ext.lower() == '.stl' and ext != '.STL':
        new_name = name + '.STL'
        old_path = os.path.join(directory, filename)
        new_path = os.path.join(directory, new_name)

        # Rename the file
        os.rename(old_path, new_path)
        print(f"Renamed: {filename} → {new_name}")

print("✅ All mesh extensions normalized to .STL")
