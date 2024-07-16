import h5py

def print_attrs(name, obj):
    """Prints the name and attributes of an object."""
    print(f"{name}:")
    for key, val in obj.attrs.items():
        print(f"    {key}: {val}")

def print_dataset(name, obj):
    """Prints the name and contents of a dataset."""
    print(f"{name}: {obj[()]}")
    print("")

def main(file_path):
    with h5py.File(file_path, 'r') as file:
        # Print file attributes
        print("File attributes:")
        for key, val in file.attrs.items():
            print(f"    {key}: {val}")

        # Traverse and print all groups, datasets and their attributes
        print("\nContents:")
        file.visititems(print_attrs)
        file.visititems(print_dataset)

if __name__ == "__main__":
    # Replace 'your_file.h5' with the path to your HDF5 file
    file_path = 'output.h5'
    main(file_path)
