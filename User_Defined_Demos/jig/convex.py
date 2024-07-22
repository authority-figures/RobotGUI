import trimesh


def decompose_to_convex(mesh):
    """
    Decompose a mesh into convex pieces using the convex decomposition algorithm.

    Parameters:
    mesh (trimesh.Trimesh): The input mesh to decompose.

    Returns:
    list of trimesh.Trimesh: A list containing the convex pieces of the mesh.
    """
    # Check if the mesh is watertight
    if not mesh.is_watertight:
        print("Warning: Mesh is not watertight, results may not be accurate.")

    # Perform convex decomposition
    convex_pieces = mesh.convex_decomposition()

    return convex_pieces


def save_decomposed_meshes(convex_pieces, output_directory):
    """
    Save each convex piece as a separate mesh file.

    Parameters:
    convex_pieces (list of trimesh.Trimesh): The list of convex pieces.
    output_directory (str): The directory to save the decomposed pieces.
    """
    for i, piece in enumerate(convex_pieces):
        filename = f"{output_directory}"
        piece.export(filename)
        print(f"Saved: {filename}")


if __name__ == "__main__":
    # Path to the input model
    input_filename = "F:/sw/urdf_files/c501-simple.SLDASM/processed-geoms/Link_C_part_down.obj"

    # Load the mesh from the file
    mesh = trimesh.load(input_filename)

    # Perform convex decomposition
    convex_pieces = decompose_to_convex(mesh)

    # Directory to save the convex pieces
    output_directory = "F:/sw/urdf_files/c501-simple.SLDASM/processed-geoms/Link_C_part_down_convex.obj"

    # Save the convex pieces to separate files
    save_decomposed_meshes(convex_pieces, output_directory)
