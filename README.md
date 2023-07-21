# NavMeshPathfinder

NavMeshPathfinder is Python-based project designed to solve the problem of finding paths in navigation meshes (navmeshes) constructed from user-provided images. The project includes a utility to build the navmeshes from the images, an interactive user interface (UI), and an implementation of a bidirectional A* search algorithm for pathfinding.

## Table of Contents

- [Getting Started](#getting-started)
- [Installing](#installing)
- [Usage](#usage)
- [Project Structure](#project-structure)

## Getting Started

To get started, you need to clone the repository and install the required dependencies. 

## Installing

Clone the repository to your local machine using the following command:

```bash
git clone https://github.com/CarterBloop/NavMeshPathfinder.git
```

Navigate to the cloned directory:

```bash
cd NavMeshPathfinder
```

## Usage

### Building a Navmesh

To create a navmesh from a provided image, use the `meshbuilder.py` program. This will produce a .mesh.pickle file used for pathfinding. Here's an example command:

```bash
python src/meshbuilder.py your_image.png
```

### Displaying and Pathfinding

The `interactive.py` script demonstrates the pathfinding on a provided image. This script takes three command line arguments: an image file to display (must be a PNG), the filename of a pickled mesh data structure (.mesh.pickle), and a subsampling factor. The subsampling factor is an integer used to scale down large images for display on small screens. Here's an example command:

```bash
python src/interactive.py image.png image.png.mesh.pickle 2
```

## Project Structure

The project is composed by input data (/input) and three Python scripts (src/):

- `src/interactive.py`: Runs the interactive UI for pathfinding. This script accepts a PNG image file, a pickled navmesh file (.mesh.pickle), and a subsampling factor as command line arguments.

- `src/meshbuilder.py`: Creates navmeshes from user-provided images. This script generates the '.mesh.pickle' files used by `interactive.py`.

- `src/pathfinder.py`: Contains the implementation of the bidirectional A* pathfinding algorithm.