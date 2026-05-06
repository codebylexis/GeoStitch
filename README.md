# GeoStitch

Computes a geodesic parameterization of a triangulated 3D mesh and visualizes
the result in an interactive 3D viewer. The parameterization maps each vertex
to a (row, stitch) coordinate suitable for generating crochet patterns.

## How it works

Two scalar fields are computed over the mesh surface:

**f (row coordinate):** geodesic distance from a seed vertex at the bottom
pole of the mesh, computed with Dijkstra's algorithm on the triangle adjacency
graph. Vertices at the same geodesic distance form a "row" in the crochet sense.

**g (stitch coordinate):** an angular coordinate that increases perpendicular
to the rows. Computed by solving an optimization problem: find g values such that
the discrete gradient of g aligns with the tangent direction perpendicular to
grad(f) at every vertex. The objective is:

    minimize  sum_v  ( rotatedGrad(f)_v  .  grad(g)_v  -  1 )^2

This is solved with the full-memory BFGS quasi-Newton method with Armijo line
search. A geodesic path from the seed to the antipodal vertex is used as the
seam (where g resets to 0) to make the parameterization well-defined on a sphere.

## Requirements

- Java 11 or later
- Gradle (or use the wrapper after running `gradle wrapper`)
- No manual dependency downloads; jMonkeyEngine is fetched from Maven Central

## Build and run

```bash
gradle run
```

On the first run Gradle downloads jMonkeyEngine (~50 MB). Subsequent runs are
fast. The optimization runs in a background thread, so the 3D window appears
immediately and the visualization fills in when computation finishes (a few
seconds for the included sphere mesh).

## Controls

| Input | Action |
|---|---|
| W / A / S / D | Fly forward / left / back / right |
| Right-click + drag | Rotate camera |
| Scroll wheel | Zoom |

## What you see

- **Gray sphere (center):** the input mesh
- **Red arrows:** grad(g) vectors on one row of the mesh
- **Colored dots on mesh:** vertices on one row, colored by their (f, g) value
- **Colored dots to the right:** the same vertices plotted in (g, f) parameter space
- **Blue dots:** the geodesic seam
- **Magenta dots (left):** the first half of all mesh vertices

## Project structure

```
src/
  Main.java      — application entry point, parameterization algorithm, visualization
  Vertex.java    — mesh vertex with adjacency and parameterization state
  Params.java    — records BFGS run statistics
res/
  sphere.obj     — input mesh (482 vertices, ~960 triangles)
  sphere.mtl     — material for the OBJ loader
build.gradle     — Gradle build configuration
settings.gradle  — project name
```

## Changing the input mesh

Replace `res/sphere.obj` with any triangulated OBJ file. The seed vertex is
currently hardcoded to the point nearest to (0, -1, 0), so a mesh centered at
the origin works best. Non-manifold meshes or meshes with disconnected components
may produce unexpected results.
