# GeoStitch

GeoStitch computes a **geodesic parameterization** of a triangulated 3D mesh and
visualizes the result in an interactive 3D viewer. The parameterization maps
every vertex to a `(row, stitch)` coordinate pair suitable for generating crochet
patterns from arbitrary surfaces.

The key idea: instead of cutting a mesh open and flattening it (which distorts
angles and areas), GeoStitch assigns coordinates that follow the actual curvature
of the surface — rows flow along lines of equal geodesic distance, and stitches
increase perpendicular to those rows.

> For a full explanation of the math and algorithms, see [ALGORITHMS.md](ALGORITHMS.md).

---

## How it works (overview)

Two scalar fields are computed over the mesh surface and stored at each vertex:

| Field | Meaning | How |
|---|---|---|
| **f** | Row coordinate — geodesic distance from the seed vertex | Dijkstra on the mesh graph |
| **g** | Stitch coordinate — angular position around each row | BFGS optimization |

Once both fields are known, a vertex at position `v` maps to crochet coordinates
`(row, stitch) = (f(v), g(v))`.

A **seam** (the geodesic path from seed to antipode) is fixed as `g = 0` so the
angular coordinate is well-defined on a closed surface.

---

## Requirements

- Java 17 or later
- Gradle (or use `./gradlew` after running `gradle wrapper` once)

No manual dependency downloads needed — jMonkeyEngine and JUnit 5 are fetched
from Maven Central on first build.

---

## Build and run

```bash
gradle run
```

The first run downloads jMonkeyEngine (~50 MB). Subsequent runs are fast. The
optimization runs in a background thread so the 3D window appears immediately;
the visualization fills in when computation finishes (a few seconds for the
bundled sphere mesh).

## Run tests

```bash
gradle test
```

Unit tests cover `areClose`, `groupByRows`, and `findClosestVertex` using
JUnit 5.

## CI

GitHub Actions runs `gradle test` on every push and pull request to `main`.
See [.github/workflows/ci.yml](.github/workflows/ci.yml).

---

## Controls

| Input | Action |
|---|---|
| W / A / S / D | Fly forward / left / back / right |
| Right-click + drag | Rotate camera |
| Scroll wheel | Zoom |

---

## What you see

The viewer shows three regions side by side:

```
[magenta dots]   [gray sphere]   [colored dots]
   (left)           (center)         (right)
```

| Object | Description |
|---|---|
| **Gray sphere (center)** | The input mesh |
| **Colored dots on sphere** | Vertices on row 11, colored by their (f, g) value |
| **Red arrows** | `grad(g)` vectors at those vertices — should point "around" the sphere |
| **Blue dots** | The geodesic seam where `g = 0` |
| **Colored dots (right)** | Row 11 vertices plotted in (g, f) parameter space |
| **Magenta dots (left)** | First half of all mesh vertices, offset left |

---

## Project structure

```
src/
  Main.java     — entry point, parameterization algorithm, visualization
  Vertex.java   — mesh vertex: position, adjacency, f/g fields, gradient fields
  Params.java   — records BFGS run statistics for logging
res/
  sphere.obj    — input mesh (482 vertices, ~960 triangles)
  sphere.mtl    — material for the OBJ loader
test/
  MainTest.java — JUnit 5 unit tests
build.gradle    — Gradle build configuration
ALGORITHMS.md  — detailed explanation of all algorithms and math
```

---

## Changing the input mesh

Replace `res/sphere.obj` with any triangulated OBJ file. The seed vertex is
found as the point nearest to `(0, -1, 0)`, so a mesh centered at the origin
works best. Non-manifold meshes or meshes with disconnected components may
produce unexpected results.
