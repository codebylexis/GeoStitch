# Algorithms and Math — GeoStitch

This document explains the mathematical ideas behind GeoStitch's geodesic
parameterization pipeline. The goal is to assign every vertex on a triangulated
3D mesh two coordinates:

- **f** — a "row" coordinate equal to geodesic distance from a chosen seed vertex
- **g** — a "stitch" coordinate that increases perpendicular to the rows

Together, `(f, g)` gives every vertex a unique address in a flat parameter
domain, analogous to `(row, stitch)` in a crochet pattern.

---

## 1. Graph representation of the mesh

A triangulated mesh is treated as an undirected weighted graph `G = (V, E)`:

- **Vertices** `V` — the mesh vertices, each storing a 3D position `pos ∈ ℝ³`
- **Edges** `E` — two vertices share an edge if they appear together in any triangle
- **Edge weight** `w(u, v) = |pos_u − pos_v|` — the Euclidean distance between them

Adjacency is built in `Main.runComputation` by iterating over every triangle and
adding each pair of triangle corners as neighbors (with duplicate checks).

---

## 2. Geodesic distance — computing f

### What is geodesic distance?

The **geodesic distance** between two points on a surface is the length of the
shortest path that stays on the surface. On a smooth sphere of radius 1 it equals
the arc length; on a triangle mesh we approximate it by the shortest path along
mesh edges.

### Dijkstra's algorithm

GeoStitch uses **Dijkstra's single-source shortest-path algorithm** seeded at the
bottom pole of the mesh (the vertex nearest to `(0, −1, 0)`).

**Algorithm** (see `Main.computeGeodesicDistance`):

```
Initialize: f(seed) = 0,  f(v) = ∞  for all other v
Priority queue Q ordered by f value

While Q is not empty:
    u = vertex with minimum f in Q
    Mark u as finalized
    For each neighbor v of u (not yet finalized):
        candidate = f(u) + |pos_u − pos_v|
        If candidate < f(v):
            f(v) = candidate
            Re-insert v in Q with updated priority
```

After the algorithm finishes, `f(v)` holds the length of the shortest edge-path
from the seed to `v`. This is a graph-distance approximation to the true
geodesic; finer meshes give a better approximation.

**Complexity:** `O((|V| + |E|) log |V|)` with a binary-heap priority queue.

---

## 3. Grouping vertices into rows

Vertices at the same geodesic distance should belong to the same crochet row.
Because floating-point Dijkstra distances are rarely exactly equal, vertices with
`|f(u) − f(v)| < ε` (where `ε = 1e-3`) are placed in the same row.

**Efficient grouping** (`Main.groupByRows`) uses a `TreeMap<Double, List<Vertex>>`
(a sorted map keyed by the row's representative f value). For each new vertex:

1. Query `floorKey(f(v))` — the largest existing key ≤ f(v)
2. Query `ceilingKey(f(v))` — the smallest existing key ≥ f(v)
3. If either is within ε of f(v), add the vertex to that row
4. Otherwise, create a new row

Because the sorted map is used, the only possible match is one of these two
immediate neighbors — no full scan is needed. This gives **O(n log n)** total
rather than O(n²).

---

## 4. The seam — making g well-defined

On a closed surface (like a sphere) any angular coordinate must have a
discontinuity somewhere, just like longitude has a seam at the antimeridian.
GeoStitch uses a **geodesic path from the seed to the antipodal vertex** as the
seam. Vertices on this path are fixed to `g = 0`; the optimizer treats them as
boundary conditions.

The seam is found with a second Dijkstra run (`Main.findLongestGeodesic`), this
time tracking predecessor pointers so the full path can be reconstructed by
walking back from the farthest vertex.

---

## 5. Discrete gradients

To compute `g` we need a notion of gradient on the mesh. For a scalar field `h`
defined at vertices, the **discrete gradient at vertex v** is estimated from its
neighbors:

```
∇h(v) ≈ normalize( Σ_{u ∈ neighbors(v)}  (pos_u − pos_v) · (h(u) − h(v)) )
```

Each term `(pos_u − pos_v) · (h(u) − h(v))` is a scaled edge vector pointing
from `v` toward `u`, weighted by how much `h` changes along that edge. Summing
over all neighbors and normalizing gives a unit vector in the direction of
greatest increase of `h`.

This is implemented in `Main.computeGradientF` and `Main.computeGradientG`.

---

## 6. The tangent direction

For `g` to increase "around" each row, its gradient should point perpendicular
to `∇f` within the tangent plane of the surface. This target direction is called
the **tangent** or rotated gradient of f, written `τ(v)`.

For a sphere, the surface normal at vertex `v` is just `n(v) = v / |v|`.
The tangent direction is then:

```
τ(v) = normalize(∇f(v)) × n(v)
```

The cross product of a vector in the tangent plane with the surface normal gives
a vector that is:
- perpendicular to `∇f` (so it points "sideways" rather than "uphill")
- still in the tangent plane (so it stays on the surface)

This is computed in `Main.computeTangent`.

---

## 7. Optimization for g — BFGS

### Objective function

We want `∇g(v)` to match `τ(v)` at every vertex, specifically to satisfy
`τ(v) · ∇g(v) = 1`. The mismatch at each vertex is:

```
e(v) = τ(v) · ∇g(v) − 1
```

The overall objective is the sum of squared errors:

```
minimize  F(g) = Σ_v  e(v)²  =  Σ_v  ( τ(v) · ∇g(v) − 1 )²
```

The variables being optimized are the scalar `g` values at every non-boundary
vertex.

### Partial derivative (gradient of F)

Differentiating with respect to `g(v)` (holding all other values fixed):

```
∂F/∂g(v) = 2 · e(v)
```

So the gradient vector is simply `[2e(v₁), 2e(v₂), ..., 2e(vₙ)]`.

### BFGS quasi-Newton method

Gradient descent would update `g ← g − α ∇F` each step, which converges slowly
on poorly-conditioned problems. **BFGS** (Broyden–Fletcher–Goldfarb–Shanno)
improves on this by maintaining an approximation `H` to the *inverse Hessian*
(the inverse of the matrix of second derivatives) and using it to scale and
rotate the gradient into a better search direction.

**Each iteration:**

1. **Search direction:**  `p = −H · ∇F`

   Instead of stepping straight down the gradient, multiply by `H` to account
   for curvature. Early on `H = I` (identity), so the first step is pure
   gradient descent.

2. **Line search:** find step size `α` via Armijo backtracking (Section 8 below)

3. **Update variables:** `g ← g + α · p`, giving displacement `s = α · p`

4. **Recompute gradient:** `∇F_new`, giving gradient change `y = ∇F_new − ∇F`

5. **BFGS Hessian update** (rank-2 correction):

```
ρ_k  =  1 / (yᵀ s)

H_{k+1}  =  H_k  −  H_k (s yᵀ / yᵀs)  −  (y sᵀ / yᵀs) H_k  +  (s sᵀ / yᵀs)
```

This update keeps `H` symmetric and ensures it satisfies the **secant condition**
`H_{k+1} y = s` (the new Hessian approximation correctly predicts how the
gradient changed along the last step).

**Convergence:** the loop terminates when:
- The change in objective value is below `1e-12` (converged), or
- The gradient norm is below `1e-12` (at a critical point), or
- `yᵀs` is NaN (numerical breakdown), or
- `maxIterations` is reached

GeoStitch uses **full-memory BFGS**, meaning the entire `n × n` matrix `H` is
stored. This is exact but costs `O(n²)` memory. For large meshes this would
become a bottleneck; L-BFGS (limited-memory BFGS) would be the natural upgrade.

---

## 8. Armijo line search

After computing the search direction `p`, a step size `α` must be chosen small
enough that the objective actually decreases. The **Armijo sufficient decrease
condition** requires:

```
F(g + α p) ≤ F(g) + c · α · ∇F · p
```

where `c ∈ (0, 1)` is a small constant (here `c ≈ 0.115`). The right-hand side
is a linear approximation to `F` along `p`, scaled down by `c` to accept any
"sufficiently steep" decrease rather than exact minimization.

If the condition fails, the step is shrunk by factor `ρ ≈ 0.758` and checked
again. This backtracking loop terminates quickly (usually within a handful of
reductions) and guarantees that `α` is not so large that it overshoots.

---

## 9. Summary of the pipeline

```
Load OBJ mesh
    │
    ▼
Build adjacency graph (vertex neighbors from triangle list)
    │
    ▼
Dijkstra from seed → f values (row coordinate)
    │
    ▼
Group vertices into rows by f value  (threshold ε = 1e-3)
    │
    ▼
Dijkstra again → seam path (geodesic from seed to antipode)
    │
    ▼
Fix seam vertices to g = 0; initialize interior to g = 1
    │
    ▼
BFGS optimization → g values (stitch coordinate)
  ├── compute discrete gradients ∇f, ∇g
  ├── compute tangent τ = normalize(∇f) × normal
  ├── evaluate objective Σ(τ·∇g − 1)²
  ├── Armijo line search for step size
  └── rank-2 BFGS Hessian update
    │
    ▼
Visualize (f, g) parameterization in jMonkeyEngine
```

---

## References

- Dijkstra, E. W. (1959). *A note on two problems in connexion with graphs.*
  Numerische Mathematik, 1(1), 269–271.
- Nocedal, J. & Wright, S. J. (2006). *Numerical Optimization* (2nd ed.).
  Springer. — Chapter 6 (quasi-Newton methods) and Chapter 3 (line search).
- Surazhsky, V., et al. (2005). *Fast exact and approximate geodesics on meshes.*
  ACM SIGGRAPH. — Context for geodesic computation on triangle meshes.
