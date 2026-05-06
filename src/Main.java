import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.TreeMap;

import com.jme3.app.SimpleApplication;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Triangle;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.debug.Arrow;
import com.jme3.scene.shape.Sphere;
import com.jme3.system.AppSettings;

/**
 * Loads a triangulated OBJ mesh, computes a geodesic parameterization of its
 * surface, and visualizes the result in 3D.
 *
 * The parameterization assigns two scalar fields to every vertex:
 *   f -- geodesic distance from a seed vertex (bottom pole), grouping vertices
 *        into roughly parallel "rows" like rows in a crochet pattern.
 *   g -- an angular coordinate computed by optimizing alignment between the
 *        gradient of g and the tangent direction perpendicular to grad(f).
 *
 * The optimization problem is: minimize sum_v (rotatedGradF_v . gradG_v - 1)^2
 * over all g values, solved with the BFGS quasi-Newton method.
 */
public class Main extends SimpleApplication {

    // Vertices within this geodesic-distance tolerance are placed in the same row.
    public static final double THRESHOLD = 1e-3;

    private List<Vertex> verticesList;
    private List<Params> paramsList = new ArrayList<>();

    public static void main(String[] args) {
        Main app = new Main();

        AppSettings settings = new AppSettings(true);
        settings.setResolution(1080, 720);
        settings.setVSync(true);
        // Gamma correction causes a black screen on some macOS/Metal drivers.
        settings.setGammaCorrection(false);
        app.setSettings(settings);
        app.setShowSettings(false);
        app.setDisplayFps(true);
        app.start();
    }

    @Override
    public void simpleInitApp() {
        viewPort.setBackgroundColor(new ColorRGBA(0.1f, 0.1f, 0.1f, 1f));

        Spatial mesh = assetManager.loadModel("sphere.obj");
        Material mat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
        mat.setColor("Diffuse", ColorRGBA.Gray);
        mat.setColor("Ambient", ColorRGBA.White.mult(0.1f));
        mat.setBoolean("UseMaterialColors", true);
        mesh.setMaterial(mat);
        rootNode.attachChild(mesh);
        mesh.setLocalTranslation(-3f, 0f, 0f);

        verticesList = new ArrayList<>();

        // OBJ files can load as either a Geometry (single mesh group) or a Node
        // (multiple groups). Handle both so the code works with any OBJ file.
        Geometry geo = null;
        if (mesh instanceof Geometry) {
            geo = (Geometry) mesh;
        } else if (mesh instanceof com.jme3.scene.Node) {
            for (Spatial child : ((com.jme3.scene.Node) mesh).getChildren()) {
                if (child instanceof Geometry) { geo = (Geometry) child; break; }
            }
        }

        if (geo != null) {
            final Geometry finalGeo = geo;
            // Run the parameterization on a background thread so the render loop
            // is not blocked. Scene objects are added via enqueue() when done.
            new Thread(() -> runComputation(finalGeo)).start();
        } else {
            System.out.println("Could not find Geometry in loaded model.");
        }

        AmbientLight ambient = new AmbientLight();
        ambient.setColor(ColorRGBA.White);
        rootNode.addLight(ambient);

        DirectionalLight sun = new DirectionalLight();
        sun.setDirection(new Vector3f(-1, -1, -1).normalizeLocal());
        sun.setColor(ColorRGBA.White);
        rootNode.addLight(sun);

        DirectionalLight backLight = new DirectionalLight();
        backLight.setDirection(new Vector3f(1, 1, 1).normalizeLocal());
        backLight.setColor(ColorRGBA.White);
        rootNode.addLight(backLight);

        cam.setLocation(new Vector3f(0, 0, 10));
        cam.lookAt(Vector3f.ZERO, Vector3f.UNIT_Y);
        flyCam.setMoveSpeed(10f);
        flyCam.setZoomSpeed(10f);
        flyCam.setDragToRotate(true);
        inputManager.setCursorVisible(true);
    }

    /**
     * Builds the vertex graph, computes the geodesic parameterization, runs BFGS
     * optimization, then hands the resulting geometry off to the render thread.
     */
    private void runComputation(Geometry geo) {
        Mesh m = geo.getMesh();

        // Build an adjacency graph from the triangle list.
        for (int i = 0; i < m.getTriangleCount(); i++) {
            Triangle t = new Triangle();
            m.getTriangle(i, t);
            Vertex v1 = findOrCreateVertex(t.get1(), verticesList);
            Vertex v2 = findOrCreateVertex(t.get2(), verticesList);
            Vertex v3 = findOrCreateVertex(t.get3(), verticesList);
            if (!v1.neighbors.contains(v2)) v1.neighbors.add(v2);
            if (!v1.neighbors.contains(v3)) v1.neighbors.add(v3);
            if (!v2.neighbors.contains(v1)) v2.neighbors.add(v1);
            if (!v2.neighbors.contains(v3)) v2.neighbors.add(v3);
            if (!v3.neighbors.contains(v1)) v3.neighbors.add(v1);
            if (!v3.neighbors.contains(v2)) v3.neighbors.add(v2);
        }

        // Seed the geodesic computation from the bottom pole of the sphere.
        Vertex seed = findClosestVertex(new Vector3f(0f, -1f, 0f), verticesList);
        computeGeodesicDistance(verticesList, seed);

        // Group vertices by geodesic distance into discrete rows.
        TreeMap<Double, List<Vertex>> rows = groupByRows(verticesList);

        // The geodesic from seed to the antipodal point serves as the seam (boundary)
        // where g wraps from 0 back to its starting value.
        List<Vertex> boundary = findLongestGeodesic(verticesList, seed);

        long start = System.nanoTime();

        // Fix boundary vertices to g=0 and initialize interior vertices to g=1.
        for (Vertex v : verticesList) {
            for (Vertex b : boundary) {
                if (v.equals(b)) { v.isBoundary = true; v.g = 0d; }
            }
        }
        for (Map.Entry<Double, List<Vertex>> entry : rows.entrySet()) {
            for (Vertex v : entry.getValue()) {
                if (!v.isBoundary) v.g = 1d;
            }
        }

        computeG(verticesList, 10000);
        System.out.println("optimization completed");

        Collections.sort(paramsList, Comparator.comparingDouble(p -> p.meanOfErrors));
        for (Params p : paramsList) System.out.println(p.toString());

        long elapsed = System.nanoTime() - start;
        long s = elapsed / 1_000_000_000, h = s / 3600;
        s %= 3600;
        System.out.printf("%02d:%02d:%02d%n", h, s / 60, s % 60);

        List<com.jme3.scene.Geometry> toAdd = buildVisualization(rows, boundary);

        // Attach all scene objects on the render thread.
        enqueue(() -> { for (com.jme3.scene.Geometry g : toAdd) rootNode.attachChild(g); });
    }

    /**
     * Constructs all visualization geometry after the optimization has finished.
     * Returns a list of objects ready to be attached to the scene graph.
     *
     * Three sets of objects are created:
     *   - Colored markers on row 11 showing the (f, g) parameterization values.
     *   - Red gradient arrows (grad g) on that same row.
     *   - Blue markers along the geodesic seam (boundary).
     *   - Magenta markers showing the first half of all vertices offset to the left.
     */
    private List<com.jme3.scene.Geometry> buildVisualization(
            TreeMap<Double, List<Vertex>> rows, List<Vertex> boundary) {

        List<com.jme3.scene.Geometry> toAdd = new ArrayList<>();

        int index = 0;
        for (Map.Entry<Double, List<Vertex>> entry : rows.entrySet()) {
            List<Vertex> r = entry.getValue();
            if (index != 11) { index++; continue; }

            for (int i = 0; i < r.size(); i++) {
                Vertex vi = r.get(i);

                // Arrow showing the gradient of g at this vertex.
                com.jme3.scene.Geometry gArrow = new com.jme3.scene.Geometry(
                        "gradG", new Arrow(new Vector3f(vi.gradG)));
                Material gmat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                gmat.getAdditionalRenderState().setWireframe(true);
                gmat.getAdditionalRenderState().setLineWidth(4);
                gmat.setColor("Color", ColorRGBA.Red);
                gArrow.setMaterial(gmat);
                gArrow.setLocalTranslation(vi.pos);
                toAdd.add(gArrow);

                // Color encodes the combined (f, g) coordinate as a hue value.
                ColorRGBA color = hslToRgb(((float) vi.f + (float) vi.g) / 2f, 0.5f, 0.5f);

                // Marker on the mesh surface at the vertex position.
                com.jme3.scene.Geometry mg = new com.jme3.scene.Geometry(
                        "marker", new Sphere(8, 8, 0.01f));
                mg.setMaterial(solidColor(color));
                mg.setLocalTranslation(vi.pos);
                toAdd.add(mg);

                // Marker in (g, f) parameter space, offset to the right of the mesh.
                com.jme3.scene.Geometry mg2 = new com.jme3.scene.Geometry(
                        "paramMarker", new Sphere(8, 8, 0.01f));
                mg2.setMaterial(solidColor(color));
                mg2.setLocalTranslation(new Vector3f((float) vi.g, (float) vi.f, 0f).addLocal(3f, 0f, 0f));
                toAdd.add(mg2);
            }
            index++;
        }

        // Mark the geodesic seam in blue.
        for (Vertex v : boundary) {
            com.jme3.scene.Geometry mg = new com.jme3.scene.Geometry(
                    "seam", new Sphere(8, 8, 0.01f));
            mg.setMaterial(solidColor(new ColorRGBA(0f, 0f, 1f, 1f)));
            mg.setLocalTranslation(v.pos);
            toAdd.add(mg);
        }

        // Show the first half of all vertices as magenta dots, offset to the left.
        for (int i = 0; i < verticesList.size() / 2; i++) {
            Vertex v = verticesList.get(i);
            com.jme3.scene.Geometry mg = new com.jme3.scene.Geometry(
                    "vert", new Sphere(8, 8, 0.01f));
            mg.setMaterial(solidColor(new ColorRGBA(1f, 0f, 1f, 1f)));
            mg.setLocalTranslation(v.pos.add(-6f, 0f, 0f));
            toAdd.add(mg);
        }

        return toAdd;
    }

    private Material solidColor(ColorRGBA color) {
        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setColor("Color", color);
        return mat;
    }

    /**
     * Computes the tangent direction at a vertex perpendicular to grad(f) within
     * the tangent plane of the surface. Used as the target direction for grad(g).
     */
    public Vector3f computeTangent(Vertex v) {
        Vector3f normalizedGradF = v.gradF.normalize();
        // For a sphere the vertex position is the surface normal.
        Vector3f normal = v.pos.normalize();
        return normalizedGradF.cross(normal).normalizeLocal();
    }

    /**
     * Runs BFGS quasi-Newton optimization to find g values such that
     * grad(g) aligns with the tangent direction perpendicular to grad(f).
     *
     * Objective: minimize sum_v (rotatedGradF_v . gradG_v - 1)^2
     *
     * The full-memory BFGS Hessian approximation (n x n matrix) is used.
     * The loop terminates on convergence, vanishing gradient, NaN, or
     * reaching maxIterations.
     */
    public void computeG(List<Vertex> verticesList, int maxIterations) {
        int n = verticesList.size();

        // Initialize inverse Hessian approximation as the identity matrix.
        double[][] H = new double[n][n];
        for (int i = 0; i < n; i++) H[i][i] = 1.0d;

        // Armijo line search parameters (c: sufficient decrease, rho: backtracking factor).
        double c = 0.11496810419916242;
        double rho = 0.7584440735714182;

        double prevObjectiveValue = Double.MAX_VALUE;
        Params params = new Params();

        for (int iter = 0; iter < maxIterations; iter++) {
            System.out.println("epoch: " + iter);

            double[] localGradients = new double[n];
            double[] p = new double[n];
            double sumOfErrors = 0d;

            for (int i = 0; i < n; i++) {
                Vertex v = verticesList.get(i);
                v.gradF = computeGradientF(v);
                v.rotatedGradF = computeTangent(v);

                if (!v.isBoundary) {
                    v.gradG = computeGradientG(v);
                } else {
                    v.gradG.set(v.rotatedGradF);
                }

                double error = v.rotatedGradF.dot(v.gradG) - 1d;
                sumOfErrors += error * error;
                localGradients[i] = 2d * error;
            }

            double meanError = sumOfErrors / (double) n;
            double currentObjectiveValue = meanError;

            if (Math.abs(currentObjectiveValue - prevObjectiveValue) < 1e-12) break;

            double gradientNorm = 0d;
            for (double grad : localGradients) gradientNorm += grad * grad;
            gradientNorm = Math.sqrt(gradientNorm);
            if (gradientNorm < 1e-12) break;

            // Compute search direction: p = -H * grad
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) p[i] -= H[i][j] * localGradients[j];
            }

            double[] s = new double[n];
            double[] y = new double[n];
            double[] newLocalGradients = new double[n];

            for (int i = 0; i < n; i++) {
                Vertex v = verticesList.get(i);
                double alpha = lineSearch(i, p, localGradients, 1.0d, c, rho);
                s[i] = alpha * p[i];
                v.g += s[i];
                v.gradF = computeGradientF(v);
                v.rotatedGradF = computeTangent(v);
                if (!v.isBoundary) {
                    v.gradG = computeGradientG(v);
                } else {
                    v.gradG.set(v.rotatedGradF);
                }
                newLocalGradients[i] = 2d * (v.rotatedGradF.dot(v.gradG) - 1d);
                y[i] = newLocalGradients[i] - localGradients[i];
            }

            double yTs = 0;
            for (int i = 0; i < n; i++) yTs += y[i] * s[i];

            // Hessian update is undefined when yTs is NaN (e.g., line search collapsed).
            if (Double.isNaN(yTs)) break;

            // BFGS rank-2 Hessian update.
            double[][] H_new = new double[n][n];
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    H_new[i][j] = H[i][j]
                            - H[i][j] * (s[i] * y[j] / yTs)
                            - (y[i] * s[j] / yTs) * H[i][j]
                            + (s[i] * s[j] / yTs);
                }
            }
            H = H_new;

            localGradients = newLocalGradients;
            prevObjectiveValue = currentObjectiveValue;
            params.update(iter, sumOfErrors, meanError, c, rho);
        }
        paramsList.add(params);
    }

    public double lineSearch(int vertexIndex, double[] p, double[] gradient,
                             double alphaStart, double c, double rho) {
        Vertex v = verticesList.get(vertexIndex);
        double alpha = alphaStart;
        double currentObjective = computeObjective(v, v.g);
        // Armijo sufficient decrease condition: backtrack until the condition is met.
        while (computeObjective(v, v.g + alpha * p[vertexIndex])
                > currentObjective + c * alpha * gradient[vertexIndex] * p[vertexIndex]) {
            alpha *= rho;
        }
        return alpha;
    }

    private double computeObjective(Vertex v, double gValue) {
        double tempG = v.g;
        v.g = gValue;
        v.gradG = computeGradientG(v);
        v.gradF = computeGradientF(v);
        v.rotatedGradF = computeTangent(v);
        double error = v.rotatedGradF.dot(v.gradG) - 1d;
        v.g = tempG;
        return error * error;
    }

    /** Discrete gradient of g at v, estimated from neighbor differences. */
    public Vector3f computeGradientG(Vertex v) {
        Vector3f gradient = new Vector3f();
        for (Vertex neighbor : v.neighbors) {
            Vector3f direction = neighbor.pos.subtract(v.pos);
            gradient = gradient.add(direction.scaleAdd((float) (neighbor.g - v.g), Vector3f.ZERO));
        }
        return gradient.normalizeLocal();
    }

    /** Discrete gradient of f at v, estimated from neighbor differences. */
    public Vector3f computeGradientF(Vertex v) {
        Vector3f gradient = new Vector3f();
        for (Vertex neighbor : v.neighbors) {
            Vector3f direction = neighbor.pos.subtract(v.pos);
            gradient = gradient.add(direction.scaleAdd((float) (neighbor.f - v.f), Vector3f.ZERO));
        }
        return gradient.normalizeLocal();
    }

    /**
     * Finds the geodesic path from seed to the farthest vertex using Dijkstra.
     * This path is used as the seam of the parameterization.
     */
    public List<Vertex> findLongestGeodesic(List<Vertex> vertices, Vertex seed) {
        PriorityQueue<Vertex> queue = new PriorityQueue<>(Comparator.comparingDouble(v -> v.distance));
        seed.distance = 0;
        queue.add(seed);

        while (!queue.isEmpty()) {
            Vertex current = queue.poll();
            if (current.distance == Double.MAX_VALUE) break;
            for (Vertex neighbor : current.neighbors) {
                double newDist = current.distance + current.pos.distance(neighbor.pos);
                if (newDist < neighbor.distance) {
                    queue.remove(neighbor);
                    neighbor.distance = newDist;
                    neighbor.predecessor = current;
                    queue.add(neighbor);
                }
            }
        }

        Vertex farthest = seed;
        for (Vertex v : vertices) {
            if (v.distance > farthest.distance) farthest = v;
        }

        List<Vertex> path = new ArrayList<>();
        while (farthest != null) { path.add(farthest); farthest = farthest.predecessor; }
        Collections.reverse(path);
        return path;
    }

    /** Dijkstra shortest-path distances from seed stored in each vertex's f field. */
    public void computeGeodesicDistance(List<Vertex> vertices, Vertex seed) {
        PriorityQueue<Vertex> queue = new PriorityQueue<>(Comparator.comparing(v -> v.f));
        seed.f = 0;
        queue.add(seed);
        while (!queue.isEmpty()) {
            Vertex current = queue.poll();
            current.finalized = true;
            for (Vertex neighbor : current.neighbors) {
                if (!neighbor.finalized) {
                    double newDist = current.f + current.pos.distance(neighbor.pos);
                    if (newDist < neighbor.f) {
                        neighbor.f = newDist;
                        queue.remove(neighbor);
                        queue.add(neighbor);
                    }
                }
            }
        }
    }

    public static boolean areClose(double a, double b) {
        return Math.abs(a - b) < THRESHOLD;
    }

    /**
     * Groups vertices into rows by their geodesic distance (f value).
     * Vertices within THRESHOLD of each other share a row key.
     */
    public static TreeMap<Double, List<Vertex>> groupByRows(List<Vertex> vertices) {
        TreeMap<Double, List<Vertex>> rows = new TreeMap<>();
        for (Vertex vertex : vertices) {
            boolean added = false;
            for (Double key : rows.keySet()) {
                if (areClose(vertex.f, key)) {
                    rows.get(key).add(vertex);
                    added = true;
                    break;
                }
            }
            if (!added) {
                List<Vertex> newRow = new ArrayList<>();
                newRow.add(vertex);
                rows.put(vertex.f, newRow);
            }
        }
        return rows;
    }

    public static Vertex findClosestVertex(Vector3f input, List<Vertex> vertices) {
        if (vertices == null || vertices.isEmpty()) return null;
        Vertex closest = vertices.get(0);
        double minDist = input.distance(closest.pos);
        for (Vertex vertex : vertices) {
            double curDist = input.distance(vertex.pos);
            if (curDist < minDist) { minDist = curDist; closest = vertex; }
        }
        return closest;
    }

    public Vertex findOrCreateVertex(Vector3f pos, List<Vertex> verticesList) {
        for (Vertex v : verticesList) {
            if (v.pos.equals(pos)) return v;
        }
        Vertex newVertex = new Vertex();
        newVertex.pos = pos;
        verticesList.add(newVertex);
        return newVertex;
    }

    // HSL to RGB conversion used to color-code parameterization values.
    public ColorRGBA hslToRgb(float h, float s, float l) {
        float r, g, b;
        if (s == 0) {
            r = g = b = l;
        } else {
            float q = l < 0.5 ? l * (1 + s) : l + s - l * s;
            float p = 2 * l - q;
            r = hueToRgb(p, q, h + 1 / 3f);
            g = hueToRgb(p, q, h);
            b = hueToRgb(p, q, h - 1 / 3f);
        }
        return new ColorRGBA(r, g, b, 1f);
    }

    private float hueToRgb(float p, float q, float t) {
        if (t < 0) t += 1;
        if (t > 1) t -= 1;
        if (t < 1 / 6f) return p + (q - p) * 6 * t;
        if (t < 1 / 2f) return q;
        if (t < 2 / 3f) return p + (q - p) * (2 / 3f - t) * 6;
        return p;
    }
}
