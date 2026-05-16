package com.pcl_mobile;

import android.app.Activity;
import android.graphics.Color;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.os.Bundle;
import android.view.Gravity;
import android.view.Window;
import android.widget.FrameLayout;
import android.widget.TextView;

import com.sirokujira.pclmobile.pclmobileJNILib;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.charset.StandardCharsets;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

public class MainActivity extends Activity {

    private static final String TAG = "MainActivity";
    private static final double VOXEL_LEAF_SIZE = 0.14;
    private final ExecutorService pclExecutor = Executors.newSingleThreadExecutor();
    private PointCloudSurface pointCloudSurface;
    private TextView statusText;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        super.onCreate(savedInstanceState);

        pointCloudSurface = new PointCloudSurface(this);
        statusText = makeStatusText();

        FrameLayout root = new FrameLayout(this);
        root.addView(pointCloudSurface, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.MATCH_PARENT));
        root.addView(statusText, makeOverlayParams());
        setContentView(root);

        pclExecutor.execute(this::runPclSample);
    }

    @Override
    protected void onResume() {
        super.onResume();
        pointCloudSurface.onResume();
    }

    @Override
    protected void onPause() {
        pointCloudSurface.onPause();
        super.onPause();
    }

    @Override
    protected void onDestroy() {
        pclExecutor.shutdownNow();
        super.onDestroy();
    }

    private TextView makeStatusText() {
        TextView text = new TextView(this);
        text.setText("pclMobile OpenGL point cloud\nLoading native PCL result...");
        text.setTextColor(Color.rgb(238, 244, 252));
        text.setTextSize(15.0f);
        text.setLineSpacing(2.0f, 1.05f);
        text.setPadding(24, 18, 24, 18);
        text.setBackgroundColor(Color.argb(180, 11, 16, 24));
        return text;
    }

    private FrameLayout.LayoutParams makeOverlayParams() {
        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.WRAP_CONTENT);
        params.gravity = Gravity.TOP | Gravity.START;
        return params;
    }

    private void runPclSample() {
        try {
            File samplePcd = writeSamplePcd();

            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            float[] rawPoints = pclmobileJNILib.getCloudPoints();
            float[] covarianceMatrix = pclmobileJNILib.computeCovarianceMatrix();
            float[] principalAxes = pclmobileJNILib.computePrincipalAxes();
            float[] momentOfInertiaAndObb = pclmobileJNILib.computeMomentOfInertiaAndOBB();
            float[] squaredDistancesToOrigin = pclmobileJNILib.computeSquaredDistancesToPoint(0.0f, 0.0f, 0.0f);
            float[] maxDistanceFromCentroid = pclmobileJNILib.computeMaxDistanceFromCentroid();
            float[] demeanedPoints = pclmobileJNILib.demeanActiveCloud();
            float[] translatedPoints = pclmobileJNILib.translateActiveCloud(0.04f, -0.02f, 0.03f);
            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            float[] rigidTransform = pclmobileJNILib.estimateRigidTransformSVD(translatedPoints);
            float[] transformedPoints = pclmobileJNILib.transformActiveCloud(rigidTransform);
            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            float[] targetIcpResult = pclmobileJNILib.alignToTargetICP(translatedPoints, 35, 0.20, 1.0e-8, 1.0e-8);
            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            float[] targetGicpResult = pclmobileJNILib.alignToTargetGICP(translatedPoints, 35, 0.20, 1.0e-8, 1.0e-8, 20);

            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            pclmobileJNILib.filterVoxelGrid(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
            float[] filteredPoints = pclmobileJNILib.getFilteredPoints();
            float[] centroidAndBounds = pclmobileJNILib.computeCentroidAndBounds();
            float[] normals = pclmobileJNILib.estimateNormals(16);
            float[] shotFeatures = pclmobileJNILib.computeSHOTFeatures(16, 0.18);
            float[] boundaryPoints = pclmobileJNILib.computeBoundaryPoints(16, 0.18, 90.0);
            float[] differenceOfNormals = pclmobileJNILib.computeDifferenceOfNormals(0.08, 0.20);
            float[] planeModel = pclmobileJNILib.segmentPlane(0.03, 100);
            float[] sphereModel = pclmobileJNILib.segmentSphere(0.05, 100);
            float[] nearestNeighbors = pclmobileJNILib.nearestKSearch(0.0f, 0.0f, 0.0f, 8);
            float[] octreeNeighbors = pclmobileJNILib.octreeRadiusSearch(0.0f, 0.0f, 0.0f, 0.10, 0.28);
            float[] clusterSizes = pclmobileJNILib.extractEuclideanClusters(0.18, 20, 5000);
            float[] convexHullPoints = pclmobileJNILib.computeConvexHull();
            float[] projectedPlanePoints = pclmobileJNILib.projectInliersToPlane(0.03, 100);
            float[] icpResult = pclmobileJNILib.alignToTranslatedCopyICP(0.05f, -0.03f, 0.02f, 35);

            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            pclmobileJNILib.filterAxisOutside("z", -0.10, 0.10);
            float[] passThroughOutsidePoints = pclmobileJNILib.getFilteredPoints();

            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            pclmobileJNILib.filterGridMinimum(0.10);
            float[] gridMinimumPoints = pclmobileJNILib.getFilteredPoints();

            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            pclmobileJNILib.filterNormalSpaceSampling(128, 17, 4, 4, 4, 16);
            float[] normalSpacePoints = pclmobileJNILib.getFilteredPoints();

            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            pclmobileJNILib.removeNaNFromActiveCloud();
            float[] finitePoints = pclmobileJNILib.getFilteredPoints();

            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            pclmobileJNILib.filterStatisticalOutlierRemoval(20, 1.0);
            float[] statisticalInliers = pclmobileJNILib.getFilteredPoints();

            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            pclmobileJNILib.filterRadiusOutlierRemoval(0.18, 3);
            float[] radiusInliers = pclmobileJNILib.getFilteredPoints();

            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            pclmobileJNILib.filterCropBox(-0.60, -0.50, -0.40, 0.85, 0.55, 0.75);
            float[] cropBoxPoints = pclmobileJNILib.getFilteredPoints();

            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            pclmobileJNILib.extractPlaneInliers(0.03, 100);
            float[] extractedPlaneInliers = pclmobileJNILib.getFilteredPoints();

            pclmobileJNILib.load(samplePcd.getAbsolutePath());
            pclmobileJNILib.extractModelOutliers(pclmobileJNILib.SACMODEL_PLANE, 0.03, 100);
            float[] extractedPlaneOutliers = pclmobileJNILib.getFilteredPoints();

            if (rawPoints.length == 0 || filteredPoints.length == 0) {
                throw new IllegalStateException("PCL returned an empty point cloud");
            }

            int rawCount = rawPoints.length / 3;
            int filteredCount = filteredPoints.length / 3;
            int centroidStatsCount = centroidAndBounds.length;
            int covarianceStatsCount = covarianceMatrix.length;
            int principalAxisStatsCount = principalAxes.length;
            int momentStatsCount = momentOfInertiaAndObb.length;
            int distanceCount = squaredDistancesToOrigin.length;
            int demeanedPointCount = demeanedPoints.length / 3;
            int translatedPointCount = translatedPoints.length / 3;
            int transformedPointCount = transformedPoints.length / 3;
            int normalCount = normals.length / 4;
            int shotDescriptorCount = shotFeatures.length / 352;
            int boundaryPointCount = boundaryPoints.length / 4;
            int differenceOfNormalsCount = differenceOfNormals.length / 4;
            int sphereInlierCount = sphereModel.length >= 5 ? Math.round(sphereModel[4]) : 0;
            int passThroughOutsidePointCount = passThroughOutsidePoints.length / 3;
            int gridMinimumPointCount = gridMinimumPoints.length / 3;
            int normalSpacePointCount = normalSpacePoints.length / 3;
            int finitePointCount = finitePoints.length / 3;
            int statisticalInlierCount = statisticalInliers.length / 3;
            int radiusInlierCount = radiusInliers.length / 3;
            int cropBoxPointCount = cropBoxPoints.length / 3;
            int planeInlierCount = planeModel.length >= 5 ? Math.round(planeModel[4]) : 0;
            int extractedPlaneInlierCount = extractedPlaneInliers.length / 3;
            int extractedPlaneOutlierCount = extractedPlaneOutliers.length / 3;
            int nearestNeighborCount = nearestNeighbors.length / 4;
            int octreeNeighborCount = octreeNeighbors.length / 4;
            int clusterCount = clusterSizes.length;
            int convexHullPointCount = convexHullPoints.length / 3;
            int projectedPlanePointCount = projectedPlanePoints.length / 3;
            boolean icpConverged = icpResult.length > 0 && icpResult[0] == 1.0f;
            boolean targetIcpConverged = targetIcpResult.length > 0 && targetIcpResult[0] == 1.0f;
            boolean targetGicpConverged = targetGicpResult.length > 0 && targetGicpResult[0] == 1.0f;
            int reduction = Math.round((1.0f - (filteredCount / (float) rawCount)) * 100.0f);
            android.util.Log.i(TAG, "PCL OpenGL sample completed: raw=" + rawCount
                    + " filtered=" + filteredCount + " centroidStats=" + centroidStatsCount
                    + " covarianceStats=" + covarianceStatsCount
                    + " pcaStats=" + principalAxisStatsCount + " momentStats=" + momentStatsCount
                    + " distances=" + distanceCount + " maxDistanceTuple=" + maxDistanceFromCentroid.length
                    + " demeaned=" + demeanedPointCount + " translated=" + translatedPointCount
                    + " rigidTransform=" + rigidTransform.length + " transformed=" + transformedPointCount
                    + " normals=" + normalCount + " shot=" + shotDescriptorCount
                    + " boundary=" + boundaryPointCount + " don=" + differenceOfNormalsCount
                    + " sphereInliers=" + sphereInlierCount
                    + " planeInliers=" + planeInlierCount + " extractedPlane=" + extractedPlaneInlierCount
                    + " planeOutliers=" + extractedPlaneOutlierCount
                    + " projectedPlane=" + projectedPlanePointCount
                    + " passThroughOutside=" + passThroughOutsidePointCount
                    + " gridMinimum=" + gridMinimumPointCount + " normalSpace=" + normalSpacePointCount
                    + " finite=" + finitePointCount
                    + " sor=" + statisticalInlierCount + " radius=" + radiusInlierCount
                    + " cropBox=" + cropBoxPointCount
                    + " nearest=" + nearestNeighborCount + " octree=" + octreeNeighborCount
                    + " clusters=" + clusterCount + " hull=" + convexHullPointCount
                    + " icp=" + icpConverged + " targetIcp=" + targetIcpConverged
                    + " targetGicp=" + targetGicpConverged
                    + " reduction=" + reduction + "% file=" + samplePcd);
            runOnUiThread(() -> {
                statusText.setText("pclMobile OpenGL point cloud\n"
                        + "Raw PCD: " + rawCount + " pts   VoxelGrid: " + filteredCount
                        + " pts   -" + reduction + "%\n"
                        + "Normals: " + normalCount + "   SHOT: " + shotDescriptorCount
                        + "   Boundary: " + boundaryPointCount
                        + "\nPlane inliers: " + planeInlierCount
                        + " / " + extractedPlaneInlierCount + "   SOR: " + statisticalInlierCount
                        + "\nKdTree: " + nearestNeighborCount + "   Octree: " + octreeNeighborCount
                        + "   Clusters: " + clusterCount + "   Hull: " + convexHullPointCount
                        + "\nSphere: " + sphereInlierCount + "   CropBox: " + cropBoxPointCount
                        + "   GridMin: " + gridMinimumPointCount + "   NormalSpace: " + normalSpacePointCount
                        + "\nPCA: " + principalAxisStatsCount + "   Dist: " + distanceCount
                        + "   Transform: " + transformedPointCount + "   ICP: " + icpConverged
                        + "/" + targetIcpConverged + "/" + targetGicpConverged);
                pointCloudSurface.setPointClouds(rawPoints, filteredPoints);
            });
        } catch (Throwable error) {
            android.util.Log.e(TAG, "PCL OpenGL sample failed", error);
            runOnUiThread(() -> statusText.setText("pclMobile OpenGL point cloud\nFailed: "
                    + error.getClass().getSimpleName() + ": " + error.getMessage()));
        }
    }

    private File writeSamplePcd() throws IOException {
        File output = new File(getCacheDir(), "pclmobile_opengl_sample.pcd");
        StringBuilder pcd = new StringBuilder();
        StringBuilder points = new StringBuilder();
        int pointCount = 0;

        pcd.append("# .PCD v0.7 - Point Cloud Data file format\n")
                .append("VERSION 0.7\n")
                .append("FIELDS x y z\n")
                .append("SIZE 4 4 4\n")
                .append("TYPE F F F\n")
                .append("COUNT 1 1 1\n");

        for (int ix = 0; ix < 62; ix++) {
            double x = -1.45 + ix * 0.047;
            for (int iy = 0; iy < 42; iy++) {
                double y = -0.98 + iy * 0.047;
                double base = 0.30 * Math.sin(x * 3.0) + 0.22 * Math.cos(y * 4.2);
                double bump = Math.exp(-((x - 0.36) * (x - 0.36) + (y + 0.16) * (y + 0.16)) * 12.0);
                double groove = Math.exp(-((x + 0.58) * (x + 0.58) + (y - 0.34) * (y - 0.34)) * 18.0);
                appendPoint(points, x, y, base + bump * 0.82 - groove * 0.35);
                pointCount++;
            }
        }

        for (int i = 0; i < 420; i++) {
            double t = i * 0.23;
            double radius = 0.20 + (i % 11) * 0.010;
            double x = 0.46 + Math.cos(t) * radius;
            double y = -0.16 + Math.sin(t) * radius;
            double z = -0.48 + i * 0.0042;
            appendPoint(points, x, y, z);
            pointCount++;
        }

        pcd.append("WIDTH ").append(pointCount).append('\n')
                .append("HEIGHT 1\n")
                .append("VIEWPOINT 0 0 0 1 0 0 0\n")
                .append("POINTS ").append(pointCount).append('\n')
                .append("DATA ascii\n")
                .append(points);

        try (FileOutputStream stream = new FileOutputStream(output)) {
            stream.write(pcd.toString().getBytes(StandardCharsets.UTF_8));
        }
        return output;
    }

    private static void appendPoint(StringBuilder points, double x, double y, double z) {
        points.append(String.format(Locale.US, "%.4f %.4f %.4f%n", x, y, z));
    }

    private static final class PointCloudSurface extends GLSurfaceView {
        private final PointCloudRenderer renderer;

        PointCloudSurface(Activity activity) {
            super(activity);
            setEGLContextClientVersion(2);
            renderer = new PointCloudRenderer();
            setRenderer(renderer);
            setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
        }

        void setPointClouds(float[] rawPoints, float[] filteredPoints) {
            queueEvent(() -> renderer.setPointClouds(rawPoints, filteredPoints));
        }
    }

    private static final class PointCloudRenderer implements GLSurfaceView.Renderer {
        private static final int FLOATS_PER_VERTEX = 6;
        private static final int BYTES_PER_FLOAT = 4;
        private static final float[] RAW_COLOR = {0.18f, 0.72f, 1.0f};
        private static final float[] FILTERED_COLOR = {1.0f, 0.72f, 0.16f};

        private final float[] projection = new float[16];
        private final float[] view = new float[16];
        private final float[] model = new float[16];
        private final float[] viewModel = new float[16];
        private final float[] mvp = new float[16];
        private FloatBuffer rawBuffer;
        private FloatBuffer filteredBuffer;
        private int rawCount;
        private int filteredCount;
        private int program;
        private int positionHandle;
        private int colorHandle;
        private int mvpHandle;
        private int pointSizeHandle;
        private long startMillis;

        @Override
        public void onSurfaceCreated(GL10 gl, EGLConfig config) {
            program = buildProgram(VERTEX_SHADER, FRAGMENT_SHADER);
            positionHandle = GLES20.glGetAttribLocation(program, "aPosition");
            colorHandle = GLES20.glGetAttribLocation(program, "aColor");
            mvpHandle = GLES20.glGetUniformLocation(program, "uMvp");
            pointSizeHandle = GLES20.glGetUniformLocation(program, "uPointSize");
            GLES20.glClearColor(0.015f, 0.021f, 0.031f, 1.0f);
            GLES20.glEnable(GLES20.GL_DEPTH_TEST);
            startMillis = System.currentTimeMillis();
        }

        @Override
        public void onSurfaceChanged(GL10 gl, int width, int height) {
            Matrix.setLookAtM(view, 0, 0.0f, -3.2f, 2.1f, 0.0f, 0.0f, 0.12f, 0.0f, 0.0f, 1.0f);
        }

        @Override
        public void onDrawFrame(GL10 gl) {
            GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);
            if (program == 0 || rawBuffer == null || filteredBuffer == null) {
                return;
            }

            int[] viewport = new int[4];
            GLES20.glGetIntegerv(GLES20.GL_VIEWPORT, viewport, 0);
            int width = viewport[2];
            int height = viewport[3];
            int topInset = Math.min(150, Math.max(92, height / 7));
            int drawHeight = Math.max(1, height - topInset);
            int gap = Math.max(8, width / 120);
            int panelWidth = Math.max(1, (width - gap) / 2);

            GLES20.glUseProgram(program);
            drawCloud(rawBuffer, rawCount, 0, 0, panelWidth, drawHeight, 3.0f, -0.16f);
            drawCloud(filteredBuffer, filteredCount, panelWidth + gap, 0, width - panelWidth - gap, drawHeight, 7.0f, 0.16f);
            GLES20.glViewport(0, 0, width, height);
        }

        void setPointClouds(float[] rawPoints, float[] filteredPoints) {
            rawBuffer = makeBuffer(rawPoints, RAW_COLOR);
            filteredBuffer = makeBuffer(filteredPoints, FILTERED_COLOR);
            rawCount = rawPoints.length / 3;
            filteredCount = filteredPoints.length / 3;
        }

        private void drawCloud(FloatBuffer buffer, int count, int x, int y, int width, int height,
                float pointSize, float verticalOffset) {
            if (width <= 0 || height <= 0 || count <= 0) {
                return;
            }

            GLES20.glViewport(x, y, width, height);
            float aspect = width / (float) height;
            Matrix.perspectiveM(projection, 0, 42.0f, aspect, 0.1f, 20.0f);

            float elapsed = (System.currentTimeMillis() - startMillis) / 1000.0f;
            Matrix.setIdentityM(model, 0);
            Matrix.translateM(model, 0, 0.0f, verticalOffset, 0.0f);
            Matrix.rotateM(model, 0, elapsed * 18.0f, 0.0f, 0.0f, 1.0f);
            Matrix.rotateM(model, 0, 62.0f, 1.0f, 0.0f, 0.0f);
            Matrix.scaleM(model, 0, 0.78f, 0.78f, 0.78f);

            Matrix.multiplyMM(viewModel, 0, view, 0, model, 0);
            Matrix.multiplyMM(mvp, 0, projection, 0, viewModel, 0);
            GLES20.glUniformMatrix4fv(mvpHandle, 1, false, mvp, 0);
            GLES20.glUniform1f(pointSizeHandle, pointSize);

            buffer.position(0);
            GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false,
                    FLOATS_PER_VERTEX * BYTES_PER_FLOAT, buffer);
            GLES20.glEnableVertexAttribArray(positionHandle);

            buffer.position(3);
            GLES20.glVertexAttribPointer(colorHandle, 3, GLES20.GL_FLOAT, false,
                    FLOATS_PER_VERTEX * BYTES_PER_FLOAT, buffer);
            GLES20.glEnableVertexAttribArray(colorHandle);

            GLES20.glDrawArrays(GLES20.GL_POINTS, 0, count);
            GLES20.glDisableVertexAttribArray(positionHandle);
            GLES20.glDisableVertexAttribArray(colorHandle);
            buffer.position(0);
        }

        private static FloatBuffer makeBuffer(float[] points, float[] color) {
            FloatBuffer buffer = ByteBuffer
                    .allocateDirect((points.length / 3) * FLOATS_PER_VERTEX * BYTES_PER_FLOAT)
                    .order(ByteOrder.nativeOrder())
                    .asFloatBuffer();
            for (int i = 0; i < points.length; i += 3) {
                float z = points[i + 2];
                float light = Math.max(0.0f, Math.min(1.0f, (z + 0.7f) / 2.1f));
                buffer.put(points[i]);
                buffer.put(points[i + 1]);
                buffer.put(z);
                buffer.put(color[0] + (1.0f - color[0]) * light * 0.32f);
                buffer.put(color[1] + (1.0f - color[1]) * light * 0.32f);
                buffer.put(color[2] + (1.0f - color[2]) * light * 0.32f);
            }
            buffer.position(0);
            return buffer;
        }

        private static int buildProgram(String vertexShader, String fragmentShader) {
            int vertex = compileShader(GLES20.GL_VERTEX_SHADER, vertexShader);
            int fragment = compileShader(GLES20.GL_FRAGMENT_SHADER, fragmentShader);
            int program = GLES20.glCreateProgram();
            GLES20.glAttachShader(program, vertex);
            GLES20.glAttachShader(program, fragment);
            GLES20.glLinkProgram(program);
            int[] status = new int[1];
            GLES20.glGetProgramiv(program, GLES20.GL_LINK_STATUS, status, 0);
            if (status[0] == 0) {
                String log = GLES20.glGetProgramInfoLog(program);
                GLES20.glDeleteProgram(program);
                throw new IllegalStateException("OpenGL program link failed: " + log);
            }
            return program;
        }

        private static int compileShader(int type, String source) {
            int shader = GLES20.glCreateShader(type);
            GLES20.glShaderSource(shader, source);
            GLES20.glCompileShader(shader);
            int[] status = new int[1];
            GLES20.glGetShaderiv(shader, GLES20.GL_COMPILE_STATUS, status, 0);
            if (status[0] == 0) {
                String log = GLES20.glGetShaderInfoLog(shader);
                GLES20.glDeleteShader(shader);
                throw new IllegalStateException("OpenGL shader compile failed: " + log);
            }
            return shader;
        }

        private static final String VERTEX_SHADER =
                "uniform mat4 uMvp;\n"
                        + "uniform float uPointSize;\n"
                        + "attribute vec3 aPosition;\n"
                        + "attribute vec3 aColor;\n"
                        + "varying vec3 vColor;\n"
                        + "void main() {\n"
                        + "  vColor = aColor;\n"
                        + "  gl_Position = uMvp * vec4(aPosition, 1.0);\n"
                        + "  gl_PointSize = uPointSize;\n"
                        + "}\n";

        private static final String FRAGMENT_SHADER =
                "precision mediump float;\n"
                        + "varying vec3 vColor;\n"
                        + "void main() {\n"
                        + "  vec2 p = gl_PointCoord - vec2(0.5);\n"
                        + "  if (dot(p, p) > 0.25) discard;\n"
                        + "  gl_FragColor = vec4(vColor, 1.0);\n"
                        + "}\n";
    }
}
