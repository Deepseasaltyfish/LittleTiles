package team.creative.littletiles.common.block.little.tile.group;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ForkJoinPool;
import java.util.stream.IntStream;

import org.joml.Matrix4f;
import org.joml.Vector3f;

import team.creative.creativecore.common.util.math.base.Facing;
import team.creative.creativecore.common.util.math.geo.VectorFan;
import team.creative.creativecore.common.util.math.vec.Vec3f;
import team.creative.littletiles.LittleTiles;
import team.creative.littletiles.common.block.little.tile.LittleTile;
import team.creative.littletiles.common.grid.LittleGrid;
import team.creative.littletiles.common.math.box.LittleBox;
import team.creative.littletiles.common.math.box.LittleTransformableBox;
import team.creative.littletiles.common.math.vec.LittleVec;
import team.creative.littletiles.common.math.vec.LittleVecGrid;

public class LittleVoxelUtils {

    private static final float EPSILON = 1e-6f;
    private static final int BVH_LEAF_SIZE = 8;

    // ========== Public API ==========

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll) {
        return rotateVoxels(group, yaw, pitch, roll, Runtime.getRuntime().availableProcessors());
    }

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll, int parallelism) {
        LittleGrid target = LittleGrid.get(group.getSmallest());
        return rotateVoxels(group, yaw, pitch, roll, parallelism, target);
    }

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll, LittleGrid targetGrid) {
        return rotateVoxels(group, yaw, pitch, roll, Runtime.getRuntime().availableProcessors(), targetGrid);
    }

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll, int parallelism, LittleGrid targetGrid) {
        long startTotal = System.currentTimeMillis();
        long stageStart;

        stageStart = System.currentTimeMillis();
        LittleGrid grid = targetGrid;
        LittleGroup copy = group.copy();
        copy.convertTo(grid);
        LittleTiles.LOGGER.info("Stage 1 - Copy & grid conversion (target: {}): {} ms", grid.count, System.currentTimeMillis() - stageStart);

        stageStart = System.currentTimeMillis();
        List<SourceBox> sourceBoxes = new ArrayList<>();
        int slopeCount = 0;

        for (LittleTile tile : copy.allTiles()) {
            for (LittleBox box : tile) {
                if (box instanceof LittleTransformableBox) {
                    slopeCount++;
                    LittleTransformableBox transformable = (LittleTransformableBox) box;

                    float[][] triVerts = buildTrianglesFromCache(transformable.requestCache());
                    if (triVerts.length == 0) {
                        LittleTiles.LOGGER.warn("No triangles generated for transformable box, skipping");
                        continue;
                    }

                    Vec3f[] corners = transformable.getTiltedCorners();
                    float minX = Float.POSITIVE_INFINITY, maxX = Float.NEGATIVE_INFINITY;
                    float minY = Float.POSITIVE_INFINITY, maxY = Float.NEGATIVE_INFINITY;
                    float minZ = Float.POSITIVE_INFINITY, maxZ = Float.NEGATIVE_INFINITY;
                    for (Vec3f c : corners) {
                        minX = Math.min(minX, c.x); maxX = Math.max(maxX, c.x);
                        minY = Math.min(minY, c.y); maxY = Math.max(maxY, c.y);
                        minZ = Math.min(minZ, c.z); maxZ = Math.max(maxZ, c.z);
                    }
                    int startX = (int)Math.floor(minX);
                    int startY = (int)Math.floor(minY);
                    int startZ = (int)Math.floor(minZ);
                    int endX = (int)Math.ceil(maxX);
                    int endY = (int)Math.ceil(maxY);
                    int endZ = (int)Math.ceil(maxZ);

                    for (int x = startX; x < endX; x++) {
                        for (int y = startY; y < endY; y++) {
                            for (int z = startZ; z < endZ; z++) {
                                float cx = x + 0.5f;
                                float cy = y + 0.5f;
                                float cz = z + 0.5f;
                                if (isPointInPolyhedron(cx, cy, cz, triVerts)) {
                                    LittleBox unitBox = new LittleBox(x, y, z, x + 1, y + 1, z + 1);
                                    sourceBoxes.add(new SourceBox(unitBox, tile));
                                }
                            }
                        }
                    }
                } else {
                    sourceBoxes.add(new SourceBox(box, tile));
                }
            }
        }

        if (sourceBoxes.isEmpty()) {
            LittleTiles.LOGGER.info("Source boxes empty, returning empty group");
            return new LittleGroup();
        }
        LittleTiles.LOGGER.info("Stage 2 - Extracted {} source boxes ({} slopes voxelized): {} ms",
                sourceBoxes.size(), slopeCount, System.currentTimeMillis() - stageStart);

        stageStart = System.currentTimeMillis();
        BVHNode root = buildBVH(sourceBoxes);
        LittleTiles.LOGGER.info("Stage 3 - BVH construction: {} ms", System.currentTimeMillis() - stageStart);

        stageStart = System.currentTimeMillis();
        Matrix4f rot = new Matrix4f().rotationXYZ(pitch, yaw, roll);
        Vector3f vec = new Vector3f();
        float minX = Float.POSITIVE_INFINITY, maxX = Float.NEGATIVE_INFINITY;
        float minY = Float.POSITIVE_INFINITY, maxY = Float.NEGATIVE_INFINITY;
        float minZ = Float.POSITIVE_INFINITY, maxZ = Float.NEGATIVE_INFINITY;

        for (SourceBox sb : sourceBoxes) {
            float[][] corners = sb.getCorners();
            for (float[] c : corners) {
                vec.set(c[0], c[1], c[2]);
                rot.transformPosition(vec);
                minX = Math.min(minX, vec.x()); maxX = Math.max(maxX, vec.x());
                minY = Math.min(minY, vec.y()); maxY = Math.max(maxY, vec.y());
                minZ = Math.min(minZ, vec.z()); maxZ = Math.max(maxZ, vec.z());
            }
        }

        int startX = (int)Math.floor(minX) - 1;
        int startY = (int)Math.floor(minY) - 1;
        int startZ = (int)Math.floor(minZ) - 1;
        int endX = (int)Math.ceil(maxX) + 1;
        int endY = (int)Math.ceil(maxY) + 1;
        int endZ = (int)Math.ceil(maxZ) + 1;

        long totalVoxelsLong = (long)(endX - startX) * (long)(endY - startY) * (long)(endZ - startZ);
        if (totalVoxelsLong <= 0 || totalVoxelsLong > Integer.MAX_VALUE) {
            LittleTiles.LOGGER.info("Bounding box too large ({} voxels), returning empty group", totalVoxelsLong);
            return new LittleGroup();
        }
        int totalVoxels = (int)totalVoxelsLong;
        LittleTiles.LOGGER.info("Stage 4 - Bounding box computed, {} target voxels: {} ms", totalVoxels, System.currentTimeMillis() - stageStart);

        stageStart = System.currentTimeMillis();
        Matrix4f invRot = new Matrix4f(rot).invert();
        LittleTiles.LOGGER.info("Stage 5 - Matrix inversion: {} ms", System.currentTimeMillis() - stageStart);

        stageStart = System.currentTimeMillis();
        Map<LittleTile, Set<LittleVec>> resultMap = new ConcurrentHashMap<>();
        int threads = parallelism > 0 ? parallelism : Runtime.getRuntime().availableProcessors();
        ForkJoinPool pool = new ForkJoinPool(threads);

        try {
            pool.submit(() -> {
                IntStream.range(0, totalVoxels).parallel().forEach(index -> {
                    Vector3f localVec = new Vector3f();
                    int x = startX + index / ((endY - startY) * (endZ - startZ));
                    int remainder = index % ((endY - startY) * (endZ - startZ));
                    int y = startY + remainder / (endZ - startZ);
                    int z = startZ + remainder % (endZ - startZ);

                    float cx = x + 0.5f;
                    float cy = y + 0.5f;
                    float cz = z + 0.5f;

                    localVec.set(cx, cy, cz);
                    invRot.transformPosition(localVec);
                    float sx = localVec.x();
                    float sy = localVec.y();
                    float sz = localVec.z();

                    LittleTile foundTile = queryBVH(root, sx, sy, sz);
                    if (foundTile != null) {
                        resultMap.computeIfAbsent(foundTile, k -> ConcurrentHashMap.newKeySet())
                                .add(new LittleVec(x, y, z));
                    }
                });
            }).join();
        } finally {
            pool.shutdown();
        }
        int hitCount = resultMap.values().stream().mapToInt(Set::size).sum();
        LittleTiles.LOGGER.info("Stage 6 - Parallel sampling ({} threads): {} ms, {} voxels hit", threads, System.currentTimeMillis() - stageStart, hitCount);

        stageStart = System.currentTimeMillis();
        LittleGroup result = new LittleGroup();
        int totalBoxesAfter = 0;

        for (Map.Entry<LittleTile, Set<LittleVec>> entry : resultMap.entrySet()) {
            LittleTile template = entry.getKey();
            Set<LittleVec> positions = entry.getValue();
            if (positions.isEmpty()) continue;

            List<LittleVec> sorted = new ArrayList<>(positions);
            sorted.sort((a, b) -> {
                if (a.y != b.y) return Integer.compare(a.y, b.y);
                if (a.z != b.z) return Integer.compare(a.z, b.z);
                return Integer.compare(a.x, b.x);
            });

            List<LittleBox> boxes = new ArrayList<>();
            int i = 0;
            while (i < sorted.size()) {
                LittleVec first = sorted.get(i);
                int runStartX = first.x;
                int y = first.y;
                int z = first.z;
                int runEndX = runStartX + 1;
                i++;
                while (i < sorted.size()) {
                    LittleVec next = sorted.get(i);
                    if (next.y == y && next.z == z && next.x == runEndX) {
                        runEndX++;
                        i++;
                    } else {
                        break;
                    }
                }
                boxes.add(new LittleBox(runStartX, y, z, runEndX, y + 1, z + 1));
            }

            LittleTile newTile = new LittleTile(template.getState(), template.color, boxes);
            result.addTileFast(grid, newTile);
            totalBoxesAfter += boxes.size();
        }

        LittleTiles.LOGGER.info("Stage 7 - Group construction (fast merge): {} tiles, {} boxes: {} ms",
                result.totalTiles(), totalBoxesAfter, System.currentTimeMillis() - stageStart);

        stageStart = System.currentTimeMillis();
        translateToOrigin(result);
        LittleTiles.LOGGER.info("Stage 8 - Grid normalization & translation: {} ms", System.currentTimeMillis() - stageStart);

        LittleTiles.LOGGER.info("Total rotation time: {} ms", System.currentTimeMillis() - startTotal);
        return result;
    }

    // ========== BVH Implementation ==========

    private static class BVHNode {
        float minX, minY, minZ, maxX, maxY, maxZ;
        BVHNode left, right;
        List<SourceBox> boxes;

        BVHNode(List<SourceBox> boxes) {
            this.boxes = boxes;
            computeBounds(boxes);
        }

        BVHNode(BVHNode left, BVHNode right) {
            this.left = left;
            this.right = right;
            this.boxes = null;
            minX = Math.min(left.minX, right.minX);
            minY = Math.min(left.minY, right.minY);
            minZ = Math.min(left.minZ, right.minZ);
            maxX = Math.max(left.maxX, right.maxX);
            maxY = Math.max(left.maxY, right.maxY);
            maxZ = Math.max(left.maxZ, right.maxZ);
        }

        private void computeBounds(List<SourceBox> boxes) {
            minX = Float.POSITIVE_INFINITY;
            maxX = Float.NEGATIVE_INFINITY;
            minY = Float.POSITIVE_INFINITY;
            maxY = Float.NEGATIVE_INFINITY;
            minZ = Float.POSITIVE_INFINITY;
            maxZ = Float.NEGATIVE_INFINITY;
            for (SourceBox sb : boxes) {
                float[][] corners = sb.getCorners();
                for (float[] c : corners) {
                    minX = Math.min(minX, c[0]); maxX = Math.max(maxX, c[0]);
                    minY = Math.min(minY, c[1]); maxY = Math.max(maxY, c[1]);
                    minZ = Math.min(minZ, c[2]); maxZ = Math.max(maxZ, c[2]);
                }
            }
        }

        boolean contains(float x, float y, float z) {
            return x >= minX && x <= maxX && y >= minY && y <= maxY && z >= minZ && z <= maxZ;
        }
    }

    private static BVHNode buildBVH(List<SourceBox> boxes) {
        return buildBVH(boxes, 0);
    }

    private static BVHNode buildBVH(List<SourceBox> boxes, int depth) {
        if (boxes.size() <= BVH_LEAF_SIZE || depth > 20) {
            return new BVHNode(boxes);
        }

        float minX = Float.POSITIVE_INFINITY, maxX = Float.NEGATIVE_INFINITY;
        float minY = Float.POSITIVE_INFINITY, maxY = Float.NEGATIVE_INFINITY;
        float minZ = Float.POSITIVE_INFINITY, maxZ = Float.NEGATIVE_INFINITY;
        for (SourceBox sb : boxes) {
            float[][] corners = sb.getCorners();
            for (float[] c : corners) {
                minX = Math.min(minX, c[0]); maxX = Math.max(maxX, c[0]);
                minY = Math.min(minY, c[1]); maxY = Math.max(maxY, c[1]);
                minZ = Math.min(minZ, c[2]); maxZ = Math.max(maxZ, c[2]);
            }
        }
        float extentX = maxX - minX;
        float extentY = maxY - minY;
        float extentZ = maxZ - minZ;

        int axis;
        if (extentX >= extentY && extentX >= extentZ) axis = 0;
        else if (extentY >= extentX && extentY >= extentZ) axis = 1;
        else axis = 2;

        float split;
        if (axis == 0) split = (minX + maxX) * 0.5f;
        else if (axis == 1) split = (minY + maxY) * 0.5f;
        else split = (minZ + maxZ) * 0.5f;

        List<SourceBox> leftList = new ArrayList<>();
        List<SourceBox> rightList = new ArrayList<>();
        for (SourceBox sb : boxes) {
            float[][] corners = sb.getCorners();
            float cx = 0, cy = 0, cz = 0;
            for (float[] c : corners) { cx += c[0]; cy += c[1]; cz += c[2]; }
            cx /= 8; cy /= 8; cz /= 8;
            float center = (axis == 0) ? cx : (axis == 1 ? cy : cz);
            if (center < split) leftList.add(sb);
            else rightList.add(sb);
        }

        if (leftList.isEmpty() || rightList.isEmpty()) {
            return new BVHNode(boxes);
        }

        return new BVHNode(buildBVH(leftList, depth + 1), buildBVH(rightList, depth + 1));
    }

    private static LittleTile queryBVH(BVHNode node, float x, float y, float z) {
        if (!node.contains(x, y, z)) return null;
        if (node.boxes != null) {
            for (SourceBox sb : node.boxes) {
                if (sb.contains(x, y, z)) return sb.tile;
            }
            return null;
        }
        LittleTile result = queryBVH(node.left, x, y, z);
        if (result != null) return result;
        return queryBVH(node.right, x, y, z);
    }

    // ========== SourceBox (plain boxes only) ==========

    private static class SourceBox {
        final LittleTile tile;
        private final float[][] corners;

        SourceBox(LittleBox box, LittleTile tile) {
            this.tile = tile;
            this.corners = new float[][] {
                    {box.minX, box.minY, box.minZ},
                    {box.maxX, box.minY, box.minZ},
                    {box.minX, box.maxY, box.minZ},
                    {box.maxX, box.maxY, box.minZ},
                    {box.minX, box.minY, box.maxZ},
                    {box.maxX, box.minY, box.maxZ},
                    {box.minX, box.maxY, box.maxZ},
                    {box.maxX, box.maxY, box.maxZ}
            };
        }

        float[][] getCorners() { return corners; }

        boolean contains(float x, float y, float z) {
            return x >= corners[0][0] && x <= corners[1][0] &&
                    y >= corners[0][1] && y <= corners[2][1] &&
                    z >= corners[0][2] && z <= corners[4][2];
        }
    }

    // ========== Slope voxelization helper (ray casting) ==========

    private static float[][] buildTrianglesFromCache(LittleTransformableBox.VectorFanCache cache) {
        List<float[]> triList = new ArrayList<>();
        for (Facing facing : Facing.VALUES) {
            LittleTransformableBox.VectorFanFaceCache faceCache = cache.get(facing);
            if (faceCache == null) continue;
            List<VectorFan> fans = new ArrayList<>();
            if (faceCache.hasAxisStrip()) {
                fans.addAll(faceCache.axisStrips);
            }
            if (faceCache.hasTiltedStrip()) {
                if (faceCache.tiltedStrip1 != null) fans.add(faceCache.tiltedStrip1);
                if (faceCache.tiltedStrip2 != null) fans.add(faceCache.tiltedStrip2);
            }
            for (VectorFan fan : fans) {
                int n = fan.count();
                if (n < 3) continue;
                Vec3f v0 = fan.get(0);
                for (int i = 1; i < n - 1; i++) {
                    Vec3f v1 = fan.get(i);
                    Vec3f v2 = fan.get(i + 1);
                    float[] tri = new float[9];
                    tri[0] = v0.x; tri[1] = v0.y; tri[2] = v0.z;
                    tri[3] = v1.x; tri[4] = v1.y; tri[5] = v1.z;
                    tri[6] = v2.x; tri[7] = v2.y; tri[8] = v2.z;
                    triList.add(tri);
                }
            }
        }
        return triList.toArray(new float[0][]);
    }

    private static boolean isPointInPolyhedron(float px, float py, float pz, float[][] triVerts) {
        int hitCount = 0;
        float[] orig = {px, py, pz};
        float[] dir = {1.0f, 0.0f, 0.0f};
        for (float[] tri : triVerts) {
            float[] v0 = {tri[0], tri[1], tri[2]};
            float[] v1 = {tri[3], tri[4], tri[5]};
            float[] v2 = {tri[6], tri[7], tri[8]};
            if (rayTriangleIntersect(orig, dir, v0, v1, v2)) {
                hitCount++;
            }
        }
        return hitCount % 2 == 1;
    }

    private static boolean rayTriangleIntersect(float[] orig, float[] dir, float[] v0, float[] v1, float[] v2) {
        float[] edge1 = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
        float[] edge2 = {v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]};
        float[] pvec = cross(dir, edge2);
        float det = dot(edge1, pvec);
        if (Math.abs(det) < 1e-8) return false;
        float invDet = 1.0f / det;
        float[] tvec = {orig[0]-v0[0], orig[1]-v0[1], orig[2]-v0[2]};
        float u = dot(tvec, pvec) * invDet;
        if (u < 0 || u > 1) return false;
        float[] qvec = cross(tvec, edge1);
        float v = dot(dir, qvec) * invDet;
        if (v < 0 || u + v > 1) return false;
        float t = dot(edge2, qvec) * invDet;
        return t > 1e-8;
    }

    private static float[] cross(float[] a, float[] b) {
        return new float[]{a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]};
    }

    private static float dot(float[] a, float[] b) {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    }

    // ========== Helper ==========

    private static void translateToOrigin(LittleGroup group) {
        LittleVec min = group.getMinVec();
        if (min.x == 0 && min.y == 0 && min.z == 0) return;
        LittleVec negative = new LittleVec(-min.x, -min.y, -min.z);
        group.move(new LittleVecGrid(negative, group.getGrid()));
    }
}