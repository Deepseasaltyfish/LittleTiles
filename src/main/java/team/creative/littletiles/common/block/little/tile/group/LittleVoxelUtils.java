package team.creative.littletiles.common.block.little.tile.group;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ForkJoinPool;
import java.util.stream.IntStream;

import org.joml.Matrix4f;
import org.joml.Vector3f;

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

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll, LittleGrid targetGrid) {
        return rotateVoxels(group, yaw, pitch, roll, Runtime.getRuntime().availableProcessors(), targetGrid);
    }

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll, int parallelism, LittleGrid targetGrid) {
        long startTotal = System.currentTimeMillis();
        long stageStart;

        // 1. 复制并统一到目标网格
        stageStart = System.currentTimeMillis();
        LittleGrid grid = targetGrid;
        LittleGroup copy = group.copy();
        copy.convertTo(grid);
        LittleTiles.LOGGER.info("Stage 1 - Copy & grid conversion (target: {}): {} ms", grid.count, System.currentTimeMillis() - stageStart);

        // 2. 提取源盒子（支持斜面）
        stageStart = System.currentTimeMillis();
        List<SourceBox> sourceBoxes = new ArrayList<>();
        int slopeCount = 0;
        for (LittleTile tile : copy.allTiles()) {
            for (LittleBox box : tile) {
                if (box instanceof LittleTransformableBox) {
                    slopeCount++;
                    sourceBoxes.add(new SourceBox((LittleTransformableBox) box, tile));
                } else {
                    sourceBoxes.add(new SourceBox(box, tile));
                }
            }
        }
        if (sourceBoxes.isEmpty()) {
            LittleTiles.LOGGER.info("Source boxes empty, returning empty group");
            return new LittleGroup();
        }
        LittleTiles.LOGGER.info("Stage 2 - Extracted {} source boxes ({} slopes): {} ms",
                sourceBoxes.size(), slopeCount, System.currentTimeMillis() - stageStart);

        // 3. 构建 BVH
        stageStart = System.currentTimeMillis();
        BVHNode root = buildBVH(sourceBoxes);
        LittleTiles.LOGGER.info("Stage 3 - BVH construction: {} ms", System.currentTimeMillis() - stageStart);

        // 4. 计算旋转后的包围盒
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

        // 5. 逆矩阵
        stageStart = System.currentTimeMillis();
        Matrix4f invRot = new Matrix4f(rot).invert();
        LittleTiles.LOGGER.info("Stage 5 - Matrix inversion: {} ms", System.currentTimeMillis() - stageStart);

        // 6. 并行采样（使用 BVH，自动处理斜面）
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

        // 7. 合并（此处仅做 X 方向游程合并，保持稳定）
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

        // 8. 最终处理（不再 convertToSmallest，因为已固定目标网格）
        stageStart = System.currentTimeMillis();
        translateToOrigin(result);
        LittleTiles.LOGGER.info("Stage 8 - Grid normalization & translation: {} ms", System.currentTimeMillis() - stageStart);

        LittleTiles.LOGGER.info("Total rotation time: {} ms", System.currentTimeMillis() - startTotal);
        return result;
    }

    // ========== BVH 与 SourceBox 实现（包含斜面支持） ==========

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

        // 计算包围盒
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
            float center;
            if (axis == 0) {
                center = (sb.getCorners()[0][0] + sb.getCorners()[4][0]) * 0.5f; // approximate
            } else if (axis == 1) {
                center = (sb.getCorners()[0][1] + sb.getCorners()[2][1]) * 0.5f;
            } else {
                center = (sb.getCorners()[0][2] + sb.getCorners()[1][2]) * 0.5f;
            }
            // 更准确的方法：计算所有角点的平均值，但此处简化
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

    private static class SourceBox {
        final LittleTile tile;
        private final float[][] corners; // 8 corners, each [x,y,z]
        private final Plane[] planes;    // 6 planes for convex hull
        private final boolean isSlope;

        // 普通盒子构造（不变）
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
            this.isSlope = false;
            this.planes = null;
        }

        // 斜面构造（支持阴角/阳角）
        SourceBox(LittleTransformableBox box, LittleTile tile) {
            this.tile = tile;
            Vec3f[] vecCorners = box.getTiltedCorners();
            this.corners = new float[8][3];
            for (int i = 0; i < 8; i++) {
                corners[i][0] = vecCorners[i].x;
                corners[i][1] = vecCorners[i].y;
                corners[i][2] = vecCorners[i].z;
            }
            this.isSlope = true;
            this.planes = computePlanes(corners);
        }

        float[][] getCorners() { return corners; }

        boolean contains(float x, float y, float z) {
            if (isSlope) {
                for (Plane p : planes) {
                    if (p.distance(x, y, z) < -EPSILON) return false;
                }
                return true;
            } else {
                return x >= corners[0][0] && x <= corners[1][0] &&
                        y >= corners[0][1] && y <= corners[2][1] &&
                        z >= corners[0][2] && z <= corners[4][2];
            }
        }

        // 用12个三角形（每个面拆成2个三角形）构建平面，法向量指向内部
        private static Plane[] computePlanes(float[][] corners) {
            int[][] triIndices = {
                    {0,1,3}, {0,3,2}, // bottom
                    {4,5,7}, {4,7,6}, // top
                    {0,1,5}, {0,5,4}, // front
                    {2,3,7}, {2,7,6}, // back
                    {0,2,6}, {0,6,4}, // left
                    {1,3,7}, {1,7,5}  // right
            };
            // 计算几何中心
            float cx = 0, cy = 0, cz = 0;
            for (float[] c : corners) { cx += c[0]; cy += c[1]; cz += c[2]; }
            cx /= 8; cy /= 8; cz /= 8;

            Plane[] planes = new Plane[6];
            int planeIdx = 0;
            // 每两个三角形共面，合并为同一个平面
            for (int i = 0; i < triIndices.length; i += 2) {
                int[] t1 = triIndices[i];
                int[] t2 = triIndices[i+1];
                // 用第一个三角形的三点计算法向量
                float[] a = corners[t1[0]], b = corners[t1[1]], c = corners[t1[2]];
                float ax = a[0], ay = a[1], az = a[2];
                float bx = b[0], by = b[1], bz = b[2];
                float cx_ = c[0], cy_ = c[1], cz_ = c[2];
                float e1x = bx - ax, e1y = by - ay, e1z = bz - az;
                float e2x = cx_ - ax, e2y = cy_ - ay, e2z = cz_ - az;
                float nx = e1y * e2z - e1z * e2y;
                float ny = e1z * e2x - e1x * e2z;
                float nz = e1x * e2y - e1y * e2x;
                // 指向内部
                float dot = nx * (cx - ax) + ny * (cy - ay) + nz * (cz - az);
                if (dot < 0) { nx = -nx; ny = -ny; nz = -nz; }
                float len = (float)Math.sqrt(nx*nx + ny*ny + nz*nz);
                if (len > 1e-8f) { nx /= len; ny /= len; nz /= len; }
                float d = -(nx * ax + ny * ay + nz * az);
                planes[planeIdx++] = new Plane(nx, ny, nz, d);
            }
            return planes;
        }

        private static class Plane {
            final float nx, ny, nz, d;
            Plane(float nx, float ny, float nz, float d) {
                this.nx = nx; this.ny = ny; this.nz = nz; this.d = d;
            }
            float distance(float x, float y, float z) {
                return nx * x + ny * y + nz * z + d;
            }
        }
    }

    // ========== 辅助方法 ==========

    private static void translateToOrigin(LittleGroup group) {
        LittleVec min = group.getMinVec();
        if (min.x == 0 && min.y == 0 && min.z == 0) return;
        LittleVec negative = new LittleVec(-min.x, -min.y, -min.z);
        group.move(new LittleVecGrid(negative, group.getGrid()));
    }
}