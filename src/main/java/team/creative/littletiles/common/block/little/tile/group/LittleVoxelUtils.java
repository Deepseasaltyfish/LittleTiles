package team.creative.littletiles.common.block.little.tile.group;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ForkJoinPool;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.IntStream;

import org.joml.Matrix4f;
import org.joml.Vector3f;

import team.creative.littletiles.LittleTiles;
import team.creative.littletiles.common.block.little.tile.LittleTile;
import team.creative.littletiles.common.grid.LittleGrid;
import team.creative.littletiles.common.math.box.LittleBox;
import team.creative.littletiles.common.math.vec.LittleVec;
import team.creative.littletiles.common.math.vec.LittleVecGrid;

public class LittleVoxelUtils {

    private static final float EPSILON = 1e-6f;
    private static final int BVH_LEAF_SIZE = 8;

    /**
     * 旋转体素组（自动使用 CPU 核心数并行）
     */
    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll) {
        return rotateVoxels(group, yaw, pitch, roll, Runtime.getRuntime().availableProcessors());
    }

    /**
     * 旋转体素组（指定并行线程数）
     */
    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll, int parallelism) {
        long startTotal = System.currentTimeMillis();
        long stageStart;

        // 1. 复制并统一网格
        stageStart = System.currentTimeMillis();
        int targetSize = group.getSmallest();
        LittleGrid grid = LittleGrid.get(targetSize);
        LittleGroup copy = group.copy();
        copy.convertTo(grid);
        LittleTiles.LOGGER.warn("阶段1-复制与网格转换: {} ms", System.currentTimeMillis() - stageStart);

        // 2. 提取源盒子
        stageStart = System.currentTimeMillis();
        List<SourceBox> sourceBoxes = new ArrayList<>();
        for (LittleTile tile : copy.allTiles()) {
            for (LittleBox box : tile) {
                sourceBoxes.add(new SourceBox(box, tile));
            }
        }
        if (sourceBoxes.isEmpty()) {
            LittleTiles.LOGGER.warn("源盒子为空，返回空组");
            return new LittleGroup();
        }
        LittleTiles.LOGGER.warn("阶段2-提取源盒子 (数量: {}): {} ms", sourceBoxes.size(), System.currentTimeMillis() - stageStart);

        // 3. 构建 BVH
        stageStart = System.currentTimeMillis();
        BVHNode root = buildBVH(sourceBoxes);
        LittleTiles.LOGGER.warn("阶段3-构建BVH: {} ms", System.currentTimeMillis() - stageStart);

        // 4. 计算旋转后的包围盒
        stageStart = System.currentTimeMillis();
        Matrix4f rot = new Matrix4f().rotationXYZ(pitch, yaw, roll);
        Vector3f vec = new Vector3f();
        float minX = Float.POSITIVE_INFINITY, maxX = Float.NEGATIVE_INFINITY;
        float minY = Float.POSITIVE_INFINITY, maxY = Float.NEGATIVE_INFINITY;
        float minZ = Float.POSITIVE_INFINITY, maxZ = Float.NEGATIVE_INFINITY;

        for (SourceBox sb : sourceBoxes) {
            LittleBox box = sb.box;
            float[][] corners = {
                    {box.minX, box.minY, box.minZ},
                    {box.maxX, box.minY, box.minZ},
                    {box.minX, box.maxY, box.minZ},
                    {box.maxX, box.maxY, box.minZ},
                    {box.minX, box.minY, box.maxZ},
                    {box.maxX, box.minY, box.maxZ},
                    {box.minX, box.maxY, box.maxZ},
                    {box.maxX, box.maxY, box.maxZ}
            };
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
            LittleTiles.LOGGER.warn("旋转区域过大，体素数 {} 超过 int 范围，或非正数，返回空组", totalVoxelsLong);
            return new LittleGroup();
        }
        int totalVoxels = (int)totalVoxelsLong;
        LittleTiles.LOGGER.warn("阶段4-包围盒计算完成，目标体素数: {}，耗时: {} ms", totalVoxels, System.currentTimeMillis() - stageStart);

        // 5. 逆矩阵
        stageStart = System.currentTimeMillis();
        Matrix4f invRot = new Matrix4f(rot).invert();
        LittleTiles.LOGGER.warn("阶段5-逆矩阵计算: {} ms", System.currentTimeMillis() - stageStart);

        // 6. 并行处理
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
        LittleTiles.LOGGER.warn("阶段6-并行体素采样 (线程数: {}): {} ms，命中体素数: {}", threads, System.currentTimeMillis() - stageStart,
                resultMap.values().stream().mapToInt(Set::size).sum());

        // 7. 构建新组 - 快速合并（按 X 方向合并连续段）
        stageStart = System.currentTimeMillis();

        LittleGroup result = new LittleGroup();
        int totalBoxesAfter = 0;

        for (Map.Entry<LittleTile, Set<LittleVec>> entry : resultMap.entrySet()) {
            LittleTile template = entry.getKey();
            Set<LittleVec> positions = entry.getValue();
            if (positions.isEmpty()) continue;

            // 转换为列表并排序（按 y, z, x 排序）
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
                // 合并同一行（y,z）上连续的 x
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

        LittleTiles.LOGGER.warn("阶段7-构建新组 (快速合并X方向) 完成 (tiles: {}, boxes: {}): {} ms", result.totalTiles(), totalBoxesAfter,
                System.currentTimeMillis() - stageStart);

        // 8. 转换并归零（不变）
        stageStart = System.currentTimeMillis();
        result.convertToSmallest();
        translateToOrigin(result);
        LittleTiles.LOGGER.warn("阶段8-转换网格与归零: {} ms", System.currentTimeMillis() - stageStart);

        LittleTiles.LOGGER.warn("旋转总耗时: {} ms", System.currentTimeMillis() - startTotal);
        return result;
    }

    // ---------- BVH 实现 ----------
    private static class BVHNode {
        float minX, minY, minZ, maxX, maxY, maxZ;
        BVHNode left, right;
        List<SourceBox> boxes; // 叶子节点使用

        BVHNode(List<SourceBox> boxes) {
            this.boxes = boxes;
            computeBounds(boxes);
        }

        BVHNode(BVHNode left, BVHNode right) {
            this.left = left;
            this.right = right;
            this.boxes = null;
            // 合并包围盒
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
                minX = Math.min(minX, sb.minX);
                maxX = Math.max(maxX, sb.maxX);
                minY = Math.min(minY, sb.minY);
                maxY = Math.max(maxY, sb.maxY);
                minZ = Math.min(minZ, sb.minZ);
                maxZ = Math.max(maxZ, sb.maxZ);
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
            minX = Math.min(minX, sb.minX); maxX = Math.max(maxX, sb.maxX);
            minY = Math.min(minY, sb.minY); maxY = Math.max(maxY, sb.maxY);
            minZ = Math.min(minZ, sb.minZ); maxZ = Math.max(maxZ, sb.maxZ);
        }
        float extentX = maxX - minX;
        float extentY = maxY - minY;
        float extentZ = maxZ - minZ;

        // 选择最长轴分割
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
                center = (sb.minX + sb.maxX) * 0.5f;
            } else if (axis == 1) {
                center = (sb.minY + sb.maxY) * 0.5f;
            } else {
                center = (sb.minZ + sb.maxZ) * 0.5f;
            }
            if (center < split) leftList.add(sb);
            else rightList.add(sb);
        }

        // 防止某一边为空
        if (leftList.isEmpty() || rightList.isEmpty()) {
            return new BVHNode(boxes);
        }

        return new BVHNode(buildBVH(leftList, depth+1), buildBVH(rightList, depth+1));
    }

    private static LittleTile queryBVH(BVHNode node, float x, float y, float z) {
        if (!node.contains(x, y, z)) return null;
        if (node.boxes != null) {
            for (SourceBox sb : node.boxes) {
                if (sb.contains(x, y, z)) return sb.tile;
            }
            return null;
        }
        // 内部节点：先查左，再查右
        LittleTile result = queryBVH(node.left, x, y, z);
        if (result != null) return result;
        return queryBVH(node.right, x, y, z);
    }

    // ---------- 辅助类 ----------
    private static class SourceBox {
        final LittleBox box;
        final LittleTile tile;
        final float minX, minY, minZ, maxX, maxY, maxZ;

        SourceBox(LittleBox box, LittleTile tile) {
            this.box = box;
            this.tile = tile;
            // 使用浮点数并添加小容差
            this.minX = box.minX - EPSILON;
            this.minY = box.minY - EPSILON;
            this.minZ = box.minZ - EPSILON;
            this.maxX = box.maxX + EPSILON;
            this.maxY = box.maxY + EPSILON;
            this.maxZ = box.maxZ + EPSILON;
        }

        boolean contains(float x, float y, float z) {
            return x >= minX && x <= maxX && y >= minY && y <= maxY && z >= minZ && z <= maxZ;
        }
    }

    private static void translateToOrigin(LittleGroup group) {
        LittleVec min = group.getMinVec();
        if (min.x == 0 && min.y == 0 && min.z == 0) return;
        LittleVec negative = new LittleVec(-min.x, -min.y, -min.z);
        group.move(new LittleVecGrid(negative, group.getGrid()));
    }
}