package team.creative.littletiles.common.block.little.tile.group;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ForkJoinPool;
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

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll) {
        return rotateVoxels(group, yaw, pitch, roll, Runtime.getRuntime().availableProcessors());
    }

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll, int parallelism) {
        long startTotal = System.currentTimeMillis();
        long stageStart;

        // 1. Copy and unify grid
        stageStart = System.currentTimeMillis();
        int targetSize = group.getSmallest();
        LittleGrid grid = LittleGrid.get(targetSize);
        LittleGroup copy = group.copy();
        copy.convertTo(grid);
        LittleTiles.LOGGER.warn("Stage 1 - Copy & grid conversion: {} ms", System.currentTimeMillis() - stageStart);

        // 2. Extract source boxes
        stageStart = System.currentTimeMillis();
        List<SourceBox> sourceBoxes = new ArrayList<>();
        for (LittleTile tile : copy.allTiles()) {
            for (LittleBox box : tile) {
                sourceBoxes.add(new SourceBox(box, tile));
            }
        }
        if (sourceBoxes.isEmpty()) {
            LittleTiles.LOGGER.warn("Source boxes empty, returning empty group");
            return new LittleGroup();
        }
        LittleTiles.LOGGER.warn("Stage 2 - Extracted {} source boxes: {} ms", sourceBoxes.size(), System.currentTimeMillis() - stageStart);

        // 3. Build BVH
        stageStart = System.currentTimeMillis();
        BVHNode root = buildBVH(sourceBoxes);
        LittleTiles.LOGGER.warn("Stage 3 - BVH construction: {} ms", System.currentTimeMillis() - stageStart);

        // 4. Compute rotated bounding box
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
            LittleTiles.LOGGER.warn("Bounding box too large ({} voxels), returning empty group", totalVoxelsLong);
            return new LittleGroup();
        }
        int totalVoxels = (int)totalVoxelsLong;
        LittleTiles.LOGGER.warn("Stage 4 - Bounding box computed, {} target voxels: {} ms", totalVoxels, System.currentTimeMillis() - stageStart);

        // 5. Invert rotation matrix
        stageStart = System.currentTimeMillis();
        Matrix4f invRot = new Matrix4f(rot).invert();
        LittleTiles.LOGGER.warn("Stage 5 - Matrix inversion: {} ms", System.currentTimeMillis() - stageStart);

        // 6. Parallel sampling
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
        LittleTiles.LOGGER.warn("Stage 6 - Parallel sampling ({} threads): {} ms, {} voxels hit", threads, System.currentTimeMillis() - stageStart, hitCount);

        // ===================== 阶段7：ArrayList列合并 + 水平合并 =====================
        stageStart = System.currentTimeMillis();

        Map<LittleTile, Map<Long, List<Integer>>> materialColumnLists = new HashMap<>();

        for (Map.Entry<LittleTile, Set<LittleVec>> entry : resultMap.entrySet()) {
            LittleTile tile = entry.getKey();
            Map<Long, List<Integer>> colMap = materialColumnLists.computeIfAbsent(tile, k -> new HashMap<>());
            for (LittleVec v : entry.getValue()) {
                long key = ((long) v.x << 32) | (v.z & 0xffffffffL);
                colMap.computeIfAbsent(key, k -> new ArrayList<>()).add(v.y);
            }
        }

        LittleGroup result = new LittleGroup();
        int totalBoxes = 0;

        for (Map.Entry<LittleTile, Map<Long, List<Integer>>> entry : materialColumnLists.entrySet()) {
            LittleTile template = entry.getKey();
            Map<Long, List<Integer>> colMap = entry.getValue();

            // 解析列，生成 (x,z) -> 范围列表
            Map<Integer, Map<Integer, List<int[]>>> rows = new HashMap<>();
            for (Map.Entry<Long, List<Integer>> colEntry : colMap.entrySet()) {
                long key = colEntry.getKey();
                int x = (int) (key >> 32);
                int z = (int) (key & 0xffffffffL);
                List<Integer> yList = colEntry.getValue();
                Collections.sort(yList);
                List<int[]> ranges = new ArrayList<>();
                int start = yList.get(0);
                int end = start + 1;
                for (int i = 1; i < yList.size(); i++) {
                    int y = yList.get(i);
                    if (y == end) {
                        end++;
                    } else {
                        ranges.add(new int[]{start, end});
                        start = y;
                        end = y + 1;
                    }
                }
                ranges.add(new int[]{start, end});
                if (!ranges.isEmpty()) {
                    rows.computeIfAbsent(x, k -> new HashMap<>()).put(z, ranges);
                }
            }

            List<LittleBox> boxes = new ArrayList<>();
            // 水平合并（同一x行，相邻z且Y范围相同的单段列）
            for (Map.Entry<Integer, Map<Integer, List<int[]>>> rowEntry : rows.entrySet()) {
                int x = rowEntry.getKey();
                Map<Integer, List<int[]>> zMap = rowEntry.getValue();
                List<Integer> zList = new ArrayList<>(zMap.keySet());
                Collections.sort(zList);

                for (int zi = 0; zi < zList.size(); zi++) {
                    int z = zList.get(zi);
                    List<int[]> ranges = zMap.get(z);
                    if (ranges.isEmpty()) continue;

                    if (ranges.size() == 1) {
                        int rangeStartY = ranges.get(0)[0];
                        int rangeEndY = ranges.get(0)[1];
                        int zStart = z;
                        int zEnd = z + 1;
                        while (zi + 1 < zList.size()) {
                            int nextZ = zList.get(zi + 1);
                            List<int[]> nextRanges = zMap.get(nextZ);
                            if (nextRanges.size() == 1 && nextRanges.get(0)[0] == rangeStartY && nextRanges.get(0)[1] == rangeEndY) {
                                zEnd = nextZ + 1;
                                zi++;
                            } else {
                                break;
                            }
                        }
                        boxes.add(new LittleBox(x, rangeStartY, zStart, x + 1, rangeEndY, zEnd));
                    } else {
                        for (int[] range : ranges) {
                            boxes.add(new LittleBox(x, range[0], z, x + 1, range[1], z + 1));
                        }
                    }
                }
            }

            LittleTile newTile = new LittleTile(template.getState(), template.color, boxes);
            result.addTileFast(grid, newTile);
            totalBoxes += boxes.size();
        }

        LittleTiles.LOGGER.warn("Stage 7 - ArrayList column merge + horizontal merge: {} tiles, {} boxes: {} ms",
                result.totalTiles(), totalBoxes, System.currentTimeMillis() - stageStart);

        // 8. Finalize grid and center
        stageStart = System.currentTimeMillis();
        result.convertToSmallest();
        translateToOrigin(result);
        LittleTiles.LOGGER.warn("Stage 8 - Grid normalization & translation: {} ms", System.currentTimeMillis() - stageStart);

        LittleTiles.LOGGER.warn("Total rotation time: {} ms", System.currentTimeMillis() - startTotal);
        return result;
    }

    // ========== BVH Implementation (unchanged) ==========

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

    // ========== Helper Classes ==========

    private static class SourceBox {
        final LittleBox box;
        final LittleTile tile;
        final float minX, minY, minZ, maxX, maxY, maxZ;

        SourceBox(LittleBox box, LittleTile tile) {
            this.box = box;
            this.tile = tile;
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