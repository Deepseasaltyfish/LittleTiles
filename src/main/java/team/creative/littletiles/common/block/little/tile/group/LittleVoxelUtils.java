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

        // 2. Build rotation and inverse matrices
        stageStart = System.currentTimeMillis();
        Matrix4f rot = new Matrix4f().rotationXYZ(pitch, yaw, roll);
        Matrix4f invRot = new Matrix4f(rot).invert();
        Vector3f vec = new Vector3f();
        LittleTiles.LOGGER.warn("Stage 2 - Matrix construction: {} ms", System.currentTimeMillis() - stageStart);

        // 3. Extract source boxes with their materials
        stageStart = System.currentTimeMillis();
        List<SourceBox> sourceBoxes = new ArrayList<>();
        for (LittleTile tile : copy.allTiles()) {
            for (LittleBox box : tile) {
                sourceBoxes.add(new SourceBox(box, tile));
            }
        }
        LittleTiles.LOGGER.warn("Stage 3 - Extracted {} source boxes: {} ms", sourceBoxes.size(), System.currentTimeMillis() - stageStart);

        if (sourceBoxes.isEmpty()) return new LittleGroup();

        // 4. Forward project each source box: scan its rotated AABB, inverse test
        stageStart = System.currentTimeMillis();
        Map<LittleTile, Map<Long, Set<Integer>>> materialColumns = new ConcurrentHashMap<>();
        long[] totalVoxelsScanned = {0};
        long[] totalVoxelsAccepted = {0};

        int threads = parallelism > 0 ? parallelism : Runtime.getRuntime().availableProcessors();
        ForkJoinPool pool = new ForkJoinPool(threads);

        try {
            pool.submit(() -> {
                sourceBoxes.parallelStream().forEach(source -> {
                    LittleBox box = source.box;
                    LittleTile tile = source.tile;

                    // Compute rotated AABB of this box
                    float[][] corners = {
                            {box.minX, box.minY, box.minZ}, {box.maxX, box.minY, box.minZ},
                            {box.minX, box.maxY, box.minZ}, {box.maxX, box.maxY, box.minZ},
                            {box.minX, box.minY, box.maxZ}, {box.maxX, box.minY, box.maxZ},
                            {box.minX, box.maxY, box.maxZ}, {box.maxX, box.maxY, box.maxZ}
                    };
                    float minX = Float.POSITIVE_INFINITY, maxX = Float.NEGATIVE_INFINITY;
                    float minY = Float.POSITIVE_INFINITY, maxY = Float.NEGATIVE_INFINITY;
                    float minZ = Float.POSITIVE_INFINITY, maxZ = Float.NEGATIVE_INFINITY;
                    for (float[] c : corners) {
                        vec.set(c[0], c[1], c[2]);
                        rot.transformPosition(vec);
                        float x = vec.x(), y = vec.y(), z = vec.z();
                        minX = Math.min(minX, x); maxX = Math.max(maxX, x);
                        minY = Math.min(minY, y); maxY = Math.max(maxY, y);
                        minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);
                    }
                    int startX = (int)Math.floor(minX);
                    int endX = (int)Math.ceil(maxX);
                    int startY = (int)Math.floor(minY);
                    int endY = (int)Math.ceil(maxY);
                    int startZ = (int)Math.floor(minZ);
                    int endZ = (int)Math.ceil(maxZ);

                    // Scan this AABB
                    for (int tx = startX; tx < endX; tx++) {
                        for (int ty = startY; ty < endY; ty++) {
                            for (int tz = startZ; tz < endZ; tz++) {
                                synchronized (totalVoxelsScanned) {
                                    totalVoxelsScanned[0]++;
                                }
                                // Inverse transform to source space
                                vec.set(tx + 0.5f, ty + 0.5f, tz + 0.5f);
                                invRot.transformPosition(vec);
                                float sx = vec.x(), sy = vec.y(), sz = vec.z();
                                // Check if inside source box
                                if (sx >= box.minX - EPSILON && sx < box.maxX + EPSILON &&
                                        sy >= box.minY - EPSILON && sy < box.maxY + EPSILON &&
                                        sz >= box.minZ - EPSILON && sz < box.maxZ + EPSILON) {
                                    synchronized (totalVoxelsAccepted) {
                                        totalVoxelsAccepted[0]++;
                                    }
                                    long key = ((long) tx << 32) | (tz & 0xffffffffL);
                                    materialColumns.computeIfAbsent(tile, k -> new ConcurrentHashMap<>())
                                            .computeIfAbsent(key, k -> ConcurrentHashMap.newKeySet())
                                            .add(ty);
                                }
                            }
                        }
                    }
                });
            }).join();
        } finally {
            pool.shutdown();
        }
        LittleTiles.LOGGER.warn("Stage 4 - Forward projection ({} threads): {} ms, scanned {} voxels, accepted {} voxels",
                threads, System.currentTimeMillis() - stageStart, totalVoxelsScanned[0], totalVoxelsAccepted[0]);

        // 5. Build result group by column merging
        stageStart = System.currentTimeMillis();
        LittleGroup result = new LittleGroup();
        int totalBoxes = 0;
        for (Map.Entry<LittleTile, Map<Long, Set<Integer>>> entry : materialColumns.entrySet()) {
            LittleTile template = entry.getKey();
            Map<Long, Set<Integer>> columns = entry.getValue();
            List<LittleBox> boxes = new ArrayList<>();
            for (Map.Entry<Long, Set<Integer>> colEntry : columns.entrySet()) {
                long key = colEntry.getKey();
                int x = (int)(key >> 32);
                int z = (int)(key & 0xffffffffL);
                List<Integer> yList = new ArrayList<>(colEntry.getValue());
                Collections.sort(yList);
                int startY = yList.get(0);
                int endY = startY + 1;
                for (int i = 1; i < yList.size(); i++) {
                    int y = yList.get(i);
                    if (y == endY) endY++;
                    else {
                        boxes.add(new LittleBox(x, startY, z, x + 1, endY, z + 1));
                        startY = y;
                        endY = y + 1;
                    }
                }
                boxes.add(new LittleBox(x, startY, z, x + 1, endY, z + 1));
            }
            LittleTile newTile = new LittleTile(template.getState(), template.color, boxes);
            result.addTileFast(grid, newTile);
            totalBoxes += boxes.size();
        }
        LittleTiles.LOGGER.warn("Stage 5 - Column merge & group construction: {} ms, {} tiles, {} boxes",
                System.currentTimeMillis() - stageStart, result.totalTiles(), totalBoxes);

        // 6. Finalize
        stageStart = System.currentTimeMillis();
        result.convertToSmallest();
        translateToOrigin(result);
        LittleTiles.LOGGER.warn("Stage 6 - Grid normalization & translation: {} ms", System.currentTimeMillis() - stageStart);

        LittleTiles.LOGGER.warn("Total rotation time: {} ms", System.currentTimeMillis() - startTotal);
        return result;
    }

    private static class SourceBox {
        final LittleBox box;
        final LittleTile tile;
        SourceBox(LittleBox box, LittleTile tile) {
            this.box = box;
            this.tile = tile;
        }
    }

    private static void translateToOrigin(LittleGroup group) {
        LittleVec min = group.getMinVec();
        if (min.x == 0 && min.y == 0 && min.z == 0) return;
        LittleVec negative = new LittleVec(-min.x, -min.y, -min.z);
        group.move(new LittleVecGrid(negative, group.getGrid()));
    }
}