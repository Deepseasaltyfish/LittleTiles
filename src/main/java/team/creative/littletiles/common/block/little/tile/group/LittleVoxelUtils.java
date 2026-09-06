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

    /**
     * Rotates a voxel group using all available CPU cores.
     */
    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll) {
        return rotateVoxels(group, yaw, pitch, roll, Runtime.getRuntime().availableProcessors());
    }

    /**
     * Rotates a voxel group with specified parallelism.
     * Uses forward mapping + per-column Y merge for optimal performance.
     */
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

        // 2. Build rotation matrix (forward)
        stageStart = System.currentTimeMillis();
        Matrix4f rot = new Matrix4f().rotationXYZ(pitch, yaw, roll);
        Vector3f vec = new Vector3f();
        LittleTiles.LOGGER.warn("Stage 2 - Matrix construction: {} ms", System.currentTimeMillis() - stageStart);

        // 3. Collect source voxels per material, forward-project and group by (x,z) column
        stageStart = System.currentTimeMillis();
        Map<LittleTile, Map<Long, Set<Integer>>> materialColumns = new ConcurrentHashMap<>();

        // Count total source voxels for logging
        long[] totalVoxelCount = {0};

        int threads = parallelism > 0 ? parallelism : Runtime.getRuntime().availableProcessors();
        ForkJoinPool pool = new ForkJoinPool(threads);

        try {
            pool.submit(() -> {
                // Parallel over all tiles and boxes
                copy.allTiles().forEach(tile -> {
                    for (LittleBox box : tile) {
                        // Enumerate every unit voxel inside this box
                        for (int x = box.minX; x < box.maxX; x++) {
                            for (int y = box.minY; y < box.maxY; y++) {
                                for (int z = box.minZ; z < box.maxZ; z++) {
                                    synchronized (totalVoxelCount) {
                                        totalVoxelCount[0]++;
                                    }
                                    // Forward transform the center of this voxel
                                    vec.set(x + 0.5f, y + 0.5f, z + 0.5f);
                                    rot.transformPosition(vec);
                                    int tx = Math.round(vec.x());
                                    int ty = Math.round(vec.y());
                                    int tz = Math.round(vec.z());
                                    // Group by (tx, tz) column, collect ty
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

        LittleTiles.LOGGER.warn("Stage 3 - Forward projection ({} threads): {} ms, {} source voxels processed",
                threads, System.currentTimeMillis() - stageStart, totalVoxelCount[0]);

        // 4. Build result group by merging columns into contiguous Y-runs
        stageStart = System.currentTimeMillis();
        LittleGroup result = new LittleGroup();
        int totalBoxes = 0;

        for (Map.Entry<LittleTile, Map<Long, Set<Integer>>> entry : materialColumns.entrySet()) {
            LittleTile template = entry.getKey();
            Map<Long, Set<Integer>> columns = entry.getValue();
            List<LittleBox> boxes = new ArrayList<>();

            for (Map.Entry<Long, Set<Integer>> colEntry : columns.entrySet()) {
                long key = colEntry.getKey();
                int x = (int) (key >> 32);
                int z = (int) (key & 0xffffffffL);
                Set<Integer> ySet = colEntry.getValue();

                // Sort Y values and merge contiguous runs
                List<Integer> yList = new ArrayList<>(ySet);
                Collections.sort(yList);

                int startY = yList.get(0);
                int endY = startY + 1;
                for (int i = 1; i < yList.size(); i++) {
                    int y = yList.get(i);
                    if (y == endY) {
                        endY++;
                    } else {
                        boxes.add(new LittleBox(x, startY, z, x + 1, endY, z + 1));
                        startY = y;
                        endY = y + 1;
                    }
                }
                boxes.add(new LittleBox(x, startY, z, x + 1, endY, z + 1));
            }

            // Create tile with merged boxes
            LittleTile newTile = new LittleTile(template.getState(), template.color, boxes);
            result.addTileFast(grid, newTile);
            totalBoxes += boxes.size();
        }

        LittleTiles.LOGGER.warn("Stage 4 - Column merge & group construction: {} ms, {} tiles, {} boxes",
                System.currentTimeMillis() - stageStart, result.totalTiles(), totalBoxes);

        // 5. Finalize grid and center
        stageStart = System.currentTimeMillis();
        result.convertToSmallest();
        translateToOrigin(result);
        LittleTiles.LOGGER.warn("Stage 5 - Grid normalization & translation: {} ms", System.currentTimeMillis() - stageStart);

        LittleTiles.LOGGER.warn("Total rotation time: {} ms", System.currentTimeMillis() - startTotal);
        return result;
    }

    // ========== Helper Methods ==========

    private static void translateToOrigin(LittleGroup group) {
        LittleVec min = group.getMinVec();
        if (min.x == 0 && min.y == 0 && min.z == 0) return;
        LittleVec negative = new LittleVec(-min.x, -min.y, -min.z);
        group.move(new LittleVecGrid(negative, group.getGrid()));
    }
}