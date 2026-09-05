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
    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll) {
        return rotateVoxels(group, yaw, pitch, roll, Runtime.getRuntime().availableProcessors());
    }

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll, int parallelism) {
        LittleTiles.LOGGER.info("Using {} thread(s) for rotating.", parallelism);

        int targetSize = group.getSmallest();
        LittleGrid grid = LittleGrid.get(targetSize);
        LittleGroup copy = group.copy();
        copy.convertTo(grid);

        List<SourceBox> sourceBoxes = new ArrayList<>();
        for (LittleTile tile : copy.allTiles()) {
            for (LittleBox box : tile) {
                sourceBoxes.add(new SourceBox(box, tile));
            }
        }
        if (sourceBoxes.isEmpty()) return new LittleGroup();

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

        Matrix4f invRot = new Matrix4f(rot).invert();

        int totalVoxels = (endX - startX) * (endY - startY) * (endZ - startZ);
        if (totalVoxels <= 0) return new LittleGroup();

        Map<LittleTile, Set<LittleVec>> resultMap = new ConcurrentHashMap<>();

        int threads = parallelism > 0 ? parallelism : Runtime.getRuntime().availableProcessors();
        ForkJoinPool pool = new ForkJoinPool(threads);

        try {
            pool.submit(() -> {
                IntStream.range(0, totalVoxels).parallel().forEach(index -> {
                    int x = startX + index / ((endY - startY) * (endZ - startZ));
                    int remainder = index % ((endY - startY) * (endZ - startZ));
                    int y = startY + remainder / (endZ - startZ);
                    int z = startZ + remainder % (endZ - startZ);

                    float cx = x + 0.5f;
                    float cy = y + 0.5f;
                    float cz = z + 0.5f;

                    vec.set(cx, cy, cz);
                    invRot.transformPosition(vec);
                    float sx = vec.x();
                    float sy = vec.y();
                    float sz = vec.z();

                    for (SourceBox sb : sourceBoxes) {
                        if (sb.contains(sx, sy, sz)) {
                            resultMap.computeIfAbsent(sb.tile, k -> ConcurrentHashMap.newKeySet())
                                    .add(new LittleVec(x, y, z));
                            break;
                        }
                    }
                });
            }).join();
        } finally {
            pool.shutdown();
        }

        LittleGroup result = new LittleGroup();
        for (Map.Entry<LittleTile, Set<LittleVec>> entry : resultMap.entrySet()) {
            LittleTile template = entry.getKey();
            Set<LittleVec> positions = entry.getValue();
            if (positions.isEmpty()) continue;

            List<LittleBox> boxes = new ArrayList<>(positions.size());
            for (LittleVec v : positions) {
                boxes.add(new LittleBox(v.x, v.y, v.z, v.x + 1, v.y + 1, v.z + 1));
            }
            LittleTile newTile = new LittleTile(template.getState(), template.color, boxes);
            newTile.combine(grid, true);
            result.addTile(grid, newTile);
        }

        result.convertToSmallest();
        translateToOrigin(result);

        return result;
    }

    private static class SourceBox {
        final LittleBox box;
        final LittleTile tile;
        private final float minX, minY, minZ, maxX, maxY, maxZ;
        private final float eps = 1e-6f; // 容差

        SourceBox(LittleBox box, LittleTile tile) {
            this.box = box;
            this.tile = tile;

            this.minX = box.minX - eps;
            this.minY = box.minY - eps;
            this.minZ = box.minZ - eps;
            this.maxX = box.maxX + eps;
            this.maxY = box.maxY + eps;
            this.maxZ = box.maxZ + eps;
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