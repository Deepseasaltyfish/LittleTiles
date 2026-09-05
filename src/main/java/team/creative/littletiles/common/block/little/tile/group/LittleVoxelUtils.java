package team.creative.littletiles.common.block.little.tile.group;

import java.util.*;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import team.creative.littletiles.common.block.little.tile.LittleTile;
import team.creative.littletiles.common.grid.LittleGrid;
import team.creative.littletiles.common.math.box.LittleBox;
import team.creative.littletiles.common.math.vec.LittleVec;
import team.creative.littletiles.common.math.vec.LittleVecGrid;

public class LittleVoxelUtils {

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll) {
        return rotateVoxels(group, yaw, pitch, roll, 2);
    }

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll, int superSample) {
        if (superSample < 1) superSample = 1;

        LittleGroup voxelGroup = group.voxelize();
        LittleGrid originalGrid = voxelGroup.getGrid();

        int superSize = originalGrid.count * superSample;
        LittleGrid superGrid = LittleGrid.get(superSize);

        voxelGroup.convertTo(superGrid);

        Map<LittleTile, Set<LittleVec>> superMap = new HashMap<>();
        for (LittleTile tile : voxelGroup.allTiles()) {
            for (LittleBox box : tile) {
                for (int x = box.minX; x < box.maxX; x++) {
                    for (int y = box.minY; y < box.maxY; y++) {
                        for (int z = box.minZ; z < box.maxZ; z++) {
                            superMap.computeIfAbsent(tile, k -> new HashSet<>())
                                    .add(new LittleVec(x, y, z));
                        }
                    }
                }
            }
        }
        if (superMap.isEmpty()) return new LittleGroup();

        int minX = Integer.MAX_VALUE, minY = Integer.MAX_VALUE, minZ = Integer.MAX_VALUE;
        int maxX = Integer.MIN_VALUE, maxY = Integer.MIN_VALUE, maxZ = Integer.MIN_VALUE;
        for (Set<LittleVec> set : superMap.values()) {
            for (LittleVec v : set) {
                minX = Math.min(minX, v.x); maxX = Math.max(maxX, v.x);
                minY = Math.min(minY, v.y); maxY = Math.max(maxY, v.y);
                minZ = Math.min(minZ, v.z); maxZ = Math.max(maxZ, v.z);
            }
        }
        float centerX = (minX + maxX) * 0.5f;
        float centerY = (minY + maxY) * 0.5f;
        float centerZ = (minZ + maxZ) * 0.5f;

        Matrix4f rot = new Matrix4f().rotationXYZ(pitch, yaw, roll);
        Vector3f vec = new Vector3f();
        Map<LittleTile, Set<LittleVec>> rotatedSuperMap = new HashMap<>();

        for (Map.Entry<LittleTile, Set<LittleVec>> entry : superMap.entrySet()) {
            LittleTile tile = entry.getKey();
            for (LittleVec v : entry.getValue()) {
                vec.set(v.x + 0.5f - centerX, v.y + 0.5f - centerY, v.z + 0.5f - centerZ);
                rot.transformPosition(vec);
                vec.add(centerX, centerY, centerZ);
                int nx = Math.round(vec.x());
                int ny = Math.round(vec.y());
                int nz = Math.round(vec.z());
                rotatedSuperMap.computeIfAbsent(tile, k -> new HashSet<>())
                        .add(new LittleVec(nx, ny, nz));
            }
        }

        Map<LittleTile, Set<LittleVec>> downsampledMap = new HashMap<>();
        for (Map.Entry<LittleTile, Set<LittleVec>> entry : rotatedSuperMap.entrySet()) {
            LittleTile tile = entry.getKey();
            for (LittleVec v : entry.getValue()) {
                int ox = Math.floorDiv(v.x, superSample);
                int oy = Math.floorDiv(v.y, superSample);
                int oz = Math.floorDiv(v.z, superSample);
                downsampledMap.computeIfAbsent(tile, k -> new HashSet<>())
                        .add(new LittleVec(ox, oy, oz));
            }
        }

        LittleGroup result = new LittleGroup();
        for (Map.Entry<LittleTile, Set<LittleVec>> entry : downsampledMap.entrySet()) {
            LittleTile template = entry.getKey();
            List<LittleBox> boxes = new ArrayList<>(entry.getValue().size());
            for (LittleVec v : entry.getValue()) {
                boxes.add(new LittleBox(v.x, v.y, v.z, v.x + 1, v.y + 1, v.z + 1));
            }
            LittleTile newTile = new LittleTile(template.getState(), template.color, boxes);
            newTile.combine(originalGrid, true); // 合并相邻体素
            result.addTile(originalGrid, newTile);
        }

        result.convertToSmallest();
        translateToOrigin(result);

        return result;
    }

    private static void translateToOrigin(LittleGroup group) {
        LittleVec min = group.getMinVec();
        if (min.x == 0 && min.y == 0 && min.z == 0) return;
        LittleVec negative = new LittleVec(-min.x, -min.y, -min.z);
        group.move(new LittleVecGrid(negative, group.getGrid()));
    }
}