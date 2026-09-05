package team.creative.littletiles.common.block.little.tile.group;

import java.util.*;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import team.creative.littletiles.common.block.little.tile.LittleTile;
import team.creative.littletiles.common.grid.LittleGrid;
import team.creative.littletiles.common.math.box.LittleBox;
import team.creative.littletiles.common.math.vec.LittleVec;

public class LittleVoxelUtils {

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll) {
        LittleGroup voxelGroup = group.voxelize();
        LittleGrid grid = voxelGroup.getGrid();

        Map<LittleTile, Set<LittleVec>> originalMap = new HashMap<>();
        for (LittleTile tile : voxelGroup.allTiles()) {
            for (LittleBox box : tile) {
                for (int x = box.minX; x < box.maxX; x++) {
                    for (int y = box.minY; y < box.maxY; y++) {
                        for (int z = box.minZ; z < box.maxZ; z++) {
                            originalMap.computeIfAbsent(tile, k -> new HashSet<>())
                                    .add(new LittleVec(x, y, z));
                        }
                    }
                }
            }
        }
        if (originalMap.isEmpty()) return new LittleGroup();

        int minX = Integer.MAX_VALUE, minY = Integer.MAX_VALUE, minZ = Integer.MAX_VALUE;
        int maxX = Integer.MIN_VALUE, maxY = Integer.MIN_VALUE, maxZ = Integer.MIN_VALUE;
        for (Set<LittleVec> set : originalMap.values()) {
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
        Map<LittleTile, Set<LittleVec>> rotatedMap = new HashMap<>();

        for (Map.Entry<LittleTile, Set<LittleVec>> entry : originalMap.entrySet()) {
            LittleTile tile = entry.getKey();
            for (LittleVec v : entry.getValue()) {
                vec.set(v.x + 0.5f - centerX, v.y + 0.5f - centerY, v.z + 0.5f - centerZ);
                rot.transformPosition(vec);
                vec.add(centerX, centerY, centerZ);
                int nx = Math.round(vec.x());
                int ny = Math.round(vec.y());
                int nz = Math.round(vec.z());
                rotatedMap.computeIfAbsent(tile, k -> new HashSet<>()).add(new LittleVec(nx, ny, nz));
            }
        }

        LittleGroup result = new LittleGroup();
        for (Map.Entry<LittleTile, Set<LittleVec>> entry : rotatedMap.entrySet()) {
            LittleTile template = entry.getKey();
            List<LittleBox> boxes = new ArrayList<>();
            for (LittleVec v : entry.getValue()) {
                boxes.add(new LittleBox(v.x, v.y, v.z, v.x + 1, v.y + 1, v.z + 1));
            }
            LittleTile newTile = new LittleTile(template.getState(), template.color, boxes);
            newTile.combine(grid, true);
            result.addTile(grid, newTile);
        }
        result.convertToSmallest();
        return result;
    }
}