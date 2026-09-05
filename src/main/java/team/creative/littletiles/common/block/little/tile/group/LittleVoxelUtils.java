package team.creative.littletiles.common.block.little.tile.group;

import java.util.*;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import team.creative.littletiles.common.block.little.tile.LittleTile;
import team.creative.littletiles.common.grid.LittleGrid;
import team.creative.littletiles.common.math.box.LittleBox;

public class LittleVoxelUtils {

    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll) {
        LittleGroup voxelGroup = group.voxelize();
        LittleGrid grid = voxelGroup.getGrid();

        List<LittleTile> tileList = new ArrayList<>();
        List<Long> positionList = new ArrayList<>();
        for (LittleTile tile : voxelGroup.allTiles()) {
            for (LittleBox box : tile) {
                for (int x = box.minX; x < box.maxX; x++) {
                    for (int y = box.minY; y < box.maxY; y++) {
                        for (int z = box.minZ; z < box.maxZ; z++) {
                            tileList.add(tile);
                            positionList.add(encode(x, y, z));
                        }
                    }
                }
            }
        }
        if (positionList.isEmpty()) return new LittleGroup();

        int minX = Integer.MAX_VALUE, minY = Integer.MAX_VALUE, minZ = Integer.MAX_VALUE;
        int maxX = Integer.MIN_VALUE, maxY = Integer.MIN_VALUE, maxZ = Integer.MIN_VALUE;
        for (long key : positionList) {
            int[] p = decode(key);
            minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0] + 1);
            minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1] + 1);
            minZ = Math.min(minZ, p[2]); maxZ = Math.max(maxZ, p[2] + 1);
        }
        float centerX = (minX + maxX) * 0.5f;
        float centerY = (minY + maxY) * 0.5f;
        float centerZ = (minZ + maxZ) * 0.5f;

        Matrix4f rot = new Matrix4f().rotationXYZ(pitch, yaw, roll);

        Map<LittleTile, Set<Long>> rotatedMap = new HashMap<>();
        Vector3f vec = new Vector3f();
        for (int i = 0; i < positionList.size(); i++) {
            LittleTile tile = tileList.get(i);
            int[] p = decode(positionList.get(i));

            vec.set(p[0] + 0.5f - centerX, p[1] + 0.5f - centerY, p[2] + 0.5f - centerZ);
            rot.transformPosition(vec);
            vec.add(centerX, centerY, centerZ);
            int nx = Math.round(vec.x());
            int ny = Math.round(vec.y());
            int nz = Math.round(vec.z());
            long nkey = encode(nx, ny, nz);
            rotatedMap.computeIfAbsent(tile, k -> new HashSet<>()).add(nkey);
        }

        LittleGroup result = new LittleGroup();
        for (Map.Entry<LittleTile, Set<Long>> entry : rotatedMap.entrySet()) {
            LittleTile template = entry.getKey();
            Set<Long> positions = entry.getValue();
            List<LittleBox> boxes = new ArrayList<>(positions.size());
            for (long key : positions) {
                int[] p = decode(key);
                boxes.add(new LittleBox(p[0], p[1], p[2], p[0] + 1, p[1] + 1, p[2] + 1));
            }
            LittleTile newTile = new LittleTile(template.getState(), template.color, boxes);
            newTile.combine(grid, true);
            result.addTile(grid, newTile);
        }
        result.convertToSmallest();
        return result;
    }

    public static long encode(int x, int y, int z) {
        return (((long)x) << 32) | (((long)y) << 16) | (z & 0xFFFFL);
    }
    public static int[] decode(long key) {
        return new int[]{(int)(key >> 32), (int)((key >> 16) & 0xFFFFL), (int)(key & 0xFFFFL)};
    }
}