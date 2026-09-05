package team.creative.littletiles.common.block.little.tile.group;

import java.util.*;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import team.creative.littletiles.common.block.little.tile.LittleTile;
import team.creative.littletiles.common.grid.LittleGrid;
import team.creative.littletiles.common.math.box.LittleBox;

public class LittleVoxelUtils {

    /**
     * 对体素组进行任意角度旋转（欧拉角，弧度）。
     * 输入组会被体素化后再旋转，返回新的体素组（无结构、无子组）。
     * 使用最近邻重采样，适用于 90° 倍数以外任意角度。
     */
    public static LittleGroup rotateVoxels(LittleGroup group, float yaw, float pitch, float roll) {
        // 先体素化
        LittleGroup voxelGroup = group.voxelize();
        LittleGrid grid = voxelGroup.getGrid();

        // 收集所有体素及其材质
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

        // 计算包围盒中心（基于原始坐标）
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

        // 构建旋转矩阵（顺序：ZYX，即先绕 X 轴旋转 roll，再绕 Y 轴 yaw，最后绕 Z 轴 pitch）
        Matrix4f rot = new Matrix4f().rotateZYX(roll, pitch, yaw); // JOML 方法：rotateZYX(angleZ, angleY, angleX)
        // 或者显式构建：rot.rotationZYX(roll, pitch, yaw);

        // 存储旋转后的体素（按材质分组）
        Map<LittleTile, Set<Long>> rotatedMap = new HashMap<>();
        for (int i = 0; i < positionList.size(); i++) {
            LittleTile tile = tileList.get(i);
            long key = positionList.get(i);
            int[] p = decode(key);
            // 将坐标平移到中心，应用旋转，再平移回
            Vector3f vec = new Vector3f(p[0] + 0.5f - centerX, p[1] + 0.5f - centerY, p[2] + 0.5f - centerZ);
            vec.mul((Vector3fc) rot);
            vec.add(centerX, centerY, centerZ);
            // 四舍五入到最近整数网格
            int nx = Math.round(vec.x());
            int ny = Math.round(vec.y());
            int nz = Math.round(vec.z());
            // 确保非负（可选，但应允许负值，因为包围盒可能偏移）
            long nkey = encode(nx, ny, nz);
            rotatedMap.computeIfAbsent(tile, k -> new HashSet<>()).add(nkey);
        }

        // 构建新组
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

    // 编码/解码（同前）
    public static long encode(int x, int y, int z) {
        return (((long)x) << 32) | (((long)y) << 16) | (z & 0xFFFFL);
    }
    public static int[] decode(long key) {
        return new int[]{(int)(key >> 32), (int)((key >> 16) & 0xFFFFL), (int)(key & 0xFFFFL)};
    }
}