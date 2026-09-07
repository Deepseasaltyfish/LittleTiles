package team.creative.littletiles.common.placement.shape.type;

import team.creative.creativecore.common.util.math.vec.Vec3d;
import team.creative.littletiles.client.tool.shaper.ShapePosition;
import team.creative.littletiles.client.tool.shaper.ShapeSelection;
import team.creative.littletiles.common.math.box.LittleBox;
import team.creative.littletiles.common.math.box.collection.LittleBoxes;
import team.creative.littletiles.common.math.vec.LittleVec;
import team.creative.littletiles.common.placement.shape.LittleShape;
import team.creative.littletiles.common.placement.shape.config.CatenaryConfig;

public class LittleShapeCatenary extends LittleShape<CatenaryConfig> {

    private static final double EPS = 1e-8;
    private static final double MIN_A = 0.01;
    private static final double MIN_HORIZONTAL_DIST = 0.5;
    private static final int MAX_ITER = 300;
    private static final int LINE_SEARCH_ITER = 20;
    private static final int MAX_VOXELS = 1_000_000;
    private static final int MIN_STEPS = 4;

    public LittleShapeCatenary() {
        super(2);
    }

    @Override
    protected void build(LittleBoxes boxes, ShapeSelection selection, CatenaryConfig config) {
        if (selection.size() < 2) return;

        ShapePosition pos1 = selection.get(0);
        ShapePosition pos2 = selection.get(1);

        LittleVec rel1 = pos1.getRelative(selection.pos);
        LittleVec rel2 = pos2.getRelative(selection.pos);

        double x1 = rel1.x, y1 = rel1.y, z1 = rel1.z;
        double x2 = rel2.x, y2 = rel2.y, z2 = rel2.z;

        Vec3d p1 = new Vec3d(x1, y1, z1);
        Vec3d p2 = new Vec3d(x2, y2, z2);

        Vec3d delta = p2.copy();
        delta.sub(p1);
        double dx = delta.x, dz = delta.z;
        double horizontalDist = Math.sqrt(dx * dx + dz * dz);

        if (horizontalDist < MIN_HORIZONTAL_DIST) {
            addColumn(boxes, p1, p2, config.thickness);
            return;
        }

        double ux = dx / horizontalDist;
        double uz = dz / horizontalDist;

        double drop = config.drop; // grid units
        double yLow = Math.min(y1, y2);
        double rise1 = y1 - yLow;
        double rise2 = y2 - yLow;

        if (drop < EPS) {
            drawLine(boxes, p1, p2, horizontalDist, ux, uz, config.thickness);
            return;
        }

        double[] params = solveParams(horizontalDist, rise1, rise2, drop);
        if (params == null) {
            drawLine(boxes, p1, p2, horizontalDist, ux, uz, config.thickness);
            return;
        }

        double a = params[0];
        double x0 = params[1];
        double b = yLow - drop - a;

        int steps = computeSteps(horizontalDist, a, x0, b, boxes.grid.pixelLength);
        if (steps > MAX_VOXELS) {
            drawLine(boxes, p1, p2, horizontalDist, ux, uz, config.thickness);
            return;
        }

        double stepX = horizontalDist / steps;
        for (int i = 0; i <= steps; i++) {
            double x = i * stepX;
            double y = a * Math.cosh((x - x0) / a) + b;
            Vec3d world = new Vec3d(x1 + x * ux, y, z1 + x * uz);
            addBox(boxes, world, config.thickness);
        }
    }

    /* Solves a and x0 via Newton's method with line search. Returns null if fails. */
    private double[] solveParams(double d, double r1, double r2, double drop) {
        double a = Math.clamp((d * d) / (8 * drop), MIN_A, Double.MAX_VALUE);
        double x0 = d * 0.5;

        if (Math.abs(r1 - r2) > EPS) {
            double sum = r1 + r2 + 2 * drop;
            if (sum > EPS) {
                x0 = d * (r1 + drop) / sum;
                x0 = Math.clamp(x0, MIN_A, d - MIN_A);
            }
        }

        for (int iter = 0; iter < MAX_ITER; iter++) {
            double cosh1 = Math.cosh(x0 / a);
            double cosh2 = Math.cosh((d - x0) / a);
            double sinh1 = Math.sinh(x0 / a);
            double sinh2 = Math.sinh((d - x0) / a);

            double F1 = a * (cosh1 - 1) - (r1 + drop);
            double F2 = a * (cosh2 - 1) - (r2 + drop);

            double dF1da = (cosh1 - 1) - (x0 / a) * sinh1;
            double dF1dx0 = sinh1;
            double dF2da = (cosh2 - 1) - ((d - x0) / a) * sinh2;
            double dF2dx0 = -sinh2;

            double det = dF1da * dF2dx0 - dF1dx0 * dF2da;
            if (Math.abs(det) < EPS) break;

            double da = (F1 * dF2dx0 - dF1dx0 * F2) / det;
            double dx0 = (dF1da * F2 - F1 * dF2da) / det;

            double step = 1.0;
            double bestRes = Math.abs(F1) + Math.abs(F2);
            double bestA = a, bestX0 = x0;

            for (int i = 0; i < LINE_SEARCH_ITER; i++) {
                double aNew = Math.clamp(a - step * da, MIN_A, Double.MAX_VALUE);
                double x0New = Math.clamp(x0 - step * dx0, MIN_A, d - MIN_A);
                double F1n = aNew * (Math.cosh(x0New / aNew) - 1) - (r1 + drop);
                double F2n = aNew * (Math.cosh((d - x0New) / aNew) - 1) - (r2 + drop);
                double res = Math.abs(F1n) + Math.abs(F2n);
                if (res < bestRes) {
                    bestA = aNew;
                    bestX0 = x0New;
                    break;
                }
                step *= 0.5;
                if (step < 1e-12) break;
            }
            a = bestA;
            x0 = bestX0;

            if (Math.abs(da) < EPS && Math.abs(dx0) < EPS) break;
        }

        if (Double.isNaN(a) || a < MIN_A || x0 < 0 || x0 > d) return null;
        return new double[]{a, x0};
    }

    private int computeSteps(double d, double a, double x0, double b, double pixelSize) {
        int subSteps = 100;
        double step = d / subSteps;
        double arcLen = 0.0;
        double prevX = 0.0, prevY = a * Math.cosh((prevX - x0) / a) + b;
        for (int i = 1; i <= subSteps; i++) {
            double currX = i * step;
            double currY = a * Math.cosh((currX - x0) / a) + b;
            double dx = currX - prevX, dy = currY - prevY;
            arcLen += Math.sqrt(dx * dx + dy * dy);
            prevX = currX;
            prevY = currY;
        }
        int steps = (int) Math.ceil(arcLen / (pixelSize * 0.8));
        return Math.max(MIN_STEPS, steps);
    }

    private void drawLine(LittleBoxes boxes, Vec3d p1, Vec3d p2, double d, double ux, double uz, int thickness) {
        int steps = Math.max(MIN_STEPS, (int) Math.ceil(d / boxes.grid.pixelLength * 2));
        double step = 1.0 / steps;
        for (int i = 0; i <= steps; i++) {
            double t = i * step;
            double x = t * d;
            double y = p1.y + (p2.y - p1.y) / d * x;
            addBox(boxes, new Vec3d(p1.x + x * ux, y, p1.z + x * uz), thickness);
        }
    }

    private void addColumn(LittleBoxes boxes, Vec3d p1, Vec3d p2, int thickness) {
        int minX = (int) Math.floor(Math.min(p1.x, p2.x));
        int minY = (int) Math.floor(Math.min(p1.y, p2.y));
        int minZ = (int) Math.floor(Math.min(p1.z, p2.z));
        int maxX = (int) Math.ceil(Math.max(p1.x, p2.x));
        int maxY = (int) Math.ceil(Math.max(p1.y, p2.y));
        int maxZ = (int) Math.ceil(Math.max(p1.z, p2.z));
        LittleBox box = new LittleBox(minX, minY, minZ, maxX, maxY, maxZ);
        if (thickness > 1) box.growCentered(thickness - 1);
        boxes.add(box);
    }

    private void addBox(LittleBoxes boxes, Vec3d pos, int thickness) {
        int cx = (int) Math.round(pos.x);
        int cy = (int) Math.round(pos.y);
        int cz = (int) Math.round(pos.z);
        LittleBox box = new LittleBox(cx, cy, cz, cx + 1, cy + 1, cz + 1);
        if (thickness > 1) box.growCentered(thickness - 1);
        boxes.add(box);
    }

    @Override
    protected boolean requiresNoOverlap(ShapeSelection selection, CatenaryConfig config) {
        return true;
    }
}