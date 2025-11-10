package org.firstinspires.ftc.teamcode;/* TrajectoryGeneratorDemo.java
 *
 * Single-file demo + lower-allocation TrajectoryBuilder for on-robot use.
 *
 * Compile:
 *   javac TrajectoryGeneratorDemo.java
 * Run:
 *   java TrajectoryGeneratorDemo
 *
 * The program will:
 *  - build a sample trajectory (line, wait, spline, multi-spline, line)
 *  - print basic diagnostics (frame counts, total time)
 *  - write a CSV "trajectory_out.csv" containing t,x,y,theta for quick plotting/inspection
 *
 * Notes:
 *  - Units: arbitrary linear units (match your robot; Python version used inches).
 *  - The OptimizedBuilder aims to avoid per-sample ArrayList allocations by doing
 *    a two-pass approach: (1) measure dense geometry length, (2) allocate buffers once,
 *    (3) fill buffers. This is far less GC-churny on embedded platforms.
 *
 * This file intentionally contains a compact but complete implementation of:
 *  - Vec2, MathUtil, TrapezoidProfile
 *  - Arc-length sampling via robust linear interpolation
 *  - Hermite multi-point chain (C1)
 *  - Dense geometry generators for line/spline/sin/multi
 *  - OptimizedTrajectoryBuilder (two-pass, primitive buffers)
 *  - Small debug harness + CSV writer
 */

import java.io.*;
import java.util.Arrays;

public class TrajectoryGeneratorDemo {

    // -------------------------
    // Basic types & utilities
    // -------------------------
    static final class Vec2 {
        double x, y;
        Vec2() { x = 0; y = 0; }
        Vec2(double x, double y) { this.x = x; this.y = y; }
        Vec2 add(Vec2 o) { return new Vec2(x + o.x, y + o.y); }
        Vec2 sub(Vec2 o) { return new Vec2(x - o.x, y - o.y); }
        Vec2 mul(double s) { return new Vec2(x * s, y * s); }
        double norm() { return Math.hypot(x, y); }
    }

    static final class MathUtil {
        private MathUtil() {}
        static double shortestAngleDiff(double a, double b) {
            double d = (b - a + Math.PI) % (2.0 * Math.PI);
            if (d < 0) d += 2.0 * Math.PI;
            d -= Math.PI;
            return d;
        }
        static double clamp(double v, double lo, double hi) {
            return v < lo ? lo : (v > hi ? hi : v);
        }
        static double[] linspace(double a, double b, int n) {
            double[] out = new double[n];
            if (n == 1) { out[0] = a; return out; }
            double step = (b - a) / (n - 1);
            for (int i = 0; i < n; i++) out[i] = a + step * i;
            return out;
        }
    }

    // -------------------------
    // Trapezoid motion profile
    // -------------------------
    static final class TrapezoidProfile {
        static final class Result {
            final double[] times;
            final double[] alphas;
            Result(double[] times, double[] alphas) { this.times = times; this.alphas = alphas; }
        }
        static Result generate(double dist, double vmax, double amax, double dt) {
            if (dist <= 1e-9) {
                return new Result(new double[]{0.0, dt}, new double[]{0.0, 0.0});
            }
            double tAccel = vmax / amax;
            double dAccel = 0.5 * amax * tAccel * tAccel;
            double tFlat, tTotal;
            if (2 * dAccel >= dist) {
                tAccel = Math.sqrt(dist / amax);
                tFlat = 0.0;
                tTotal = 2.0 * tAccel;
            } else {
                double dFlat = dist - 2.0 * dAccel;
                tFlat = dFlat / vmax;
                tTotal = 2.0 * tAccel + tFlat;
            }
            int estN = Math.max(2, (int)Math.ceil(tTotal / dt) + 1);
            double[] times = new double[estN + 2]; // small margin
            double[] pos = new double[times.length];
            int idx = 0;
            for (double t = 0.0; t < tTotal + 1e-9; t += dt) {
                double p;
                if (t < tAccel) p = 0.5 * amax * t * t;
                else if (t < tAccel + tFlat) p = dAccel + vmax * (t - tAccel);
                else {
                    double tDec = t - (tAccel + tFlat);
                    p = dAccel + tFlat * vmax + (vmax * tDec - 0.5 * amax * tDec * tDec);
                }
                times[idx] = t;
                pos[idx] = p;
                idx++;
            }
            if (Math.abs(times[idx-1] - tTotal) > 1e-9) {
                times[idx] = tTotal;
                pos[idx] = dist;
                idx++;
            }
            double[] tOut = Arrays.copyOf(times, idx);
            double[] aOut = new double[idx];
            for (int i=0;i<idx;i++) aOut[i] = Math.max(0.0, Math.min(1.0, pos[i]/dist));
            return new Result(tOut, aOut);
        }
    }

    // -------------------------
    // Arc-length utilities
    // -------------------------
    static final class ArcLength {
        static double[] cumulativeLengths(double[] xs, double[] ys) {
            int n = xs.length;
            double[] cum = new double[n];
            cum[0] = 0.0;
            for (int i = 1; i < n; i++) {
                double dx = xs[i] - xs[i-1];
                double dy = ys[i] - ys[i-1];
                cum[i] = cum[i-1] + Math.hypot(dx, dy);
                if (cum[i] <= cum[i-1]) cum[i] = cum[i-1] + 1e-12;
            }
            return cum;
        }

        static Vec2[] mapToSamples(double[] xs, double[] ys, double[] targetDists) {
            double[] cum = cumulativeLengths(xs, ys);
            double total = cum[cum.length-1];
            Vec2[] out = new Vec2[targetDists.length];
            if (total <= 0) {
                for (int i=0;i<out.length;i++) out[i] = new Vec2(xs[0], ys[0]);
                return out;
            }
            int n = xs.length;
            for (int k=0;k<targetDists.length;k++) {
                double td = targetDists[k];
                if (td <= 0) { out[k] = new Vec2(xs[0], ys[0]); continue; }
                if (td >= total) { out[k] = new Vec2(xs[n-1], ys[n-1]); continue; }
                // binary search cum to find i s.t. cum[i] <= td < cum[i+1]
                int lo=0, hi=n-1;
                while (lo <= hi) {
                    int mid = (lo + hi) >>> 1;
                    if (cum[mid] <= td) lo = mid + 1; else hi = mid - 1;
                }
                int i0 = Math.max(0, lo-1);
                int i1 = Math.min(n-1, i0+1);
                double denom = cum[i1]-cum[i0];
                double a = denom <= 1e-12 ? 0.0 : (td - cum[i0]) / denom;
                double x = xs[i0] * (1.0-a) + xs[i1] * a;
                double y = ys[i0] * (1.0-a) + ys[i1] * a;
                out[k] = new Vec2(x, y);
            }
            return out;
        }
    }

    // -------------------------
    // Hermite chain (C1)
    // -------------------------
    static final class HermiteChain {
        // two-pass style: we return a compact Vec2[] chain
        static Vec2[] build(Vec2[] pts, double arcHeight, int samplesPerSeg, int maxTotal) {
            int n = pts.length;
            if (n < 2) return pts.clone();
            double[] segLen = new double[n-1];
            for (int i=0;i<n-1;i++) segLen[i] = pts[i+1].sub(pts[i]).norm();
            double[] segLenSafe = new double[segLen.length];
            for (int i=0;i<segLen.length;i++) segLenSafe[i] = Math.max(segLen[i], 1e-9);

            Vec2[] T = new Vec2[n];
            for (int i=0;i<n;i++) T[i] = new Vec2(0,0);
            // endpoint tangents
            Vec2 d0 = pts[1].sub(pts[0]); double n0 = d0.norm() + 1e-9; T[0] = d0.mul(arcHeight * segLenSafe[0] / n0);
            Vec2 dN = pts[n-1].sub(pts[n-2]); double nN = dN.norm() + 1e-9; T[n-1] = dN.mul(arcHeight * segLenSafe[n-2] / nN);
            for (int i=1;i<n-1;i++) {
                double local = 0.5*(segLenSafe[i-1] + segLenSafe[i]);
                Vec2 dir = pts[i+1].sub(pts[i-1]);
                double norm = dir.norm();
                if (norm < 1e-9) T[i] = new Vec2(0,0);
                else T[i] = dir.mul(arcHeight * local / norm);
            }

            // estimate count
            int estCount = (n-1) * samplesPerSeg;
            if (estCount > maxTotal) estCount = maxTotal;
            Vec2[] temp = new Vec2[estCount + 4]; // some slack
            int idx = 0;
            for (int seg=0; seg<n-1; seg++) {
                Vec2 p0 = pts[seg], p1 = pts[seg+1], t0 = T[seg], t1 = T[seg+1];
                int samples = samplesPerSeg;
                if (seg == n-2) samples = samplesPerSeg; // include endpoint
                for (int i=0;i<samples;i++) {
                    double u = i/(double)(samples-1);
                    double u2 = u*u, u3 = u2*u;
                    double h00 = 2*u3 - 3*u2 + 1;
                    double h10 = u3 - 2*u2 + u;
                    double h01 = -2*u3 + 3*u2;
                    double h11 = u3 - u2;
                    double x = h00*p0.x + h10*t0.x + h01*p1.x + h11*t1.x;
                    double y = h00*p0.y + h10*t0.y + h01*p1.y + h11*t1.y;
                    if (idx < temp.length) temp[idx++] = new Vec2(x,y);
                }
            }
            if (idx > maxTotal) {
                Vec2[] reduced = new Vec2[maxTotal];
                for (int i=0;i<maxTotal;i++) {
                    int pick = (int)Math.round(i * (idx-1) / (double)(maxTotal-1));
                    reduced[i] = temp[pick];
                }
                return reduced;
            } else {
                Vec2[] out = new Vec2[idx];
                System.arraycopy(temp, 0, out, 0, idx);
                return out;
            }
        }
    }

    // -------------------------
    // Dense geometry helpers (returns primitive arrays)
    // -------------------------
    static double[] denseLineX(double x0,double y0,double x1,double y1, int[] outN) {
        int n = Math.max(4, (int)Math.round(Math.hypot(x1-x0,y1-y0)*6) + 2);
        outN[0] = n;
        double[] xs = new double[n];
        for (int i=0;i<n;i++) {
            double t = i/(double)(n-1);
            xs[i] = x0 + (x1-x0)*t;
        }
        return xs;
    }
    static double[] denseLineY(double x0,double y0,double x1,double y1, int[] outN) {
        int n = Math.max(4, (int)Math.round(Math.hypot(x1-x0,y1-y0)*6) + 2);
        outN[0] = n;
        double[] ys = new double[n];
        for (int i=0;i<n;i++) {
            double t = i/(double)(n-1);
            ys[i] = y0 + (y1-y0)*t;
        }
        return ys;
    }
    static double[] denseSplineX(double x0,double y0,double th0,double x1,double y1,double th1,int n) {
        Vec2 p0 = new Vec2(x0,y0), p3 = new Vec2(x1,y1);
        double dist = Math.hypot(x1-x0,y1-y0);
        Vec2 p1 = p0.add(new Vec2(Math.cos(th0), Math.sin(th0)).mul(dist*0.4));
        Vec2 p2 = p3.sub(new Vec2(Math.cos(th1), Math.sin(th1)).mul(dist*0.4));
        double[] out = new double[n];
        for (int i=0;i<n;i++) {
            double t = i/(double)(n-1);
            double t2 = t*t, t3 = t2*t;
            double b0 = (1-t)*(1-t)*(1-t);
            double b1 = 3*(1-t)*(1-t)*t;
            double b2 = 3*(1-t)*t*t;
            double b3 = t3;
            out[i] = b0*p0.x + b1*p1.x + b2*p2.x + b3*p3.x;
        }
        return out;
    }
    static double[] denseSplineY(double x0,double y0,double th0,double x1,double y1,double th1,int n) {
        Vec2 p0 = new Vec2(x0,y0), p3 = new Vec2(x1,y1);
        double dist = Math.hypot(x1-x0,y1-y0);
        Vec2 p1 = p0.add(new Vec2(Math.cos(th0), Math.sin(th0)).mul(dist*0.4));
        Vec2 p2 = p3.sub(new Vec2(Math.cos(th1), Math.sin(th1)).mul(dist*0.4));
        double[] out = new double[n];
        for (int i=0;i<n;i++) {
            double t = i/(double)(n-1);
            double t2 = t*t, t3 = t2*t;
            double b0 = (1-t)*(1-t)*(1-t);
            double b1 = 3*(1-t)*(1-t)*t;
            double b2 = 3*(1-t)*t*t;
            double b3 = t3;
            out[i] = b0*p0.y + b1*p1.y + b2*p2.y + b3*p3.y;
        }
        return out;
    }
    static double[] denseSinX(double x0,double y0,double x1,double y1,int n) {
        double[] out = new double[n];
        for (int i=0;i<n;i++) out[i] = x0 + (x1-x0)*i/(double)(n-1);
        return out;
    }
    static double[] denseSinY(double x0,double y0,double x1,double y1,int n,double freq) {
        double[] out = new double[n];
        for (int i=0;i<n;i++) {
            double t = i/(double)(n-1);
            out[i] = y0 + (y1-y0)*t + 5.0*Math.sin(2.0*Math.PI*freq*t);
        }
        return out;
    }

    // -------------------------
    // Optimized TrajectoryBuilder - two-pass no-ArrayList heavy usage
    // -------------------------
    static final class OptimizedTrajectoryBuilder {
        // Simple path representation types
        static final int PT_LINE = 1, PT_SPLINE = 2, PT_SIN = 3, PT_MULTI = 4, PT_WAIT = 5;

        // Configuration / constraints
        final double DT;
        final double MAX_VEL;
        final double MAX_ACCEL;
        final double MAX_ANG_VEL;
        final double MAX_ANG_ACCEL;
        final double MAX_SERVO_SPEED;

        final int MAX_MULTI_SAMPLES;
        final int MAX_UNIFORM_SAMPLES;
        final int DENSE_SPLINE_SAMPLES;
        final int DENSE_SIN_SAMPLES;
        final int DENSE_MULTI_PERSEG;
        final double CURVATURE_SCALE;
        final double CURVATURE_CLIP;

        // internal path arrays (simple primitive-driven storage)
        final int MAX_PATHS = 200;
        final int[] pathType = new int[MAX_PATHS];
        final double[][] pathParams = new double[MAX_PATHS][]; // variable param arrays
        int pathCount = 0;

        // poses (associated with path endpoints). store as (x,y,theta)
        final double[][] poses = new double[MAX_PATHS+1][3];
        int poseCount = 0;

        public OptimizedTrajectoryBuilder(double dt, double maxV, double maxA,
                                          int maxMultiSamples, int maxUniformSamples,
                                          int denseSpline, int denseSin, int denseMultiPerSeg,
                                          double curvatureScale, double curvatureClip) {
            this.DT = dt; this.MAX_VEL = maxV; this.MAX_ACCEL = maxA;
            this.MAX_ANG_VEL = Math.toRadians(180); this.MAX_ANG_ACCEL = Math.toRadians(180);
            this.MAX_SERVO_SPEED = Math.toRadians(360);

            this.MAX_MULTI_SAMPLES = maxMultiSamples;
            this.MAX_UNIFORM_SAMPLES = maxUniformSamples;
            this.DENSE_SPLINE_SAMPLES = denseSpline;
            this.DENSE_SIN_SAMPLES = denseSin;
            this.DENSE_MULTI_PERSEG = denseMultiPerSeg;
            this.CURVATURE_SCALE = curvatureScale;
            this.CURVATURE_CLIP = curvatureClip;

            this.pathCount = 0;
            this.poseCount = 0;
        }

        public void setStartPose(double x, double y, double th) {
            poses[0][0] = x; poses[0][1] = y; poses[0][2] = th;
            poseCount = 1;
        }

        public void lineTo(double x, double y, double th) {
            if (pathCount >= MAX_PATHS) throw new IllegalStateException("too many paths");
            pathType[pathCount] = PT_LINE;
            pathParams[pathCount] = new double[]{x,y,th};
            pathCount++;
            poses[poseCount][0]=x; poses[poseCount][1]=y; poses[poseCount][2]=th; poseCount++;
        }

        public void splineTo(double x, double y, double th) {
            if (pathCount >= MAX_PATHS) throw new IllegalStateException("too many paths");
            pathType[pathCount] = PT_SPLINE;
            pathParams[pathCount] = new double[]{x,y,th};
            pathCount++;
            poses[poseCount][0]=x; poses[poseCount][1]=y; poses[poseCount][2]=th; poseCount++;
        }

        public void sinTo(double x,double y,double th,double freq) {
            if (pathCount >= MAX_PATHS) throw new IllegalStateException("too many paths");
            pathType[pathCount] = PT_SIN;
            pathParams[pathCount] = new double[]{x,y,th,freq};
            pathCount++;
            poses[poseCount][0]=x; poses[poseCount][1]=y; poses[poseCount][2]=th; poseCount++;
        }

        public void multiSplineTo(double[][] poseList, double arcH, int samplesPerSeg) {
            if (pathCount >= MAX_PATHS) throw new IllegalStateException("too many paths");
            // compact store: params = {arcH, samplesPerSeg, n, x1,y1, x2,y2, ...}
            int n = poseList.length;
            double[] params = new double[3 + 2*n];
            params[0]=arcH; params[1]=samplesPerSeg; params[2]=n;
            for (int i=0;i<n;i++){ params[3+2*i]=poseList[i][0]; params[3+2*i+1]=poseList[i][1]; }
            pathType[pathCount] = PT_MULTI;
            pathParams[pathCount] = params;
            pathCount++;
            // append final pose: use last point's heading as previous if not provided
            double x = poseList[n-1][0], y = poseList[n-1][1];
            double prevTh = poses[poseCount-1][2];
            poses[poseCount][0]=x; poses[poseCount][1]=y; poses[poseCount][2]=prevTh; poseCount++;
        }

        public void waitSeconds(double t) {
            if (pathCount >= MAX_PATHS) throw new IllegalStateException("too many paths");
            pathType[pathCount] = PT_WAIT;
            pathParams[pathCount] = new double[]{t};
            pathCount++;
            // poses unchanged (duplicate last)
            poses[poseCount][0] = poses[poseCount-1][0]; poses[poseCount][1] = poses[poseCount-1][1]; poses[poseCount][2] = poses[poseCount-1][2];
            poseCount++;
        }

        // Result container
        static final class Trajectory {
            final double[] t, x, y, th;
            Trajectory(double[] t, double[] x, double[] y, double[] th) { this.t=t; this.x=x; this.y=y; this.th=th; }
        }

        // Heavy method: two-pass to avoid repeated array list churn
        public Trajectory build(boolean debug) {
            // PASS 1: measure dense geometry size and compute per-block data (no big allocs)
            final int maxBlocks = pathCount + 4;
            int blockCount = 0;
            int[] blockType = new int[maxBlocks]; // 1 wait, 2 move
            int[] blockStartIdx = new int[maxBlocks]; // index into paths for block
            int[] blockLen = new int[maxBlocks]; // number of path entries in block (for move)
            // We'll walk paths and create blocks: "wait" blocks are standalone, contiguous moves grouped
            int p = 0;
            while (p < pathCount) {
                if (pathType[p] == PT_WAIT) {
                    blockType[blockCount]=1;
                    blockStartIdx[blockCount]=p;
                    blockLen[blockCount]=1;
                    blockCount++;
                    p++;
                } else {
                    // move block: accumulate until next wait
                    blockType[blockCount]=2;
                    blockStartIdx[blockCount]=p;
                    int cnt=0;
                    while (p < pathCount && pathType[p] != PT_WAIT) { cnt++; p++; }
                    blockLen[blockCount]=cnt;
                    blockCount++;
                }
            }

            // measure dense total length to allocate arrays in one shot
            long totalDenseSamplesEstimate = 0;
            double totalTimeEstimate = 0.0;
            for (int bi=0; bi<blockCount; bi++) {
                if (blockType[bi] == 1) {
                    double waitT = pathParams[blockStartIdx[bi]][0];
                    totalTimeEstimate += waitT;
                } else {
                    // estimate dense size for this move block
                    int startPath = blockStartIdx[bi];
                    int len = blockLen[bi];
                    // sum of per-segment dense counts
                    int denseCountBlock = 0;
                    // find the starting pose index for the block
                    // We need to reconstruct start poses efficiently: we walk the path indices to find pose indices.
                    // For simplicity we will compute per-path dense count using rules consistent with dense helpers:
                    int poseIdx = 0; // walking through poses: each path appends a pose
                    // But that's expensive here — we can instead approximate: each path yields at least 4 samples
                    // Simpler: estimate denseCountBlock = len * max(DENSE_SPLINE_SAMPLES, DENSE_MULTI_PERSEG)
                    denseCountBlock = len * Math.max(DENSE_SPLINE_SAMPLES, DENSE_MULTI_PERSEG);
                    if (denseCountBlock <= 0) denseCountBlock = 8;
                    totalDenseSamplesEstimate += denseCountBlock;
                    // estimate nominal time for block assuming average speed
                    totalTimeEstimate += 1.0 * len; // rough
                }
            }
            // cap
            if (totalDenseSamplesEstimate > MAX_MULTI_SAMPLES) totalDenseSamplesEstimate = MAX_MULTI_SAMPLES;

            // allocate dense arrays
            int denseCap = (int)Math.max(64, Math.min(totalDenseSamplesEstimate + 8, MAX_MULTI_SAMPLES));
            double[] denseX = new double[denseCap];
            double[] denseY = new double[denseCap];
            int denseFill = 0;

            // PASS 2: fill dense arrays per block, then process into final uniform-time arrays.
            // We'll create temporary lists of block outputs (small number of blocks) using arrays of arrays.
            final int MAX_BLOCKS = blockCount + 4;
            double[][] blockXs = new double[MAX_BLOCKS][];
            double[][] blockYs = new double[MAX_BLOCKS][];
            double[][] blockTimes = new double[MAX_BLOCKS][];
            double[][] blockThs = new double[MAX_BLOCKS][];
            int bOutCount = 0;
            double globalTimeOffset = 0.0;
            int pathCursorPoseIdx = 0; // index into poses for path iteration

            // We'll iterate through original paths and build block outputs
            int pathIndex = 0;
            int poseIdxForPath = 0; // which pose corresponds to start of current path
            // We'll maintain a pointer to poses array: poses[poseCursor] is current start pose
            int poseCursor = 0;

            for (int bi=0; bi<blockCount; bi++) {
                if (blockType[bi] == 1) {
                    // wait block
                    int pIdx = blockStartIdx[bi];
                    double waitT = pathParams[pIdx][0];
                    // starting pose is the pose at the moment (we need to find the pose index)
                    // Count how many non-wait paths we've seen to get poseCursor; for simplicity, we derive from block start path index:
                    int poseIndexForThisWait = 0;
                    // compute pose index by walking path types up to pIdx
                    int poseCounter = 0;
                    for (int k=0;k<pIdx;k++) {
                        // every non-wait path increments pose index by 1, each wait increments as well (we kept pose duplicates),
                        // but we stored poses in insertion order in the builder. We can use simple mapping: the pose at path i corresponds to pose index = i
                        // Actually, the builder stored a pose for each path appended; startPose is at poses[0].
                        // For path i, starting pose is poses[i]
                        poseCounter = pIdx; // approximate mapping
                    }
                    poseIndexForThisWait = Math.min(poseCount-1, pIdx);
                    double x0 = poses[poseIndexForThisWait][0];
                    double y0 = poses[poseIndexForThisWait][1];
                    double th0 = poses[poseIndexForThisWait][2];
                    int n = Math.max(1, (int)Math.ceil(waitT / DT));
                    double[] times = new double[n];
                    double[] xs = new double[n];
                    double[] ys = new double[n];
                    double[] ths = new double[n];
                    for (int i=0;i<n;i++) {
                        times[i] = i * DT + globalTimeOffset;
                        xs[i] = x0; ys[i]=y0; ths[i]=th0;
                    }
                    blockXs[bOutCount] = xs; blockYs[bOutCount] = ys; blockTimes[bOutCount] = times; blockThs[bOutCount] = ths;
                    bOutCount++;
                    globalTimeOffset = times[n-1] + DT;
                } else {
                    // movement block: accumulate dense geometry across block paths
                    int startPathIdx = blockStartIdx[bi];
                    int len = blockLen[bi];
                    denseFill = 0;
                    int localPoseIdx = startPathIdx; // starting pose index (approximation)
                    // To keep the demo straightforward and robust, we will reconstruct dense geometry by iterating over the paths inside the block
                    // and append dense pieces to denseX/denseY.
                    int pathPtr = startPathIdx;
                    double lastX = Double.NaN, lastY = Double.NaN;
                    for (int pi=0; pi<len; pi++, pathPtr++) {
                        int typ = pathType[pathPtr];
                        double[] params = pathParams[pathPtr];
                        double x0 = poses[pathPtr][0]; double y0 = poses[pathPtr][1]; double th0 = poses[pathPtr][2];
                        // in the simplified mapping above, poses[pathPtr] should represent the start pose of that path.
                        if (typ == PT_LINE) {
                            double x1 = params[0], y1 = params[1];
                            int[] nOut = new int[1];
                            double[] px = denseLineX(x0,y0,x1,y1,nOut);
                            double[] py = denseLineY(x0,y0,x1,y1,nOut);
                            int n = nOut[0];
                            int start = (denseFill==0)?0: denseFill;
                            for (int i=0;i<n;i++) {
                                double pxv = px[i], pyv = py[i];
                                if (denseFill > 0 && Math.abs(pxv - lastX) < 1e-12 && Math.abs(pyv - lastY) < 1e-12) continue;
                                if (denseFill >= denseX.length) break;
                                denseX[denseFill] = pxv; denseY[denseFill] = pyv; denseFill++;
                                lastX = pxv; lastY = pyv;
                            }
                        } else if (typ == PT_SPLINE) {
                            double x1 = params[0], y1 = params[1], th1 = params[2];
                            double[] px = denseSplineX(x0,y0,th0,x1,y1,th1,DENSE_SPLINE_SAMPLES);
                            double[] py = denseSplineY(x0,y0,th0,x1,y1,th1,DENSE_SPLINE_SAMPLES);
                            for (int i=0;i<px.length;i++) {
                                double pxv = px[i], pyv = py[i];
                                if (denseFill > 0 && Math.abs(pxv - lastX) < 1e-12 && Math.abs(pyv - lastY) < 1e-12) continue;
                                if (denseFill >= denseX.length) break;
                                denseX[denseFill] = pxv; denseY[denseFill] = pyv; denseFill++;
                                lastX = pxv; lastY = pyv;
                            }
                        } else if (typ == PT_SIN) {
                            double x1 = params[0], y1 = params[1], th1 = params[2], freq = params[3];
                            double[] px = denseSinX(x0,y0,x1,y1,DENSE_SIN_SAMPLES);
                            double[] py = denseSinY(x0,y0,x1,y1,DENSE_SIN_SAMPLES,freq);
                            for (int i=0;i<px.length;i++) {
                                double pxv = px[i], pyv = py[i];
                                if (denseFill > 0 && Math.abs(pxv - lastX) < 1e-12 && Math.abs(pyv - lastY) < 1e-12) continue;
                                if (denseFill >= denseX.length) break;
                                denseX[denseFill] = pxv; denseY[denseFill] = pyv; denseFill++;
                                lastX = pxv; lastY = pyv;
                            }
                        } else if (typ == PT_MULTI) {
                            double arcH = params[0];
                            int sps = (int)params[1];
                            int npts = (int)params[2];
                            Vec2[] pts = new Vec2[npts+1];
                            pts[0] = new Vec2(x0,y0);
                            for (int i=0;i<npts;i++) pts[i+1] = new Vec2(params[3+2*i], params[3+2*i+1]);
                            Vec2[] chain = HermiteChain.build(pts, arcH, sps, DENSE_MULTI_PERSEG * Math.max(1, npts));
                            for (int i=0;i<chain.length;i++) {
                                double pxv = chain[i].x, pyv = chain[i].y;
                                if (denseFill > 0 && Math.abs(pxv - lastX) < 1e-12 && Math.abs(pyv - lastY) < 1e-12) continue;
                                if (denseFill >= denseX.length) break;
                                denseX[denseFill] = pxv; denseY[denseFill] = pyv; denseFill++;
                                lastX = pxv; lastY = pyv;
                            }
                        } else {
                            // unknown
                        }
                    } // end per-path in block

                    // Now we have dense geometry in denseX[0..denseFill-1], denseY similarly.
                    // Compute arc-length and curvature-based adaptive samples for the block.
                    if (denseFill < 2) {
                        // degenerate
                        double[] xs = new double[]{denseX[0], denseX[0]};
                        double[] ys = new double[]{denseY[0], denseY[0]};
                        double[] times = new double[]{globalTimeOffset, globalTimeOffset + DT};
                        double[] ths = new double[]{poses[0][2], poses[0][2]};
                        blockXs[bOutCount] = xs; blockYs[bOutCount] = ys; blockTimes[bOutCount] = times; blockThs[bOutCount] = ths;
                        bOutCount++;
                        globalTimeOffset = times[times.length-1] + DT;
                        continue;
                    }

                    double[] cum = ArcLength.cumulativeLengths(Arrays.copyOf(denseX, denseFill), Arrays.copyOf(denseY, denseFill));
                    double totalLen = cum[cum.length-1];
                    if (totalLen < 1e-9) {
                        double[] xs = new double[]{denseX[0], denseX[0]};
                        double[] ys = new double[]{denseY[0], denseY[0]};
                        double[] times = new double[]{globalTimeOffset, globalTimeOffset + DT};
                        double[] ths = new double[]{poses[0][2], poses[0][2]};
                        blockXs[bOutCount] = xs; blockYs[bOutCount] = ys; blockTimes[bOutCount] = times; blockThs[bOutCount] = ths;
                        bOutCount++;
                        globalTimeOffset = times[times.length-1] + DT;
                        continue;
                    }

                    // desired sample count for block (limit by MAX_MULTI_SAMPLES)
                    double nominalTime = Math.max(totalLen / Math.max(0.8*MAX_VEL, 1e-9), DT);
                    int desiredNum = Math.max((int)Math.ceil(nominalTime / DT) + 1, DENSE_MULTI_PERSEG * Math.max(1, len));
                    desiredNum = Math.min(desiredNum, MAX_MULTI_SAMPLES);

                    // compute curvature (finite diff)
                    int N = denseFill;
                    double[] sArr = cum;
                    double[] dx_ds = new double[N], dy_ds = new double[N];
                    for (int i=0;i<N;i++) {
                        if (i==0) {
                            double ds = sArr[1]-sArr[0];
                            dx_ds[0] = (denseX[1]-denseX[0]) / Math.max(1e-12, ds);
                            dy_ds[0] = (denseY[1]-denseY[0]) / Math.max(1e-12, ds);
                        } else if (i==N-1) {
                            double ds = sArr[N-1]-sArr[N-2];
                            dx_ds[N-1] = (denseX[N-1]-denseX[N-2]) / Math.max(1e-12, ds);
                            dy_ds[N-1] = (denseY[N-1]-denseY[N-2]) / Math.max(1e-12, ds);
                        } else {
                            double ds = sArr[i+1]-sArr[i-1];
                            dx_ds[i] = (denseX[i+1]-denseX[i-1]) / Math.max(1e-12, ds);
                            dy_ds[i] = (denseY[i+1]-denseY[i-1]) / Math.max(1e-12, ds);
                        }
                    }
                    double[] d2x = new double[N], d2y = new double[N];
                    for (int i=0;i<N;i++) {
                        if (i==0) {
                            double ds = sArr[1]-sArr[0];
                            d2x[0] = (dx_ds[1]-dx_ds[0]) / Math.max(1e-12, ds);
                            d2y[0] = (dy_ds[1]-dy_ds[0]) / Math.max(1e-12, ds);
                        } else if (i==N-1) {
                            double ds = sArr[N-1]-sArr[N-2];
                            d2x[N-1] = (dx_ds[N-1]-dx_ds[N-2]) / Math.max(1e-12, ds);
                            d2y[N-1] = (dy_ds[N-1]-dy_ds[N-2]) / Math.max(1e-12, ds);
                        } else {
                            double ds = sArr[i+1]-sArr[i-1];
                            d2x[i] = (dx_ds[i+1]-dx_ds[i-1]) / Math.max(1e-12, ds);
                            d2y[i] = (dy_ds[i+1]-dy_ds[i-1]) / Math.max(1e-12, ds);
                        }
                    }
                    double[] curvature = new double[N];
                    double meanCurv = 0.0;
                    for (int i=0;i<N;i++) {
                        double denom = Math.pow(dx_ds[i]*dx_ds[i] + dy_ds[i]*dy_ds[i], 1.5);
                        if (denom < 1e-12) denom = 1e-12;
                        curvature[i] = Math.abs(dx_ds[i]*d2y[i] - dy_ds[i]*d2x[i]) / denom;
                        if (curvature[i] > CURVATURE_CLIP) curvature[i] = CURVATURE_CLIP;
                        meanCurv += curvature[i];
                    }
                    meanCurv /= Math.max(1.0, N);
                    double[] importance = new double[N];
                    for (int i=0;i<N;i++) importance[i] = 1.0 + CURVATURE_SCALE * (curvature[i] / (meanCurv + 1e-9));
                    // segment importance
                    int segCount = N-1;
                    double[] segImp = new double[segCount];
                    for (int i=0;i<segCount;i++) segImp[i] = 0.5 * (importance[i] + importance[i+1]) * (sArr[i+1]-sArr[i]);
                    double[] cumImp = new double[segCount + 1];
                    cumImp[0]=0.0;
                    for (int i=0;i<segCount;i++) cumImp[i+1] = cumImp[i] + segImp[i];
                    double totalImp = cumImp[cumImp.length-1];
                    if (totalImp <= 0) totalImp = 1.0;
                    // pick sample positions in importance space
                    double[] t_imp = MathUtil.linspace(0.0, totalImp, desiredNum);
                    double[] sPositions = new double[desiredNum];
                    for (int k=0;k<desiredNum;k++) {
                        double v = t_imp[k];
                        // binary search cumImp
                        int lo=0, hi=cumImp.length-1;
                        while (lo <= hi) {
                            int mid = (lo+hi)>>>1;
                            if (cumImp[mid] <= v) lo = mid+1; else hi = mid-1;
                        }
                        int i0 = Math.max(0, lo-1), i1 = Math.min(cumImp.length-1, i0+1);
                        double denom = cumImp[i1] - cumImp[i0];
                        double w = denom <= 1e-12 ? 0.0 : (v - cumImp[i0]) / denom;
                        // linear interpolate sArr
                        int sLo = Math.min(N-1, i0);
                        int sHi = Math.min(N-1, i1);
                        double sVal = sArr[sLo] * (1.0-w) + sArr[sHi] * w;
                        sPositions[k] = sVal;
                    }

                    // map sPositions -> samples
                    Vec2[] samples = ArcLength.mapToSamples(Arrays.copyOf(denseX, denseFill), Arrays.copyOf(denseY, denseFill), sPositions);
                    double[] xsBlock = new double[samples.length], ysBlock = new double[samples.length];
                    for (int i=0;i<samples.length;i++){ xsBlock[i]=samples[i].x; ysBlock[i]=samples[i].y; }

                    // trapezoid mapping block times
                    TrapezoidProfile.Result tp = TrapezoidProfile.generate(totalLen, MAX_VEL, MAX_ACCEL, (int)DT==DT?DT:DT);
                    double[] timesRel = tp.times;
                    double[] alphas = tp.alphas;
                    double s0 = sPositions[0], sN_ = sPositions[sPositions.length-1];
                    double[] alphaS = new double[sPositions.length];
                    for (int i=0;i<alphaS.length;i++) alphaS[i] = (sPositions[i] - s0) / Math.max(1e-9, sN_ - s0);
                    double[] timesBlock = new double[alphaS.length];
                    // timesBlock via interpolation alphas->timesRel
                    for (int i=0;i<alphaS.length;i++) {
                        double a = alphaS[i];
                        // linear search for monotonic alphas
                        int pos=0;
                        while (pos < alphas.length-1 && alphas[pos+1] < a) pos++;
                        if (pos >= alphas.length-1) timesBlock[i] = timesRel[timesRel.length-1];
                        else {
                            double da = alphas[pos+1] - alphas[pos];
                            double w = da <= 1e-12 ? 0.0 : (a - alphas[pos]) / da;
                            timesBlock[i] = timesRel[pos]*(1.0-w) + timesRel[pos+1]*w;
                        }
                    }
                    if (timesBlock.length != xsBlock.length) {
                        double lastT = timesRel[timesRel.length-1];
                        for (int i=0;i<timesBlock.length;i++) timesBlock[i]= lastT * i / Math.max(1, timesBlock.length-1);
                    }

                    // headings
                    int M = xsBlock.length;
                    double[] dx_t = new double[M], dy_t = new double[M];
                    for (int i=0;i<M;i++) {
                        if (i==0) { dx_t[i]=xsBlock[1]-xsBlock[0]; dy_t[i]=ysBlock[1]-ysBlock[0]; }
                        else if (i==M-1) { dx_t[i]=xsBlock[M-1]-xsBlock[M-2]; dy_t[i]=ysBlock[M-1]-ysBlock[M-2]; }
                        else { dx_t[i] = xsBlock[i+1]-xsBlock[i-1]; dy_t[i] = ysBlock[i+1]-ysBlock[i-1]; }
                    }
                    double[] thBlock = new double[M];
                    for (int i=0;i<M;i++) thBlock[i] = Math.atan2(dy_t[i], dx_t[i]);
                    // anchor to start pose heading
                    int startPoseIdx = Math.min(poseCount-1, Math.max(0, startPathIdx)); // approximation
                    double th0 = poses[startPoseIdx][2];
                    double shift = MathUtil.shortestAngleDiff(thBlock[0], th0);
                    for (int i=0;i<M;i++) thBlock[i] += shift;
                    for (int i=1;i<M;i++) {
                        double d = MathUtil.shortestAngleDiff(thBlock[i-1], thBlock[i]);
                        thBlock[i] = thBlock[i-1] + d;
                    }

                    // absolute times
                    double[] absTimes = new double[timesBlock.length];
                    for (int i=0;i<absTimes.length;i++) absTimes[i] = timesBlock[i] + globalTimeOffset;

                    // pack block (append)
                    blockXs[bOutCount] = xsBlock; blockYs[bOutCount] = ysBlock; blockTimes[bOutCount] = absTimes; blockThs[bOutCount] = thBlock;
                    bOutCount++;
                    globalTimeOffset = absTimes[absTimes.length-1] + DT;
                } // end movement block
            } // end blocks

            // STITCH blocks into single long arrays
            int totalLen = 0;
            for (int i=0;i<bOutCount;i++) totalLen += blockXs[i].length;
            double[] xsAll = new double[totalLen], ysAll = new double[totalLen], thAll = new double[totalLen], timesAll = new double[totalLen];
            int pos = 0;
            for (int i=0;i<bOutCount;i++) {
                System.arraycopy(blockXs[i], 0, xsAll, pos, blockXs[i].length);
                System.arraycopy(blockYs[i], 0, ysAll, pos, blockYs[i].length);
                System.arraycopy(blockThs[i], 0, thAll, pos, blockThs[i].length);
                System.arraycopy(blockTimes[i], 0, timesAll, pos, blockTimes[i].length);
                pos += blockXs[i].length;
            }
            // enforce strictly increasing times
            for (int i=1;i<timesAll.length;i++) if (timesAll[i] <= timesAll[i-1]) timesAll[i] = timesAll[i-1] + 1e-6;

            if (timesAll.length < 2 || !allFinite(xsAll) || !allFinite(ysAll)) {
                System.out.println("[WARN] results invalid, returning raw");
                return new Trajectory(timesAll, xsAll, ysAll, thAll);
            }

            // FINAL: resample on uniform time grid for physics
            double totalTime = timesAll[timesAll.length-1];
            int frames = (int)Math.ceil(totalTime / DT) + 1;
            frames = Math.min(frames, MAX_UNIFORM_SAMPLES);
            double[] uniformT = new double[frames];
            for (int i=0;i<frames;i++) uniformT[i] = i * (totalTime / Math.max(1, frames-1));

            double[] xsU = new double[frames], ysU = new double[frames], thU = new double[frames];
            for (int k=0;k<frames;k++) {
                double tt = uniformT[k];
                // binary search timesAll
                int idxFound = Arrays.binarySearch(timesAll, tt);
                if (idxFound >= 0) {
                    xsU[k] = xsAll[idxFound]; ysU[k] = ysAll[idxFound]; thU[k] = thAll[idxFound];
                } else {
                    int ins = -idxFound - 1;
                    int i0 = Math.max(0, ins-1), i1 = Math.min(timesAll.length-1, ins);
                    double denom = timesAll[i1] - timesAll[i0];
                    double w = denom <= 1e-12 ? 0.0 : (tt - timesAll[i0]) / denom;
                    xsU[k] = xsAll[i0] * (1.0-w) + xsAll[i1] * w;
                    ysU[k] = ysAll[i0] * (1.0-w) + ysAll[i1] * w;
                    double c0 = Math.cos(thAll[i0]), c1 = Math.cos(thAll[i1]);
                    double s0 = Math.sin(thAll[i0]), s1 = Math.sin(thAll[i1]);
                    double c = c0*(1.0-w) + c1*w;
                    double s = s0*(1.0-w) + s1*w;
                    thU[k] = Math.atan2(s, c);
                }
            }

            if (debug) {
                System.out.println("[build] final uniform frames: " + xsU.length + " total_time: " + totalTime + " dense samples used (approx): " + denseCap);
            }
            return new Trajectory(uniformT, xsU, ysU, thU);
        }

        private static boolean allFinite(double[] a) {
            for (double v : a) if (!Double.isFinite(v)) return false;
            return true;
        }
    }

    // -------------------------
    // Demo main (build & write CSV)
    // -------------------------
    public static void main(String[] args) throws Exception {
        // Configuration - similar to your Python defaults
        double DT = 0.05;
        double MAX_VEL = 30.0;
        double MAX_ACCEL = 30.0;
        int MAX_MULTI_SAMPLES = 4000;
        int MAX_UNIFORM_SAMPLES = 900;
        int DENSE_SPLINE_SAMPLES = 800;
        int DENSE_SIN_SAMPLES = 800;
        int DENSE_MULTI_PERSEG = 140;
        double CURVATURE_SCALE = 8.0;
        double CURVATURE_CLIP = 1e3;

        // Build a sample trajectory
        OptimizedTrajectoryBuilder builder = new OptimizedTrajectoryBuilder(
                DT, MAX_VEL, MAX_ACCEL,
                MAX_MULTI_SAMPLES, MAX_UNIFORM_SAMPLES,
                DENSE_SPLINE_SAMPLES, DENSE_SIN_SAMPLES, DENSE_MULTI_PERSEG,
                CURVATURE_SCALE, CURVATURE_CLIP
        );
        builder.setStartPose(0.0, 0.0, 0.0);
        builder.lineTo(20.0, 0.0, Math.PI/4.0);
        builder.waitSeconds(1.0);
        builder.splineTo(40.0, 20.0, Math.PI/2.0);
        double[][] multi = new double[][] {
                {45.0, 30.0},
                {20.0, 40.0},
                {0.0, 35.0},
                {-20.0, 10.0},
                {-10.0, -10.0}
        };
        builder.multiSplineTo(multi, 0.6, 140);
        builder.lineTo(20.0, 0.0, Math.PI/2.0);

        System.out.println("Building trajectory (this may take a second)...");
        OptimizedTrajectoryBuilder.Trajectory traj = builder.build(true);

        System.out.println("Trajectory built:");
        System.out.println("  frames: " + traj.t.length);
        System.out.println("  total_time: " + traj.t[traj.t.length-1]);
        // quick velocity check (magnitudes)
        double maxSpeed = 0.0;
        for (int i=1;i<traj.x.length;i++) {
            double dt = traj.t[i] - traj.t[i-1];
            double vx = (traj.x[i]-traj.x[i-1]) / dt;
            double vy = (traj.y[i]-traj.y[i-1]) / dt;
            double speed = Math.hypot(vx, vy);
            if (speed > maxSpeed) maxSpeed = speed;
        }
        System.out.printf("  max speed estimate: %.3f (units/s)\n", maxSpeed);

        // Write CSV
        writeCSV("trajectory_out.csv", traj);
        System.out.println("Wrote trajectory_out.csv (t,x,y,theta). Inspect in Excel/Matplotlib.");

        // extra: print last 8 points
        System.out.println("Last 8 samples:");
        for (int i=Math.max(0, traj.t.length-8); i<traj.t.length; i++) {
            System.out.printf("  t=%.3f x=%.3f y=%.3f th=%.3f\n", traj.t[i], traj.x[i], traj.y[i], traj.th[i]);
        }

        System.out.println("Done.");
    }

    static void writeCSV(String path, OptimizedTrajectoryBuilder.Trajectory traj) throws IOException {
        try (BufferedWriter w = new BufferedWriter(new FileWriter(path))) {
            w.write("t,x,y,theta\n");
            for (int i=0;i<traj.t.length;i++) {
                w.write(String.format("%.6f,%.6f,%.6f,%.6f\n", traj.t[i], traj.x[i], traj.y[i], traj.th[i]));
            }
        }
    }
}
