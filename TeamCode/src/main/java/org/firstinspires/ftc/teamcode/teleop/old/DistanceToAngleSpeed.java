package org.firstinspires.ftc.teamcode.teleop.old;

import java.util.Arrays;
import java.util.Comparator;

/**
 * Maps a measured distance to an (angle, speed) pair by linear interpolation
 * over a sorted table of calibration rows. Angle and speed are normalized 0..1.
 *
 * This version contains a built-in calibration table (defaultTable()) so callers
 * do not need to supply calibration data.
 */
public class DistanceToAngleSpeed {
    public static class Row {
        public final double distance;
        public final double angle; // expected 0..1
        public final double speed; // expected 0..1

        public Row(double distance, double angle, double speed) {
            this.distance = distance;
            this.angle = clamp01(angle);
            this.speed = clamp01(speed);
        }
    }

    public static class Output {
        public final double angle; // 0..1
        public final double speed; // 0..1

        public Output(double angle, double speed) {
            this.angle = clamp01(angle);
            this.speed = clamp01(speed);
        }
    }

    private final Row[] table;

    /**
     * Construct with an array of rows (unsorted allowed). Table is copied and sorted by distance.
     */
    public DistanceToAngleSpeed(Row[] rows) {
        if (rows == null || rows.length == 0) throw new IllegalArgumentException("table must contain at least one row");
        this.table = Arrays.copyOf(rows, rows.length);
        Arrays.sort(this.table, Comparator.comparingDouble(r -> r.distance));
    }

    /**
     * Convenience no-arg constructor that uses the embedded default calibration table.
     */
    public DistanceToAngleSpeed() {
        this(defaultTableRows());
    }

    /**
     * Given an input distance, returns an interpolated Output (angle, speed).
     * If input is outside the table range, returns the nearest endpoint row (no extrapolation).
     */
    public Output getInterpolated(double distanceInput) {
        // single-row table => return that row
        if (table.length == 1) {
            Row r = table[0];
            return new Output(r.angle, r.speed);
        }

        // if below first or above last, clamp to endpoints
        if (distanceInput <= table[0].distance) {
            Row r = table[0];
            return new Output(r.angle, r.speed);
        }
        if (distanceInput >= table[table.length - 1].distance) {
            Row r = table[table.length - 1];
            return new Output(r.angle, r.speed);
        }

        // binary search to find interval [i, i+1] that contains distanceInput
        int lo = 0, hi = table.length - 1;
        while (lo + 1 < hi) {
            int mid = (lo + hi) >>> 1;
            if (table[mid].distance <= distanceInput) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        Row a = table[lo];
        Row b = table[hi];

        // linear interpolation factor t in [0,1]
        double t = (distanceInput - a.distance) / (b.distance - a.distance);

        double interpAngle = lerp(a.angle, b.angle, t);
        double interpSpeed = lerp(a.speed, b.speed, t);

        return new Output(interpAngle, interpSpeed);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double clamp01(double v) {
        if (v <= 0.0) return 0.0;
        if (v >= 1.0) return 1.0;
        return v;
    }

    // --- Built-in default calibration table (edit values here if you want different defaults) ---
    private static Row[] defaultTableRows() {
        return new Row[] {
               // new Row(22.26, 0.35, 0.3),
                //new Row(31.1, 0.35, 0.4),
               // new Row(38.78, 0.33, 0.45),
               // new Row(46.21, 0.2, 0.45),
               // new Row(52, 0.2, 0.45),
               // new Row(54.32, 0.2, 0.45),
                new Row(57, .2, .45),
                new Row(58.64,0.18, 0.45),
                new Row(60.92, .18, .46),
                new Row(62.5, 0.17, 0.46),
                new Row(63.86, .2, .45),
                new Row(64.82, .21, .452),
                new Row(68, .23, .47),
               // new Row(70.36, 0.2, 0.43),
                new Row(81.34, 0.185, 0.47),
                new Row(91.8, .16, .48),
                new Row(141.4, .17, .55)
        };
    }

    /**
     * Expose the default table so external code can inspect it if needed.
     */
    public static Row[] defaultTable() {
        return Arrays.copyOf(defaultTableRows(), defaultTableRows().length);
    }
}