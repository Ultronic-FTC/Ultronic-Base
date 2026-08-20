package org.firstinspires.ftc.teamcode.lib.util;

import java.util.TreeMap;

public class InterpolatingDoubleMap {
    private final TreeMap<Double, Double> map = new TreeMap<>();

    public void put(double key, double value) {
        map.put(key, value);
    }

    public void clear() {
        map.clear();
    }

    public double get(double key) {
        Double exact = map.get(key);
        if (exact != null) return exact;

        Double lo = map.floorKey(key);
        Double hi = map.ceilingKey(key);

        if (lo == null) return map.firstEntry().getValue();
        if (hi == null) return map.lastEntry().getValue();

        double t = (key - lo) / (hi - lo);
        return map.get(lo) * (1 - t) + map.get(hi) * t;
    }
}