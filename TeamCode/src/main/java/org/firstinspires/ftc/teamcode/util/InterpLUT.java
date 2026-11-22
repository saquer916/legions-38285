package org.firstinspires.ftc.teamcode.util;

import java.util.Map;
import java.util.TreeMap;

// Piecewise-linear lookup table. Inputs outside the table clamp to the first/last value.
public class InterpLUT {
    private final TreeMap<Double, Double> points = new TreeMap<>();

    public InterpLUT add(double input, double output) {
        points.put(input, output);
        return this;
    }

    public double get(double input) {
        Map.Entry<Double, Double> lower = points.floorEntry(input);
        Map.Entry<Double, Double> upper = points.ceilingEntry(input);
        if (lower == null) return upper.getValue();
        if (upper == null || lower.getKey().equals(upper.getKey())) return lower.getValue();

        double t = (input - lower.getKey()) / (upper.getKey() - lower.getKey());
        return lower.getValue() + t * (upper.getValue() - lower.getValue());
    }
}
