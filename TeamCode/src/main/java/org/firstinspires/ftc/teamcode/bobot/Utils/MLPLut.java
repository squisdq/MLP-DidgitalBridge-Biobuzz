package org.firstinspires.ftc.teamcode.bobot.Utils;

import java.util.ArrayList;
import java.util.List;

public class MLPLut {

    private final List<Double> xData;
    private final List<Double> yData;
    private final boolean allowExtrapolation;

    private MLPLut(List<Double> xData, List<Double> yData, boolean allowExtrapolation) {
        this.xData = xData;
        this.yData = yData;
        this.allowExtrapolation = allowExtrapolation;
    }

    public static Builder builder() {
        return new Builder();
    }

    public double get(double x) {
        int n = xData.size();
        if (n == 0) {
            return 0.0;
        }
        if (n == 1) {
            return yData.get(0);
        }

        double xMin = xData.get(0);
        double xMax = xData.get(n - 1);

        if (x < xMin) {
            return allowExtrapolation
                    ? extrapolate(x, 0, 1)
                    : yData.get(0);
        }
        if (x > xMax) {
            return allowExtrapolation
                    ? extrapolate(x, n - 2, n - 1)
                    : yData.get(n - 1);
        }

        for (int i = 0; i < n - 1; i++) {
            double x0 = xData.get(i);
            double x1 = xData.get(i + 1);
            if (x >= x0 && x <= x1) {
                return interpolate(x, x0, yData.get(i), x1, yData.get(i + 1));
            }
        }

        return yData.get(n - 1);
    }

    private double interpolate(double x, double x0, double y0, double x1, double y1) {
        if (x1 == x0) {
            return y0;
        }
        double t = (x - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    private double extrapolate(double x, int i0, int i1) {
        double x0 = xData.get(i0);
        double y0 = yData.get(i0);
        double x1 = xData.get(i1);
        double y1 = yData.get(i1);
        if (x1 == x0) {
            return y0;
        }
        double slope = (y1 - y0) / (x1 - x0);
        return y0 + slope * (x - x0);
    }

    public int size() {
        return xData.size();
    }

    public boolean isExtrapolationAllowed() {
        return allowExtrapolation;
    }

    public static class Builder {
        private final List<Double> xData = new ArrayList<>();
        private final List<Double> yData = new ArrayList<>();
        private boolean allowExtrapolation = false;

        public Builder add(double x, double y) {
            xData.add(x);
            yData.add(y);
            return this;
        }

        public Builder allowExtrapolation() {
            this.allowExtrapolation = true;
            return this;
        }

        public Builder allowExtrapolation(boolean value) {
            this.allowExtrapolation = value;
            return this;
        }

        public MLPLut build() {
            int n = xData.size();
            for (int i = 1; i < n; i++) {
                double xKey = xData.get(i);
                double yKey = yData.get(i);
                int j = i - 1;
                while (j >= 0 && xData.get(j) > xKey) {
                    xData.set(j + 1, xData.get(j));
                    yData.set(j + 1, yData.get(j));
                    j--;
                }
                xData.set(j + 1, xKey);
                yData.set(j + 1, yKey);
            }
            return new MLPLut(xData, yData, allowExtrapolation);
        }
    }
}