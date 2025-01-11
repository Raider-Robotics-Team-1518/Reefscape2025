// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Utils {

    public static float getHue(float r, float g, float b) {
        float[] hsl = RgbToHsv(r, g, b);
        return hsl[0];
    }

    /**
     * Convert a RGB Color to it corresponding HSL values.
     * modified from https://gist.github.com/Yona-Appletree/0c4b58763f070ae8cdff7db583c82563
     *
     * @return an array containing the 3 HSL values.
     */
    public static float[] RgbToHsv(float r, float g, float b) {
        // Minimum and Maximum RGB values are used in the HSL calculations

        float min = Math.min(r, Math.min(g, b));
        float max = Math.max(r, Math.max(g, b));

        // Calculate the Hue

        float h = 0;
        float diff = max - min;
        if (max == min)
            h = 0;
        else if (max == r)
            h = ((60 * (g - b) / diff) + 360) % 360;
        else if (max == g)
            h = (60 * (b - r) / diff) + 120;
        else if (max == b)
            h = (60 * (r - g) / diff) + 240;

        // Calculate the Luminance

        float l = (max + min) * 0.5f;

        // Calculate the Saturation

        float s = 0;

        if (max == min)
            s = 0;
        else if (l <= .5f)
            s = diff / (max + min);
        else
            s = diff / (2 - diff);

        return new float[] { h, s * 100, l * 100 };
    }

    public static double round2prec (double value, int precision) {
        int scale = (int) Math.pow(10, precision);
        return (double) Math.round(value * scale) / scale;
    } 
}
