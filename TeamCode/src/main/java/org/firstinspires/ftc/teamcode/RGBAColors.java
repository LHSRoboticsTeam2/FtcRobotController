package org.firstinspires.ftc.teamcode;

public class RGBAColors {
    private int red;
    private int blue;
    private int green;
    private int alpha;
    public RGBAColors (int red, int blue, int green, int alpha){
        this.red = red;
        this.blue = blue;
        this.green = green;
        this.alpha = alpha;
    }
    public int getRed() {
        return this.red;
    }
    public int getBlue() {
        return this.blue;
    }
    public int getGreen() {
        return this.green;
    }
    public int getAlpha() {
        return this.alpha;
    }
}
