package org.firstinspires.ftc.teamcode.util;

public class ColorTelemetry {

    public static String getFontFormatted(String f, int s, Color c){
        return "<font "+(s == -1 ? "" : "size=\""+s+"\" ")+(f.equals("none") ? "" : "face=\""+f+"\" ")
                +(c == Color.NO_COLOR ? "" : "color=#"+c.getHexValue()+" ")+">";
    }
    //amogus
}
