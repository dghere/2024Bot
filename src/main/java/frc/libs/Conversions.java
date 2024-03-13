package frc.libs;

public class Conversions {
    public static double neoToMeters(double clicks, double circumference, double gearRatio)
    {
        return clicks * (circumference / ( gearRatio * 48));
    }
}
