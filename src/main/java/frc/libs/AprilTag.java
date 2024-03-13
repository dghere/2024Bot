package frc.libs;

public class AprilTag// implements Comparable {
{
    private double fID;
    private double ta;
    private double tx;
    private double txp;
    private double ty;
    private double typ;
    
    public double getfID() { return fID; }
    public double getta() { return ta; }
    public double gettx() { return tx; }
    public double gettxp() { return txp; }
    public double getty() { return ty; }
    public double gettyp() { return typ; }

    public int compareTo(Object t)
    {
        AprilTag tag = (AprilTag)t;
        if(ty > tag.getty())
            return 1;
        else if(ty < tag.getty())
            return -1;
        else
            return 0;
    }

    public String toString()
    {
        return "" + getfID();
    }

}
