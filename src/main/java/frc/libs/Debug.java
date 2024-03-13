package frc.libs;

public class Debug {

    public static Boolean enabled = false;
    public static void log(String message)
    {
        if(enabled)
            System.out.println(message);

    }
    
}
