package frc.libs;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightLib {

    
    


    public static double GetDistance(AprilTag tag){
        double targetOffsetAngle_Vertical = tag.getty();;//8.5;
        double limelightMountAngleDegree = 8.5;//15;
        double limelightLensHeightInches = 17.;
        
        double goalHeightInches = 50;

        double angleToGoalDegree = limelightMountAngleDegree + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegree * (Math.PI / 180);

        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
}

    

    public static String getJSON(String hostName)
    {
        return NetworkTableInstance.getDefault().getTable(hostName).getEntry("json").getString("No Data");
    }


    public static AprilTag[] GetVisibleTags(String limeJson)
    {
        AprilTag[] tags;

        try
        {
            //  object mapper to create the JsonNodes
            ObjectMapper objectMapper = new ObjectMapper();
        
            // Find the list of April Tags ("Fiducial") seen by limelight "Results"
            JsonNode results = objectMapper.readTree(limeJson).at("/Results");
            JsonNode fiducials = results.get("Fiducial");

            //  configure to parse into Array and ignore fields not in used AprilTag class
            objectMapper.configure(DeserializationFeature.USE_JAVA_ARRAY_FOR_JSON_ARRAY, true);
            objectMapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

            //  This is the magic part.  the Jackson library will automatically create the arary of objects
            //  just send it the JSON string and tell it the .class to use
            tags = objectMapper.readValue(fiducials.toPrettyString(), AprilTag[].class);
        }
        catch(Exception e)
        {
            //  send back empty array if exception is thrown.
            tags = new AprilTag[0];
        }
        return tags;
    }


    
}
