package frc.robot.constants;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public final class DynamicConstants {
    /*
     * Add static classes below for Shuffleboard dynamic contants. Each class represents 
     * a subsystem and will have its own tab. These constants are stored on the RIO through 
     * NetworkTables json and should be saved to the driver station computer after changes.
     * 
     * WARNING: Dynamic constants from the RIO may be read after initialization. Ensure functionality 
     * of any constants that are only read on startup.
     */

    public static class ArmSetpoints {
        public static double armL4 = 0;
        public static double armL3 = 0;
        public static double armL2 = 0;
        public static double armL1 = 0;

        public static double armTestPos = 0.1;

    }

    public static class ElevatorSetpoints {
        public static double elevL4 = 0;
        public static double elevL3 = 0;
        public static double elevL2 = 0;
        public static double elevL1 = 0;
        public static double elevAlgaeTop = 0;
        public static double elevAlgaeBot = 0;
        public static double elevClimb = 0;

        public static double elevTestPos = 0.5;
    }

    public static class Algae {
        public static double intakePercent = .5;
        public static double outtakePercent = -.5;
    } 

    public static class TestVoltages {
        public static double bucketTestOut = 0;
        public static double agitatorTestOut = 0;
        public static double algaeTestOut = 0;
    }

    public static class IRThresholds {
        public static double bucketIRthreshold = 1.9;
        public static double coralIRthreshold = 1.5;
    }


    private static HashMap<Field, SimpleWidget> entries;

    /*
     * Initializes all the Shuffleboard tabs and widgets by pulling their fields from the provided classes
     */
    public static void init() {
        ShuffleboardTab subsystemIOTab;// = Shuffleboard.getTab("SubsystemIO");
        entries = new HashMap<>();

        //add all .class values of the static classes above
        Class<?>[] subsystems = {ArmSetpoints.class, ElevatorSetpoints.class, TestVoltages.class, Algae.class};
        
        for(Class<?> subsystem : subsystems) {
            Field[] fields = subsystem.getDeclaredFields();
            subsystemIOTab = Shuffleboard.getTab(subsystem.getSimpleName());
            for (Field field : fields) {
                if (java.lang.reflect.Modifier.isStatic(field.getModifiers()) && field.getType() == double.class) {
                    field.setAccessible(true);
                    double value = 0;
                    try{
                        value = field.getDouble(null);
                    }
                    catch(IllegalAccessException a){
                        System.out.println("Access Exception when reading subsystem IO");
                    }
                    entries.put(field, subsystemIOTab.addPersistent(subsystem.getSimpleName() + ": " + field.getName(), "double", value).withSize(2, 1));
                }
            }
        }
    }

    /*
     * Must be periodically called. Updates the constants values from the Shuffleboard
     */
    public static void periodic(){
        for(Entry<Field, SimpleWidget> entry : entries.entrySet()){
            try{
                entry.getKey().setDouble(null, entry.getValue().getEntry().getDouble(0));
            }
            catch(IllegalAccessException a){
                System.out.println("Access Exception when writing subsystem IO");
            }
        }
    }
}
