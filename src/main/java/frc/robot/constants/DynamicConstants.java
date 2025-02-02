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
    public static class Example{
        public static double exampleConstant = 0;
    }
    public static class Testing {
        //voltage of certain movements
        public static double testElevatorVoltage = 0;
        public static double algaePercentOUt = 0;
        public static double testCoralIntakeVoltage = 0;
        public static double globalTestPercentOut = 0;
    }


    private static HashMap<Field, SimpleWidget> entries;

    /*
     * Initializes all the Shuffleboard tabs and widgets by pulling their fields from the provided classes
     */
    public static void init(){
        ShuffleboardTab subsystemIOTab;// = Shuffleboard.getTab("SubsystemIO");
        entries = new HashMap<>();

        //add all .class values of the static classes above
        Class<?>[] subsystems = {Testing.class};
        
        for(Class<?> subsystem : subsystems){
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
