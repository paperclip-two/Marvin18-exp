package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private static LED INSTANCE;

    public static synchronized LED getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LED();
        }

        return INSTANCE;
    }

    private LED() {
        // Initialize the AddressableLED
        led = new AddressableLED(0);

        // Create a buffer to hold the LED color data
        ledBuffer = new AddressableLEDBuffer(82);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);

        // initialize the section map
        initMap();

        // set initial led state
        led.start();
    }

    // state map
    private Map<LEDSection, State> map = new HashMap<>();
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int counter = 3;
    private int initialCounter = 3;
    private double m_rainbowFirstPixelHue = 0;

    @Override
    public void periodic() {
        if (counter == 0) {
            // loop through each led section and update the leds based on the current state
            for (Entry<LEDSection, State> entry : map.entrySet()) {
                updateState(entry.getValue());
            }

            // apply the changes
            led.setData(ledBuffer);
            counter = initialCounter;
        } else {
            counter--;
        }
    }

    /**
     * This method will set a single led section to a single color
     * 
     * @param name  - The name of the led section in the map
     * @param color - The color to set the led section
     * @return
     */
    public Command setState(LEDSection name, LEDColor state) {
        return run(
                () -> {
                    map.get(name).setColor(state);
                });
    }

    /**
     * This method will set the flashing of an led section.
     * Color does NOT change
     * 
     * @param name     - The name of the led section in the map
     * @param flashing - boolean for turning on/off flashing
     * @return
     */
    public Command setState(LEDSection name, boolean flashing) {
        return run(
                () -> {
                    map.get(name).setFlashing(flashing);
                    map.get(name).setIterations(0);
                });
    }

    /**
     * This method will set the rolling of an led section.
     * Color does NOT change
     * 
     * @param name    - The name of the led section in the map
     * @param rolling - Rolling enumeration to set rolling
     * @return
     */
    public Command setState(LEDSection name, Rolling rolling) {
        return run(
                () -> {
                    map.get(name).setRolling(rolling);
                    map.get(name).setIterations(0);
                });
    }

    /**
     * This method will set the rolling of an led section
     * 
     * @param name    - The name of the led section in the map
     * @param color   - The new color of the led section
     * @param rolling - Rolling enumeration to set rolling
     * @return
     */
    public Command setState(LEDSection name, LEDColor color, boolean flashing) {
        return run(
                () -> {
                    map.get(name).setColor(color);
                    map.get(name).setFlashing(flashing);
                    map.get(name).setIterations(0);
                });
    }

    /**
     * This method will set the rolling of an led section
     * 
     * @param name    - The name of the led section in the map
     * @param color   - The new color of the led section
     * @param rolling - Rolling enumeration to set rolling
     * @return
     */
    public Command setState(LEDSection name, LEDColor color, Rolling rolling) {
        return run(
                () -> {
                    map.get(name).setColor(color);
                    map.get(name).setRolling(rolling);
                    map.get(name).setIterations(0);
                });
    }

    /**
     * This method will set multiple led section to a single color
     * 
     * @param name  - The names of the led sections in the map
     * @param color - The color to set the led section
     * @return
     */
    public Command setState(Set<LEDSection> names, LEDColor state) {
        return run(
                () -> {
                    for (LEDSection name : names) {
                        map.get(name).setColor(state);
                    }
                });
    }

    /**
     * This method will set the flashing of multiple led sections.
     * Color does NOT change
     * 
     * @param name     - The names of the led sections in the map
     * @param flashing - boolean for turning on/off flashing
     * @return
     */
    public Command setState(Set<LEDSection> names, boolean flashing) {
        return run(
                () -> {
                    for (LEDSection name : names) {
                        map.get(name).setFlashing(flashing);
                        map.get(name).setIterations(0);
                    }
                });
    }

    /**
     * This method will set the rolling of multiple led sections.
     * Color does NOT change
     * 
     * @param name    - The names of the led sections in the map
     * @param rolling - Rolling enumeration to set rolling
     * @return
     */
    public Command setState(Set<LEDSection> names, Rolling rolling) {
        return run(
                () -> {
                    for (LEDSection name : names) {
                        map.get(name).setRolling(rolling);
                        map.get(name).setIterations(0);
                    }
                });
    }

    /**
     * This method will set the rolling of multiple led sections
     * 
     * @param name    - The names of the led sections in the map
     * @param color   - The new color of the led section
     * @param rolling - Rolling enumeration to set rolling
     * @return
     */
    public Command setState(Set<LEDSection> names, LEDColor color, boolean flashing) {
        return run(
                () -> {
                    for (LEDSection name : names) {
                        map.get(name).setColor(color);
                        map.get(name).setFlashing(flashing);
                        map.get(name).setIterations(0);
                    }
                });
    }

    /**
     * This method will set the rolling of multiple led sections
     * 
     * @param name    - The names of the led sections in the map
     * @param color   - The new color of the led section
     * @param rolling - Rolling enumeration to set rolling
     * @return
     */
    public Command setState(Set<LEDSection> names, LEDColor color, Rolling rolling) {
        return run(
                () -> {
                    for (LEDSection name : names) {
                        map.get(name).setColor(color);
                        map.get(name).setRolling(rolling);
                        map.get(name).setIterations(0);
                    }
                });
    }

    /**
     * This method will set one or more led sections to the mapped state
     * 
     * @param colorMap - A map of name to desired state
     * @return
     */
    public Command setState(Map<LEDSection, State> stateMap) {
        return run(
                () -> {
                    for (Entry<LEDSection, State> entry : stateMap.entrySet()) {
                        map.get(entry.getKey()).setColor(entry.getValue().getColor());
                        map.get(entry.getKey()).setFlashing(entry.getValue().isFlashing());
                        map.get(entry.getKey()).setRolling(entry.getValue().getRolling());
                        map.get(entry.getKey()).setIterations(0);
                        map.get(entry.getKey()).setForwardIndex(0);
                        map.get(entry.getKey()).setReverseIndex(map.get(entry.getKey()).getView().getLength() - 1);
                    }
                });
    }

    /**
     * This method is used to initialize the led section map. Add more here if
     * needed and known.
     */
    private void initMap() {
        map.put(LEDSection.CORALINTAKEUP, new State(new AddressableLEDBufferView(ledBuffer, 0, 13)));
        map.put(LEDSection.CORALINTAKEDOWN, new State(new AddressableLEDBufferView(ledBuffer, 14, 27)));
        map.put(LEDSection.LEFT45, new State(new AddressableLEDBufferView(ledBuffer, 28, 40)));
        map.put(LEDSection.RIGHT45, new State(new AddressableLEDBufferView(ledBuffer, 41, 53)));
        map.put(LEDSection.LEFTVERTICAL, new State(new AddressableLEDBufferView(ledBuffer, 54, 67)));
        map.put(LEDSection.INNERCARRIAGE, new State(new AddressableLEDBufferView(ledBuffer, 68, 81)));
    }

    private void updateState(State state) {
        LEDColor color = state.getColor();

        if (color == LEDColor.RAINBOW) {
            // this is done by hue saturation so cannot use RGB
            for (var i = 0; i < state.getView().getLength(); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
                final var hue = (m_rainbowFirstPixelHue + (i * 180 / state.getView().getLength())) % 180;
                // Set the value
                state.getView().setHSV(i, (int) hue, 255, 128);
            }
            // Increase by to make the rainbow "move"
            m_rainbowFirstPixelHue += 3;
            // Check bounds
            m_rainbowFirstPixelHue %= 180;
        } else {
            var R = 0;
            var G = 0;
            var B = 0;

            if (color == LEDColor.WHITE) {
                R = 255;
                G = 255;
                B = 255;
            } else if (color == LEDColor.RED) {
                R = 255;
                G = 0;
                B = 0;
            } else if (color == LEDColor.GREEN) {
                R = 0;
                G = 255;
                B = 0;
            } else if (color == LEDColor.BLUE) {
                R = 0;
                G = 0;
                B = 255;
            } else if (color == LEDColor.BLACK) {
                R = 0;
                G = 0;
                B = 0;
            }

            // if state is flashing cannot roll or set solid
            if (state.isFlashing()) {
                for (var i = 0; i < state.getView().getLength(); i++) {
                    // this will light up the lights every 10th loop
                    if (state.getIterations() % 3 == 0) {
                        state.getView().setRGB(i, 0, 0, 0);
                    } else {
                        state.getView().setRGB(i, R, G, B);
                    }
                }

                state.setIterations(state.getIterations() + 1);
            }

            // if the rolling state is set handle it
            else if (state.getRolling() != Rolling.OFF) {
                // have to handle both forward (ie. 0 -> 10) and reverse (ie. 10 -> 0)
                int indexToSet = 0;
                if (state.getRolling() == Rolling.FORWARD) {
                    indexToSet = state.getForwardIndex();
                } else {
                    indexToSet = state.getReverseIndex();
                }

                for (var i = 0; i < state.getView().getLength(); i++) {
                    if (state.getRolling() == Rolling.FORWARD && i <= indexToSet) {
                        state.getView().setRGB(i, R, G, B);
                    } else if (state.getRolling() == Rolling.REVERSE && i >= indexToSet) {
                        state.getView().setRGB(i, R, G, B);
                    } else {
                        state.getView().setRGB(i, 0, 0, 0);
                    }
                }

                if (state.getRolling() == Rolling.FORWARD) {
                    indexToSet++;
                    if (indexToSet > state.getView().getLength() - 1) {
                        indexToSet = 0;
                    }
                    state.setForwardIndex(indexToSet);
                } else if (state.getRolling() == Rolling.REVERSE) {
                    indexToSet--;
                    if (indexToSet < 0) {
                        indexToSet = state.getView().getLength() - 1;
                    }
                    state.setReverseIndex(indexToSet);
                }

            }

            // otherwise just set the color
            else {
                for (var i = 0; i < state.getView().getLength(); i++) {
                    // Sets the specified LED to the RGB values for red
                    state.getView().setRGB(i, R, G, B);
                }
            }
        }
    }

    // public definitions for led sections
    // This section should change when adding new led sections
    public static enum LEDSection {
        CORALINTAKEUP,
        CORALINTAKEDOWN,
        LEFT45,
        RIGHT45,
        LEFTVERTICAL,
        INNERCARRIAGE
    }

    // public definition for Color
    public static enum LEDColor {
        RED,
        BLUE,
        GREEN,
        WHITE,
        YELLOW,
        BLACK,
        RAINBOW,
    }

    // public definition for rolling direction
    public static enum Rolling {
        OFF,
        FORWARD,
        REVERSE
    }

    // public class to hold state information for each led section
    public static class State {
        AddressableLEDBufferView view;
        LEDColor color = LEDColor.RAINBOW;
        int iterations = 0;
        int forwardIndex = 0;
        int reverseIndex = 0;
        boolean flashing = false;
        Rolling rolling = Rolling.OFF;

        /**
         * DO NOT USE THIS CONSTRUCTOR OUTSIDE OF THE LED SUBSYSTEM
         * 
         * @param view
         */
        public State(AddressableLEDBufferView view) {
            this.view = view;
            reverseIndex = view.getLength() - 1;
        }

        public State(LEDColor color, boolean flashing) {
            this.color = color;
            this.flashing = flashing;
        }

        public State(LEDColor color, Rolling rolling) {
            this.color = color;
            this.rolling = rolling;
        }

        public State(LEDColor color) {
            this.color = color;
        }

        public void setColor(LEDColor color) {
            this.color = color;
        }

        public LEDColor getColor() {
            return color;
        }

        public AddressableLEDBufferView getView() {
            return view;
        }

        public void setFlashing(boolean flashing) {
            this.flashing = flashing;
        }

        public boolean isFlashing() {
            return flashing;
        }

        public void setRolling(Rolling rolling) {
            this.rolling = rolling;
        }

        public Rolling getRolling() {
            return rolling;
        }

        public int getIterations() {
            return iterations;
        }

        public void setIterations(int iterations) {
            this.iterations = iterations;
        }

        public int getForwardIndex() {
            return this.forwardIndex;
        }

        public void setForwardIndex(int index) {
            this.forwardIndex = index;
        }

        public int getReverseIndex() {
            return this.reverseIndex;
        }

        public void setReverseIndex(int index) {
            this.reverseIndex = index;
        }
    }
}
