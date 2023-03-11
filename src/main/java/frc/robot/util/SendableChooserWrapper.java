package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class SendableChooserWrapper<V> extends SendableChooser<V> {
    V previousSelection = getSelected();
    public boolean didValueChange() {
        boolean didItChange = false;
        try {
            didItChange = previousSelection.equals(getSelected());
        } catch (NullPointerException ignored) {

        }
        previousSelection = getSelected();
        return didItChange;
    }
}
