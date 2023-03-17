package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class SendableChooserWrapper<V> extends SendableChooser<V> {
    V previousSelection = getSelected();
    public boolean didValueChange() {
        boolean didItChange = false;
        if (getSelected() != null)
            didItChange = !getSelected().equals(previousSelection);
        else if (previousSelection != null) // current value is null and old value is not null
            didItChange = true;
        previousSelection = getSelected();
        return didItChange;
    }
}
