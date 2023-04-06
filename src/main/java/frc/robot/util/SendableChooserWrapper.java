package frc.robot.util;

public class SendableChooserWrapper<V> extends SettableSendableChooser<V> {
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
