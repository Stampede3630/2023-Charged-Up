package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class SendableChooserEnum<V> extends SendableChooserWrapper<V> {
    public SendableChooserEnum(Class<V> enumType, V defaultOption) {
        V[] values = enumType.getEnumConstants();
        for(V value: values) {
            addOption(value.toString(), value);
        }
        setDefaultOption(defaultOption.toString(), defaultOption);
    }

    public SendableChooserEnum(Class<V> enumType) {
        V[] values = enumType.getEnumConstants();
        for(V value: values) {
            addOption(value.toString(), value);
        }
    }
}
