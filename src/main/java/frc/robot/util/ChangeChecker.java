package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ChangeChecker<T> implements BooleanSupplier{
    private final Supplier<T> valueGetter;
    private T oldValue;
    private final BooleanSupplier condition;

    public ChangeChecker(Supplier<T> valueSupplier, T initialValue, BooleanSupplier conditionForChecking) {
        valueGetter = valueSupplier;
        oldValue = initialValue;
        condition = conditionForChecking;
    }

    public ChangeChecker(Supplier<T> valueSupplier) {
        this.valueGetter = valueSupplier;
        this.condition = () -> true;
        this.oldValue = null;
    }

    @Override
    public boolean getAsBoolean() {
        if (condition.getAsBoolean()) { // only eval if the condition is true
            boolean didItChange = false;
            T currentValue = valueGetter.get();
            if (currentValue != null)
                didItChange = !currentValue.equals(oldValue);
            else if (oldValue != null) // current value is null and old value is not null
                didItChange = true;
            oldValue = currentValue;
            return didItChange;
        } else 
            return false;
    }
}
