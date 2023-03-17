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
    @Override
    public boolean getAsBoolean() {
        if (condition.getAsBoolean()) {
            boolean didItChange = false;
            T currentValue = valueGetter.get();
            try {
                didItChange = currentValue.equals(oldValue);
            } catch (NullPointerException ignored) {}
            oldValue = currentValue;
            return didItChange;
        } else 
            return false;
    }
}
