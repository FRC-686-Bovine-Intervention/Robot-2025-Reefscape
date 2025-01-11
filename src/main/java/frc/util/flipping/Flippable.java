package frc.util.flipping;

import frc.util.flipping.AllianceFlipUtil.FieldFlipType;

public interface Flippable<T> {
    public T flip(FieldFlipType flipType);
    public default T flip() {
        return flip(AllianceFlipUtil.defaultFlipType);
    }
}
