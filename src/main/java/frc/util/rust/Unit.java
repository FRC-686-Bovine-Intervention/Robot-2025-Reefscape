package frc.util.rust;

public class Unit {
    private Unit() {}

    public static final Unit unit = new Unit();

    @Override
    public String toString() {
        return "()";
    }
}
