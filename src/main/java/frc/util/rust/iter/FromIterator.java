package frc.util.rust.iter;

import java.util.ArrayList;

@FunctionalInterface
public interface FromIterator<Item, DataStruct> {
    public DataStruct from_iter(Iterator<Item> iter);

    public static <Item> ArrayList<Item> arrayList(Iterator<Item> iter) {
        var buffer = new ArrayList<Item>();
        iter.for_each(buffer::add);
        return buffer;
    }
}
