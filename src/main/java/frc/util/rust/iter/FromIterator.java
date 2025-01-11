package frc.util.rust.iter;

import java.util.ArrayList;
import java.util.function.IntFunction;

@FunctionalInterface
public interface FromIterator<Item, DataStruct> {
    public DataStruct from_iter(Iterator<Item> iter);

    public static <Item> Item[] array(Iterator<Item> iter, IntFunction<Item[]> generator) {
        var buffer = generator.apply(iter.size_hint().upperBound.unwrap_or(iter.size_hint().lowerBound));
        for (int i = 0; i < buffer.length; i++) {
            buffer[i] = iter.next().unwrap();
        }
        return buffer;
    }
    public static <Item> ArrayList<Item> arrayList(Iterator<Item> iter) {
        var buffer = new ArrayList<Item>(iter.size_hint().upperBound.unwrap_or(iter.size_hint().lowerBound));
        iter.for_each(buffer::add);
        return buffer;
    }
}
