package frc.util.rust.iter;

import frc.util.rust.Option;

public interface IntoIterator<Item> extends Iterable<Item> {
    public Iterator<Item> into_iter();

    @Override
    public default java.util.Iterator<Item> iterator() {
        var iter = this.into_iter();
        return new java.util.Iterator<Item>() {
            private final Option<Option<Item>> cachedItem = Option.none();
            public boolean hasNext() {
                var item = cachedItem.unwrap_or_else(iter::next);
                cachedItem.mut_replace(item);
                return item.is_some();
            }
            public Item next() {
                return Option.flatten(cachedItem.mut_take()).or_else(iter::next).unwrap();
            }
        };
    }
}
