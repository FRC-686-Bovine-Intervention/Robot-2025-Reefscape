package frc.util.rust.iter;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.IntFunction;
import java.util.function.Predicate;

import edu.wpi.first.math.Pair;
import frc.util.rust.Option;

public interface Iterator<Item> extends IntoIterator<Item> {
    public Option<Item> next();
    public SizeHint size_hint();
    public static class SizeHint {
        public int lowerBound;
        public Option<Integer> upperBound;
        private SizeHint(int lowerBound, Option<Integer> upperBound) {
            this.lowerBound = lowerBound;
            this.upperBound = upperBound;
        }
        public static SizeHint exact(int bound) {
            return new SizeHint(
                bound,
                Option.some(bound)
            );
        }
        public static SizeHint unknown() {
            return new SizeHint(
                0,
                Option.none()
            );
        }
        public SizeHint withLowerBound(int lowerBound) {
            return new SizeHint(
                lowerBound,
                this.upperBound.map((up) -> Math.max(lowerBound, up))
            );
        }
        public SizeHint withUpperBound(int upperBound) {
            return new SizeHint(
                Math.min(this.lowerBound, upperBound),
                Option.some(upperBound)
            );
        }

        public void dec() {
            this.lowerBound -= 1;
            this.upperBound.mut_replace(this.upperBound.map((up) -> up - 1));
        }

        public SizeHint add(SizeHint other) {
            return new SizeHint(
                this.lowerBound + other.lowerBound,
                Option.if_then(this.upperBound.is_some() && other.upperBound.is_some(), () -> this.upperBound.unwrap() + other.upperBound.unwrap())
            );
        }
        public SizeHint sub(SizeHint other) {
            return new SizeHint(
                this.lowerBound - other.lowerBound,
                Option.if_then(this.upperBound.is_some() && other.upperBound.is_some(), () -> this.upperBound.unwrap() - other.upperBound.unwrap())
            );
        }
        public SizeHint min(SizeHint other) {
            Option<Integer> upper;
            if (this.upperBound.is_some() && other.upperBound.is_some()) {
                upper = Option.some(Math.min(this.upperBound.unwrap(), other.upperBound.unwrap()));
            } else if (this.upperBound.is_some()) {
                upper = this.upperBound;
            } else if (other.upperBound.is_some()) {
                upper = other.upperBound;
            } else {
                upper = Option.none();
            }
            return new SizeHint(
                Math.min(this.lowerBound, other.lowerBound),
                upper
            );
        }
    }
    @Override
    public default Iterator<Item> into_iter() {return this;}

    // Constructors
    public static <Item> DoubleEndedIterator<Item> empty() {
        return new DoubleEndedIterator<Item>() {
            @Override
            public Option<Item> next() {
                return Option.none();
            }

            @Override
            public Option<Item> next_back() {
                return Option.none();
            }

            private final SizeHint sizeHint = SizeHint.exact(0);
            @Override
            public SizeHint size_hint() {
                return sizeHint;
            }
        };
    }
    public static <Item> DoubleEndedIterator<Item> once(Item once) {
        return new DoubleEndedIterator<Item>() {
            private boolean consumed = false;
            @Override
            public Option<Item> next() {
                if (consumed) {
                    return Option.none();
                }
                sizeHint.dec();
                consumed = true;
                return Option.some(once);
            }
            @Override
            public Option<Item> next_back() {
                return next();
            }
            private final SizeHint sizeHint = SizeHint.exact(1);
            @Override
            public SizeHint size_hint() {
                return sizeHint;
            }
        };
    }
    public static <Item> DoubleEndedIterator<Item> of(Item[] array) {
        return new DoubleEndedIterator<Item>() {
            private int front_index = 0;
            private int back_index = array.length;
            @Override
            public Option<Item> next() {
                return Option.if_then(front_index < back_index, () -> array[front_index++]).inspect((item) -> sizeHint.dec());
            }
            @Override
            public Option<Item> next_back() {
                return Option.if_then(front_index < back_index, () -> array[--back_index]).inspect((item) -> sizeHint.dec());
            }
            private final SizeHint sizeHint = SizeHint.exact(array.length);
            @Override
            public SizeHint size_hint() {
                return sizeHint;
            }
        };
    }
    public static <Item> DoubleEndedIterator<Item> of(List<Item> list) {
        return new DoubleEndedIterator<Item>() {
            private int front_index = 0;
            private int back_index = list.size();
            @Override
            public Option<Item> next() {
                return Option.if_then(front_index <= back_index, () -> list.get(front_index++)).inspect((item) -> sizeHint.dec());
            }
            @Override
            public Option<Item> next_back() {
                return Option.if_then(front_index <= back_index, () -> list.get(--back_index)).inspect((item) -> sizeHint.dec());
            }
            private final SizeHint sizeHint = SizeHint.exact(list.size());
            @Override
            public SizeHint size_hint() {
                return sizeHint;
            }
        };
    }
    public static <Item> Iterator<Item> of(Iterable<Item> iter) {
        return new Iterator<Item>() {
            private final java.util.Iterator<Item> iterator = iter.iterator();
            @Override
            public Option<Item> next() {
                return Option.if_then(iterator.hasNext(), iterator::next);
            }
            private final SizeHint sizeHint = SizeHint.unknown();
            @Override
            public SizeHint size_hint() {
                return sizeHint;
            }
        };
    }

    // Adapters
    public default Iterator<Item> inspect(Consumer<Item> inspect_function) {
        var parent = this;
        return new Iterator<Item>() {
            @Override
            public Option<Item> next() {
                return parent.next().inspect(inspect_function);
            }
            @Override
            public SizeHint size_hint() {
                return parent.size_hint();
            }
        };
    }
    public default <U> Iterator<U> map(Function<Item, U> map_function) {
        var parent = this;
        return new Iterator<U>() {
            @Override
            public Option<U> next() {
                return parent.next().map(map_function);
            }
            @Override
            public SizeHint size_hint() {
                return parent.size_hint();
            }
        };
    }
    public default Iterator<Item> filter(Predicate<Item> predicate) {
        var parent = this;
        return new Iterator<Item>() {
            @Override
            public Option<Item> next() {
                for (var item : parent) {
                    if(predicate.test(item)) {
                        return Option.some(item);
                    }
                }
                return Option.none();
            }
            @Override
            public SizeHint size_hint() {
                return parent.size_hint().withLowerBound(0);
            }
        };
    }
    public default <U> Iterator<U> filter_map(Function<Item, Option<U>> filter_map_function) {
        var parent = this;
        return new Iterator<U>() {
            @Override
            public Option<U> next() {
                for (var item : parent) {
                    var test = filter_map_function.apply(item);
                    if (test.is_some()) {
                        return test;
                    }
                }
                return Option.none();
            }
            @Override
            public SizeHint size_hint() {
                return parent.size_hint().withLowerBound(0);
            }
        };
    }
    public default Iterator<Item> chain(IntoIterator<Item> other) {
        var parent = this;
        var other_iter = other.into_iter();
        return new Iterator<Item>() {
            @Override
            public Option<Item> next() {
                return parent.next().or_else(other_iter::next);
            }
            @Override
            public SizeHint size_hint() {
                return parent.size_hint().add(other_iter.size_hint());
            }
        };
    }
    public default <U> Iterator<Pair<Item, U>> zip(IntoIterator<U> other) {
        var parent = this;
        var other_iter = other.into_iter();
        return new Iterator<Pair<Item, U>>() {
            @Override
            public Option<Pair<Item, U>> next() {
                var a = parent.next();
                if (a.is_none()) {
                    return Option.none();
                }
                var b = other_iter.next();
                if (b.is_none()) {
                    return Option.none();
                }
                return Option.some(Pair.of(a.unwrap(), b.unwrap()));
            }
            @Override
            public SizeHint size_hint() {
                return parent.size_hint().min(other_iter.size_hint());
            }
        };
    }
    public default Iterator<Pair<Integer, Item>> enumerate() {
        var parent = this;
        return new Iterator<Pair<Integer,Item>>() {
            private int current_index = 0;
            @Override
            public Option<Pair<Integer, Item>> next() {
                return parent.next().map((ele) -> Pair.of(current_index++, ele));
            }
            @Override
            public SizeHint size_hint() {
                return parent.size_hint();
            }
        };
    }
    public default Iterator<Item> take(int n) {
        var parent = this;
        return new Iterator<Item>() {
            private int current_index = 0;
            @Override
            public Option<Item> next() {
                if (current_index >= n) return Option.none();
                current_index++;
                return parent.next();
            }
            @Override
            public SizeHint size_hint() {
                return parent.size_hint().min(SizeHint.exact(n));
            }
        };
    }
    public default Iterator<Item> skip(int n) {
        var parent = this;
        return new Iterator<Item>() {
            private int current_index = 0;
            @Override
            public Option<Item> next() {
                for (; current_index < n; current_index++) {
                    parent.next();
                }
                return parent.next();
            }
            @Override
            public SizeHint size_hint() {
                return parent.size_hint().sub(SizeHint.exact(n));
            }
        };
    }
    public default Iterator<Item> take_while(Predicate<Item> predicate) {
        var parent = this;
        return new Iterator<Item>() {
            private boolean taking = true;
            @Override
            public Option<Item> next() {
                if (!taking) {
                    return Option.none();
                }
                var item = parent.next();
                taking = item.is_some_and(predicate);
                if (!taking) {
                    return Option.none();
                }
                return item;
            }
            @Override
            public SizeHint size_hint() {
                return parent.size_hint().withLowerBound(0);
            }
        };
    }
    public default Iterator<Item> skip_while(Predicate<Item> predicate) {
        var parent = this;
        return new Iterator<Item>() {
            private boolean skipping = true;
            @Override
            public Option<Item> next() {
                var item = parent.next();
                while (skipping) {
                    skipping = item.is_some_and(predicate);
                    if (!skipping) {
                        break;
                    }
                    item = parent.next();
                }
                return item;
            }
            @Override
            public SizeHint size_hint() {
                return parent.size_hint().withLowerBound(0);
            }
        };
    }
    public default <U> Iterator<U> map_while(Function<Item, Option<U>> predicate) {
        var parent = this;
        return new Iterator<U>() {
            private boolean mapping = true;
            @Override
            public Option<U> next() {
                var item = parent.next().and_then(predicate);
                while (mapping) {
                    mapping = item.is_some();
                    if (!mapping) {
                        break;
                    }
                    item = parent.next().and_then(predicate);
                }
                return item;
            }
            @Override
            public SizeHint size_hint() {
                return parent.size_hint().withLowerBound(0);
            }
        };
    }
    public static <I> Iterator<I> flatten(Iterator<Iterator<I>> fat) {
        return new Iterator<I>() {
            private Option<Iterator<I>> current_iterator;
            @Override
            public Option<I> next() {
                if (current_iterator == null) {
                    current_iterator = fat.next();
                }
                if (current_iterator.is_none()) {
                    return Option.none();
                }
                var item = current_iterator.unwrap().next();
                if (item.is_none()) {
                    current_iterator = fat.next();
                    return next();
                }
                return item;
            }
            @Override
            public SizeHint size_hint() {
                return SizeHint.unknown();
            }
        };
    }
    public default <U> Iterator<U> flat_map(Function<Item, IntoIterator<U>> map) {
        var parent = this;
        return new Iterator<U>() {
            private Option<Iterator<U>> current_iterator;
            @Override
            public Option<U> next() {
                if (current_iterator == null) {
                    current_iterator = parent.next().map(map).map((iter) -> iter.into_iter());
                }
                if (current_iterator.is_none()) {
                    return Option.none();
                }
                var item = current_iterator.unwrap().next();
                if (item.is_none()) {
                    current_iterator = parent.next().map(map).map((iter) -> iter.into_iter());
                    return next();
                }
                return item;
            }
            @Override
            public SizeHint size_hint() {
                return SizeHint.unknown();
            }
        };
    }

    // Terminators
    public default void for_each(Consumer<Item> for_each) {
        for(var ele = next(); ele.is_some(); ele = next()) {
            for_each.accept(ele.unwrap());
        }
    }
    
    public default <DataStruct> DataStruct collect(FromIterator<Item, DataStruct> collector) {
        return collector.from_iter(this);
    }
    public default Item[] collect_array(IntFunction<Item[]> generator) {
        return FromIterator.array(this, generator);
    }
    public default ArrayList<Item> collect_arraylist() {
        return FromIterator.arrayList(this);
    }
    
    @SuppressWarnings("unused")
    public default int count() {
        var count = 0;
        for (var item : this) {
            count++;
        }
        return count;
    }
    public default Option<Item> reduce(BiFunction<Item, Item, Item> reduction_function) {
        return this.next().map((item) -> fold(item, reduction_function));
    }
    public default Item fold(Item initial, BiFunction<Item, Item, Item> folding_function) {
        Item accumulator = initial;
        for (var item : this) {
            accumulator = folding_function.apply(accumulator, item);
        }
        return accumulator;
    }
    public default Option<Item> find(Predicate<Item> predicate) {
        for (var item : this) {
            if (predicate.test(item)) {
                return Option.some(item);
            }
        }
        return Option.none();
    }
    public default Option<Integer> position(Predicate<Item> predicate) {
        var index = 0;
        for (var item : this) {
            if (predicate.test(item)) {
                return Option.some(index);
            }
            index++;
        }
        return Option.none();
    }
    public default boolean any(Predicate<Item> predicate) {
        for (var item : this) {
            if (predicate.test(item)) {
                return true;
            }
        }
        return false;
    }
    public default boolean all(Predicate<Item> predicate) {
        for (var item : this) {
            if (!predicate.test(item)) {
                return false;
            }
        }
        return true;
    }

    public static interface DoubleEndedIterator<Item> extends Iterator<Item> {
        public Option<Item> next_back();

        public default DoubleEndedIterator<Item> rev() {
            var parent = this;
            return new DoubleEndedIterator<Item>() {
                @Override
                public Option<Item> next() {
                    return parent.next_back();
                }

                @Override
                public Option<Item> next_back() {
                    return parent.next();
                }

                @Override
                public SizeHint size_hint() {
                    return parent.size_hint();
                }
            };
        }
    }
}
