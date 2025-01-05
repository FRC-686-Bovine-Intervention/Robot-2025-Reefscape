package frc.util.rust;

import java.util.Objects;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.Supplier;

import frc.util.rust.iter.IntoIterator;
import frc.util.rust.iter.Iterator.DoubleEndedIterator;

/**
 * A Java version of Rust's Option<T> enum <p>
 * {@code Option<T>} is isomorphic to {@code Optional<T>} and {@code Result<T, Unit>}<p>
 * {@code Option<Unit>} is isomorphic to {@code boolean}, {@code Optional<Unit>} and {@code Result<Unit, Unit>}
 */
public class Option<T> implements IntoIterator<T> {
    private T inner;

    private Option(T some) {
        this.inner = some;
    }

    /**
     * Wraps {@code some} in an {@code Option<T>}
     * @param <T> The type of {@code some}
     * @param some The value to wrap
     * @return The wrapper {@code Option} OR {@code None} if {@code some} is {@code null}
     */
    public static <T> Option<T> some(T some) {
        return new Option<T>(some);
    }
    /**
     * @param <T> The type of the (nonexistent) wrapped value
     * @return {@code None}
     */
    public static <T> Option<T> none() {
        return new Option<T>(null);
    }
    /**
     * Converts a {@code Optional<T>} into {@code Option<T>}
     * @param <T> The type of the possible wrapped value
     * @param optional 
     * @return The converted {@code Option}
     */
    public static <T> Option<T> from(Optional<T> optional) {
        return new Option<T>(optional.orElse(null));
    }
    /**
     * @param <T> The type of the possible wrapped value
     * @param cond The condition to check
     * @param some The value of the option if {@code cond} is {@code true}
     * @return If {@code cond} is true returns {@code Some(some)}, otherwise returns {@code None}
     */
    public static <T> Option<T> if_then_some(boolean cond, T some) {
        if(cond) {
            return some(some);
        } else {
            return none();
        }
    }
    /**
     * @param <T> The type of the possible wrapped value
     * @param cond The condition to check
     * @param some_supplier The supplier to poll if {@code cond} is {@code true}
     * @return If {@code cond} is true returns {@code Some(some_supplier.get())}, otherwise returns {@code None}
     */
    public static <T> Option<T> if_then(boolean cond, Supplier<T> some_supplier) {
        if(cond) {
            return some(some_supplier.get());
        } else {
            return none();
        }
    }

    public boolean is_some() {
        return inner != null;
    }
    public boolean is_none() {
        return inner == null;
    }

    public boolean is_some_and(Predicate<T> predicate) {
        return is_some() && predicate.test(inner);
    }

    /**
     * @return The wrapped value
     * @throws NullPointerException If {@code None}
     */
    public T unwrap() throws NullPointerException {
        if(is_none()) throw new NullPointerException("Unwrapped a None Option");
        return inner;
    }
    /**
     * @return The wrapped value, if {@code None} returns {@code null}
     */
    public T unwrap_to_nullable() {
        return inner;
    }
    /**
     * @return The wrapped value, if {@code None} returns {@code other}
     */
    public T unwrap_or(T other) {
        return is_some() ? inner : other;
    }
    /**
     * @return The wrapped value, if {@code None} returns {@code other.get()}
     */
    public T unwrap_or_else(Supplier<T> other) {
        return is_some() ? inner : other.get();
    }
    /**
     * Converts a {@code Option<T>} into {@code Optional<T>}
     * @return The converted {@code Optional}
     */
    public Optional<T> to_optional() {
        return Optional.ofNullable(inner);
    }

    /**
     * Maps the wrapped value using {@code map_function}
     * @param <U> The new type
     * @param map_function The function to use to map the wrapped value
     * @return An {@code Option<U>} containing the mapped value, or {@code None}
     */
    public <U> Option<U> map(Function<T, U> map_function) {
        return if_then(is_some(), () -> map_function.apply(inner));
    }

    /**
     * @param inspect_function Function to call if {@code Some}
     * @return {@code this}
     */
    public Option<T> inspect(Consumer<T> inspect_function) {
        if(is_some()) {
            inspect_function.accept(inner);
        }
        return this;
    }

    /**
     * Flattens {@code Option<Option<T>>} into {@code Option<T>}
     * @param <T> The type of the wrapped value
     * @param fat The {@code Option} to flatten
     * @return The flattened {@code Option}
     */
    public static <T> Option<T> flatten(Option<Option<T>> fat) {
        return fat.and_then((o) -> o);
    }

    /**
     * If {@code None} returns {@code None}, otherwise returns {@code other}
     * @param <U> The type of the wrapped value
     * @param other The {@code Option} to return if {@code Some}
     * @return {@code other} if {@code Some}
     */
    public <U> Option<U> and(Option<U> other) {
        return is_none() ? Option.none() : other;
    }
    /**
     * If {@code None} returns {@code None}, otherwise returns {@code other.get()}<p>
     * Also known as {@code flat_map}, as it is equivalent to {@code flatten(this.map(other))}
     * @param <U> The type of the wrapped value
     * @param other The function to poll and return if {@code Some}
     * @return {@code other.get()} if {@code Some}
     */
    public <U> Option<U> and_then(Function<T, Option<U>> other) {
        return is_none() ? Option.none() : other.apply(inner);
    }
    /**
     * If {@code None} returns {@code other}, otherwise returns {@code this}
     * @param other The {@code Option} to return if {@code None}
     * @return {@code other} if {@code None}, otherwise {@code this}
     */
    public Option<T> or(Option<T> other) {
        return is_some() ? this : other;
    }
    /**
     * If {@code None} returns {@code other.get()}, otherwise returns {@code this}
     * @param other The supplier to poll and return if {@code None}
     * @return {@code other.get()} if {@code None}, otherwise {@code this}
     */
    public Option<T> or_else(Supplier<Option<T>> other) {
        return is_some() ? this : other.get();
    }

    /**
     * Converts {@code this} into {@code Result<T, E>} using {@code err} as the error value if {@code None}
     * @param <E> The error type
     * @param err The error value to use if {@code None}
     * @return The converted {@code Result}, mapping {@code Some(some)} to {@code Result.ok(some)} and {@code None} to {@code Result.err(err)}
     */
    public <E> Result<T, E> ok_or(E err) {
        return is_some() ? Result.ok(inner) : Result.err(err);
    }
    /**
     * Converts a {@code Option<T>} into {@code Result<T, E>} using {@code err} as the error value if {@code None}
     * @param <E> The error type
     * @param err The supplier to poll for the error value if {@code None}
     * @return The converted {@code Result}, mapping {@code Some(some)} to {@code Result.ok(some)} and {@code None} to {@code Result.err(err.get())}
     */
    public <E> Result<T, E> ok_or_else(Supplier<E> err) {
        return is_some() ? Result.ok(inner) : Result.err(err.get());
    }

    public Option<T> mut_take() {
        var ret = Option.some(this.inner);
        this.inner = null;
        return ret;
    }
    public Option<T> mut_replace(T value) {
        var ret = this.clone();
        this.inner = value;
        return ret;
    }
    public Option<T> mut_replace(Option<T> value) {
        var ret = this.clone();
        this.inner = value.unwrap_to_nullable();
        return ret;
    }

    /**
     * Converts a {@code Option<T>} into an {@code Iterator<T>} that returns {@code this} once then {@code None} after
     * @return The converted {@code Iterator<T>}
     */
    @Override
    public DoubleEndedIterator<T> into_iter() {
        var opt = this;
        return new DoubleEndedIterator<T>() {
            private boolean consumed = false;
            @Override
            public Option<T> next() {
                if (consumed) {
                    return Option.none();
                }
                size_hint.dec();
                consumed = true;
                return opt;
            }
            @Override
            public Option<T> next_back() {
                return next();
            }
            private final SizeHint size_hint = SizeHint.exact(opt.is_some() ? 1 : 0);
            @Override
            public SizeHint size_hint() {
                return size_hint;
            }
        };
    }

    @Override
    public boolean equals(Object obj) {
        if(obj instanceof Option<?> other) {
            if(Objects.equals(inner, other.inner)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public String toString() {
        return is_some() ? String.format("Some(%s)", inner) : "None";
    }

    @Override
    public Option<T> clone() {
        return Option.some(inner);
    }
}
