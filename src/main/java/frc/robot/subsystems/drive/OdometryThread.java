package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.Timer;

public class OdometryThread extends Thread {
    private static OdometryThread instance;
    public static OdometryThread getInstance() {if (instance == null) {instance = new OdometryThread();} return instance;}

    private OdometryThread() {
        this.setName("OdometryThread");
        this.setDaemon(true);
    }

    private final Lock signalsLock = new ReentrantLock();
    public final Lock odometryLock = new ReentrantLock();
    private final List<Signal<?>> signals = new ArrayList<>(12);
    private final List<Queue<Double>> timestampQueues = new ArrayList<>(1);

    public <T> Queue<T> registerPhoenixSignal(StatusSignal<T> statusSignal) {
        var queue = new ArrayBlockingQueue<T>(20);
        this.signalsLock.lock();
        this.odometryLock.lock();
        try {
            this.signals.add(new PhoenixSignal<>(statusSignal, queue));
        } finally {
            this.signalsLock.unlock();
            this.odometryLock.unlock();
        }
        return queue;
    }
    public <T> Queue<T> registerGenericSignal(Supplier<T> supplier) {
        var queue = new ArrayBlockingQueue<T>(20);
        this.signalsLock.lock();
        this.odometryLock.lock();
        try {
            this.signals.add(new GenericSignal<>(supplier, queue));
        } finally {
            this.signalsLock.unlock();
            this.odometryLock.unlock();
        }
        return queue;
    }
    public Queue<Double> generateTimestampQueue() {
        var queue = new ArrayBlockingQueue<Double>(20);
        this.odometryLock.lock();
        try {
            this.timestampQueues.add(queue);
        } finally {
            this.odometryLock.unlock();
        }
        return queue;
    }

    private static interface Signal<T> {
        public void poll();
        public OptionalDouble getLatency();
    }
    private static record PhoenixSignal<T>(StatusSignal<T> statusSignal, Queue<T> queue) implements Signal<T> {
        @Override
        public void poll() {
            this.statusSignal.refresh();
            this.queue.offer(this.statusSignal.getValue());
        }

        @Override
        public OptionalDouble getLatency() {
            return OptionalDouble.of(this.statusSignal.getTimestamp().getLatency());
        }
    }
    private static record GenericSignal<T>(Supplier<T> supplier, Queue<T> queue) implements Signal<T> {
        @Override
        public void poll() {
            this.queue.offer(this.supplier.get());
        }

        @Override
        public OptionalDouble getLatency() {
            return OptionalDouble.empty();
        }
    }

    @Override
    public void start() {
        if (!this.timestampQueues.isEmpty()) {
            super.start();
        }
    }

    @Override
    public void run() {
        while (true) {
            this.signalsLock.lock();
            try {
                Thread.sleep((long) (1000.0 / DriveConstants.odometryLoopFrequencyHz));
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                this.signalsLock.unlock();
            }

            this.odometryLock.lock();
            try {
                var timestamp = Timer.getFPGATimestamp();
                var totalLatency = 0.0;
                var latencySources = 0;
                for (var signal : this.signals) {
                    signal.poll();
                    var latency = signal.getLatency();
                    if (latency.isEmpty()) {continue;}
                    totalLatency += latency.getAsDouble();
                    latencySources += 1;
                }
                if (latencySources > 0) {
                    timestamp -= totalLatency / latencySources;
                }
                for (var queue : this.timestampQueues) {
                    queue.offer(timestamp);
                }
            } finally {
                this.odometryLock.unlock();
            }
        }
    }
}
