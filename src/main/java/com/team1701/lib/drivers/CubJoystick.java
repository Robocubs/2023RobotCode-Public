package com.team1701.lib.drivers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import com.team1701.frc2023.Constants;
import com.team1701.lib.util.Callback;
import com.team1701.lib.util.DeadZone;
import com.team1701.lib.util.LatchedBoolean;
import edu.wpi.first.wpilibj.Joystick;

public class CubJoystick extends Joystick {
    private static final int kMaxJoystickAxes = 12;
    private Map<Integer, List<Callback>> mButtonPressedHandlers = new HashMap<Integer, List<Callback>>();
    private Map<Integer, List<Callback>> mButtonReleasedHandlers = new HashMap<Integer, List<Callback>>();
    private Map<Integer, List<Callback>> mAxisPressedHandlers = new HashMap<Integer, List<Callback>>();
    private Map<Integer, List<Callback>> mAxisReleasedHandlers = new HashMap<Integer, List<Callback>>();
    private Map<Integer, LatchedBoolean> mAxisPressedLatches;
    private Map<Integer, LatchedBoolean> mAxisReleasedLatches;
    private Map<Integer, DeadZone> mDeadZones;

    public CubJoystick(int port) {
        super(port);

        mAxisPressedLatches = IntStream.range(0, kMaxJoystickAxes)
                .boxed()
                .collect(Collectors.toMap(axis -> axis, axis -> new LatchedBoolean()));
        mAxisReleasedLatches = IntStream.range(0, kMaxJoystickAxes)
                .boxed()
                .collect(Collectors.toMap(axis -> axis, axis -> new LatchedBoolean()));

        var defaultDeadZone = new DeadZone(Constants.kDefaultDeadZone);
        mDeadZones = IntStream.range(0, kMaxJoystickAxes)
                .boxed()
                .collect(Collectors.toMap(axis -> axis, axis -> defaultDeadZone));
    }

    public double getXWithDeadZone() {
        return getRawAxisWithDeadZone(getXChannel());
    }

    public double getYWithDeadZone() {
        return getRawAxisWithDeadZone(getYChannel());
    }

    public double getZWithDeadZone() {
        return getRawAxisWithDeadZone(getZChannel());
    }

    public double getRawAxisWithDeadZone(int axis) {
        var deadZone = mDeadZones.get(axis);
        return deadZone.apply(getRawAxis(axis));
    }

    public void setXDeadZone(DeadZone deadZone) {
        setDeadZone(getXChannel(), deadZone);
    }

    public void setYDeadZone(DeadZone deadZone) {
        setDeadZone(getYChannel(), deadZone);
    }

    public void setZDeadZone(DeadZone deadZone) {
        setDeadZone(getZChannel(), deadZone);
    }

    public void setDeadZone(int axis, DeadZone deadZone) {
        mDeadZones.put(axis, deadZone);
    }

    public void onButtonPressed(int button, Callback handler) {
        mButtonPressedHandlers.putIfAbsent(button, new ArrayList<Callback>());
        mButtonPressedHandlers.get(button).add(handler);
    }

    public void onButtonReleased(int button, Callback handler) {
        mButtonReleasedHandlers.putIfAbsent(button, new ArrayList<Callback>());
        mButtonReleasedHandlers.get(button).add(handler);
    }

    public void onXPressed(Callback handler) {
        onAxisPressed(getXChannel(), handler);
    }

    public void onXReleased(Callback handler) {
        onAxisReleased(getXChannel(), handler);
    }

    public void onYPressed(Callback handler) {
        onAxisPressed(getYChannel(), handler);
    }

    public void onYReleased(Callback handler) {
        onAxisReleased(getYChannel(), handler);
    }

    public void onZPressed(Callback handler) {
        onAxisPressed(getZChannel(), handler);
    }

    public void onZReleased(Callback handler) {
        onAxisReleased(getZChannel(), handler);
    }

    public void onAxisPressed(int axis, Callback handler) {
        mAxisPressedHandlers.putIfAbsent(axis, new ArrayList<Callback>());
        mAxisPressedHandlers.get(axis).add(handler);
    }

    public void onAxisReleased(int axis, Callback handler) {
        mAxisReleasedHandlers.putIfAbsent(axis, new ArrayList<Callback>());
        mAxisReleasedHandlers.get(axis).add(handler);
    }

    private boolean getRawAxisPressed(int axis) {
        var value = getRawAxisWithDeadZone(axis);
        var latch = mAxisPressedLatches.get(axis);
        return latch.update(value != 0);
    }

    private boolean getRawAxisReleased(int axis) {
        var value = getRawAxisWithDeadZone(axis);
        var latch = mAxisReleasedLatches.get(axis);
        return latch.update(value == 0);
    }

    public void invokeHandlers() {
        invokeHandlersMatchingFilter(mButtonPressedHandlers, this::getRawButtonPressed);
        invokeHandlersMatchingFilter(mButtonReleasedHandlers, this::getRawButtonReleased);
        invokeHandlersMatchingFilter(mAxisPressedHandlers, this::getRawAxisPressed);
        invokeHandlersMatchingFilter(mAxisReleasedHandlers, this::getRawAxisReleased);
    }

    private void invokeHandlersMatchingFilter(Map<Integer, List<Callback>> handlers, Predicate<Integer> predicate) {
        handlers.entrySet().stream()
                .filter(entry -> predicate.test(entry.getKey()))
                .flatMap(entry -> entry.getValue().stream())
                .forEach(Callback::invoke);
    }

    public void resetHandlers() {
        mButtonPressedHandlers.keySet().forEach(this::getRawButtonPressed);
        mButtonReleasedHandlers.keySet().forEach(this::getRawButtonReleased);
        mAxisPressedHandlers.keySet().forEach(this::getRawAxisPressed);
        mAxisReleasedHandlers.keySet().forEach(this::getRawAxisReleased);
    }
}
