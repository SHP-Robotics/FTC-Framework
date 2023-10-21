package org.firstinspires.ftc.teamcode.shplib.utility.filters;

import java.util.ArrayList;
import java.util.Collections;

public class MedianFilter implements Filter {
    private final ArrayList<Double> window;
    private final ArrayList<Double> store;
    private final double windowSize;

    public MedianFilter(int windowSize) {
        this.window = new ArrayList<>();
        this.store = new ArrayList<>();
        this.windowSize = windowSize;
    }

    public double calculate(double value) {
        // find insertion point for value
        int index = Collections.binarySearch(window, value);
        // handle error
        if (index < 0) index = Math.abs(index + 1);

        // add value to window
        window.add(index, value);

        // window size
        int currentSize = window.size();

        // if window size is greater than max, remove last value from window and store
        if (currentSize > windowSize) {
            window.remove(store.get(0));
            store.remove(0);
            --currentSize;
        }

        // add value to store
        store.add(value);

        // if window size is odd, return middle element
        // else window size is even, return average of middle elements
        if (currentSize % 2 != 0) return window.get(currentSize / 2);
        else return (window.get(currentSize / 2 - 1) + window.get(currentSize / 2)) / 2.0;
    }
}
