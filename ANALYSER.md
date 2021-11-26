# Displaying monitor information

The monitor project enables the study of running realtime systems with ways of
triggering on infrequent fault conditions. Up to now it has required a logic
analyser or a scope (preferably with more than two channels) to display the
results.

This alternative back-end uses a Pico and an inexpensive display to show up to
eight data channels. Component cost should be under $20. This solution has no
benefits compared to a logic analyser with pre-trigger data capture: it is a
low-cost alternative.

![Image](./images/la_hw.jpg)

The device under test runs `monitor.py` as per the docs.

Key features are:
 1. Flexible triggering options allow detection of rare events in a realtime
 system including slow running code or CPU hogging.
 2. Substantial data capture pre- and post-tigger enabling the circumstances
 leading to the trigger event to be studied.
 3. Timing measurements may be made using a pair of cursors.
 4. Data may be sideways-scrolled using the encoder.
 5. Zoom in and out buttons.
 
Limitations:
 1. No realtime display: the device captures a set of samples which may then be
 displayed.

User interface:  
![Image](./images/la_ui.jpg)

This shows a capture of data from a realtime system.

# Request for feedback

This project is almost complete, however it is undocumented. I'm unsure whether
there is any demand for this solution: perhaps everyone doing serious realtime
work already has a quality LA. Perhaps it might have application in education?

If the project is of interest please let me know.
