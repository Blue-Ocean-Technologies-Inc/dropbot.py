import numpy as np
import time


def high_voltage_source_rms_error(proxy, n=10):
    '''
    Test the measured voltage for a range of target voltages.

    Sweep target output voltage between minimum and maximum voltages and
    calculate error between target and measured output.

    Parameters
    ----------
    proxy : Proxy
    n : int
        Number of voltages to measure between minimum and maximum voltage.

    Returns
    -------
    float
        Average root mean squared error between target voltage and measured
        output voltage.
    '''
    # Save current state of high-voltage output.
    hv_output_enabled = proxy.hv_output_enabled
    original_voltage = proxy.voltage

    try:
        proxy.voltage = proxy.min_waveform_voltage
        proxy.hv_output_enabled = True
        measured_voltage = []

        voltage = np.linspace(proxy.min_waveform_voltage,
                              proxy.max_waveform_voltage, n)[1:-1]
        for v in voltage:
            proxy.voltage = v
            time.sleep(.5)
            measured_voltage.append(proxy.measured_voltage)

        measured_voltage = np.array(measured_voltage)
        proxy.voltage = proxy.min_waveform_voltage + 5

        # calculate the average rms error
        error = measured_voltage - voltage
        rms_error = np.sqrt(np.mean((error / voltage)**2))
        return rms_error
    finally:
        # Save current state of high-voltage output.
        proxy.hv_output_enabled = hv_output_enabled
        proxy.voltage = original_voltage


def detect_shorted_channels(proxy):
    '''
    Check for electrical channel shorts.

    Parameters
    ----------
    proxy : Proxy

    Returns
    -------

    '''
    # Save state of attributes that we will be modifying.
    start_attrs = {attr_i: getattr(proxy, attr_i)
                   for attr_i in ('hv_output_enabled', 'hv_output_selected',
                                  'state_of_channels')}

    shorts = []
    try:
        # Disable high-voltage output.
        proxy.hv_output_enabled = False
        # Select low-voltage (i.e., 3.3V) output.
        proxy.hv_output_selected = False

        # Actuate each channel one at a time and measure analog channel 0 to
        # detect a short.
        number_of_channels = proxy.number_of_channels
        for i in range(0, number_of_channels):
            state = np.zeros(number_of_channels)
            state[i] = 1
            proxy.state_of_channels = state
            time.sleep(0.05)
            short = proxy.analog_read(0) / 1024.0 * 3.3 < 1
            print "\rChecking for shorts. Channel %d: %s        " % (i, "SHORT"
                                                                     if short
                                                                     else
                                                                     "OK"),
            print ''
            if short:
                shorts.append(i)
        if shorts:
            print "Shorts on channels %s" % ", ".join(map(str, shorts))
        else:
            print "No shorts"
    finally:
        # Restore state of attributes that we were modified.
        for attr_i, value_i in start_attrs.iteritems():
            setattr(proxy, attr_i, value_i)
        if shorts:
            # Shorts were detected.  Disable all channels.
            proxy.state_of_channels = np.zeros(number_of_channels)
