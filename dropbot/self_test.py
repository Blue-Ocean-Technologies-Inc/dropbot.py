import datetime as dt
import io
import logging
import time

import numpy as np

from .hardware_test import (ALL_TESTS, system_info, test_i2c, test_voltage,
                            test_shorts, test_on_board_feedback_calibration,
                            test_channels)

logger = logging.getLogger(name=__name__)

__all__ = ['format_system_info_results', 'format_test_i2c_results',
           'format_test_voltage_results', 'format_test_shorts_results',
           'format_test_on_board_feedback_calibration_results',
           'format_test_channels_results']


def format_system_info_results(info):
    output = io.BytesIO()

    print >> output, '\nControl board:\n' + '-' * 80

    print >> output, '\n  uuid:', info['control board']['uuid']
    print >> output, '\n  Properties:'
    for k, v in info['control board']['properties'].items():
        print >> output, '    %s: %s' % (k, v)
    print >> output, '\n  Config:'
    for k, v in info['control board']['config'].items():
        print >> output, '    %s: %s' % (k, v)

    print >> output, '\nsoft i2c scan:', info['soft_i2c_scan']
    print >> output, 'number_of_channels:', info['number_of_channels']

    return output.getvalue()


def format_test_i2c_results(results):
    output = io.BytesIO()

    print >> output, '\ni2c scan:\n' + '-' * 80

    for address in sorted(results['i2c_scan']):
        data = results['i2c_scan'][address]
        info_string = ''
        if 'name' in data.keys():
            info_string = data['name']
        if 'hardware_version' in data.keys():
            info_string += " v%s" % data['hardware_version']
        if 'software_version' in data.keys():
            info_string += ", firmware: v%s" % data['software_version']
        if 'uuid' in data.keys():
            info_string += ", uuid: %s" % data['uuid']

        print >> output, '  %s: %s' % (address, info_string)

    return output.getvalue()


def format_test_voltage_results(results):
    output = io.BytesIO()

    print >> output, '\nTest voltage results:\n' + '-' * 80

    measured_voltage = np.array(results['measured_voltage'])
    target_voltage = np.array(results['target_voltage'])

    print >> output, '  target_voltage:', target_voltage
    print >> output, '  measured_voltage:', measured_voltage

    # calculate the average rms error
    r = measured_voltage - target_voltage
    print >> output, ('  rms_error = %.1f%%' %
                      (100 * np.sqrt(np.mean((r / target_voltage)**2))))

    # plt.figure()
    # # plot the measured vs target votage
    # plt.plot(target_voltage, measured_voltage, 'o')
    # plt.plot(target_voltage, target_voltage, 'k--')
    # plt.xlabel('Target voltage')
    # plt.ylabel('Measured voltage')

    return output.getvalue()


def format_test_on_board_feedback_calibration_results(results):
    output = io.BytesIO()

    c_measured = np.array(results['c_measured'])

    print >> output, ('\nTest on-board feedback calibration results:\n' + '-' *
                      80)

    print >> output, '  Measured capacitance:', c_measured

    # C_nominal = np.array([0, 10e-12, 100e-12, 470e-12])
    # plt.figure()
    # plt.plot(C_nominal * 1e12, c_measured * 1e12, 'o')
    # plt.plot(C_nominal * 1e12, C_nominal * 1e12, 'k--')
    # plt.xlabel('Nominal capacitance (pF)')
    # plt.ylabel('Measured capacitance (pF)')

    return output.getvalue()


def format_test_shorts_results(results):
    output = io.BytesIO()

    shorts = results['shorts']
    print >> output, '\nTest shorts results:\n' + '-' * 80
    if len(shorts):
        print >> output, "  Shorts on channels %s" % ", ".join([str(x) for x in
                                                                shorts])
    else:
        print >> output, "  No shorts"

    return output.getvalue()


def format_test_channels_results(results):
    output = io.BytesIO()

    print >> output, '\nTest channels results:\n' + '-' * 80

    c = np.array(results['c'])
    test_channels = np.array(results['test_channels'])
    shorts = results['shorts']
    n_channels = len(test_channels)
    n_reps = c.shape[1]

    if len(c) == 0:
        return

    nc = test_channels[np.min(c, 1) < 5e-12].tolist()
    for x in shorts:
        nc.remove(x)

    # plt.figure()
    # plt.bar(range(c.shape[0]), np.mean(c, 1) / 1e-12,
    # yerr=np.std(c, 1) / 1e-12)
    # plt.title('DropBot system: %s' % str(proxy.uuid)[-8:])
    # plt.xlabel("Channel");
    # plt.ylabel("Capacitance (pF)")

    # plt.figure()
    # plt.hist(c.flatten() / 1e-12, 20)
    # plt.ylabel("# of channels")
    # plt.xlabel("Capacitance (pF)")

    if len(shorts) or len(nc):
        print >> output, ("  The following channels failed (%d of %d / %.1f "
                          "%%):" % (len(shorts) + len(nc), n_channels,
                                    float(len(shorts) + len(nc)) / n_channels *
                                    100))
        if len(shorts):
            print >> output, ("    shorts (%d of %d / %.1f %%): %s" %
                              (len(shorts), n_channels,
                               float(len(shorts)) / n_channels * 100,
                               ", ".join([str(x) for x in shorts])))
        if len(nc):
            print >> output, ("    no connection (%d of %d / %.1f %%): %s" %
                              (len(nc), n_channels,
                               float(len(nc)) / n_channels * 100,
                               ", ".join([str(x) for x in nc])))
            if n_reps > 1:
                for x in nc:
                    n_fails = np.count_nonzero(c[x, :] < 5e-12)
                    print >> output, ("\n    Channel %d failed %d of %d reps "
                                      "(%.1f %%)" % (x, n_fails, n_reps, 100.0
                                                     * n_fails / n_reps))
    else:
        print >> output, "  All channels passed"

    return output.getvalue()


def self_test(proxy, tests=None):
    '''
    Perform quality control tests.

    Parameters
    ----------
    proxy : dropbot.SerialProxy
        DropBot control board reference.
    tests : list, optional
        List of names of test functions to run.

        By default, run all tests.

    Returns
    -------
    dict
        Results from all tests.
    '''
    total_time = 0

    if tests is None:
        tests = ALL_TESTS
    results = {}

    for test_name_i in tests:
        start_time_i = time.time()
        test_func_i = eval(test_name_i)
        results[test_name_i] = test_func_i(proxy)
        results[test_name_i]['utc_timestamp'] = (dt.datetime.utcnow()
                                                 .isoformat())
        duration_i = time.time() - start_time_i
        results[test_name_i]['test_duration'] = duration_i
        logger.info('%s: %.1f s', test_name_i, duration_i)
        total_time += duration_i

    logger.info('**Total time: %.1f s**', total_time)

    return results
