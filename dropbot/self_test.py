import datetime as dt
import logging
import subprocess as sp
import tempfile
import time

import jinja2
import matplotlib as mpl
import matplotlib.pyplot
import matplotlib.ticker
import numpy as np
import path_helpers as ph
import si_prefix as si

# Import test functions used by `self_test`.
from .hardware_test import (ALL_TESTS, system_info, test_i2c, test_voltage,
                            test_shorts, test_on_board_feedback_calibration,
                            test_channels)

logger = logging.getLogger(name=__name__)

__all__ = ['format_system_info_results', 'format_test_i2c_results',
           'format_test_voltage_results', 'format_test_shorts_results',
           'format_test_on_board_feedback_calibration_results',
           'format_test_channels_results']

CAPACITANCE_FORMATTER = mpl.ticker.FuncFormatter(lambda x, *args: '%sF' %
                                                 si.si_format(x))


def format_system_info_results(info):
    '''
    .. versionadded:: 1.28

    Parameters
    ----------
    info : dict
        Results from :func:`system_info`.

    Returns
    -------
    str
        Summary of :func:`system_info` results in Markdown format.
    '''
    template = jinja2.Template(r'''
# Control board (UUID: `{{ info['control board']['uuid'] }}`) #

## Properties ##
{% for name_i, property_i in info['control board']['properties'].iteritems() %}
 - **{{ name_i }}**: `{{ property_i }}`
{%- endfor %}

## Config ##
{% for name_i, value_i in info['control board']['config'].iteritems() %}
 - **{{ name_i }}**: `{{ value_i }}`
{%- endfor %}

{% for key_i in ('soft_i2c_scan', 'number_of_channels') -%}
**{{ key_i.replace('_', ' ') }}**: `{{ info[key_i] }}`{{ '  ' }}
{% endfor -%}'''.strip())

    return template.render(info=info).strip()


def format_test_i2c_results(results):
    '''
    .. versionadded:: 1.28

    Parameters
    ----------
    results : dict
        Results from :func:`test_i2c`.

    Returns
    -------
    str
        Summary of :func:`test_i2c` results in Markdown format.
    '''
    template = jinja2.Template(r'''
# I2C scan: #

{% if results['i2c_scan'] %}
{%- for address in results['i2c_scan']|sort %}
{%- set data = results['i2c_scan'][address] -%}
 - **`{{ address }}`**:
{%- if 'name' in data %} {{ data['name'] }}{% endif %}
{%- if 'hardware_version' in data %} v{{ data['hardware_version'] }}{% endif %}
{%- if 'software_version' in data %}, firmware: v{{ data['software_version'] }}
{%- endif %}
{%- if 'uuid' in data %}, uuid: `{{ data['uuid'] }}`{% endif -%}
{% endfor -%}
{% else -%}
No devices found on I2C bus.
{% endif -%}'''.strip())

    return template.render(results=results).strip()


def format_test_voltage_results(results, figure_path=None):
    '''
    .. versionadded:: 1.28

    Parameters
    ----------
    results : dict
        Results from :func:`test_voltage`.
    figure_path : str, optional
        If specified, include summary figure reference in text and write figure
        image to specified path.

        Filepath must have web browser compatible image extension (e.g.,
        ``.jpg``, ``.png``).

    Returns
    -------
    str
        Summary of :func:`test_voltage` results in Markdown format.

        If :data:`figure_path` was specified, summary figure is written to the
        specified path.
    '''
    measured_voltage = np.array(results['measured_voltage'])
    target_voltage = np.array(results['target_voltage'])
    # Calculate the average rms error
    error = measured_voltage - target_voltage
    rms_error = 100 * np.sqrt(np.mean((error / target_voltage)**2))

    if figure_path:
        figure_path = ph.path(figure_path).realpath()
        # Make parent directories if they don't exist.
        figure_path.parent.makedirs_p()
        axis = plot_test_voltage_results(results)
        fig = axis.get_figure()
        fig.tight_layout()
        fig.savefig(figure_path, bbox_inches='tight')

    template = jinja2.Template(r'''
# Test voltage results: #

 - **Target voltage**:   `{{ target_voltage }}`
 - **Measured voltage**: `{{ measured_voltage }}`
 - **Root-mean-squared (RMS) error**: {{ '{:.1f}'.format(rms_error) }}%
{%- if figure_path %}
   ![Measured vs target voltage]({{ figure_path }})
{%- endif %}
    '''.strip())

    return template.render(results=results, target_voltage=target_voltage,
                           measured_voltage=measured_voltage,
                           rms_error=rms_error,
                           figure_path=figure_path).strip()


def plot_test_voltage_results(results, axis=None):
    '''
    .. versionadded:: 1.28

    Plot summary of results from :func:`test_voltage`.

    Parameters
    ----------
    results : dict
        Results from :func:`test_voltage`.
    axis : matplotlib.axes._subplots.AxesSubplot, optional
        A `matplotlib` axis to plot to.

        If not set, axis is generated within a new `matplotlib` figure.

    Returns
    -------
    axis : matplotlib.axes._subplots.AxesSubplot
        A `matplotlib` axis plotting the measured voltage against the target
        voltage.
    '''
    if axis is None:
        fig, axis = mpl.pyplot.subplots(figsize=(3.5, 3.5))
        axis.set_aspect(True)

    # Plot the measured vs target voltage
    axis.plot(results['target_voltage'], results['measured_voltage'], 'o')
    axis.plot(results['target_voltage'], results['target_voltage'], 'k--')
    axis.set_xlabel('Target voltage')
    axis.set_ylabel('Measured voltage')

    return axis


def format_test_on_board_feedback_calibration_results(results,
                                                      figure_path=None):
    '''
    .. versionadded:: 1.28

    Parameters
    ----------
    results : dict
        Results from :func:`test_on_board_feedback_calibration`.
    figure_path : str, optional
        If specified, include summary figure reference in text and write figure
        image to specified path.

        Filepath must have web browser compatible image extension (e.g.,
        ``.jpg``, ``.png``).

    Returns
    -------
    str
        Summary of :func:`test_on_board_feedback_calibration` results in
        Markdown format.

        If :data:`figure_path` was specified, summary figure is written to the
        specified path.
    '''
    c_measured = np.array(results['c_measured'])

    if figure_path:
        figure_path = ph.path(figure_path).realpath()
        # Make parent directories if they don't exist.
        figure_path.parent.makedirs_p()
        axis = plot_test_on_board_feedback_calibration_results(results)
        fig = axis.get_figure()
        fig.tight_layout()
        fig.savefig(figure_path, bbox_inches='tight')

    template = jinja2.Template(r'''
# Test on-board feedback calibration results: #

 - **Measured capacitance**: `{{ c_measured }}`
{%- if figure_path %}
   ![On-board feedback calibration capacitors]({{ figure_path }})
{%- endif %}'''.strip())
    return template.render(results=results, c_measured=c_measured,
                           figure_path=figure_path)


def plot_test_on_board_feedback_calibration_results(results, axis=None):
    '''
    .. versionadded:: 1.28

    Plot summary of results from :func:`test_on_board_feedback_calibration`.

    Parameters
    ----------
    results : dict
        Results from :func:`test_on_board_feedback_calibration`.
    axis : matplotlib.axes._subplots.AxesSubplot, optional
        A `matplotlib` axis to plot to.

        If not set, axis is generated within a new `matplotlib` figure.

    Returns
    -------
    axis : matplotlib.axes._subplots.AxesSubplot
        A `matplotlib` axis plotting the measured on-board capacitance against
        the nominal values of the on-board feedback calibration capacitors.
    '''
    if axis is None:
        fig, axis = mpl.pyplot.subplots(figsize=(3, 3))
        axis.set_aspect(True)

    # TODO These nominal capacitance values should be added somewhere more
    # general (perhaps in `dropbot/__init__.py`)?
    C_nominal = np.array([0, 10e-12, 100e-12, 470e-12])

    axis.plot(C_nominal, results['c_measured'], 'o')
    axis.plot(C_nominal, C_nominal, 'k--')
    axis.set_xlabel('Nominal capacitance')
    axis.set_ylabel('Measured capacitance')
    # Use SI unit prefixes for axis capacitance tick labels.
    axis.xaxis.set_major_formatter(CAPACITANCE_FORMATTER)
    axis.yaxis.set_major_formatter(CAPACITANCE_FORMATTER)

    return axis


def format_test_shorts_results(results):
    '''
    .. versionadded:: 1.28

    Parameters
    ----------
    results : dict
        Results from :func:`test_shorts`.

    Returns
    -------
    str
        Summary of :func:`test_shorts` results in Markdown format.
    '''
    template = jinja2.Template(r'''
# Test shorts results: #
{% if results['shorts'] %}
 - Shorts on channels {{ results['shorts']|join(', ') }}
{%- else %}
 - No shorts.
{%- endif -%}'''.strip())

    return template.render(results=results)


def format_test_channels_results(results, figure_path=None):
    '''
    .. versionadded:: 1.28

    Parameters
    ----------
    results : dict
        Results from :func:`test_channels`.
    figure_path : str, optional
        If specified, include summary figure reference in text and write figure
        image to specified path.

        Filepath must have web browser compatible image extension (e.g.,
        ``.jpg``, ``.png``).

    Returns
    -------
    str
        Summary of :func:`test_channels` results in Markdown format.

        If :data:`figure_path` was specified, summary figure is written to the
        specified path.
    '''
    c = np.array(results['c'])
    test_channels_ = np.array(results['test_channels'])
    shorts = results['shorts']

    # Threshold for connected electrode.
    c_threshold = 5e-12

    n_channels = len(test_channels_)
    n_reps = c.shape[1] if len(c.shape) > 1 else 0

    if not c.tolist():
        no_connection = []
    else:
        no_connection = test_channels_[np.min(c, 1) < c_threshold].tolist()

    for x in shorts:
        try:
            no_connection.remove(x)
        except ValueError:
            pass

    if len(c) and figure_path:
        figure_path = ph.path(figure_path).realpath()
        # Make parent directories if they don't exist.
        figure_path.parent.makedirs_p()
        axes = plot_test_channels_results(results)
        fig = axes[0].get_figure()
        fig.tight_layout()
        fig.savefig(figure_path, bbox_inches='tight')

    context = dict(c=c, c_threshold=c_threshold, test_channels=test_channels_,
                   shorts=shorts, n_channels=n_channels, n_reps=n_reps,
                   no_connection=no_connection, figure_path=figure_path)

    template = jinja2.Template(r'''
# Test channels results: #
{% if c|length < 1 %}
No channels tested.
{% else %}{% if shorts|length or no_connection|length %}
{%- set bad_channels = shorts + no_connection %}
{%- set bad_channels_count = (bad_channels|length) %}
The following channels failed ({{ bad_channels_count }} of {{ n_channels }} / **{{ '{:.1f}'.format((bad_channels_count | float) / n_channels * 100) }}%**):
{% if shorts|length %}
 - **Shorts** ({{ shorts | length }} of {{ n_channels }} / {{ '{:.1f}'.format((shorts|length|float) / n_channels * 100) }}%): **{{ shorts | join(', ') }}**
{%- endif %}
{%- if no_connection|length %}
 - **No connection** ({{ no_connection | length }} of {{ n_channels }} / {{ '{:.1f}'.format((no_connection|length|float) / n_channels * 100) }}%): **{{ no_connection | join(', ') }}**
{%- if n_reps > 1 -%}
{% for x in no_connection -%}
{%- set n_fails = (c[x] < c_threshold).sum() %}
     * **Channel {{ x }} failed** {{ n_fails }} of {{ n_reps }} reps **({{ '{:.1f}'.format(n_fails / n_reps * 100) }}%)**
{%- endfor %}
{%- endif %}
{%- endif %}
{%- else %}
**All channels passed.**
{%- endif %}
{%- if figure_path %}
   ![Channel capacitance summary]({{ figure_path }})
{%- endif %}
{%- endif %}
'''.strip())

    return template.render(results=results, **context)


def plot_test_channels_results(results, axes=None):
    '''
    .. versionadded:: 1.28

    Plot summary of results from :func:`test_channels`.

    Parameters
    ----------
    results : dict
        Results from :func:`test_channels`.
    axes : list, optional
        List of two `matplotlib` axes to plot to.

        If not set, axes are generated within a new `matplotlib` figure.

    Returns
    -------
    axes : list
        List of two `matplotlib` axes.

        The measured capacitance of each channel is plotted on the first axis
        as a bar chart.

        The histogram of the measured capacitances across all channels is
        plotted on the second axis.
    '''
    c = np.array(results['c'])

    if axes is None:
        fig, axes = mpl.pyplot.subplots(2, figsize=(4, 4))

    axis_i = axes[0]
    axis_i.bar(range(c.shape[0]), np.mean(c, 1), yerr=np.std(c, 1))
    axis_i.set_xlabel("Channel")
    axis_i.set_ylabel("Capacitance")
    # Use SI unit prefixes for y-axis capacitance tick labels.
    axis_i.yaxis.set_major_formatter(CAPACITANCE_FORMATTER)

    axis_i = axes[1]
    axis_i.hist(c.flatten(), 20)
    axis_i.set_ylabel("# of channels")
    axis_i.set_xlabel("Capacitance")
    # Use SI unit prefixes for x-axis capacitance tick labels.
    axis_i.xaxis.set_major_formatter(CAPACITANCE_FORMATTER)

    return axes


def self_test(proxy, tests=None):
    '''
    .. versionadded:: 1.28

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


def _generate_test_channels_results(channels=20, n_reps=3, c_min=3e-12,
                                    c_max=12e-12, short_count=3, seed=0):
    '''
    .. versionadded:: 1.28

    Generate simulated results in form returned by :func:`test_channels`.

    Useful, for example, to test :func:`format_test_channels_results`.

    Parameters
    ----------
    channels : list or int, optional
        List of channels numbers.

        If specified as an integer, use ``range(channels)``.

        Default: ``range(20)``.
    n_reps : int, optional
        Number of capacitance measurements to simulate for each channel.
    c_min : float, optional
        Minimum simulated capacitance.

        Default: 3e-12.
    c_max : float, optional
        Maximum simulated capacitance.

        Default: 12e-12.
    short_count : int, optional
        Number of simulated shorted connections.
    seed : int or array_like, optional
        Seed for `numpy.random.RandomState`.

        Must be convertible to 32 bit unsigned integers.

        Default: 0.
    '''
    if isinstance(channels, int):
        test_channels_ = np.arange(20, dtype=int)
    else:
        test_channels_ = channels

    random = np.random.RandomState(seed)
    # Simulate capacitance measurements for each test channel.
    c = np.array([random.rand(n_reps) * (c_max - c_min) + c_min
                  for i in xrange(len(test_channels_))])
    # Simulate shorts by randomly selecting test channels.
    shorts = random.choice(test_channels_, size=min(short_count,
                                                    len(test_channels_)),
                           replace=False).tolist()

    return {'c': c.tolist(), 'test_channels': test_channels_, 'shorts': shorts}


def generate_report(results, output_path=None, force=False):
    '''
    .. versionadded:: 1.28

    Generate summary report of :func:`self_test` results either as Markdown or
    a Word document.

    Parameters
    ----------
    results : dict
        Results from :func:`self_test`.
    output_path : str, optional
        Report output path.

        If not specified, a text-only Markdown report is generated.

        If extension of output path is ``docx``, write output as Word document.

        Otherwise, output path is interpreted as a directory path and a
        Markdown file is written to the output directory, along with ``.png``
        images for test-related plots (where applicable).  Output directory
        will be created if it does not exist.
    force : bool, optional
        Overwrite output path if it exists.

    Returns
    -------
    str or None
        If :data:`output_path` is not specified, a text-only Markdown report is
        returned.

    Raises
    ------
    IOError
        If :data:`output_path` exists and :data:`force` is not ``True``.
    '''
    if output_path is not None:
        output_path = ph.path(output_path).realpath()
        if output_path.exists() and not force:
            if output_path.isdir() and output_path.listdir():
                # Output path is a directory with existing contents.
                raise IOError('Output directory already exists and is '
                              'non-empty.  Use `force` to overwrite.')
            elif output_path.ext.lower() == '.docx':
                raise IOError('Output path exists.  Use `force` to overwrite.')
            elif output_path.isfile():
                raise IOError('Output path exists and is a file.  Output path '
                              'must either be a directory or a filepath with '
                              'the `.docx` extension.')

    tests_with_figure = set(['test_channels', 'test_voltage',
                             'test_on_board_feedback_calibration'])

    if output_path is None:
        # Execute `format_<test name>_results` for each test to generate each
        # respective Markdown report.
        md_results_cmds = ['format_{test_name}_results(results["{test_name}"])'
                           .format(test_name=name_i) for name_i in ALL_TESTS]
        md_results = map(eval, md_results_cmds)

        # Join Markdown reports, separated by horizontal bars.
        md_report = (2 * '\n' + (72 * '-') + 2 * '\n').join(md_results)

        # No output path was specified.  Return text-only Markdown report.
        return md_report

    if output_path.ext.lower() == '.docx':
        output_path.parent.makedirs_p()
        parent_dir = ph.path(tempfile.mkdtemp(prefix='dropbot-self-test'))
    else:
        parent_dir = output_path
        output_path.makedirs_p()

    markdown_path = parent_dir.joinpath('results-summary.markdown')

    try:
        # Execute `format_<test name>_results` for each test to generate each
        # respective Markdown report.
        md_results = [eval('format_{test_name}_results'
                           .format(test_name=name_i))
                      (results[name_i],
                       **({'figure_path': parent_dir.joinpath(name_i + '.png')}
                          if name_i in tests_with_figure else {}))
                      for name_i in ALL_TESTS]

        # Join Markdown reports, separated by horizontal bars.
        md_report = (2 * '\n' + (72 * '-') + 2 * '\n').join(md_results)

        with markdown_path.open('w') as output:
            output.write(md_report)

        if output_path.ext.lower() == '.docx':
            sp.check_call(['pandoc', markdown_path, '-o', output_path],
                          shell=True)
    finally:
        if output_path.ext.lower() == '.docx':
            parent_dir.rmtree()
