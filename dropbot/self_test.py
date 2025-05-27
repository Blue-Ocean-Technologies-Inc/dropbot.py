# coding: utf-8
import bs4
import pint
import copy
import jinja2
import logging
import pkgutil
import tempfile
import pypandoc
import json_tricks

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
# Set non-interactive backend to avoid GUI warnings
mpl.use('Agg')

import numpy as np
import pandas as pd
import datetime as dt
# import subprocess as sp

import path_helpers as ph

from tqdm import tqdm

from . import NOMINAL_ON_BOARD_CALIBRATION_CAPACITORS
# Import test functions used by `self_test`.
from .hardware_test import (ALL_TESTS, system_info, test_system_metrics,
                            test_i2c, test_voltage, test_shorts,
                            test_on_board_feedback_calibration,
                            test_channels)

ureg = pint.UnitRegistry()

logger = logging.getLogger(name=__name__)

__all__ = ['format_system_info_results', 'format_test_channels_results',
           'format_test_i2c_results',
           'format_test_on_board_feedback_calibration_results',
           'format_test_shorts_results', 'format_test_system_metrics_results',
           'format_test_voltage_results']

CAPACITANCE_FORMATTER = ticker.FuncFormatter(lambda x, *args: f"{ureg.Quantity(x, ureg.F).to('pF'):.0f~#P}")
VOLTAGE_FORMATTER = ticker.FuncFormatter(lambda x, *args: f"{ureg.Quantity(x, ureg.V):.0f~#P}")

def format_system_info_results(info):
    """
    .. versionadded:: 1.28

    Parameters
    ----------
    info : dict
        Results from :func:`dropbot.hardware_test.system_info`.

    Returns
    -------
    str
        Summary of :func:`dropbot.hardware_test.system_info` results in
        Markdown format.
    """
    template = jinja2.Template(r'''
# Control board information: #

## Properties ##
 - **UUID**: `{{ info['control board']['uuid'] }}`
{% for name_i, property_i in info['control board']['properties'].items() %}
 - **{{ name_i }}**: `{{ property_i }}`
{%- endfor %}

## Config ##
{% for name_i, value_i in info['control board']['config'].items() %}
 - **{{ name_i }}**: `{{ value_i }}`
{%- endfor %}

{% for key_i in ('soft_i2c_scan', 'number_of_channels') -%}
**{{ key_i.replace('_', ' ') }}**: `{{ info[key_i] }}`{{ '  ' }}
{% endfor -%}'''.strip())

    # make a copy of the dictionary to avoid modifying the input dictionary
    info_ = copy.deepcopy(info)
    # replace the value of C16 in the dictionary with the formatted value
    config = info_['control board']['config']
    config['C16'] = f"{ureg.Quantity(config['C16'], ureg.F).to('nF'):.2f~#P}"
    config['R7'] = f"{ureg.Quantity(config['R7'], ureg.ohm).to('kohm'):.2f~#P}"
    config['pot_max'] = f"{ureg.Quantity(config['pot_max'], ureg.ohm).to('kohm'):.2f~#P}"
    config['max_voltage'] = f"{ureg.Quantity(config['max_voltage'], ureg.V):.1f~#P}"
    config['min_frequency'] = f"{ureg.Quantity(config['min_frequency'], ureg.Hz):.1f~#P}"
    config['max_frequency'] = f"{ureg.Quantity(config['max_frequency'], ureg.Hz).to('kHz'):.1f~#P}"

    return template.render(info=info_).strip()


def format_test_system_metrics_results(results):
    """
    .. versionadded:: 1.39

    Parameters
    ----------
    results : dict
        Results from :func:`dropbot.hardware_test.test_system_metrics`.

    Returns
    -------
    str
        Summary of :func:`dropbot.hardware_test.test_system_metrics` results in
        Markdown format.
    """
    template = jinja2.Template(r'''
# System Metrics: #

{% for name_i, property_i in results.items() %}
 - **{{ name_i }}**: `{{ property_i }}`
{%- endfor %}'''.strip())
    # Remove utc_timestamp and test_duration from results dict before
    # rendering.  Make a copy to avoid modifying the input dictionary.
    results = results.copy()
    del results['utc_timestamp']
    del results['duration']

    # Specify SI units for recognized properties.
    units = {'analog_reference': ureg.V,
             'temperature': ureg.degC,
             'voltage_limit': ureg.V}
    for property_i, unit_i in units.items():
        results[property_i] = f"{ureg.Quantity(results[property_i], unit_i ):.1f~#P}"

    return template.render(results=results).strip()


def format_test_i2c_results(results):
    """
    .. versionadded:: 1.28

    .. versionchanged:: 1.29.2
        Fix new lines between list items.

        Prior to version *1.29.2*, the output did not have new lines separating
        list items.

    Parameters
    ----------
    results : dict
        Results from :func:`dropbot.hardware_test.test_i2c`.

    Returns
    -------
    str
        Summary of :func:`dropbot.hardware_test.test_i2c` results in Markdown
        format.
    """
    template = jinja2.Template(r'''
# I2C scan: #
{% if results['i2c_scan'] %}
{%- for address in results['i2c_scan']|sort %}
{%- set data = results['i2c_scan'][address] %}
 - **`{{ address }}`**:
{%- if 'name' in data %} {{ data['name'] }}{% endif %}
{%- if 'hardware_version' in data %} v{{ data['hardware_version'] }}{% endif %}
{%- if 'software_version' in data %}, firmware: v{{ data['software_version'] }}
{%- endif %}
{%- if 'uuid' in data %}, uuid: `{{ data['uuid'] }}`{% endif -%}
{% endfor -%}
{% else %}
No devices found on I2C bus.
{%- endif -%}'''.strip())
    return template.render(results=results).strip()


def format_test_voltage_results(results, figure_path=None):
    """
    .. versionadded:: 1.28

    .. versionchanged:: 1.30
        Format measured/target voltages as a table.

    Parameters
    ----------
    results : dict
        Results from :func:`dropbot.hardware_test.test_voltage`.
    figure_path : str, optional
        If specified, include summary figure reference in text and write figure
        image to specified path.

        Filepath must have web browser compatible image extension (e.g.,
        ``.jpg``, ``.png``).

    Returns
    -------
    str
        Summary of :func:`dropbot.hardware_test.test_voltage` results in
        Markdown format.

        If :data:`figure_path` was specified, summary figure is written to the
        specified path.
    """
    voltages = pd.DataFrame(np.column_stack([results['target_voltage'], results['measured_voltage']]),
                            columns=['target', 'measured'])
    # Calculate the average rms error
    error = voltages['measured'] - voltages['target']
    rms_error = 100 * np.sqrt(np.mean((error / voltages['target']) ** 2))

    if figure_path:
        figure_path = ph.path(figure_path).realpath()
        # Make parent directories if they don't exist.
        figure_path.parent.makedirs(exist_ok=True)
        axis = plot_test_voltage_results(results)
        fig = axis.get_figure()
        fig.tight_layout()
        fig.savefig(figure_path, bbox_inches='tight')

    template = jinja2.Template(r'''
# Test voltage results: #

 - **Output voltages**:

{{ voltages.T|string|indent(8, True) }}

- **Root-mean-squared (RMS) error**: {{ '{:.1f}'.format(rms_error) }}%

{%- if figure_path %}
<div style="margin-top: 20px;">
   <img src="{{ figure_path }}" alt="" style="max-width: 100%; height: auto;">
</div>
{%- endif %}
    '''.strip())

    return template.render(results=results, voltages=voltages
                           .map(lambda x: f"{ureg.Quantity(x, ureg.V):.1f~#P}"),
                           rms_error=rms_error,
                           figure_path=figure_path).strip()


def plot_test_voltage_results(results, axis=None):
    """
    .. versionadded:: 1.28

    Plot summary of results from :func:`dropbot.hardware_test.test_voltage`.

    Parameters
    ----------
    results : dict
        Results from :func:`dropbot.hardware_test.test_voltage`.
    axis : matplotlib.axes._subplots.AxesSubplot, optional
        A `matplotlib` axis to plot to.

        If not set, axis is generated within a new `matplotlib` figure.

    Returns
    -------
    axis : matplotlib.axes._subplots.AxesSubplot
        A `matplotlib` axis plotting the measured voltage against the target
        voltage.
    """
    if axis is None:
        fig, axis = plt.subplots(figsize=(3.5, 3.5))
        axis.set_aspect(True)

    # Plot the measured vs target voltage
    axis.plot(results['target_voltage'], results['measured_voltage'], 'o')
    axis.plot(results['target_voltage'], results['target_voltage'], 'k--')
    axis.set_xlabel('Target voltage')
    axis.set_ylabel('Measured voltage')
    axis.xaxis.set_major_formatter(VOLTAGE_FORMATTER)
    axis.yaxis.set_major_formatter(VOLTAGE_FORMATTER)
    axis.set_title('Measured vs Target Voltage')
    return axis


def format_test_on_board_feedback_calibration_results(results, figure_path=None):
    """
    .. versionadded:: 1.28

    .. versionchanged:: 1.30
        Format measured/nominal voltages as a table.

    .. versionchanged:: 1.46
        Accept capacitance input in list or 2-d array format (where
        the dimension 1 represents replicates).

    Parameters
    ----------
    results : dict
        Results from
        :func:`dropbot.hardware_test.test_on_board_feedback_calibration`.
    figure_path : str, optional
        If specified, include summary figure reference in text and write figure
        image to specified path.

        Filepath must have web browser compatible image extension (e.g.,
        ``.jpg``, ``.png``).

    Returns
    -------
    str
        Summary of
        :func:`dropbot.hardware_test.test_on_board_feedback_calibration`
        results in Markdown format.

        If :data:`figure_path` was specified, summary figure is written to the
        specified path.
    """
    C_nominal = NOMINAL_ON_BOARD_CALIBRATION_CAPACITORS
    c_measured = np.array(results['c_measured'])

    # If c_measured is a 1-d array or a list, convert it to a 2-d array
    if len(c_measured.shape) == 1:
        c_measured = np.reshape(c_measured, [len(C_nominal), 1])

    capacitances = (pd.DataFrame(np.column_stack([C_nominal.values, np.mean(c_measured, 1)]),
                                 columns=['nominal', 'measured']).T
                    .map(lambda x: f"{ureg.Quantity(x, ureg.F).to('pF'):.1f~#P}"))

    if figure_path:
        figure_path = ph.path(figure_path).realpath()
        # Make parent directories if they don't exist.
        figure_path.parent.makedirs(exist_ok=True)
        axis = plot_test_on_board_feedback_calibration_results(results)
        axis.set_title('Measured vs Nominal Capacitance')
        fig = axis.get_figure()
        fig.tight_layout()
        fig.savefig(figure_path, bbox_inches='tight')

    template = jinja2.Template(r'''
# Test on-board feedback calibration results: #

 - **Measured capacitance**:

{{ capacitances|string|indent(8, True) }}
{%- if figure_path %}
<div style="margin-top: 20px;">
   <img src="{{ figure_path }}" alt="" style="max-width: 100%; height: auto;">
</div>
{%- endif %}'''.strip())
    return template.render(results=results, capacitances=capacitances,
                           figure_path=figure_path)


def plot_test_on_board_feedback_calibration_results(results, axis=None):
    """
    .. versionadded:: 1.28

    .. versionchanged:: 1.30
        Use :data:`NOMINAL_ON_BOARD_CALIBRATION_CAPACITORS` from
        :module:`__init__`.

    .. versionchanged:: 1.46
        Accept capacitance input in list or 2-d array format (where
        the dimension 1 represents replicates).


    Plot summary of results from
    :func:`dropbot.hardware_test.test_on_board_feedback_calibration`.

    Parameters
    ----------
    results : dict
        Results from
        :func:`dropbot.hardware_test.test_on_board_feedback_calibration`.
    axis : matplotlib.axes._subplots.AxesSubplot, optional
        A `matplotlib` axis to plot to.

        If not set, axis is generated within a new `matplotlib` figure.

    Returns
    -------
    axis : matplotlib.axes._subplots.AxesSubplot
        A `matplotlib` axis plotting the measured on-board capacitance against
        the nominal values of the on-board feedback calibration capacitors.
    """
    if axis is None:
        fig, axis = plt.subplots(figsize=(3, 3))
        axis.set_aspect(True)

    C_nominal = NOMINAL_ON_BOARD_CALIBRATION_CAPACITORS.values
    c_measured = np.array(results['c_measured'])

    # If c_measured is a 1-d array or a list, convert it to a 2-d array
    if len(np.array(c_measured).shape) == 1:
        c_measured = np.reshape(c_measured, [len(C_nominal), 1])

    axis.errorbar(C_nominal, np.mean(c_measured, 1),
                  yerr=np.std(c_measured, 1), fmt='o')
    axis.plot(C_nominal, C_nominal, 'k--')
    axis.set_xlabel('Nominal capacitance')
    axis.set_ylabel('Measured capacitance')
    # Use SI unit prefixes for axis capacitance tick labels.
    axis.xaxis.set_major_formatter(CAPACITANCE_FORMATTER)
    axis.yaxis.set_major_formatter(CAPACITANCE_FORMATTER)

    return axis


def format_test_shorts_results(results):
    """
    .. versionadded:: 1.28

    Parameters
    ----------
    results : dict
        Results from :func:`dropbot.hardware_test.test_shorts`.

    Returns
    -------
    str
        Summary of :func:`dropbot.hardware_test.test_shorts` results in
        Markdown format.
    """
    template = jinja2.Template(r'''
# Test shorts results: #
{% if results['shorts'] %}
 - Shorts on channels {{ results['shorts']|join(', ') }}
{%- else %}
 - No shorts.
{%- endif -%}'''.strip())

    return template.render(results=results)


def format_test_channels_results(results, figure_path=None):
    """
    .. versionadded:: 1.28

    .. versionchanged:: 1.30
        Align plot to left.

        Include channel count in message when all channels pass.

    Parameters
    ----------
    results : dict
        Results from :func:`dropbot.hardware_test.test_channels`.
    figure_path : str, optional
        If specified, include summary figure reference in text and write figure
        image to specified path.

        Filepath must have web browser compatible image extension (e.g.,
        ``.jpg``, ``.png``).

    Returns
    -------
    str
        Summary of :func:`dropbot.hardware_test.test_channels` results in
        Markdown format.

        If :data:`figure_path` was specified, summary figure is written to the
        specified path.
    """
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
        figure_path.parent.makedirs(exist_ok=True)
        axes = plot_test_channels_results(results)
        axes[0].set_title('Channel Capacitance Summary')
        fig = axes[0].get_figure()
        fig.tight_layout()
        fig.savefig(figure_path, bbox_inches='tight')

    context = dict(c=c, c_threshold=c_threshold, figure_path=figure_path,
                   n_channels=n_channels, n_reps=n_reps,
                   no_connection=no_connection, shorts=shorts,
                   test_channels=test_channels_)

    template = jinja2.Template(r'''
# Test channels results: #
{% if c|length < 1 %}
No channels tested.
{% else %}{% if shorts|length or no_connection|length %}
{%- set bad_channels = shorts + no_connection %}
{%- set bad_channels_count = (bad_channels|length) %}
 - The following channels failed ({{ bad_channels_count }} of {{ n_channels }} / **{{ '{:.1f}'.format((bad_channels_count | float) / n_channels * 100) }}%**):
{% if shorts|length %}
    * **Shorts** ({{ shorts | length }} of {{ n_channels }} / {{ '{:.1f}'.format((shorts|length|float) / n_channels * 100) }}%): **{{ shorts | join(', ') }}**
{%- endif %}
{%- if no_connection|length %}
    * **No connection** ({{ no_connection | length }} of {{ n_channels }} / {{ '{:.1f}'.format((no_connection|length|float) / n_channels * 100) }}%): **{{ no_connection | join(', ') }}**
{%- if n_reps > 1 -%}
{% for x in no_connection -%}
{%- set n_fails = (c[x] < c_threshold).sum() %}
     * **Channel {{ x }} failed** {{ n_fails }} of {{ n_reps }} reps **({{ '{:.1f}'.format(n_fails / n_reps * 100) }}%)**
{%- endfor %}
{%- endif %}
{%- endif %}
{%- else %}
- **All {{ n_channels }} channels passed.**
{%- endif %}
{%- if figure_path %}
<div style="margin-top: 20px;">
   <img src="{{ figure_path }}" alt="" style="max-width: 100%; height: auto;">
</div>
{%- endif %}
{%- endif %}
'''.strip())

    return template.render(results=results, **context)


def plot_test_channels_results(results, axes=None):
    """
    .. versionadded:: 1.28

    Plot summary of results from :func:`dropbot.hardware_test.test_channels`.

    Parameters
    ----------
    results : dict
        Results from :func:`dropbot.hardware_test.test_channels`.
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
    """
    c = np.array(results['c'])

    if axes is None:
        fig, axes = plt.subplots(2, figsize=(4, 4))

    axis_i = axes[0]
    axis_i.bar(list(range(c.shape[0])), np.mean(c, 1), yerr=np.std(c, 1))
    # change the color of the bars to red for any readings below the threshold
    for i in range(c.shape[0]):
        if np.mean(c[i]) < 5e-12:
            axis_i.patches[i].set_facecolor('tab:red')
    axis_i.set_xlabel("Channel")
    axis_i.set_ylabel("Capacitance")
    # Use SI unit prefixes for y-axis capacitance tick labels.
    axis_i.yaxis.set_major_formatter(CAPACITANCE_FORMATTER)

    axis_i = axes[1]
    hist = np.histogram(c, bins=20)
    # plot the readings above the threshold as blue and below as red
    # find the indices of the readings below the threshold
    hist_below = np.histogram(c[c < 5e-12], hist[1])
    axis_i.stairs(hist_below[0], hist_below[1], color='tab:red', fill=True)
    hist_above = np.histogram(c[c >= 5e-12], hist[1])
    axis_i.stairs(hist_above[0], hist_above[1], color='tab:blue', fill=True)

    axis_i.set_ylabel("# of channels")
    axis_i.set_xlabel("Capacitance")
    # Use SI unit prefixes for x-axis capacitance tick labels.
    axis_i.xaxis.set_major_formatter(CAPACITANCE_FORMATTER)

    return axes


def self_test(proxy, tests=None):
    """
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
    """
    total_time = 0

    if tests is None:
        tests = ALL_TESTS
    results = {}

    for test_name_i in (pbar:= tqdm(tests)):
        pbar.set_description(test_name_i)
        test_func_i = eval(test_name_i)
        results[test_name_i] = test_func_i(proxy)
        duration_i = results[test_name_i]['duration']
        logger.info('%s: %.1f s', test_name_i, duration_i)
        total_time += duration_i

    logger.info('**Total time: %.1f s**', total_time)

    return results


def _generate_test_channels_results(channels=20, n_reps=3, c_min=3e-12,
                                    c_max=12e-12, short_count=3, seed=0):
    """
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
    """
    if isinstance(channels, int):
        test_channels_ = np.arange(20, dtype=int)
    else:
        test_channels_ = channels

    random = np.random.RandomState(seed)
    # Simulate capacitance measurements for each test channel.
    c = np.array([random.rand(n_reps) * (c_max - c_min) + c_min
                  for i in range(len(test_channels_))])
    # Simulate shorts by randomly selecting test channels.
    shorts = random.choice(test_channels_, size=min(short_count, len(test_channels_)),
                           replace=False).tolist()

    return {'c': c.tolist(), 'test_channels': test_channels_, 'shorts': shorts}


def generate_report(results, output_path=None, force=False):
    """
    .. versionadded:: 1.28

    .. versionchanged:: 1.29.1
        Only try to format results for tests that have data in the
        :data:`results` dictionary.

        Prior to version ``1.29.1``, this function would fail unless the
        :data:`results` dictionary contained data existed for **all tests** in
        :data:`ALL_TESTS` .

    .. versionchanged:: 1.54
        If output extension is ``.html``, output self-contained HTML report
        with ``<script id="results">...</script>`` tag containing JSON report
        results.


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

        If extension of output path is ``html``, write output as self-contained
        HTML report with ``<script id="results">...</script>`` tag containing
        JSON report results.

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
    """
    if output_path is not None:
        output_path = ph.path(output_path).realpath()
        if output_path.exists() and not force:
            if output_path.isdir() and output_path.listdir():
                # Output path is a directory with existing contents.
                raise IOError('Output directory already exists and is non-empty.  Use `force` to overwrite.')
            elif output_path.ext.lower() == '.docx':
                raise IOError('Output path exists.  Use `force` to overwrite.')
            elif output_path.ext.lower() == '.html':
                raise IOError('Output path exists.  Use `force` to overwrite.')
            elif output_path.isfile():
                raise IOError('Output path exists and is a file.  Output path '
                              'must either be a directory or a filepath with '
                              'the `.docx` extension.')

    tests_with_figure = {'test_channels', 'test_voltage', 'test_on_board_feedback_calibration'}

    # Find starting time of earliest test (or current date and time if no
    # timestamp is available).
    min_timestamp = min([result_i['utc_timestamp'] for result_i in results.values()
                         if 'utc_timestamp' in result_i] + [dt.datetime.now(dt.timezone.utc).isoformat()])
    header = f"---\ntitle: 'DropBot Self Test Report (*{min_timestamp.split('.')[0]}*)'\n---\n\n"

    if output_path is None:
        # Execute `format_<test name>_results` for each test to generate each
        # respective Markdown report.
        md_results_cmds = ['format_{test_name}_results(results["{test_name}"])'
                           .format(test_name=name_i) for name_i in ALL_TESTS if name_i in results]
        md_results = list(map(eval, md_results_cmds))

        # Join Markdown reports, separated by horizontal bars.
        md_report = header + (2 * '\n' + (72 * '-') + 2 * '\n').join(md_results)

        # No output path was specified.  Return text-only Markdown report.
        return md_report

    if output_path.ext.lower() in ('.docx', '.html'):
        output_path.parent.makedirs(exist_ok=True)
        parent_dir = ph.path(tempfile.mkdtemp(prefix='dropbot-self-test'))
    else:
        parent_dir = output_path
        output_path.makedirs(exist_ok=True)

    markdown_path = parent_dir.joinpath('results-summary.markdown')

    if output_path.ext.lower() in ('.docx', '.html'):
        print(f"Writing report to {output_path}")
    else:
        print(f"Writing report to {markdown_path}")

    try:
        # Execute `format_<test name>_results` for each test to generate each
        # respective Markdown report.
        md_results = [eval('format_{test_name}_results'
                           .format(test_name=name_i))
                      (results[name_i],
                       **({'figure_path': parent_dir.joinpath(name_i + '.png')}
                          if name_i in tests_with_figure else {}))
                      for name_i in ALL_TESTS if name_i in results]

        # Join Markdown reports, separated by horizontal bars.
        md_report = header + (2 * '\n' + (72 * '-') + 2 * '\n').join(md_results)

        markdown_path.write_text(md_report)

        if output_path.ext.lower() == '.docx':
            # sp.check_call(['pandoc', markdown_path, '-o', output_path], shell=True)
            pypandoc.convert_file(markdown_path, 'docx', format='markdown', outputfile=output_path)
        elif output_path.ext.lower() == '.html':
            # Write template to file for use with `pandoc`.
            template = pkgutil.get_data('dropbot', 'static/templates/SelfTestTemplate.html5')
            template_path = parent_dir.joinpath('SelfTestTemplate.html5')
            template_path.write_text(template)
            # Use `pandoc` to create self-contained `.html` report.
            # sp.check_call(['pandoc', markdown_path, '-o', output_path,
            #                '--standalone', '--self-contained', '--template',
            #                template_path], shell=True, stderr=sp.PIPE)
            #
            # data = output_path.read_text()
            data = pypandoc.convert_file(markdown_path, 'html', format='markdown',
                                         extra_args=['--standalone', '--embed-resources', '--template', template_path])

            output_path.write_text(data, encoding='utf-8')

            # Inject JSON result data into HTML report.
            soup = bs4.BeautifulSoup(data, 'lxml')
            results_script = soup.select_one('script#results')
            # Format JSON with indents.  Works around [`json_tricks`
            # issue][i51].
            #
            # [i51]: https://github.com/mverleg/pyjson_tricks/issues/51
            json_data = json_tricks.dumps(results, indent=4)
            results_script.string = bs4.NavigableString(json_data)
            output_path.write_text(str(soup), encoding='utf-8')

    finally:
        if output_path.ext.lower() in ('.docx', '.html'):
            parent_dir.rmtree()
