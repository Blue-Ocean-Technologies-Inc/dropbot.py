# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.1'
#       jupytext_version: 0.8.1
#   kernelspec:
#     display_name: Python 2
#     language: python
#     name: python2
#   language_info:
#     codemirror_mode:
#       name: ipython
#       version: 2
#     file_extension: .py
#     mimetype: text/x-python
#     name: python
#     nbconvert_exporter: python
#     pygments_lexer: ipython2
#     version: 2.7.14
#   toc:
#     base_numbering: 1
#     nav_menu: {}
#     number_sections: true
#     sideBar: true
#     skip_h1_title: false
#     title_cell: Table of Contents
#     title_sidebar: Contents
#     toc_cell: false
#     toc_position: {}
#     toc_section_display: true
#     toc_window_display: false
# ---

# %% {"ExecuteTime": {"start_time": "2018-12-10T04:55:44.608000Z", "end_time": "2018-12-10T04:55:51.951000Z"}}
from __future__ import print_function, unicode_literals, division
import logging
import time
logging.basicConfig(level=logging.DEBUG)

import arrow
# Access Teensy header constants, e.g., `A10`, `A11`, etc.
import arduino_helpers.hardware.teensy as teensy
import dropbot as db
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import si_prefix as si
%matplotlib inline


def actuate_random_channels(N, seed=0):
    if seed is not None:
        np.random.seed(seed)
    random_channels = sorted(np.random.choice(range(120), size=N, replace=False))

    proxy.turn_off_all_channels()
    state_of_channels = pd.Series(1, index=random_channels)
    proxy.state_of_channels = state_of_channels
    return proxy.state_of_channels


try:
    proxy.terminate()
except:
    pass

proxy = db.SerialProxy(ignore=True)

proxy.update_state(event_mask=db.EVENT_SHORTS_DETECTED | db.EVENT_ENABLE |
                   db.EVENT_CHANNELS_UPDATED,
                   hv_output_enabled=True,
                   capacitance_update_interval_ms=0,
                   # XXX Disable chip load feedback threshold.
                   chip_load_range_margin=0)
proxy.disabled_channels_mask = np.zeros_like(proxy.disabled_channels_mask)

# %% {"ExecuteTime": {"start_time": "2018-12-10T04:55:51.958000Z", "end_time": "2018-12-10T04:55:51.988000Z"}}
def dump(message):
    print(message)
    
proxy.signals.signal('halted').connect(dump)

# %% {"ExecuteTime": {"start_time": "2018-12-10T04:55:51.997000Z", "end_time": "2018-12-10T04:55:52.011000Z"}}
proxy.signals.signal('shorts-detected').connect(dump)

# %% {"ExecuteTime": {"start_time": "2018-12-10T04:55:52.018000Z", "end_time": "2018-12-10T04:55:52.032000Z"}}
import analog_save_config as ac

# %% {"ExecuteTime": {"start_time": "2018-12-10T04:59:15.947000Z", "end_time": "2018-12-10T04:59:15.960000Z"}}
proxy.setConversionSpeed(2, 0)

# %% {"ExecuteTime": {"start_time": "2018-12-10T04:59:17.191000Z", "end_time": "2018-12-10T04:59:17.221000Z"}}
_adc_config = ac._adc_config(proxy)
_adc_config

# %%
ac._adc_load_config(proxy)

# %% {"ExecuteTime": {"start_time": "2018-12-10T04:55:52.556000Z", "end_time": "2018-12-10T04:55:52.561000Z"}}
# proxy.terminate()

# %% {"ExecuteTime": {"start_time": "2018-12-10T05:00:19.275000Z", "end_time": "2018-12-10T05:00:19.297000Z"}}
adc_config = proxy.analog_save_config(0)
adc_config

# %% {"ExecuteTime": {"start_time": "2018-12-10T05:00:50.962000Z", "end_time": "2018-12-10T05:00:50.990000Z"}}
proxy.setConversionSpeed(2, 0)
adc_config = proxy.analog_save_config(0)
adc_config

# %% {"ExecuteTime": {"start_time": "2018-12-10T05:01:29.464000Z", "end_time": "2018-12-10T05:01:29.505000Z"}}
# adc_config = proxy.analog_save_config(0)
proxy.analog_load_config(0, adc_config)
_adc_config = ac._adc_config(proxy)
_adc_config

# %% {"ExecuteTime": {"start_time": "2018-12-10T05:04:07.414000Z", "end_time": "2018-12-10T05:04:07.637000Z"}}
_adc_config['averaging'] = 4
_adc_config['sampling_speed'] = 'VERY_LOW_SPEED'
_adc_config['conversion_speed'] = 'HI_SPEED_16_BITS'
_adc_config['pga_gain'] = 0
_adc_config['reference'] = '1V2'

ac._adc_load_config(proxy, _adc_config)
display(ac._adc_config(proxy))

N = 10e3
duration = proxy.benchmark_analog_read(11, int(N))
N / duration

# %% {"ExecuteTime": {"start_time": "2018-12-10T05:22:20.687000Z", "end_time": "2018-12-10T05:22:20.696000Z"}}
proxy.terminate()

# %% {"ExecuteTime": {"start_time": "2018-12-10T05:03:42.345000Z", "end_time": "2018-12-10T05:03:42.442000Z"}}
N = 100
proxy.benchmmark_u16_percentile_diff(11, 50, 25, 75, N) / N

# %% {"ExecuteTime": {"start_time": "2018-12-07T22:46:46.134000Z", "end_time": "2018-12-07T22:46:47.745000Z"}}
SAMPLING_SPEEDS_BY_ID = {v: k for k, v in ac.SAMPLING_SPEEDS.items()}
CONVERSION_SPEEDS_BY_ID = {v: k for k, v in ac.CONVERSION_SPEEDS.items()}

# df_best = pd.read_clipboard()
sampling_rates = []
for i, row_i in df_best[['averaging', 'sampling_speed', 'conversion_speed']].iterrows():
    adc_config_i = row_i.to_dict()
    
    adc_config['pga_gain'] = 0
    adc_config['reference'] = '1V2'
#     adc_config['averaging'] = 4
    adc_config['sampling_speed'] = SAMPLING_SPEEDS_BY_ID[min(4, row_i['sampling_speed'])]
    adc_config['conversion_speed'] = CONVERSION_SPEEDS_BY_ID[row_i['conversion_speed']]

    ac._adc_load_config(proxy, adc_config)
#     display(ac._adc_config(proxy))

    N = 10e3
    duration = proxy.benchmark_analog_read(11, int(N))
    adc_config_i['sampling_rate_hz'] = N / duration
    sampling_rates.append(adc_config_i)

df_sampling_rates = pd.DataFrame(sampling_rates, columns=['averaging',
                                                          'sampling_speed',
                                                          'conversion_speed',
                                                          'sampling_rate_hz'])
df_sampling_rates['period/10 samples'] = (10 / df_sampling_rates['sampling_rate_hz']).map(lambda x: '%ss' % si.si_format(x))
df_sampling_rates['sampling_rate_hz'] = df_sampling_rates['sampling_rate_hz'].map(lambda x: '%sHz' % si.si_format(x))
df_sampling_rates

# %% {"ExecuteTime": {"start_time": "2018-12-07T21:51:01.746000Z", "end_time": "2018-12-07T21:51:01.753000Z"}}
import functools as ft

# %% {"ExecuteTime": {"start_time": "2018-12-07T22:34:14.841000Z", "end_time": "2018-12-07T22:34:26.751000Z"}}
config_i = {'n_channels': 1,
            'averaging': 4,
            'sampling_speed': 0,
            'conversion_speed': 4}

amplitudes = []
N = 100
n_samples = range(10, 51, 5)
# n_samples = (15, )

for n_samples_i in n_samples:
    channels = actuate_random_channels(config_i['n_channels'])
    proxy.setReference(teensy.ADC_REF_1V2, 0)
    proxy.setAveraging(config_i['averaging'], 0)
    proxy.setConversionSpeed(config_i['conversion_speed'], 0)
    proxy.setSamplingSpeed(config_i['sampling_speed'], 0)

    read_f = ft.partial(proxy.differential_reads_simple, teensy.A10, teensy.A11)

    data = pd.concat([pd.Series(read_f(n_samples_i))
                      for i in range(N)],
                     keys=range(N)).unstack(level=0)
    stats = data.describe().T

    d = pd.DataFrame([stats['75%'] - stats['25%'], stats['max'] - stats['min']],
                     index=['%-diff', 'peak-to-peak'])
    amplitudes.append(d.T)
    
df_stats = pd.concat(amplitudes, keys=n_samples)


# %% {"ExecuteTime": {"start_time": "2018-12-07T22:34:26.757000Z", "end_time": "2018-12-07T22:34:28.157000Z"}}
df_stats.index.set_names(['n_samples', 'i'], inplace=True)
df_stats.reset_index(level=0).boxplot(by='n_samples')
df_summary = df_stats.groupby('n_samples').describe().T.unstack(level=0)
display(df_summary)
display(df_summary.loc['std'] / df_summary.loc['50%'])

# %% {"ExecuteTime": {"start_time": "2018-12-07T22:39:10.850000Z", "end_time": "2018-12-07T22:39:11.012000Z"}}
N = 10e3
30 * (proxy.benchmark_analog_read(11, N) / N)

# %% {"ExecuteTime": {"start_time": "2018-12-07T22:34:37.774000Z", "end_time": "2018-12-07T22:34:38.206000Z"}}
# _max, _min, _25, _75 = data.describe()[['max', 'min', '25%', '75%']]
# xlim = ax.get_xlim()
# ax.plot(xlim, 2 * [_25])
# ax.plot(xlim, 2 * [_75])
# ax.plot(xlim, 2 * [_min])
# ax.plot(xlim, 2 * [_max])
# _max - _min, _75 - _25
ax = data[data.columns[:5]].plot()
# ax.set_ylim(-(2 ** 15), 2 ** 15)

# %% {"ExecuteTime": {"start_time": "2018-12-07T21:40:06.614000Z", "end_time": "2018-12-07T21:40:14.712000Z"}}
%timeit duration = proxy.benchmark_analog_read(11, int(N))

# %% {"ExecuteTime": {"start_time": "2018-12-07T21:29:29.566000Z", "end_time": "2018-12-07T21:29:29.575000Z"}}
print('%ss' % si.si_format(duration / N))

# %% {"ExecuteTime": {"start_time": "2018-12-06T22:13:35.275000Z", "end_time": "2018-12-06T22:13:35.281000Z"}}
import json

# %% {"ExecuteTime": {"start_time": "2018-12-06T22:13:41.975000Z", "end_time": "2018-12-06T22:13:45.051000Z"}}
with open('exp(adc-config-00) differential.json', 'r') as input_:
    results = json.load(input_)

# %% {"ExecuteTime": {"start_time": "2018-12-06T22:13:46.932000Z", "end_time": "2018-12-06T22:13:46.957000Z"}}
%load_ext line_profiler

# %% {"ExecuteTime": {"start_time": "2018-12-07T19:03:04.493000Z", "end_time": "2018-12-07T19:03:04.508000Z"}}
range(10)[:None]

# %% {"ExecuteTime": {"start_time": "2018-12-07T19:04:59.615000Z", "end_time": "2018-12-07T19:04:59.631000Z"}}
def summarize_configuration_results(result_i, n_samples=None):
    result_i = result_i.copy()
    readings_i = result_i.pop('readings')
    stats_i = pd.concat([pd.Series(r['data'][:n_samples]).describe()
                         for r in readings_i],
                        keys=range(len(readings_i))).unstack(level=1)
    amplitude_i = stats_i['75%'] - stats_i['25%']
    result_i['amplitude'] = amplitude_i.median()
    result_i['std/median'] = amplitude_i.std() / result_i['amplitude']
    if n_samples is not None:
        result_i['n_samples'] = int(stats_i.iloc[0]['count'])
    return result_i

# %% {"ExecuteTime": {"start_time": "2018-12-07T19:05:00.537000Z", "end_time": "2018-12-07T19:05:00.544000Z"}}
# %lprun -f summarize_configuration_results summarize_configuration_results(result_i)
# %timeit summarize_configuration_results(result_i)
# summarize_configuration_results(result_i)

# %% {"ExecuteTime": {"start_time": "2018-12-07T19:23:42.678000Z", "end_time": "2018-12-07T19:27:22.242000Z"}}
summaries = []

for i, result_i in enumerate(results):
    print('\r%-80s' % ('Processing %3d/%-3d: %s' % (i + 1, len(results),
                                                    {k: v for k, v in result_i.items()
                                                     if k != 'readings'})), end='')
    summaries.append(summarize_configuration_results(result_i, n_samples=20))

# %% {"ExecuteTime": {"start_time": "2018-12-07T19:27:22.250000Z", "end_time": "2018-12-07T19:27:22.340000Z"}}
df_results = pd.DataFrame(summaries)
df_best = df_results.loc[df_results.groupby('n_channels')['std/median'].idxmin()]
df_best[['n_channels', 'averaging', 'sampling_speed', 'conversion_speed',
         'amplitude', 'std/median']]

# %% {"ExecuteTime": {"start_time": "2018-12-07T18:47:02.403000Z", "end_time": "2018-12-07T18:47:03.213000Z"}}
# with open('single-ended 1.2V (summary).json', 'w') as output:
#     json_tricks.dump(df_results, output)

# %% {"ExecuteTime": {"start_time": "2018-12-07T19:19:18.822000Z", "end_time": "2018-12-07T19:19:18.861000Z"}}
df_best = df_results.loc[(df_results.averaging == 8) &
                         (df_results.sampling_speed == 5) &
                         (df_results.conversion_speed == 4)]
df_best[['n_channels', 'averaging', 'sampling_speed', 'conversion_speed',
         'amplitude', 'std/median']]

# %% {"ExecuteTime": {"start_time": "2018-12-07T20:42:45.477000Z", "end_time": "2018-12-07T20:42:51.471000Z"}}
N = df_best.shape[0]
fig, axes = plt.subplots(N, 2, figsize=(10, N * 5))

for ax_i, i in zip(axes, df_best.index):
    result_i = results[i].copy()
    readings_i = result_i.pop('readings')
    df_i = pd.concat([pd.Series(r['data']) for r in readings_i],
                     keys=range(len(readings_i))).unstack(level=0)
    df_i[range(5)].plot(ax=ax_i[0])
    df_i[range(5)].plot(ax=ax_i[1])
#     ax_i[0].set_ylim(-(2 ** 15), 2 ** 15)
    ax_i[0].set_ylim(0, 2 ** 16)
    ax_i[0].set_title('%s' % result_i)
# plt.close('all')

# %% {"ExecuteTime": {"start_time": "2018-12-07T18:53:09.676000Z", "end_time": "2018-12-07T18:53:09.684000Z"}}
plt.close('all')

# %%
df_all.groupby().describe().head()

# %%
df_all.memory_usage()

# %%
for c in df_all.columns:
    if c != 'device_load':
        df_all[c] = df_all[c].astype('category')

# %%
del df_all

# %%
proxy.terminate()

# %% {"ExecuteTime": {"start_time": "2018-12-06T22:14:05.405000Z", "end_time": "2018-12-06T22:14:05.997000Z"}}
proxy.turn_off_all_channels()
proxy.voltage = 100
proxy.disabled_channels_mask = np.zeros_like(proxy.disabled_channels_mask)

# Access Teensy header constants, e.g., `A10`, `A11`, etc.
import arduino_helpers.hardware.teensy as teensy

channel_states = actuate_random_channels(10)
display(channel_states[channel_states > 0].to_frame().T)
proxy.setReference(teensy.ADC_REF_3V3, 0)
V_chip_load = pd.Series(proxy.analog_reads_simple(11, 50)) / (2 ** 16)
ax = V_chip_load.plot()
ax.set_ylim(0, 1)
display(proxy.measure_output_current(20))
V_chip_load.describe().to_frame().T

# %%
# 3.3 / 1.2 * .03
(V_chip_load * 3.3).plot(ylim=(0, 3.3))

# %%
pga_value = 1 << 6
pga_value

# %%
proxy

# %% {"ExecuteTime": {"start_time": "2018-12-06T22:14:11.702000Z", "end_time": "2018-12-06T22:14:11.804000Z"}}
from __future__ import print_function, unicode_literals, absolute_import, division

import arduino_helpers.hardware.teensy as teensy
import pandas as pd


ADC_SC2_REFSEL0_BIT = 0
ADC_CFG2_ADHSC = 0x04
ADC_CFG1_ADLSMP_BIT = 4
#define ADC_CFG2_ADLSTS1_BIT (1)
#define ADC_CFG2_ADLSTS0_BIT (0)
# Sampling speed
ADC_CFG1_ADLSMP_BIT = 4
ADC_CFG2_ADLSTS1_BIT = 1
ADC_CFG2_ADLSTS0_BIT = 0

ADC_SC3_AVGE_BIT = (2)
ADC_SC3_AVGS1_BIT = (1)
ADC_SC3_AVGS0_BIT = (0)

def ADC_CFG1_ADIV(n):
    return (((n) & 3) << 5)  # Clock divide select, 0=direct, 1=div2, 2=div4, 3=div8

def ADC_CFG1_ADICLK(n):
    return (((n) & 3) << 0)  # Input clock, 0=bus, 1=bus/2, 2=OSCERCLK, 3=async

# 36 MHz bus clock (see `proxy.D__F_BUS()`)
ADC_CFG1_2_25MHZ = (ADC_CFG1_ADIV(3) + ADC_CFG1_ADICLK(1))
ADC_CFG1_4_5MHZ = (ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1))
ADC_CFG1_9MHZ = (ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1))
ADC_CFG1_18MHZ = (ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1))

ADC_CFG1_LOW_SPEED = (ADC_CFG1_2_25MHZ)
ADC_CFG1_VERY_LOW_SPEED = ADC_CFG1_LOW_SPEED
ADC_CFG1_MED_SPEED = (ADC_CFG1_9MHZ)
ADC_CFG1_HI_SPEED_16_BITS = (ADC_CFG1_9MHZ)
ADC_CFG1_HI_SPEED = (ADC_CFG1_18MHZ)
ADC_CFG1_VERY_HIGH_SPEED = ADC_CFG1_HI_SPEED

def _averaging(adc_config):
    if not (adc_config.SC3 & (1 << ADC_SC3_AVGE_BIT)):
        return 0
    else:
        avgs = adc_config.SC3 & ((1 << ADC_SC3_AVGS1_BIT) + (1 << ADC_SC3_AVGS0_BIT))
        return 2 ** (avgs + 2)


def _conversion_speed(adc_config):
    speed = adc_config.CFG1 & 0b1100011

    if (adc_config.CFG2 & ADC_CFG2_ADHSC):
        # Either HI_SPEED_16_BITS, HI_SPEED, or VERY_HIGH_SPEED
        if speed == ADC_CFG1_VERY_HIGH_SPEED:
            return 'VERY_HIGH_SPEED'
        elif speed == ADC_CFG1_HI_SPEED:
            return 'HI_SPEED'
        else:  # if speed == ADC_CFG1_HI_SPEED_16_BITS:
            return 'HI_SPEED_16_BITS'
    else:
        # Either MED_SPEED, LOW_SPEED, or VERY_LOW_SPEED
        if speed == ADC_CFG1_MED_SPEED:
            return 'MED_SPEED'
        elif speed == ADC_CFG1_LOW_SPEED:
            return 'LOW_SPEED'
        else:  # if speed == ADC_CFG1_VERY_LOW_SPEED:
            return 'VERY_LOW_SPEED'

def _sampling_speed(adc_config):
    if not (adc_config.CFG1 & (1 << ADC_CFG1_ADLSMP_BIT)):
        sampling_speed = 'VERY_HIGH_SPEED'
    elif adc_config.CFG2 & ((1 << ADC_CFG2_ADLSTS1_BIT) | (1 << ADC_CFG2_ADLSTS0_BIT)) == ((1 << ADC_CFG2_ADLSTS1_BIT) | (1 << ADC_CFG2_ADLSTS0_BIT)):
        sampling_speed = 'HIGH_SPEED'
    elif adc_config.CFG2 & ((1 << ADC_CFG2_ADLSTS1_BIT) | (1 << ADC_CFG2_ADLSTS0_BIT)) == ((1 << ADC_CFG2_ADLSTS1_BIT) | (0 << ADC_CFG2_ADLSTS0_BIT)):
        sampling_speed = 'MED_SPEED'
    elif adc_config.CFG2 & ((1 << ADC_CFG2_ADLSTS1_BIT) | (1 << ADC_CFG2_ADLSTS0_BIT)) == ((0 << ADC_CFG2_ADLSTS1_BIT) | (1 << ADC_CFG2_ADLSTS0_BIT)):
        sampling_speed = 'LOW_SPEED'
    elif adc_config.CFG2 & ((1 << ADC_CFG2_ADLSTS1_BIT) | (1 << ADC_CFG2_ADLSTS0_BIT)) == ((0 << ADC_CFG2_ADLSTS1_BIT) | (0 << ADC_CFG2_ADLSTS0_BIT)):
        sampling_speed = 'VERY_LOW_SPEED'
    return sampling_speed

def _reference(adc_config):
    return 'ALT' if adc_config.SC2 & (1 << ADC_SC2_REFSEL0_BIT) else 'NONE'

# %%
proxy.setConversionSpeed(3, 0)
proxy.setSamplingSpeed(3, 0)
proxy.setAveraging(4, 0)
n = int(10e3)
sample_hz = n / proxy.benchmark_analog_read(11, n)
display(_adc_config(proxy))
sample_hz

# %%
adc_config_i = adc_config

# %% {"ExecuteTime": {"start_time": "2018-12-06T22:14:13.003000Z", "end_time": "2018-12-06T22:14:13.015000Z"}}
# proxy.setConversionSpeed(3, 0)
# proxy.setSamplingSpeed(3, 0)
# proxy.setAveraging(4, 0)

def _adc_config(proxy):
    adc_config_reg = pd.Series(proxy.analog_save_config(0), index=['SC1A', 'SC2', 'SC3', 'CFG1', 'CFG2'])
    adc_config = dict(zip(['reference', 'sampling_speed', 'conversion_speed', 'pga_gain', 'averaging'],
                          [_reference(adc_config_reg), _sampling_speed(adc_config_reg),
                           _conversion_speed(adc_config_reg),
                           proxy.getPGA(0) if proxy.isPGAEnabled(0) else 0,
                           _averaging(adc_config_reg)]))
    # init_adc_config = adc_config
    return adc_config

CONVERSION_SPEEDS = dict(zip(['VERY_LOW_SPEED', 'LOW_SPEED', 'MED_SPEED',
                              'HI_SPEED', 'HI_SPEED_16_BITS',
                              'VERY_HIGH_SPEED'], range(6)))
SAMPLING_SPEEDS = dict(zip(['VERY_LOW_SPEED', 'LOW_SPEED', 'MED_SPEED',
                            'HIGH_SPEED', 'VERY_HIGH_SPEED'], range(5)))
        
def _adc_load_config(proxy, adc_config, adc_num=0):
    '''
    Apply configuration to ADC.
    
    Parameters
    ----------
    adc_config : dict
        ADC configuration including the following keys::
        - ``averaging``
        - ``conversion_speed``
        - ``sampling_speed``
        - ``pga_gain``
        - ``reference``
    adc_num : int, optional
        ADC index number (default=``0``).
    '''
    proxy.setAveraging(adc_config['averaging'], adc_num)
    proxy.setConversionSpeed(CONVERSION_SPEEDS[adc_config['conversion_speed']],
                             adc_num)
    proxy.setSamplingSpeed(SAMPLING_SPEEDS[adc_config['sampling_speed']], adc_num)
    proxy.setReference(teensy.ADC_REF_3V3
                       if adc_config.get('reference') == 'NONE'
                       else teensy.ADC_REF_1V2, adc_num)
    if not adc_config['pga_gain']:
        proxy.disablePGA(adc_num)
    else:
        proxy.enablePGA(adc_config['pga_gain'], adc_num)

# %%
proxy.disablePGA(0)
proxy.setConversionSpeed(1, 0)
proxy.setSamplingSpeed(1, 0)
proxy.setReference(teensy.ADC_REF_3V3, 0)
proxy.setAveraging(16, 0)

adc_config = _adc_config(proxy)
display(adc_config)
_adc_load_config(proxy, adc_config)

_adc_config(proxy)

# %%
# _adc_load_config(proxy, adc_config_i)
# display(_adc_config(proxy))
# _adc_load_config(proxy, adc_config)
# display(_adc_config(proxy))
# _adc_load_config(proxy, adc_config_i)
n = int(10e3)
sample_hz = n / proxy.benchmark_analog_read(11, n)
display(_adc_config(proxy))
sample_hz

# %% {"ExecuteTime": {"start_time": "2018-12-06T22:26:34.836000Z", "end_time": "2018-12-06T22:26:34.853000Z"}}
configurations = [dict(zip(['n_channels', 'n_samples', 'averaging',
                            'sampling_speed', 'conversion_speed'],
                           [n_channels, n_samples, averaging,
                            sampling_speed, conversion_speed]))
                  for n_channels in range(1, 40, 5)
                  for n_samples in [50]
                  for averaging in [0, 4, 8, 16, 32]
                  for sampling_speed in [0, 1, 2, 3, 5]
                  for conversion_speed in range(5)]

# %% {"ExecuteTime": {"start_time": "2018-12-06T22:22:47.637000Z", "end_time": "2018-12-06T22:22:47.671000Z"}}
proxy.state

# %% {"ExecuteTime": {"start_time": "2018-12-06T22:26:38.578000Z", "end_time": "2018-12-06T22:58:52.728000Z"}}
from collections import OrderedDict
import functools as ft
    
    
def measure_configs(configurations, read_f, n=100):
    results = []
    
    for i, config_i in enumerate(configurations):
        print('\r%-50s' % ('%d/%d - %s' % (i + 1, len(configurations), config_i)), end='')

        channels = actuate_random_channels(config_i['n_channels'])
        proxy.setAveraging(config_i['averaging'], 0)
        proxy.setConversionSpeed(config_i['conversion_speed'], 0)
        proxy.setSamplingSpeed(config_i['sampling_speed'], 0)
        
        result_i = config_i.copy()
        
        data_i = []
        
        for j in range(n):
            start_j = time.time()
            data_j = np.array(read_f(config_i['n_samples']))
            end_j = time.time()
            data_i.append({'start': start_j, 'end': end_j, 'data': data_j})
        result_i['readings'] = data_i
        results.append(result_i)
    return results

# %% {"ExecuteTime": {"start_time": "2018-12-07T18:01:56.427000Z", "end_time": "2018-12-07T18:32:33.363000Z"}}
# proxy.setReference(teensy.ADC_REF_1V2, 0)
# proxy.enablePGA(0, 0)
# proxy.analog_wait_for_calibration(0)
# time.sleep(.1)
# read_f = ft.partial(proxy.differential_reads_simple, teensy.A10, teensy.A11)

# results = measure_configs(configurations, read_f)


proxy.setReference(teensy.ADC_REF_1V2, 0)
proxy.enablePGA(0, 0)
proxy.analog_wait_for_calibration(0)
time.sleep(.1)
read_f = ft.partial(proxy.analog_reads_simple, 11)

# config_i = {'n_channels': 50,
#             'averaging': 4,
#             'conversion_speed': 2,
#             'sampling_speed': 1}

# channels = actuate_random_channels(config_i['n_channels'])
# proxy.setAveraging(config_i['averaging'], 0)
# proxy.setConversionSpeed(config_i['conversion_speed'], 0)
# proxy.setSamplingSpeed(config_i['sampling_speed'], 0)

# data = pd.Series(read_f(50))
# ax = data.plot()
# ax.set_ylim(0, 2 ** 16)
results = measure_configs(configurations, read_f)

# %% {"ExecuteTime": {"start_time": "2018-12-07T15:16:21.530000Z", "end_time": "2018-12-07T15:16:21.594000Z"}}
df_i.head()

# %% {"ExecuteTime": {"start_time": "2018-12-07T18:40:29.027000Z", "end_time": "2018-12-07T18:40:46.195000Z"}}
import json_tricks


with open('exp(adc-config-00) single-ended 1.2V.json', 'w') as output:
    json_tricks.dump(results, output)

# %% {"ExecuteTime": {"start_time": "2018-12-07T15:21:29.564000Z", "end_time": "2018-12-07T15:21:29.870000Z"}}
result_i = results[900].copy()
readings_i = result_i.pop('readings')
df_i = pd.concat([pd.Series(r['data']) for r in readings_i],
                 keys=range(len(readings_i))).unstack(level=0)
df_i[0].plot()
result_i

# %%
import json_tricks

# %%
results_d = []

for k, v in results.items():
    result_i = dict(k)
    result_i['readings'] = v.values()
    for d in result_i['readings']:
        d['data'] = d['data'].values.tolist()
    results_d.append(result_i)

# %%
len(results_d[0]['readings'])

# %%
import cPickle as pickle

with open('exp(adc-config-00) differential.pickle', 'w') as output:
    pickle.dump(results_d, output)

# %%
results_d[0]['readings']

# %%
with open('exp(adc-config-00) differential.json', 'w') as output:
    json_tricks.dump(results_d, output)

# %%
# pd.Series(1, index=pd.Index(list(results.keys()[0])))
# df_i = results.values()[0]
# keys_i = results.keys()[0]

# for k, v in keys_i:
#     df_i[k] = v

# %%
ax = ((df_summary[[('differential', '75%'), ('single-ended', '75%')]] -
       df_summary[[('differential', '25%'), ('single-ended', '25%')]].values) / 1.2).boxplot()
# ax.set_ylim(0, 1)

# %%
%timeit proxy.setReference(teensy.ADC_REF_3V3, 0); proxy.analog_wait_for_calibration(0)

# %%
proxy.terminate()

# %% {"ExecuteTime": {"start_time": "2018-12-06T22:14:47.125000Z", "end_time": "2018-12-06T22:14:47.150000Z"}}
proxy.update_state(output_current_limit=.09)
# proxy.state.output_current_limit
# proxy.state

# %%
mask = pd.Series(proxy.disabled_channels_mask)
mask[mask > 0].to_frame().T

# %%
# pd.Series(proxy.analog_reads_simple(11, 50)).plot()
proxy.setReference(teensy.ADC_REF_1V2, 0)
pd.Series(proxy.differential_reads_simple(teensy.A11, teensy.A1, 50)).plot()
