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
#     sideBar: false
#     skip_h1_title: false
#     title_cell: Table of Contents
#     title_sidebar: Contents
#     toc_cell: false
#     toc_position:
#       height: 229px
#       left: 787px
#       top: 142px
#       width: 257px
#     toc_section_display: true
#     toc_window_display: true
# ---

# %% [markdown]
# # Initialize DropBot connection

# %% {"ExecuteTime": {"start_time": "2018-12-11T20:45:17.174000Z", "end_time": "2018-12-11T20:45:18.085000Z"}}
from __future__ import print_function, unicode_literals, division
import logging
import time
logging.basicConfig(level=logging.DEBUG)

import arrow
# Access Teensy header constants, e.g., `A10`, `A11`, etc.
import arduino_helpers.hardware.teensy as teensy
import dropbot as db
import dropbot.analog.config as ac
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

# %% [markdown]
# # Measure temperature regardless of ADC reference voltage

# %% {"ExecuteTime": {"start_time": "2018-12-11T20:45:19.950000Z", "end_time": "2018-12-11T20:45:20.099000Z"}}
proxy.setReference(0, 0)
display(ac._adc_config(proxy))
display(proxy.measure_temperature())
proxy.setReference(1, 0)
display(ac._adc_config(proxy))
display(proxy.measure_temperature())

# %% [markdown]
# # Measure high voltage regardless of ADC reference voltage

# %% {"ExecuteTime": {"start_time": "2018-12-11T20:45:21.477000Z", "end_time": "2018-12-11T20:45:21.999000Z"}}
proxy.update_state(hv_output_enabled=True)
proxy.setReference(0, 0)
display(ac._adc_config(proxy))
display(pd.Series([proxy.high_voltage() for i in range(10)]).mean())
proxy.setReference(1, 0)
display(ac._adc_config(proxy))
display(pd.Series([proxy.high_voltage() for i in range(10)]).mean())
# proxy.setSamplingSpeed(ac.SAMPLING_SPEEDS['VERY_LOW_SPEED'], 0)
# proxy.setConversionSpeed(ac.CONVERSION_SPEEDS['HI_SPEED'], 0)
# proxy.setAveraging(4, 0)

# %% [markdown]
# # Plot distribution of repeated capacitance measurements

# %% {"ExecuteTime": {"start_time": "2018-12-11T20:45:23.663000Z", "end_time": "2018-12-11T20:45:23.679000Z"}}
proxy.enablePGA(2, 0)

# %% {"ExecuteTime": {"start_time": "2018-12-11T20:56:19.956000Z", "end_time": "2018-12-11T20:56:22.224000Z"}}
import matplotlib as mpl

F_formatter = mpl.ticker.FuncFormatter(lambda x, args: '%sF' % si.si_format(x))

time.sleep(.25)
display(proxy.state.to_frame().T)
proxy.update_state(hv_output_enabled=True)
channels = actuate_random_channels(40)
n_channels = channels[channels > 0].shape[0]
d = pd.Series([proxy.capacitance(25) for i in range(100)])
ax = d.plot(kind='box')
ax.yaxis.set_major_formatter(F_formatter)
std_, mean_, _25_, _50_, _75_ = d.describe()[['std', 'mean', '25%', '50%', '75%']]
IQR = _75_ - _25_
print('%.2f%%' % (100 * std_ / _50_))
d_ = d[(d >= _25_ - 1.5 * IQR) & (d <= _75_ + 1.5 * IQR)]
std_, mean_, _25_, _50_, _75_ = d_.describe()[['std', 'mean', '25%', '50%', '75%']]
print('%.2f%%' % (100 * std_ / _50_))
mpl.pyplot.figure()
d.plot()
si.si_format(_50_ / (n_channels if n_channels >= 1 else 1))

# %%


# %% {"ExecuteTime": {"start_time": "2018-12-11T20:56:52.366000Z", "end_time": "2018-12-11T20:56:52.672000Z"}}
proxy.setSamplingSpeed(ac.SAMPLING_SPEEDS['VERY_LOW_SPEED'], 0)
proxy.setConversionSpeed(ac.CONVERSION_SPEEDS['HI_SPEED'], 0)
proxy.setAveraging(0, 0)
proxy.setReference(teensy.ADC_REF_1V2, 0)
d = pd.Series(proxy.differential_reads_simple(teensy.A10, teensy.A11, 20))
ax = d.plot()
ax.set_ylim(-(2 ** 15), 2 ** 15)

# %% {"ExecuteTime": {"start_time": "2018-12-11T20:57:39.707000Z", "end_time": "2018-12-11T20:57:40.752000Z"}}
proxy.setSamplingSpeed(ac.SAMPLING_SPEEDS['VERY_LOW_SPEED'], 0)
proxy.setConversionSpeed(ac.CONVERSION_SPEEDS['HI_SPEED'], 0)
proxy.setAveraging(0, 0)
proxy.setReference(teensy.ADC_REF_1V2, 0)
d = pd.Series([proxy.analogReadDifferential(teensy.A10, teensy.A11, 0) / 2
               for i in range(100)])
ax = d.plot()
ax.set_ylim(-(2 ** 15), 2 ** 15)

# %%

