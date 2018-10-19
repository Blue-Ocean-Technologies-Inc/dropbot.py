# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:percent
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
#       height: 137px
#       left: 1097.97px
#       top: 142.597px
#       width: 288px
#     toc_section_display: true
#     toc_window_display: true
# ---

# %%
import threading
import gobject
import gtk

gtk.gdk.threads_init()

from matplotlib.backends.backend_gtkagg import FigureCanvas
import matplotlib as mpl
import matplotlib.pyplot

fig, axis = mpl.pyplot.subplots(1)
canvas = {}


def gtk_thread(fig):
    window = gtk.Window()
    canvas['canvas'] = FigureCanvas(fig)
    window.add(canvas['canvas'])
    window.connect('delete-event', lambda *args: gtk.main_quit())
    window.connect('destroy-event', lambda *args: gtk.main_quit())
    window.show_all()
    gtk.main()


def refresh():
    gobject.idle_add(canvas['canvas'].draw_idle)

thread = threading.Thread(target=gtk_thread, args=(fig, ))
thread.daemon = True
thread.start()

# %% {"ExecuteTime": {"start_time": "2018-04-13T18:31:08.926000Z", "end_time": "2018-04-13T18:31:12.742000Z"}}
from __future__ import (absolute_import, print_function, unicode_literals,
                        division)

import logging
logging.basicConfig(level=logging.DEBUG)

import joblib
import dropbot as db
import dropbot.chip
import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.ticker
import numpy as np
import pandas as pd
import si_prefix as si

# %matplotlib inline

F_formatter = mpl.ticker.FuncFormatter(lambda x, *args: si.si_format(x) + 'F')

# Load Sci-Bots device file and extract neighbouring channels info.
svg_path = dropbot.DATA_DIR.joinpath('SCI-BOTS 90-pin array', 'device.svg')

# Used cached neighbours result (if available).  Otherwise, cache neighbours.
memcache = joblib.memory.Memory('.')
get_channel_neighbours = memcache.cache(db.chip.get_channel_neighbours)
neighbours = get_channel_neighbours(svg_path)

# %% [markdown]
# # Open DropBot connection

# %%
import dropbot.monitor

# %%
import threading
import time

from asyncio_helpers import cancellable
import blinker
import trollius as asyncio

signals = blinker.Namespace()

connected = threading.Event()
proxy = None
_debug_data = {}

@asyncio.coroutine
def dump(*args, **kwargs):
    print('args=`%s`, kwargs=`%s`' % (args, kwargs))

@asyncio.coroutine
def on_connected(*args, **kwargs):
    global proxy

    proxy = kwargs['dropbot']
    proxy.turn_off_all_channels()
    proxy.stop_switching_matrix()
    proxy.neighbours = neighbours

    proxy.enable_events()

    proxy.update_state(hv_output_enabled=True, hv_output_selected=True,
                       voltage=100, frequency=10e3)

    # Disable channels in contact with copper tape.
    disabled_channels_mask_i = proxy.disabled_channels_mask
    disabled_channels_mask_i[[89, 30]] = 1
    # Disable channels with no neighbours defined.
    neighbour_counts = neighbours.groupby(level='channel').count()
    disabled_channels_mask_i[neighbour_counts.loc[neighbour_counts < 1].index] = 1
    proxy.disabled_channels_mask = disabled_channels_mask_i
    
    def _on_sensitive_capacitances(message):
        '''
        Update bar plot showing measured capacitance for each sensitive channel.
        
        Parameters
        ----------
        message : dict
            Example message::
    
                {"event": "sensitive-capacitances",
                 "wall_time": ...,
                 "C": [[<channel>, <capacitance>], ...]}
        '''
        try:
            _debug_data['sensitive_capacitances'] = _debug_data.get('sensitive_capacitances', [])
            _debug_data['sensitive_capacitances'].append(message)
            _debug_data['sensitive_capacitances'] = _debug_data['sensitive_capacitances'][1:6]
            channels, capacitances = zip(*message['C'])
            capacitances = pd.Series(capacitances, index=channels)
            axis.cla()
            capacitances.plot(kind='bar', ax=axis)
            axis.set_ylim(0, max(30e-12, capacitances.max()))
            axis.yaxis.set_major_formatter(F_formatter)
            fig.canvas.draw()
        except Exception:
            logging.debug('sensitive capacitances event error.', exc_info=True)
            return

    proxy.signals.signal('sensitive-capacitances').connect(_on_sensitive_capacitances, weak=False)

    connected.set()

@asyncio.coroutine
def on_disconnected(*args, **kwargs):
    connected.clear()
    global proxy

    proxy = None

@asyncio.coroutine
def ignore(*args, **kwargs):
    raise asyncio.Return('ignore')

def connect(*args):
    global signals
    global monitor_task 
    
    signals.clear()
    signals.signal('chip-inserted').connect(dump, weak=False)
    signals.signal('connected').connect(on_connected, weak=False)
    signals.signal('disconnected').connect(on_disconnected, weak=False)
    signals.signal('version-mismatch').connect(ignore, weak=False)
    
    monitor_task = cancellable(db.monitor.monitor)
    thread = threading.Thread(target=monitor_task, args=(signals, ))
    thread.daemon = True
    thread.start()
    
    while not connected.wait(1):
        pass

connect()

import ipywidgets as ipw


def close(*args):
    connected.clear()
    proxy.stop_switching_matrix()
    proxy.update_state(drops_update_interval_ms=int(0))
    time.sleep(1.)
    monitor_task.cancel()

buttons = {'disconnect': ipw.Button(description='Disconnect'),
           'connect': ipw.Button(description='Connect')}
buttons['disconnect'].on_click(close)
buttons['connect'].on_click(connect)
ipw.HBox(buttons.values())

# %%
_debug_data

# %%
# proxy.turn_off_all_channels()
# proxy.state_of_channels = pd.Series(1, index=[17, 22, 18, 16])
# proxy.state_of_channels = pd.Series(1, index=[18, 16, 25])
# proxy.state_of_channels = pd.Series(1, index=[16])
# proxy.turn_off_all_channels()

# %% [markdown]
# ## Identify all electrodes connected to a single drop

# %%
# proxy.refresh_drops(0)
disabled_mask = pd.Series(proxy.disabled_channels_mask)
channels = disabled_mask[disabled_mask < 1].index
drops = proxy.get_drops()

if drops:
    selection = None
    while selection not in range(len(drops)):
        display(list(enumerate(drops)))
        selection = raw_input('Select drop (blank to cancel): ').strip()
        if not selection:
            raise RuntimeError('No drop selected.')
        try:
            drop_i = drops[int(selection)]
            break
        except Exception:
            logging.debug('Invalid selection', exc_info=True)

# %% [markdown]
# ## Measure combined capacitance of all electrodes

# %%
proxy.stop_switching_matrix()
proxy.set_sensitive_channels(drop_i)
proxy.set_duty_cycle(0, drop_i)
proxy.resume_switching_matrix()
time.sleep(.35)

def measure_sensitive():
    return pd.Series(proxy.y(), index=proxy.sensitive_channels)

drop_capacitance_i = measure_sensitive()
display(drop_capacitance_i.map(si.si_format))
si.si_format(drop_capacitance_i.sum())

# %% [markdown]
# ## Select middle electrode(s)

# %%
selection = None
while selection not in range(len(drop_i)):
    channels_i = pd.Series(drop_i)
    display(channels_i)
    selection = raw_input('Select **middle** channel (blank to cancel): ').strip()
    if not selection:
        raise RuntimeError('No middle channel selected.')
    try:
        middle_i = [channels_i[int(selection)]]
        break
    except Exception:
        logging.debug('Invalid selection', exc_info=True)

# %%
drop_i, middle_i

# %% [markdown]
# ## Starting from **middle** electrode(s), add _left_ neighbour to **a** group, _right_ neighbour to **b** group

# %%
middle_neighbours_i = proxy.neighbours.loc[middle_i, ['left', 'right']].reset_index(level=0, drop=True)
middle_neighbours_i = middle_neighbours_i.loc[~middle_neighbours_i.isin(middle_i)].dropna().astype(int)
a_i = [middle_neighbours_i.loc['left']]
b_i = [middle_neighbours_i.loc['right']]
a_i, b_i

# %% [markdown]
# ## Set **a**, **b** and **middle** electrodes to maximum duty cycle.

# %%
drop_channels_i = a_i + middle_i + b_i
proxy.set_sensitive_channels(drop_channels_i)
proxy.set_sensitive_channels(middle_i)
# proxy.set_duty_cycle(1, drop_channels_i)
# proxy.start_switching_matrix(row_count, .005)
proxy.resume_switching_matrix()

# %% [markdown]
# ## Center drop by alternating between `(a, middle)` and `(middle, b)` electrodes.  _PID control may be used to adjust duty cycle of active group._

# %%
import functools as ft


def pulse_channels(channels):
    proxy.turn_off_all_channels()
    proxy.state_of_channels = pd.Series(1, index=channels)
    time.sleep(.025)
    proxy.turn_off_all_channels()

a_middle = ipw.Button(description='a + middle')
b_middle = ipw.Button(description='b + middle')

proxy.stop_switching_matrix()
a_middle.on_click(lambda *args: ft.partial(pulse_channels, a_i + middle_i)())
b_middle.on_click(lambda *args: ft.partial(pulse_channels, b_i + middle_i)())
ipw.HBox([a_middle, b_middle])

# %% [markdown]
# ### Map range `[-1, 1]` to duty cycles `(1, 0)` to `(0, 1)`
#
# Ideally, during a split, both **a** and **b** groups would have maximum duty cycle.  However, for various reasons (e.g., surface imperfections), the same duty cycle applied to **a** and **b** _MAY_ not result in equal distribution.
#
# This can be addressed by mapping
#
# ```
#   -1               0                1
# (1, 0) --------- (1, 1) --------- (0, 1)
# ```

# %%
def output_duty_cycles(normalized_force):
    a = 1
    b = 1 - abs(normalized_force)
    if normalized_force > 0:
        a, b = b, a
    return a, b

# %%
forces = [-1, -.5, 0, .5, 1]
pd.Series(forces, index=forces).map(output_duty_cycles)

# %% [markdown]
# ## Find _extended_ neighbours of **a** and **b**

# %%
a_neighbours_i = proxy.neighbours.loc[a_i, ['left']].reset_index(level=0, drop=True)
a_neighbours_i = a_neighbours_i.loc[~a_neighbours_i.isin(a_i)].astype(int).tolist()
b_neighbours_i = proxy.neighbours.loc[b_i, ['right']].reset_index(level=0, drop=True)
b_neighbours_i = b_neighbours_i.loc[~b_neighbours_i.isin(b_i)].astype(int).tolist()
a_neighbours_i, b_neighbours_i

# %% [markdown]
# ## Set **middle** duty cycle to 0; init **a** and **b** to 1; and adjust force ratio and amplitude to split

# %%
min_voltage = 90
max_voltage = 125

def apply_duty_cycles(duty_cycles, set_sensitive=True):
    '''
    Apply duty cycle to each channel; optionally set respective channels as sensitive.
    
    Parameters
    ----------
    duty_cycles : pandas.Series
        Duty cycles indexed by sensitive channel number.
    '''
    proxy.stop_switching_matrix()
    channels = duty_cycles.index
    for d_i in duty_cycles.unique():
        channels_i = duty_cycles[duty_cycles == d_i].index
        proxy.set_duty_cycle(d_i, channels_i)
    if set_sensitive:
        proxy.set_sensitive_channels(channels)
    proxy.resume_switching_matrix()
    
def output_voltage(normalized_amplitude):
    range_ = max_voltage - min_voltage
    normalized_amplitude = min(1, max(0, normalized_amplitude))
    return min_voltage + normalized_amplitude * range_

def on_force_change(message):
    proxy.stop_switching_matrix()
    a_duty_i, b_duty_i = output_duty_cycles(message['new'])
    duty_cycles = pd.concat([pd.Series(a_duty_i, index=a_i + a_neighbours_i),
                             pd.Series(b_duty_i, index=b_i + b_neighbours_i)])
    # XXX Use existing sensitive channels, which include **middle** channel(s).
    apply_duty_cycles(duty_cycles, set_sensitive=False)
    proxy.resume_switching_matrix()
    
def on_amplitude_change(message):
    voltage = output_voltage(message['new'])
    proxy.voltage = voltage
    print('\r%-50s' % ('voltage: %sV' % si.si_format(voltage)), end='')

proxy.stop_switching_matrix()
drop_channels_i = a_neighbours_i + a_i + middle_i + b_i + b_neighbours_i
apply_duty_cycles(pd.Series(0, index=drop_channels_i))
    
force_slider = ipw.FloatSlider(description='Force:', min=-1, max=1, value=0)
force_slider.observe(on_force_change, names=['value'])
on_force_change({'new': 0})

amplitude_slider = ipw.FloatSlider(description='Amplitude:', min=0, max=1, value=0)
amplitude_slider.observe(on_amplitude_change, names=['value'])
on_amplitude_change({'new': 0})

ipw.VBox([force_slider, amplitude_slider])

# %% [markdown]
# ## Re-merge drop

# %%
# Create button to turn on _only_ the channels associated with **a** and **b** groups, i.e.,
# _not_ the **middle** group or the extended neighbours of **a** and **b**.
button_finalize_split = ipw.Button(description='Finalize split')
button_finalize_split.on_click(lambda *args: apply_duty_cycles(pd.Series(1, index=a_i + b_i)))

# Create button to merge the daughter drops in **a** and **b** back together across the
# **middle** electrode(s).
button_merge = ipw.Button(description='Merge')
button_merge.on_click(lambda *args:
                      apply_duty_cycles(pd.concat([pd.Series(.1, index=a_i + b_i),
                                                   pd.Series(1, index=middle_i)])))

ipw.HBox([button_finalize_split, button_merge])
