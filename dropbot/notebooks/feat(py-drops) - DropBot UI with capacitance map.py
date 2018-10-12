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

from asyncio_helpers import cancellable
import blinker
import trollius as asyncio

signals = blinker.Namespace()

proxy = None

@asyncio.coroutine
def dump(*args, **kwargs):
    print('args=`%s`, kwargs=`%s`' % (args, kwargs))
    
@asyncio.coroutine
def on_connected(*args, **kwargs):
    global proxy
    
    proxy = kwargs['dropbot']
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
    
@asyncio.coroutine
def on_disconnected(*args, **kwargs):
    global proxy
    
    proxy = None
    
@asyncio.coroutine
def ignore(*args, **kwargs):
    raise asyncio.Return('ignore')

signals.signal('chip-inserted').connect(dump, weak=False)
signals.signal('connected').connect(on_connected, weak=False)
signals.signal('disconnected').connect(on_disconnected, weak=False)
signals.signal('version-mismatch').connect(ignore, weak=False)

monitor_task = cancellable(db.monitor.monitor)
thread = threading.Thread(target=monitor_task, args=(signals, ))
thread.daemon = True
thread.start()

# %% {"ExecuteTime": {"start_time": "2018-04-13T18:31:23.622000Z", "end_time": "2018-04-13T18:31:24.858000Z"}}
%matplotlib notebook
import functools as ft
import blinker
import dropbot as db
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches
import matplotlib.collections
import svg_model
import threading
import traceback
import si_prefix as si
import time


display_channels = True
svg_path = db.DATA_DIR.joinpath('SCI-BOTS 90-pin array', 'device.svg')
df_shapes = svg_model.svg_shapes_to_df(svg_path)

electrode_channels = df_shapes.drop_duplicates(['id', 'data-channels']).set_index('id')['data-channels'].map(int)
channel_electrodes = pd.Series(electrode_channels.index, index=electrode_channels.values)
# Do not include edge electrodes in capacitance measurements.
# XXX Should probaaly add the edge electrodes to disabled channel mask.
channels = (electrode_channels.drop(['electrode%03d' % i
                                     for i in [57, 58, 73, 74]])
            .drop_duplicates().sort_values())

patches = {id_: mpl.patches.Polygon(df_shape_i[['x', 'y']].values,
                                    closed=False, label=id_)
           for id_, df_shape_i in df_shapes.groupby('id')}

fig, ax = plt.subplots(1, 1, figsize=(10, 10))
ax.set_aspect(True)

electrode_capacitances = pd.Series([10e-12] * len(patches.keys()),
                                   index=patches.keys())

my_cmap = cm.get_cmap('Blues') # or any other one
norm_capacitance = mpl.colors.Normalize(0, 20e-12)  #electrode_capacitances.max())

state_of_channels = proxy.state_of_channels


def electrode_state(electrode_id):
    if electrode_id in electrode_channels:
        channel = electrode_channels.loc[electrode_id]
        electrode_state_i = state_of_channels[channel]
        if isinstance(channel, pd.Series):
            electrode_state_i = electrode_state_i.any()
        return electrode_state_i
    
    
def render_electrode(electrode_id, capacitance=None):
    patch_i = patches[electrode_id]
    if capacitance is not None:
        patch_i.set_facecolor(my_cmap(norm_capacitance(capacitance_i)))
    electrode_state_ = electrode_state(electrode_id)
    patch_i.set_alpha(1. if electrode_state_ else .6)
    patch_i.set_edgecolor('black' if electrode_state_ else None)
        

for id_i, patch_i in patches.items():
    electrode_id = patch_i.get_label()
    electrode_state_i = electrode_state(id_i)
    capacitance_i = (0 if electrode_state_i is None
                     else electrode_capacitances.loc[electrode_id])
    render_electrode(electrode_id, capacitance_i)
    ax.add_patch(patch_i)
    patch_i.set_picker(True)

# Compute center `(x, y)` for each electrode.
electrode_centers = df_shapes.groupby('id')[['x', 'y']].mean()
# Index by **channel number** instead of **electrode id**.
electrode_centers.index = electrode_channels.reindex(electrode_centers
                                                     .index)
    
if display_channels:
    for channel_i, center_i in electrode_centers.iterrows():
        ax.text(center_i.x, center_i.y, str(channel_i),
                horizontalalignment='center', verticalalignment='center',
                color='black', fontsize=9, bbox={'facecolor': 'white',
                                                 'alpha': 0.8, 'pad': 1,
                                                 'edgecolor': 'none'})

ax.set_xlim(df_shapes.x.min(), df_shapes.x.max())
ax.set_ylim(df_shapes.y.max(), df_shapes.y.min())

# connection_id = fig.canvas.mpl_connect('button_press_event', onclick)
picks = []


def on_channels_updated(event):
    try:
        actuated_channels = set(event['actuated'])
        state_of_channels[:] = 0
        state_of_channels[list(actuated_channels)] = 1

        for channel_i, electrode_id_i in channel_electrodes.items():
            render_electrode(electrode_id_i)
    except Exception as exception:
        text_capacitance.set_text(str(actuated_channels) + '\n' + traceback.format_exc(exception))
        

def on_capacitance_updated(event):
    text_capacitance.set_text('capacitance: {}F'.format(si.si_format(event['new_value'])))
    fig.canvas.draw_idle()
    
    
def on_channel_capacitances(capacitances):
    electrode_capacitances[:] = capacitances.reindex(electrode_channels)
    electrode_capacitances.index = electrode_channels.index
    
    try:
        for electrode_id_i, patch_i in patches.items():
            capacitance_i = electrode_capacitances.loc[electrode_id_i]
#             norm_capacitance.vmax = electrode_capacitances.max()
            color_i = my_cmap(norm_capacitance(capacitance_i))
            patch_i.set_facecolor(color_i)
        fig.canvas.draw_idle()
    except Exception as exception:
        text_capacitance.set_text(traceback.format_exc(exception))


def on_drops_detected(event):
    drops_events.append(event)
    if not event['drops']['channels']:
        return
    drops_channels = np.concatenate(event['drops']['channels'])
    drops_capacitances = np.concatenate(event['drops']['capacitances'])
    
    channel_capacitances = pd.Series(drops_capacitances, index=drops_channels)

    electrode_capacitances[:] = channel_capacitances.reindex(electrode_channels)
    electrode_capacitances.index = electrode_channels.index
    
    try:
        for electrode_id_i, patch_i in patches.items():
            capacitance_i = electrode_capacitances.loc[electrode_id_i]
            color_i = my_cmap(norm_capacitance(capacitance_i))
            patch_i.set_facecolor(color_i)
        fig.canvas.draw_idle()
    except Exception as exception:
        text_capacitance.set_text(traceback.format_exc(exception))


signals = blinker.Namespace()
for signal in ('onpick', 'channel-capacitances'):
    for key, receiver in signals.signal(signal).receivers.items():
        signal.disconnect(receiver)
signals.signal('onpick').connect(picks.append, weak=False)
signals.signal('channel-capacitances').connect(on_channel_capacitances, weak=False)

drops_events = []
proxy.signals.clear()
proxy.signals.signal('drops-detected').connect(on_drops_detected, weak=False)
channels_signal = proxy.signals.signal('channels-updated')
# Attach channels update listener
channels_signal.connect(on_channels_updated, weak=False)

capacitance_signal = proxy.signals.signal('capacitance-updated')
# Attach capacitance update listener
capacitance_signal.connect(on_capacitance_updated, weak=False)


@cancellable
@asyncio.coroutine
def logger():
    try:
        while True:
            yield asyncio.From(asyncio.sleep(.01))
    except asyncio.CancelledError:
        pass
        

def on_pick(event):
    if event.mouseevent.button != 1:
        return
    signals.signal('onpick').send(event)
    electrode_id = event.artist.get_label()
    
    try:
        if electrode_id in electrode_channels:
            channel_i = electrode_channels[electrode_id]
            electrode_state_i = not state_of_channels[channel_i].any()
            state_of_channels[channel_i] = electrode_state_i 
        else:
            channel_i = None
            electrode_state_i = 0
    except Exception as exception:
        text.set_text('onpick: {}'.format(traceback.format_exc(exception)))
        text.set_text('onpick: {}'.format((event.artist.get_label(), channel_i)))
    proxy.state_of_channels = state_of_channels
    
scroll_events = []
    
def on_scroll(*args, **kwargs):
    message = 'args=`%s`, kwargs=`%s`' % (args, kwargs)
    text.set_text('onscroll 1: {}'.format(message))
    print(message)
    text.set_text('onscroll 2: {}'.format(message))
    scroll_events.append(args[0])
    

text = ax.text(0,0, "", va="bottom", ha="left")
text_capacitance = ax.text(0, 0, "", va="bottom", ha="left", transform=ax.transAxes)

logger_thread = threading.Thread(target=logger)
logger_thread.daemon = True
logger_thread.start()

fig.canvas.mpl_connect('pick_event', on_pick)
# fig.canvas.mpl_connect('pick_event', lambda *args, **kwargs:
#                        logger.started.loop.call_soon_threadsafe(ft.partial(on_scroll, *args, **kwargs)))
fig.canvas.mpl_connect('scroll_event', lambda *args, **kwargs:
                       logger.started.loop.call_soon_threadsafe(ft.partial(on_scroll, *args, **kwargs)))

# %%
# proxy.set_switching_matrix_row?

# %%
proxy.set_sensitive_channels([25, 29, 33])
proxy.set_duty_cycle(0, [29])
proxy.set_duty_cycle(1, [25, 33])
proxy.start_switching_matrix(20, .006)

# %%
proxy.state

# %%
proxy.stop_switching_matrix()
channels = [29]
# channels = [25, 29, 33]
# channels = [24, 25, 33, 32]
proxy.set_sensitive_channels(channels)
proxy.set_duty_cycle(1, channels)
proxy.start_switching_matrix(30, .006)

# %%
proxy.ram_free()

# %%
pick_i = picks[-1]

# %%
logger.cancel()

# %%


# %% {"ExecuteTime": {"start_time": "2018-04-13T18:30:49.504000Z", "end_time": "2018-04-13T18:30:49.523000Z"}}
proxy.stop_switching_matrix()

# %% {"ExecuteTime": {"start_time": "2018-04-13T18:30:50.955000Z", "end_time": "2018-04-13T18:30:50.975000Z"}}
# proxy.update_state(drops_update_interval_ms=int(0))
proxy.update_state(capacitance_update_interval_ms=500)

# %% [markdown]
# -----------------------------------------------------------------------------------------------

# %% [markdown]
# # Close DropBot connection

# %% {"ExecuteTime": {"start_time": "2018-04-13T18:30:53.208000Z", "end_time": "2018-04-13T18:30:53.214000Z"}}
proxy.terminate()

# %% [markdown]
# -----------------------------------------------------------------------------------------------
