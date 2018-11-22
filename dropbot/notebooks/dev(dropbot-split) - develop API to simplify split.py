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

# thread = threading.Thread(target=gtk_thread, args=(fig, ))
# thread.daemon = True
# thread.start()

# %%
from __future__ import (absolute_import, print_function, unicode_literals,
                        division)
import datetime as dt
import logging
import lxml
import re
logging.basicConfig(level=logging.DEBUG)

import dropbot as db
import dropbot.chip
import joblib
import json_tricks
import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.ticker
import networkx as nx
import numpy as np
import pandas as pd
import pint
import semantic_version
import si_prefix as si
import svg_model
import svg_model.data_frame as sdf

# %matplotlib inline

F_formatter = mpl.ticker.FuncFormatter(lambda x, *args: si.si_format(x) + 'F')

# Load Sci-Bots device file and extract neighbouring channels info.
svg_path = dropbot.DATA_DIR.joinpath('SCI-BOTS 90-pin array', 'device.svg')

# Used cached neighbours result (if available).  Otherwise, cache neighbours.
memcache = joblib.memory.Memory('.')
get_channel_neighbours = memcache.cache(db.chip.get_channel_neighbours)
neighbours = get_channel_neighbours(svg_path)

ureg = pint.UnitRegistry()
root = lxml.etree.parse(svg_path)
namespaces = svg_model.NSMAP
namespaces.update(svg_model.INKSCAPE_NSMAP)
inkscape_version = root.xpath('/svg:svg/@inkscape:version', namespaces=namespaces)[0]

# See http://wiki.inkscape.org/wiki/index.php/Release_notes/0.92#Important_changes
pixel_density = (96 if inkscape_version >= semantic_version.Version('0.92.0') else 90) * ureg.PPI

df_shapes = svg_model.svg_shapes_to_df(svg_path)
df_shapes.loc[:, ['x', 'y']] = (df_shapes.loc[:, ['x', 'y']].values * ureg.pixel / pixel_density).to('mm')
df_shape_infos = sdf.get_shape_infos(df_shapes, 'id')

electrodes_by_channel = \
    pd.concat(pd.Series(p.attrib['id'],
                        index=map(int, re.split(r',\s*',
                                                p.attrib['data-channels'])))
              for p in root.xpath('//svg:g[@inkscape:label="Device"]'
                                  '/svg:path[not(@data-channels="")]',
                                  namespaces=svg_model.INKSCAPE_NSMAP))
electrodes_by_channel.sort_index(inplace=True)
channels_by_electrode = pd.Series(electrodes_by_channel.index, index=electrodes_by_channel.values)
channels_by_electrode.sort_index(inplace=True)
channel_areas = df_shape_infos.loc[channels_by_electrode.index, 'area']
channel_areas.index = channels_by_electrode
channel_areas.name = 'area (mm^2)'

G = nx.Graph()
adjacency_list = neighbours.dropna().to_frame().reset_index(level=0).astype(int).values
adjacency_list.sort(axis=1)
G.add_edges_from(map(tuple, pd.DataFrame(adjacency_list).drop_duplicates().values))


def shortest_path(source, target):
    return nx.shortest_path(G, source, target)[1::]

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

    def plot_message(capacitances):
        axis.cla()
        capacitances.plot(kind='bar', ax=axis)
        axis.set_ylim(0, max(30e-12, capacitances.max()))
        axis.yaxis.set_major_formatter(F_formatter)
        fig.canvas.draw()

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
            while len(_debug_data['sensitive_capacitances']) > 5:
                _debug_data['sensitive_capacitances'].pop(0)
            channels, capacitances = zip(*message['C'])
            capacitances = pd.Series(capacitances, index=channels)
            gobject.idle_add(plot_message, capacitances)
        except Exception:
            logging.debug('sensitive capacitances event error.', exc_info=True)
            return

#     proxy.signals.signal('sensitive-capacitances').connect(_on_sensitive_capacitances, weak=False)

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
def apply_duty_cycles(proxy, duty_cycles, set_sensitive=True):
    '''
    Apply duty cycle to each channel; optionally set respective channels as sensitive.

    Parameters
    ----------
    duty_cycles : pandas.Series
        Duty cycles indexed by sensitive channel number.
    set_sensitive : bool, optional
        Configure switching matrix controller with indexed channels in ``duty_cycles`` as
        the active sensitive channels **(clears any existing sensitive channels)**.
    '''
    proxy.stop_switching_matrix()
    channels = duty_cycles.index
    if set_sensitive:
        proxy.set_sensitive_channels(channels)
    duty_cycles.clip(lower=0, upper=1)
    for d_i in duty_cycles.unique():
        channels_i = duty_cycles[duty_cycles == d_i].index
        proxy.set_duty_cycle(d_i, channels_i)
    proxy.resume_switching_matrix()

# %%
EPSILON = 5e-12

# %%
proxy.stop_switching_matrix()

proxy.set_sensitive_channels([])

# %%
import collections


@asyncio.coroutine
def move_to(target, epsilon=EPSILON):
    proxy.stop_switching_matrix()
    proxy.set_sensitive_channels([])

    state = {'i': 0}
    move_done = asyncio.Event()
    loop = asyncio.get_event_loop()

    if not isinstance(target, collections.Iterable):
        target = [target]

    head = [target[-1]]
    tail = [target[0]]

    def _on_sensitive_capacitances(message):
        try:
            channels, capacitances = zip(*message['C'])
            capacitances = pd.Series(capacitances, index=channels)
            if state['i'] == 0:
                state['start'] = time.time()
                state['C_0'] = capacitances
            if capacitances[head].sum() > epsilon:
                state['end'] = time.time()
                state['C_T'] = capacitances
                proxy.signals.signal('sensitive-capacitances').disconnect(_on_sensitive_capacitances)
                loop.call_soon_threadsafe(move_done.set)
            state['i'] += 1
        except Exception:
            proxy.signals.signal('sensitive-capacitances').disconnect(_on_sensitive_capacitances)
            logging.debug('sensitive capacitances event error.', exc_info=True)
            return

    try:
        proxy.signals.signal('sensitive-capacitances').connect(_on_sensitive_capacitances, weak=False)
        apply_duty_cycles(proxy, pd.concat([pd.Series(1, index=head),
                                            pd.Series(1, index=target),
                                            pd.Series(0, index=tail)]))
        yield asyncio.From(move_done.wait())
        raise asyncio.Return(state)
    finally:
        proxy.signals.signal('sensitive-capacitances').disconnect(_on_sensitive_capacitances)


@asyncio.coroutine
def read_C():
    read_done = asyncio.Event()
    loop = asyncio.get_event_loop()

    def _on_sensitive_capacitances(message):
        try:
            channels, capacitances = zip(*message['C'])
            capacitances = pd.Series(capacitances, index=channels)
            read_done.capacitances = capacitances
            loop.call_soon_threadsafe(read_done.set)
        except Exception:
            proxy.signals.signal('sensitive-capacitances').disconnect(_on_sensitive_capacitances)
            logging.debug('sensitive capacitances event error.', exc_info=True)
            return

    try:
        proxy.signals.signal('sensitive-capacitances').connect(_on_sensitive_capacitances, weak=False)
        yield asyncio.From(read_done.wait())
        raise asyncio.Return(read_done.capacitances)
    finally:
        proxy.signals.signal('sensitive-capacitances').disconnect(_on_sensitive_capacitances)


@asyncio.coroutine
def wait_on_capacitance(callback):
    move_done = asyncio.Event()
    loop = asyncio.get_event_loop()

    state = {'i': 0}
    def _on_sensitive_capacitances(message):
        try:
            channels, capacitances = zip(*message['C'])
            capacitances = pd.Series(capacitances, index=channels)
            if state['i'] == 0:
                state['C_0'] = capacitances
            if callback(capacitances):
                state['C_T'] = capacitances
                proxy.signals.signal('sensitive-capacitances').disconnect(_on_sensitive_capacitances)
                loop.call_soon_threadsafe(move_done.set)
            state['i'] += 1
        except Exception:
            proxy.signals.signal('sensitive-capacitances').disconnect(_on_sensitive_capacitances)
            logging.debug('sensitive capacitances event error.', exc_info=True)
            return

    try:
        proxy.signals.signal('sensitive-capacitances').connect(_on_sensitive_capacitances, weak=False)
        yield asyncio.From(move_done.wait())
        raise asyncio.Return(state)
    finally:
        proxy.signals.signal('sensitive-capacitances').disconnect(_on_sensitive_capacitances)

# %%
loop = asyncio.ProactorEventLoop()
asyncio.set_event_loop(loop)

# %%
proxy.voltage = 105

# %%


# %%
import ivPID

@asyncio.coroutine
def wait_for_split(proxy, theta, a_neighbours, a, neck, b, b_neighbours,
                   pid, neck_epsilon=2e-12):
    log_ = []
    pid.setPoint = .5

    loop = asyncio.get_event_loop()
    
    def apply_force(force):
        proxy.stop_switching_matrix()
        
        a_duty_cycle, b_duty_cycle = output_duty_cycles(force)
        a_duty_cycle = max(0, min(1, a_duty_cycle))
        b_duty_cycle = max(0, min(1, b_duty_cycle))
        duty_cycles = pd.concat([pd.Series(b_duty_cycle, index=b + b_neighbours),
                                 pd.Series(a_duty_cycle, index=a + a_neighbours)])
        # XXX Use existing sensitive channels, which include **middle** channel(s).
        apply_duty_cycles(proxy, duty_cycles, set_sensitive=False)
#         print('\r%-50s' % ('%-.03f <- -> %.03f' % (a_duty_cycle, b_duty_cycle)), end='')
        log_.append({'action': 'apply-force', 'duty_cycles': duty_cycles,
                     'a_duty_cycle': a_duty_cycle, 'b_duty_cycle': b_duty_cycle,
                     'time': time.time()})
    
    neck_breaks = []

    def on_capcacitances(capacitances):
        C_neck = capacitances[neck].sum() 
        if C_neck < neck_epsilon:
            neck_breaks.append(C_neck)
        else:
            del neck_breaks[:]
        if len(neck_breaks) >= 2:
            return True
        try:
            # XXX Set PID update based on difference between 
            C_a = capacitances[a_neighbours + a].sum()
            C_b = capacitances[b + b_neighbours].sum()
            pid.update(theta - (C_a / C_b))
            loop.call_soon_threadsafe(apply_force, pid.output)
        except AttributeError:
            logging.debug('Error', exc_info=True)

    state = yield asyncio.From(wait_on_capacitance(on_capcacitances))
    log_.append({'action': 'split-complete',
                 'state': state,
                 'args': 
                 dict(zip(('theta', 'a_neighbours', 'a', 'neck', 'b', 'b_neighbours'),
                          (theta, a_neighbours, a, neck, b, b_neighbours))),
                 'time': time.time()})
    raise asyncio.Return(state)

# %%
def test_split(proxy, theta, a_neighbours, a, neck, b, b_neighbours):
    # See https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
#     K_u = 5 # Determined empirically, consistent oscillations
    K_u = 2.5 # Determined empirically, consistent oscillations
    T_u = 2 * .365  # Oscilation period

    # pid = ivPID.PID(P=K_u)
    # pid = ivPID.PID(P=.5 * K_u)  # P
    # pid = ivPID.PID(P=.45 * K_u, I=T_u / 1.2)  # PI
    # pid = ivPID.PID(P=.8 * K_u, D=T_u / 8)  # PD
    # pid = ivPID.PID(P=.6 * K_u, I=T_u / 2, D=T_u / 8)  # ["classic" PID][1]
    # pid = ivPID.PID(P=.7 * K_u, I=T_u / 2.5, D=3 * T_u / 20)  # [Pessen Integral Rule][1]
    # pid = ivPID.PID(P=.33 * K_u, I=T_u / 2, D=T_u / 3)  # ["some" overshoot][1]
    # pid = ivPID.PID(P=.2 * K_u, I=T_u / 2, D=T_u / 3)  # ["no" overshoot][1]
    # [1]: http://www.mstarlabs.com/control/znrule.html

    # pid = ivPID.PID(P=K_u)  # Set for tuning; find minimum K_u where liquid oscillates consistently from one side to the other
    pid = ivPID.PID(P=.6 * K_u, I=T_u / 2, D=T_u / 8)  # ["classic" PID][1]
    # pid = ivPID.PID(P=.33 * K_u, I=T_u / 2, D=T_u / 3)  # ["some" overshoot][1]

    try:
        result = loop.run_until_complete(asyncio.wait_for(
            wait_for_split(proxy, theta, a_neighbours, a, neck, b,
                           b_neighbours, pid), 15))
    finally:
        apply_duty_cycles(proxy, pd.Series(0, index=a_neighbours + a + neck + b + b_neighbours))
        for r in proxy.signals.signal('sensitive-capacitances').receivers.values():
            proxy.signals.signal('sensitive-capacitances').disconnect(r)

    apply_duty_cycles(proxy, pd.Series(0.75, index=a_neighbours + a + b + b_neighbours))
    time.sleep(.5)
    capacitances = loop.run_until_complete(asyncio.wait_for(read_C(), 15))
    return capacitances

# %%
import functools as ft

def threshold(channels, capacitances, C_threshold=EPSILON):
    if all(c in capacitances for c in channels) and \
            capacitances[channels].sum() > C_threshold:
        return True
        

def align(a_neighbours, a, neck, b, b_neighbours, split_epsilon = 2 * EPSILON):
    apply_duty_cycles(proxy, pd.concat([pd.Series(1, index=neck),
                                        pd.Series(.25, index=b)]))

    loop.run_until_complete(wait_on_capacitance(ft.partial(threshold, b, C_threshold=split_epsilon)))

    apply_duty_cycles(proxy, pd.concat([pd.Series(1, index=neck),
                                        pd.Series(.25, index=a)]))

    loop.run_until_complete(wait_on_capacitance(ft.partial(threshold, a, C_threshold=split_epsilon)))

    apply_duty_cycles(proxy, pd.Series(0.25, index=a + neck + b))
    apply_duty_cycles(proxy, pd.Series(0, index=a_neighbours + a + neck + b + b_neighbours))

# %%
# a_neighbours, a, neck, b, b_neighbours = [101], [103], [94], [90], [86]
a_neighbours, a, neck, b, b_neighbours = [16], [25], [29], [33], [40]
theta = 1.

results = []
for k in range(100):
    for i in range(3):
        align(a_neighbours, a, neck, b, b_neighbours)

    capacitances = test_split(proxy, theta, a_neighbours, a, neck, b, b_neighbours)
    C_a = capacitances[a_neighbours + a].sum()
    C_b = capacitances[b_neighbours + b].sum()
    result_k = {'time': time.time(), 'C_a': C_a, 'C_b': C_b, 'theta': theta}
    results.append(result_k)
    print('\r%-60s' % ('%-2d. theta: %.2f, C_a/C_b: %.2f, C_a: %sF, C_b: %sF' % 
          tuple([k + 1, theta, C_a / C_b] + map(si.si_format, [C_a, C_b]))), end='')
    
# Save results
now = dt.datetime.now()
experiment_log = {'timestamp': now.isoformat(),
                  'split_runs': results,
                  'chip': 'c3ad05dd-b753-4bfb-807a-7828e524064d',
                  'liquid': 'propylene glycol',
                  'dropbot_state': proxy.state.to_dict()}
output_name = '%s - PID split results.json' % now.strftime('%Y-%m-%dT%Hh%M')
with open(output_name, 'w') as output:
    json_tricks.dump(experiment_log, output, indent=4)

# %% [markdown]
# # Open-loop split

# %%
proxy.voltage = 105

# %%
# apply_duty_cycles(proxy, pd.Series(1, index=[28, 29]))
a_neighbours, a, neck, b, b_neighbours = [16], [25], [29], [33], [40]

for i in range(3):
    align(a_neighbours, a, neck, b, b_neighbours)

# %%
results = []

for k in range(100):
    for i in range(3):
        align(a_neighbours, a, neck, b, b_neighbours)
    apply_duty_cycles(proxy, pd.Series(1, index=[28, 29, 90]))
    time.sleep(.5)
    proxy.stop_switching_matrix()
    proxy.turn_off_all_channels()
    time.sleep(.5)
    apply_duty_cycles(proxy, pd.Series(1, index=a_neighbours + a +
                                       b + b_neighbours))
    time.sleep(3.5)
    capacitances = loop.run_until_complete(asyncio.wait_for(read_C(), 15))
    C_a = capacitances[a_neighbours + a].sum()
    C_b = capacitances[b_neighbours + b].sum()
    result_k = {'time': time.time(), 'C_a': C_a, 'C_b': C_b}
    results.append(result_k)
    print('\r%-60s' % ('%2d. C_a/C_b: %.2f, C_a: %sF, C_b: %sF' % 
          tuple([k + 1, C_a / C_b] + map(si.si_format, [C_a, C_b]))), end='')
    
# Save results
now = dt.datetime.now()
experiment_log = {'timestamp': now.isoformat(),
                  'split_runs': results,
                  'chip': 'c3ad05dd-b753-4bfb-807a-7828e524064d',
                  'liquid': 'propylene glycol',
                  'dropbot_state': proxy.state.to_dict()}
output_name = '%s - open-loop split results.json' % now.strftime('%Y-%m-%dT%Hh%M')
with open(output_name, 'w') as output:
    json_tricks.dump(experiment_log, output, indent=4)

# %% [markdown]
# # Plot results

# %%
import path_helpers as ph

root = ph.path('~/Dropbox (Sci-Bots)/Sci-Bots experiments').expand()
experiment = 'open-loop-split-02'
exp_dir = root.dirs('* results - `dropbot.py@exp(%s)`' % experiment)[0]
data_file = exp_dir.files('*.json')[0]

with open(data_file, 'r') as input_:
    experiment_log = json_tricks.load(input_)
    results = experiment_log['split_runs']

df_results = pd.DataFrame(results)
df_results.set_index(df_results.time.map(dt.datetime.fromtimestamp), inplace=True)
del df_results['time']
ax = (df_results['C_a'] / df_results['C_b']).plot(kind='box')
ax.set_title('Open-loop, Propylene glycol, ratio 1:1, 100 splits')
ax.set_xticks([])
ax.set_ylabel(r'$\frac{C_a}{C_b}$', fontsize=25)
ax.get_figure().tight_layout()
(df_results['C_a'] / df_results['C_b']).describe()

# %%
apply_duty_cycles(proxy, pd.Series(0, index=[94, 25, 29]))
# apply_duty_cycles(proxy, pd.Series(1, index=[90, 86, 94]))

# %%
# capacitances = loop.run_until_complete(read_C())
# capacitances

# %%
import functools as ft

# %%
from itertools import islice

def window(seq, n):
    "Returns a sliding window (of width n) over data from the iterable"
    "   s -> (s0,s1,...s[n-1]), (s1,s2,...,sn), ...                   "
    it = iter(seq)
    result = tuple(islice(it, n))
    if len(result) == n:
        yield result
    for elem in it:
        result = result[1:] + (elem,)
        yield result

# %%
for e in list(G.edges):
    if any(c in e for c in (30, 89, 1)):
        G.remove_edge(*e)

# %%
import itertools as it

# %%
# route = nx.shortest_path(G, 90, 22)
# route = 26, 20, 21, 22, 16, 103
# loop.run_until_complete(move_liquid(route, trail_length=2))

# %%
# apply_duty_cycles(proxy, pd.Series(1, index=[20]))
# loop.run_until_complete(read_C())

# %%
# apply_duty_cycles(proxy, pd.Series(1, index=[101]))
# time.sleep(.2)
# apply_duty_cycles(proxy, pd.Series(0, index=[101]))
# proxy.stop_switching_matrix()

# %%
# def check_head_C(channel, C_0, capacitances):
#     if (channel in capacitances):
#         if capacitances[channel] > EPSILON + C_0[channel]:
#             return True
#     loop.run_until_complete(asyncio.wait_for(wait_on_capacitance(ft.partial(check_head_C, head, C_0)), timeout=5))

# # route = nx.shortest_path(G, 25, 79)
# route = nx.shortest_path(G, 90, 16)

# for route_i in window(route, 2):
#     tail = list(route_i[:-1])
#     head = route_i[-1]
#     print(head, body, tail)
#     apply_duty_cycles(proxy, pd.Series(0, index=route_i))
#     C_0 = loop.run_until_complete(read_C())
#     apply_duty_cycles(proxy, pd.Series(1, index=[head] + tail))
#     loop.run_until_complete(asyncio.wait_for(wait_on_capacitance(ft.partial(check_head_C, head, C_0)), timeout=5))
# apply_duty_cycles(proxy, pd.Series(1, index=[head]))

# %%
import matplotlib.pyplot as plt

# %%
for r in proxy.signals.signal('sensitive-capacitances').receivers.values():
    proxy.signals.signal('sensitive-capacitances').disconnect(r)

# %%
# def remerge(route):
#     for i in route[:3]:
#         apply_duty_cycles(proxy, pd.Series(1, index=[i]))
#         time.sleep(1)

#     # 1. Move liquid along route, one electrode at a time until **head** capacitance exceeds EPSILON.
#     def _check_head(route, capacitances):
#         try:
#             return capacitances[route[-1:]].sum() >= .7 * target_C_head
#         except KeyError:
#             pass

#     for i in range(len(route) - 2):
#         route_i = route[i:i + 3]
#         apply_duty_cycles(proxy, pd.concat([pd.Series(1, index=route_i[-1:]),
#                                             pd.Series(1, index=route_i[1:-1]),
#                                             pd.Series(0, index=route[:1])]))
#         state_i = loop.run_until_complete(
#             asyncio.wait_for(wait_on_capacitance(ft.partial(_check_head, route_i)), timeout=5))

# %%
# route = nx.shortest_path(G, 16, 26)
# remerge(route)

# %%
# head_neighbours = neighbours.loc[head].dropna().astype(int).tolist()
# other_neighbours = neighbours.loc[head_neighbours].dropna().astype(int)
# other_neighbours = other_neighbours[~other_neighbours.isin([head] + head_neighbours)].tolist()
# apply_duty_cycles(proxy, pd.concat([pd.Series(1, index=[head]),
#                                     pd.Series(0, head_neighbours + other_neighbours)]))
# C_0 = loop.run_until_complete(read_C())
# # XXX Use initial capacitance as target for moving.
# C_0.sort_values().map(si.si_format)

state_ = {'n': 0}

def check_first(channel, threshold, capacitances):
    if (channel in capacitances) and capacitances[channel] > threshold:
        return True

def check_transition(channel, threshold, capacitances):
    if (channel in capacitances) and capacitances[channel] > threshold:
        state_['n'] += 1
        if state_['n'] >= 4:
            state_['n'] = 0
            return True
    else:
        state_['n'] = 0


@asyncio.coroutine
def move_liquid(route, trail_length=1, neighbour_epsilon=3e-12):
    # Pull liquid to cover first electrode and _at least slightly_ overlap next electrode on route.
    for route_i in it.imap(list, window(route, trail_length + 1)):
        apply_duty_cycles(proxy, pd.Series(1, index=route_i))
        print('\r%-50s' % ('actuated: `%s`' % route_i), end='')
        state = yield asyncio.From(wait_on_capacitance(ft.partial(check_first,
                                                                  route_i[-1],
                                                                  neighbour_epsilon)))
        apply_duty_cycles(proxy, pd.Series(1, index=route_i[1:]))
        print('\r%-50s' % ('actuated: `%s`' % route_i[1:]), end='')
        state = yield asyncio.From(wait_on_capacitance(ft.partial(check_first,
                                                                  route_i[-1],
                                                                  10e-12)))
    state = apply_duty_cycles(proxy, pd.Series(1, index=route[-trail_length:]))
    raise asyncio.Return(state)

# %%
electrode_C = pd.Series()

log_ = []


def log_sensitive_capacitances(message):
    message['time'] = time.time()
    log_.append(message)


@asyncio.coroutine
def _attempt_dispense(proxy, route, alpha=1., beta=1.1, head_epsilon=10e-12, epsilon=EPSILON,
                      neck_epsilon=2e-12):
    # 1. Move liquid along route, one electrode at a time until **head** capacitance exceeds EPSILON.
    move_state = yield asyncio.From(move_liquid(route, trail_length=1, neighbour_epsilon=epsilon))

    head = route[-2:-1]
    head_neighbours = route[-1:]
    head_index = route.index(head[0])
    neck = [route[head_index - 1]]

    apply_duty_cycles(proxy, pd.Series(1, index=head))
    C = yield asyncio.From(read_C())

    for h in head:
        electrode_C.loc[h] = max(electrode_C.loc[h] if h in electrode_C else 0,
                                 C[head].sum())
    target_C_head = beta * electrode_C[head].sum()

    # 2. Turn off head neighbour and detect tail.
    def check_head(capacitances):
         return capacitances[head_neighbours].sum() < epsilon

    apply_duty_cycles(proxy, pd.concat([pd.Series(1, index=head),
                                        pd.Series(.2, index=route[:head_index]),
                                        pd.Series(0, index=head_neighbours)]))
    state = yield asyncio.From(wait_on_capacitance(check_head))

    C_route = state['C_T'][route[:-1]]
    tail_back = C_route[C_route > EPSILON].index[0]
    tail_index = route.index(tail_back)

    tail_neighbours = route[:tail_index]
    tail = route[tail_index:head_index - 1]

    apply_duty_cycles(proxy, pd.concat([pd.Series(1, index=head),
                                        pd.Series(0.5, index=head_neighbours)]))

    yield asyncio.From(wait_for_split(proxy, alpha * target_C_head,
                                      tail_neighbours, tail, neck, head,
                                      head_neighbours, neck_epsilon=neck_epsilon))
    apply_duty_cycles(proxy, pd.Series(1, index=tail + head + head_neighbours))
    capacitances = yield asyncio.From(read_C())
    C_head = capacitances[head + head_neighbours].sum()
    apply_duty_cycles(proxy, pd.Series(1, index=head))
    raise asyncio.Return(target_C_head, C_head)


@asyncio.coroutine
def dispense(proxy, route, alpha=1., beta=1.1, epsilon=EPSILON, neck_epsilon=2e-12):
    alpha_ = alpha
    while True:
        target_C_head, C_head = \
            yield asyncio.From(_attempt_dispense(proxy, route, alpha=alpha, beta=beta,
                                                 neck_epsilon=neck_epsilon))
#         display(pd.Series({'target': target_C_head, 'result': C_head}).map(si.si_format))
        error = (C_head - target_C_head) / target_C_head
        print('%s%.2f%%' % ('+' if error > 0 else '', error * 100))

        message = {'error': error,
                   'target_C': target_C_head,
                   'C': C_head,
                   'alpha': alpha,
                   'args': dict(zip(('route', 'alpha', 'beta', 'epsilon'),
                                    (route, alpha_, beta, epsilon))),
                   'time': time.time()}
        error_tolerance = .075
        if abs(error) < error_tolerance:
            message['action'] = 'dispense-complete'
            log_.append(message)
            break
        else:
            alpha *= target_C_head / C_head if C_head != 0 else 1.
            message['action'] = 'dispense-retry'
            message['alpha'] = alpha
            log_.append(message)
#             print('alpha: %.2f' % alpha)
            yield asyncio.From(move_liquid(route[::-1][:4], neighbour_epsilon=3e-12))
            yield asyncio.From(move_liquid(route[:-3], neighbour_epsilon=3e-12))
    raise asyncio.Return(target_C_head, C_head, alpha)

# %%
import ivPID

@asyncio.coroutine
def wait_for_split(proxy, target_C_head, tail_neighbours, tail, neck, head, head_neighbours,
                   neck_epsilon=2e-12):
    # See https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    K_u = .2 / 1e-12 # Determined empirically, consistent oscillations
    T_u=2 * .365  # Oscilation period

    # pid = ivPID.PID(P=K_u)
    # pid = ivPID.PID(P=.5 * K_u)  # P
    # pid = ivPID.PID(P=.45 * K_u, I=T_u / 1.2)  # PI
    # pid = ivPID.PID(P=.8 * K_u, D=T_u / 8)  # PD
    # pid = ivPID.PID(P=.6 * K_u, I=T_u / 2, D=T_u / 8)  # ["classic" PID][1]
    # pid = ivPID.PID(P=.7 * K_u, I=T_u / 2.5, D=3 * T_u / 20)  # [Pessen Integral Rule][1]
    # pid = ivPID.PID(P=.33 * K_u, I=T_u / 2, D=T_u / 3)  # ["some" overshoot][1]
    # pid = ivPID.PID(P=.2 * K_u, I=T_u / 2, D=T_u / 3)  # ["no" overshoot][1]
    # [1]: http://www.mstarlabs.com/control/znrule.html

    pid = ivPID.PID(P=.6 * K_u, I=T_u / 2, D=T_u / 8)  # ["classic" PID][1]

    pid.setPoint = .5

    loop = asyncio.get_event_loop()

    def apply_force(force):
        proxy.stop_switching_matrix()

        tail_duty_cycle, head_duty_cycle = output_duty_cycles(force)
        tail_duty_cycle = max(0, min(1, tail_duty_cycle))
        head_duty_cycle = max(0, min(1, head_duty_cycle))
        duty_cycles = pd.concat([pd.Series(head_duty_cycle, index=head + head_neighbours),
                                 pd.Series(tail_duty_cycle, index=tail + tail_neighbours)])
        # XXX Use existing sensitive channels, which include **middle** channel(s).
        apply_duty_cycles(proxy, duty_cycles, set_sensitive=False)
#         print('\r%-50s' % ('%-.03f <- -> %.03f' % (tail_duty_cycle, head_duty_cycle)), end='')
        log_.append({'action': 'apply-force', 'duty_cycles': duty_cycles,
                     'tail_duty_cycle': tail_duty_cycle, 'head_duty_cycle': head_duty_cycle,
                     'time': time.time()})

    neck_breaks = []

    def on_capcacitances(capacitances):
        C_neck = capacitances[neck].sum()
        if C_neck < neck_epsilon:
            neck_breaks.append(C_neck)
        else:
            del neck_breaks[:]
        if len(neck_breaks) >= 4:
            return True
        try:
            pid.update(capacitances[head + head_neighbours].sum() - target_C_head)
            loop.call_soon_threadsafe(apply_force, pid.output)
        except AttributeError:
            logging.debug('Error', exc_info=True)

    apply_duty_cycles(proxy, pd.concat([pd.Series(0, index=tail_neighbours + tail + neck),
                                        pd.Series(.5, index=head + head_neighbours)]))
    state = yield asyncio.From(wait_on_capacitance(on_capcacitances))
    log_.append({'action': 'split-complete',
                 'args':
                 dict(zip(('target_C_head', 'tail_neighbours', 'tail', 'neck', 'head', 'head_neighbours'),
                          (target_C_head, tail_neighbours, tail, neck, head, head_neighbours))),
                 'time': time.time()})
    raise asyncio.Return(state)


def gather_liquid(sources, target, **kwargs):
    for s in sources:
        route = nx.shortest_path(G, s, target)
        loop.run_until_complete(asyncio.wait_for(move_liquid(route, neighbour_epsilon=2e-12, trail_length=1), timeout=20))
        apply_duty_cycles(proxy, pd.Series(1, index=route[-1:]))

# %%
# apply_duty_cycles(proxy, pd.Series(1, index=[26]))
# apply_duty_cycles(proxy, pd.Series(1, index=[82, 81, 79]))

# %%
# loop.run_until_complete(asyncio.wait_for(read_C(), timeout=10))

# %%
# loop.run_until_complete(asyncio.wait_for(move_liquid([20, 21, 22, 16]), timeout=10))

# %%
# alpha_i = all_results[-1].get(d, {'alpha': alpha_i})['alpha']
# target_C_head_i, C_head_i, alpha_i =\
#     loop.run_until_complete(asyncio.wait_for(dispense(proxy, nx.shortest_path(G, 26, 103),
#                                                       beta=1.1, alpha=alpha_i), timeout=60))
# results_i = pd.Series([target_C_head_i, C_head_i, alpha_i], index=['target_C', 'C', 'alpha'])
# display(results_i)
# results[d] = results_i
# route = nx.shortest_path(G, 16, d)
# loop.run_until_complete(asyncio.wait_for(move_liquid(route, neighbour_epsilon=2e-12, trail_length=1), timeout=20))
# apply_duty_cycles(proxy, pd.Series(1, index=route[-1:]))

# %%
# proxy.voltage = 105

# %%
# route = nx.shortest_path(G, 40, 85)
# loop.run_until_complete(asyncio.wait_for(move_liquid(route, neighbour_epsilon=2e-12,
#                                                      trail_length=3), timeout=20))
# apply_duty_cycles(proxy, pd.Series(1, index=route[-3:]))
# apply_duty_cycles(proxy, pd.Series(1, index=[85, 83]))

# %%
duty_cycles = pd.Series(proxy.sensitive_duty_cycles())[proxy.sensitive_channels.astype(int)]
# duty_cycles
# duty_cycles[85] = .5
del duty_cycles[79]
# duty_cycles
apply_duty_cycles(proxy, duty_cycles)

# %%
def gather_liquid(sources, target, **kwargs):
    for s in sources:
        route = nx.shortest_path(G, s, target)
        loop.run_until_complete(asyncio.wait_for(move_liquid(route, neighbour_epsilon=2e-12, trail_length=1), timeout=20))
        apply_duty_cycles(proxy, pd.Series(1, index=route[-1:]))

# %%
gather_liquid([16], 98, trail_length=1)
# gather_liquid([22], 26, trail_length=2)
# gather_liquid([22, 97], 83, trail_length=1)
# apply_duty_cycles(proxy, pd.Series(1, index=[16, 98, 97, 103]))
# proxy.stop_switching_matrix()
# proxy.turn_off_all_channels()
# proxy.state_of_channels = pd.Series(1, index=[82, 83])
# proxy.state_of_channels[proxy.state_of_channels > 0]

# %%



# %%
# proxy.disabled_channels_mask = np.zeros(120)
disabled = pd.Series(proxy.disabled_channels_mask)
disabled[disabled > 0]

# %% [markdown]
# --------------------------------
#
# # Run PID dispense from 85 to 79

# %%
import os
import datetime as dt
import json_tricks


move_epsilon = 5e-12


def run_experiment(route, destinations, tunings=None):
    results = {}

    for d in destinations:
        start_i = time.time()
        alpha_i = 1.
        if tunings is not None:
            alpha_i = tunings.get(d, {'alpha': alpha_i})['alpha']
        target_C_head_i, C_head_i, alpha_i =\
            loop.run_until_complete(asyncio.wait_for(dispense(proxy, route,
                                                              beta=BETA, alpha=alpha_i,
                                                              epsilon=EPSILON,
                                                              neck_epsilon=NECK_EPSILON),
                                                     timeout=60))
        end_i = time.time()
        results_i = pd.Series([target_C_head_i, C_head_i, alpha_i, start_i, end_i],
                              index=['target_C', 'C', 'alpha', 'start', 'end'])
        display(results_i, results_i['end'] - results_i['start'])
        results[d] = results_i
        route_i = nx.shortest_path(G, route[-2], d)
        loop.run_until_complete(asyncio.wait_for(move_liquid(route_i, neighbour_epsilon=move_epsilon,
                                                             trail_length=1), timeout=20))
        apply_duty_cycles(proxy, pd.Series(1, index=route_i[-1:]))

    gather_liquid(destinations[::-1], route[2])

    loop.run_until_complete(asyncio.wait_for(move_liquid(route[:4][::-1],
                                                         neighbour_epsilon=move_epsilon,
                                                         trail_length=3), timeout=20))
    apply_duty_cycles(proxy, pd.Series(1, index=route[:2]))
    return results


source = 93
target = 16
route = nx.shortest_path(G, source, target)
# route = [85, 83, 82, 81, 79, 86]
proxy.voltage = 98
EPSILON = 5e-12
NECK_EPSILON = 4e-12
BETA = 1.3

# destinations = [22, 97]  #, 53, 66]
# destinations = [25, 48]
# destinations = [111, 86]
destinations = [21, 29]

log_ = []
proxy.signals.signal('sensitive-capacitances').connect(log_sensitive_capacitances, weak=False)

original_apply_duty_cycles = apply_duty_cycles

@ft.wraps(original_apply_duty_cycles)
def apply_duty_cycles(proxy, duty_cycles, set_sensitive=True):
    log_.append({'action': 'apply-duty-cycles', 'duty_cycles': duty_cycles,
                 'set_sensitive': set_sensitive})
    return original_apply_duty_cycles(proxy, duty_cycles, set_sensitive=set_sensitive)

try:
#     results = None
#     while True:
    for i in range(10):
        print('\n' + 72 * '-', end='\n\n')
        print('Iteration %d' % i, end='\n\n')
        results = run_experiment(route, destinations, tunings=results)
        # Save results
        now = dt.datetime.now()
        experiment_log = {'timestamp': now.isoformat(),
                          'dispense_runs': results,
                          'chip': 'c3ad05dd-b753-4bfb-807a-7828e524064d',
                          'beta': BETA}
        output_name = '%s - PID dispense results.json' % now.strftime('%Y-%m-%dT%Hh%M')
        with open(output_name, 'w') as output:
            json_tricks.dump(experiment_log, output, indent=4)
        with open(os.path.splitext(output_name)[0] + '-action_log.json', 'w') as output:
            json_tricks.dump(log_, output, indent=4)
except KeyboardInterrupt:
    pass
finally:
    apply_duty_cycles = original_apply_duty_cycles
    proxy.signals.signal('sensitive-capacitances').disconnect(log_sensitive_capacitances)

    for r in proxy.signals.signal('sensitive-capacitances').receivers.values():
        proxy.signals.signal('sensitive-capacitances').disconnect(r)

# %%
# apply_duty_cycles(proxy, pd.Series(1, index=[85]))
proxy.turn_off_all_channels()
proxy.stop_switching_matrix()

# %%
capacitance_messages = [message for message in log_ if 'action' not in message]
capacitances = pd.concat([pd.Series(*zip(*message['C'])[::-1]) for message in capacitance_messages],
                         keys=(message['time'] for message in capacitance_messages)).unstack()
capacitances.index = pd.to_datetime(capacitances.index * 1e9)

# %%
import matplotlib.animation as manimation

FFMpegWriter = manimation.writers['ffmpeg']
metadata = dict(title='Movie Test', artist='Matplotlib',
                comment='Movie support!')
writer = FFMpegWriter(fps=4, metadata=metadata)

fig, axis = plt.subplots(1)

axis.yaxis.set_major_formatter(F_formatter)
axis.set_ylim(0, 120e-12)

with writer.saving(fig, os.path.splitext(output_name)[0] + ".mp4", 100):
    for time_s, s_capacitances_i in capacitances.resample('.25S').mean().iterrows():
        axis.cla()
        s_capacitances_i.plot(kind='bar', ax=axis)
        axis.yaxis.set_major_formatter(F_formatter)
        writer.grab_frame()

# %%
# df_shapes.head()
# channels_by_electrode


# %%
electrode_channels = (df_shapes.drop_duplicates(['id', 'data-channels'])
                      .set_index('id')['data-channels'].map(int))

# Compute center `(x, y)` for each electrode.
electrode_centers = df_shapes.groupby('id')[['x', 'y']].mean()
# Index by **channel number** instead of **electrode id**.
electrode_centers.index = electrode_channels.reindex(electrode_centers
                                                     .index)

patches = [mpl.patches.Polygon(df_shape_i[['x', 'y']].values,
                               closed=False, label=id_)
           for id_, df_shape_i in df_shapes.groupby('id')]

if ax is None:
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
ax.set_aspect(True)

for patch_i in patches:
    patch_i.set_facecolor('blue')
    patch_i.set_edgecolor('black')
    ax.add_patch(patch_i)

ax.set_xlim(df_shapes.x.min(), df_shapes.x.max())
ax.set_ylim(df_shapes.y.max(), df_shapes.y.min())
fig.show()

# %%
%matplotlib inline

# %%
time_s, s_capacitances_i

# %%
import matplotlib as mpl

# %%
capacitances.index.to_series().diff

# %%
# apply_duty_cycles(proxy, pd.Series(1, index=[85]))
proxy.stop_switching_matrix()
proxy.set_sensitive_channels([])

# %%
import datetime as dt
import json_tricks

# %%



# %%
!start .

# %%
len(all_results)

# %%
apply_duty_cycles(proxy, pd.Series(1, index=[20, 21, 22, 16]))

# %%
C = pd.DataFrame([loop.run_until_complete(read_C()) for i in range(20)])
ax = C[[i for i in C if i != 20]].plot(kind='box')
ax.yaxis.set_major_formatter(F_formatter)

# %%
# all_alphas = [_210]

# %%
all_results

# %%
[pd.Series(r).loc[destinations].map(lambda x: x['alpha']) for r in all_results]

# %%
# @asyncio.coroutine
# def move_liquid(proxy, route, head_epsilon=10e-12):
#     # 1. Move liquid along route, one electrode at a time until **head** capacitance exceeds EPSILON.
#     def _check_head(route, capacitances):
#         try:
#             return capacitances[route[-1:]].sum() >= head_epsilon
#         except KeyError:
#             pass

#     state_i = {}
#     for i in range(len(route) - 2):
#         route_i = route[i:i + 3]
#         duty_cycles = [pd.Series(1, index=route_i[-1:])]
#         if len(route_i) > 2:
#             duty_cycles += [pd.Series(.5, index=route_i[1:-1]),
#                             pd.Series(0, index=route_i[:1])]
#         else:
#             duty_cycles += [pd.Series(.5, index=route_i[:1])]
#         s_duty_cycles = pd.concat(duty_cycles)
# #         print(route_i, s_duty_cycles)
# #     duty_cycles = 0, .8, 1

# #     for i in range(len(route) - 1):
# #         route_i = route[i:i + 3]
# #         duty_cycles_i = pd.Series(duty_cycles[-len(route_i):], index=route_i)
# #         print(route_i, duty_cycles_i)
# #         apply_duty_cycles(proxy, duty_cycles_i)
#         apply_duty_cycles(proxy, s_duty_cycles)
#         state_i = yield asyncio.From(wait_on_capacitance(ft.partial(_check_head, route_i)))
#     raise asyncio.Return(state_i)

# %%
# route = nx.shortest_path(G, 40, 33)
# duty_cycles = 0, 1, 1

# for i in range(len(route) - 1):
#     route_i = route[i:i + 3]
#     duty_cycles_i = pd.Series(duty_cycles[-len(route_i):], index=route_i)
#     print(route_i, duty_cycles_i)

# %%
proxy.stop_switching_matrix()
apply_duty_cycles(proxy, pd.Series(1, index=[26]))
# loop.run_until_complete(asyncio.wait_for(move_liquid(proxy, nx.shortest_path(G, 22, 20)), timeout=5))
# loop.run_until_complete(asyncio.wait_for(move_liquid(proxy, [26, 20, 22]), timeout=5))
# # route[-3:][::-1]
# # route[-5:-3]

# %%
proxy.voltage = 115

# %%
dispenses = []

destinations = [5, 38, 107, 91]

alphas = [.75] * len(destinations)
for i, (d, alpha_i) in enumerate(zip(destinations, alphas)):
    target_C_head_i, C_head_i, alpha_i =\
        loop.run_until_complete(asyncio.wait_for(dispense(proxy, nx.shortest_path(G, 26, 103),
                                                          alpha=alpha_i), timeout=20))
    loop.run_until_complete(asyncio.wait_for(move_liquid(proxy, nx.shortest_path(G, 16, d)), timeout=30))
    dispenses.append((target_C_head_i, C_head_i, alpha_i, d))
    apply_duty_cycles(proxy, pd.Series(1, index=[d]))
alphas = zip(*dispenses)[2]
apply_duty_cycles(proxy, pd.Series(1, index=[destination_i for _, _, _, destination_i in dispenses]))

# %%
head_epsilon = 22e-12

for destination_i in destinations:
    loop.run_until_complete(asyncio.wait_for(move_liquid(proxy, nx.shortest_path(G, destination_i, 21),
                                                         head_epsilon=head_epsilon), timeout=20))
loop.run_until_complete(asyncio.wait_for(move_liquid(proxy, nx.shortest_path(G, 21, 26),
                                                     head_epsilon=head_epsilon), timeout=10))

# %%
# proxy.stop_switching_matrix()
# proxy.turn_off_all_channels()
apply_duty_cycles(proxy, pd.Series(1, index=[26]))

# %%
# loop.run_until_complete(asyncio.wait_for(move_liquid(proxy, nx.shortest_path(G, 16, 21)), timeout=20))


# loop.run_until_complete(asyncio.wait_for(move_liquid(proxy, nx.shortest_path(G, 16, 113)), timeout=30))
# loop.run_until_complete(asyncio.wait_for(move_liquid(proxy, nx.shortest_path(G, 16, 107)), timeout=30))
# loop.run_until_complete(asyncio.wait_for(move_liquid(proxy, nx.shortest_path(G, 16, 79)), timeout=30))
# loop.run_until_complete(asyncio.wait_for(move_liquid(proxy, nx.shortest_path(G, 16, 91)), timeout=30))

# %%



# %%
# attempts = []
attempts.append((target_C_head, C_head))
target_C_head / C_head

# %%
state_i['C_T'].plot(kind='bar')

# %%
target_C_head
# apply_duty_cycles(proxy, pd.Series(1, index=[16] + neighbours.loc[16].dropna().astype(int).tolist()))
# si.si_format(loop.run_until_complete(read_C()).sum())

# %%
proxy.signals.signal('sensitive-capacitances').receivers

# %%
# apply_duty_cycles(proxy, pd.Series([0.01, 1], index=[29, 33]))
# apply_duty_cycles(proxy, pd.Series([0.01, 1], index=[29, 33]))
# apply_duty_cycles(proxy, pd.Series(1, index=[22, 16]))

# %%
pd.DataFrame([dict(d['C']) for d in _debug_data['sensitive_capacitances']]).applymap(si.si_format)

# %%
# neighbours_i = proxy.neighbours.loc[[18, 16, 25], 'down'].astype(int)

# %%
proxy.turn_off_all_channels()
# proxy.state_of_channels = pd.Series(1, index=[20, 21, 22])
# proxy.state_of_channels
# neighbours_i = proxy.neighbours.loc[neighbours_i.tolist(), 'down'].astype(int)
# proxy.state_of_channels = pd.Series(1, index=neighbours_i)
# proxy.state_of_channels = pd.Series(1, index=[18, 16, 25])
# proxy.state_of_channels = pd.Series(1, index=[17, 22, 18, 16])
# proxy.state_of_channels = pd.Series(1, index=[18, 16, 25])
# proxy.state_of_channels = pd.Series(1, index=[16])
# proxy.turn_off_all_channels()

# %%



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
middle_i = [20]

# %%
drop_i, middle_i

# %% [markdown]
# ## Starting from **middle** electrode(s), add _left_ neighbour to **a** group, _right_ neighbour to **b** group

# %%
# a_dir = 'left'
# b_dir = 'right'

def split_channels(middle_i, a_dir = 'up', b_dir = 'down', a_count=1, b_count=1, a_neighbour_count=1, b_neighbour_count=1):
    a_i = []
    for i in range(a_count):
        neighbours_i = proxy.neighbours.loc[a_i + middle_i, [a_dir]].reset_index(level=0, drop=True)
        neighbours_i = neighbours_i.loc[~neighbours_i.isin(a_i + middle_i)].dropna().astype(int).tolist()
        a_i.extend(neighbours_i)
    b_i = []
    for i in range(b_count):
        neighbours_i = proxy.neighbours.loc[b_i + middle_i, [b_dir]].reset_index(level=0, drop=True)
        neighbours_i = neighbours_i.loc[~neighbours_i.isin(b_i + middle_i)].dropna().astype(int).tolist()
        b_i.extend(neighbours_i)

    ## Find _extended_ neighbours of **a** and **b**
    a_neighbours_i = []
    for i in range(a_neighbour_count):
        neighbours_i = proxy.neighbours.loc[a_i + a_neighbours_i, [a_dir]].reset_index(level=0, drop=True)
        neighbours_i = neighbours_i.loc[~neighbours_i.isin(a_i + a_neighbours_i)].dropna().astype(int).tolist()
        a_neighbours_i.extend(neighbours_i)
    b_neighbours_i = []
    for i in range(b_neighbour_count):
        neighbours_i = proxy.neighbours.loc[b_i + b_neighbours_i, [b_dir]].reset_index(level=0, drop=True)
        neighbours_i = neighbours_i.loc[~neighbours_i.isin(b_i + b_neighbours_i)].dropna().astype(int).tolist()
        b_neighbours_i.extend(neighbours_i)
    return {'middle': middle_i, 'a': a_i, 'b': b_i, 'a_neighbours': a_neighbours_i,
            'b_neighbours': b_neighbours_i}

split_channels_ = split_channels(middle_i)
a_i =  split_channels_['a']
b_i =  split_channels_['b']
a_neighbours_i =  split_channels_['a_neighbours']
b_neighbours_i =  split_channels_['b_neighbours']
# split_channels([22], a_count=2, a_neighbour_count=2, b_count=2, b_neighbour_count=2)

# %% [markdown]
# ## Set **a**, **b** and **middle** electrodes to maximum duty cycle.

# %%
proxy.stop_switching_matrix()
drop_channels_i = a_i + middle_i + b_i
proxy.set_sensitive_channels(drop_channels_i)
# proxy.set_sensitive_channels()
# proxy.set_duty_cycle(0, a_i)
proxy.set_duty_cycle(1, a_i + middle_i)
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

# %%
forces = [-1, -.5, 0, .5, 1]
pd.Series(forces, index=forces).map(output_duty_cycles)

# %% [markdown]
# ## Set **middle** duty cycle to 0; init **a** and **b** to 1; and adjust force ratio and amplitude to split

# %%
import time
import ivPID


class DropSplitController(object):
    def __init__(self, proxy, a, b, a_neighbours, b_neighbours, middle):
        self.a = a
        self.b = b
        self.a_neighbours = a_neighbours
        self.b_neighbours = b_neighbours
        self.middle = middle
        self.proxy = proxy
        self.outputs = {}
        self._update_force = None
        self._sensitive_callback = None
        self.signals = blinker.Namespace()
        self._i = 0

    def _apply_force(self, force, include_neighbours=True):
        self.proxy.stop_switching_matrix()

        a_i = self.a
        a_neighbours_i = self.a_neighbours
        b_i = self.b
        b_neighbours_i = self.b_neighbours

        a_duty_i, b_duty_i = output_duty_cycles(force)
        duty_cycles = pd.concat([pd.Series(a_duty_i, index=a_i + (a_neighbours_i if include_neighbours else [])),
                                 pd.Series(b_duty_i, index=b_i + (b_neighbours_i if include_neighbours else []))])
        # XXX Use existing sensitive channels, which include **middle** channel(s).
        apply_duty_cycles(self.proxy, duty_cycles, set_sensitive=False)
        self.proxy.resume_switching_matrix()

    def start(self, d_0, include_neighbours, **kwargs):
        self.stop()
        proxy = self.proxy
        a_i = self.a
        a_neighbours_i = self.a_neighbours
        b_i = self.b
        b_neighbours_i = self.b_neighbours
        self.outputs = {}
        self._i = 0

        def _on_sensitive_capacitances(message):
            '''
            Update PID controller output based on new sensor inputs.

            Parameters
            ----------
            message : dict
                Example message::

                    {"event": "sensitive-capacitances",
                     "wall_time": ...,
                     "C": [[<channel>, <capacitance>], ...]}
            '''
            channels, capacitances = zip(*message['C'])
            capacitances = pd.Series(capacitances, index=channels)
            middle_c_i = capacitances[middle_i].sum()
            a_c_i = capacitances[a_i + a_neighbours_i].sum()
            b_c_i = capacitances[b_i + b_neighbours_i].sum()

            message = {'sensitive_capacitances': capacitances,
                       'split_capacitances': {'a': a_c_i, 'b': b_c_i, 'middle': middle_c_i}}
            self.signals.signal('split-capacitances-updated').send('split-ctrl', **message)
        queue_set = threading.Event()

        @cancellable
        @asyncio.coroutine
        def _manage_force(include_neighbours):
            queue = asyncio.Queue()
            queue_set.queue = queue
            queue_set.set()
            previous_force = None
            try:
                while True:
                    force = yield asyncio.From(queue.get())
                    if previous_force != force:
                        self._apply_force(force, include_neighbours)
                    self.outputs[time.time()] = force
            except asyncio.CancelledError:
                pass
        apply_duty_cycles(proxy, pd.Series(d_0, a_i + a_neighbours_i + middle_i + b_i + b_neighbours_i),
                          set_sensitive=True)

        force_monitor = threading.Thread(target=_manage_force, args=(include_neighbours, ))
        force_monitor.daemon = True
        force_monitor.start()
        queue_set.wait()

        self.set_force = ft.partial(_manage_force.started.loop.call_soon_threadsafe,
                                    queue_set.queue.put_nowait)

        self._update_force = _manage_force
        self._sensitive_callback = _on_sensitive_capacitances
        _manage_force.started.loop.call_soon_threadsafe(queue_set.queue.put_nowait, 0)
        proxy.signals.signal('sensitive-capacitances').connect(_on_sensitive_capacitances, weak=False)

    def stop(self):
        if self._update_force is not None:
            self.proxy.stop_switching_matrix()
            self.proxy.turn_off_all_channels()
            self.proxy.signals.signal('sensitive-capacitances').disconnect(self._sensitive_callback)
            self._update_force.cancel()

# %%
def measure_sensitive():
    capacitances_received = threading.Event()

    def _on_sensitive_capacitances(message):
        channels, capacitances = zip(*message['C'])
        capacitances_received.capacitances = pd.Series(capacitances, index=channels)
        capacitances_received.set()

    try:
        proxy.signals.signal('sensitive-capacitances').connect(_on_sensitive_capacitances, weak=False)
        if capacitances_received.wait(5):
            return capacitances_received.capacitances
        else:
            raise IOError('Timed out waiting for sensitive capacitance measurement.')
    finally:
        proxy.signals.signal('sensitive-capacitances').disconnect(_on_sensitive_capacitances)

# %%
def do_load(min_voltage):
    load_epsilon = 5e-12

    state = {'i': 0}
    middle_completed = threading.Event()
    load_completed = threading.Event()

    [proxy.signals.signal('sensitive-capacitances').disconnect(r)
     for r in proxy.signals.signal('sensitive-capacitances').receivers.values()]

    def on_sensitive_C_updated(message):
        channels, capacitances = zip(*message['C'])
        capacitances = pd.Series(capacitances, index=channels)
        try:
            middle_C = capacitances[middle_i].sum()
        except KeyError:
            middle_C = 0
        try:
            b_neighbours_C = capacitances[b_neighbours_i].sum()
        except KeyError:
            b_neighbours_C = 0

        if not middle_completed.is_set() and middle_C > load_epsilon:
            middle_completed.set()
            proxy.signals.signal('sensitive-capacitances').disconnect(on_sensitive_C_updated)
        if b_neighbours_C > load_epsilon:
            load_completed.set()
            proxy.signals.signal('sensitive-capacitances').disconnect(on_sensitive_C_updated)
        # print('\r' + 50 * ' ', end='')
        # print('\r%-50s' % ('[%d] b neighbours: %sF' %
        #                    (state['i'], si.si_format(b_neighbours_C))), end='')
        state['i'] += 1

    proxy.voltage = min_voltage
#     print('%sV' % si.si_format(proxy.voltage))
    apply_duty_cycles(proxy, pd.Series(1, index=a_i + middle_i))
    proxy.signals.signal('sensitive-capacitances').connect(on_sensitive_C_updated, weak=False)
    if middle_completed.wait(10):
        logging.debug('Covered middle')
    apply_duty_cycles(proxy, pd.Series(1, index=middle_i + b_i + b_neighbours_i))
    proxy.signals.signal('sensitive-capacitances').connect(on_sensitive_C_updated, weak=False)

    start = time.time()
    if load_completed.wait(10):
        end = time.time()
        logging.debug('load completed: %ss', si.si_format(end - start))
    capacitances = pd.concat([pd.Series(proxy.y(), index=proxy.sensitive_channels) for i in range(10)]).groupby(level=0).mean()

    apply_duty_cycles(proxy, pd.Series(1, index=b_i))
    time.sleep(.5)
    target_C_b = pd.Series(proxy.y(), index=proxy.sensitive_channels).sum()
    return target_C_b


def do_dispense_split(target_C_b, min_voltage, max_voltage, ready_wait=3, split_wait=2.5, alpha=1.,
                      K_ux=1.):
    neck_epsilon = 4e-12
    split_ctrl = DropSplitController(proxy, a_i, b_i, a_neighbours_i, b_neighbours_i, middle_i)

    # See https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    K_u = .2 / 1e-12 # Determined empirically, consistent oscillations
    K_u *= K_ux
    T_u=2 * .365  # Oscilation period

    # pid = ivPID.PID(P=K_u)
    # pid = ivPID.PID(P=.5 * K_u)  # P
    # pid = ivPID.PID(P=.45 * K_u, I=T_u / 1.2)  # PI
    # pid = ivPID.PID(P=.8 * K_u, D=T_u / 8)  # PD
    # pid = ivPID.PID(P=.6 * K_u, I=T_u / 2, D=T_u / 8)  # ["classic" PID][1]
    # pid = ivPID.PID(P=.7 * K_u, I=T_u / 2.5, D=3 * T_u / 20)  # [Pessen Integral Rule][1]
    # pid = ivPID.PID(P=.33 * K_u, I=T_u / 2, D=T_u / 3)  # ["some" overshoot][1]
    # pid = ivPID.PID(P=.2 * K_u, I=T_u / 2, D=T_u / 3)  # ["no" overshoot][1]
    # [1]: http://www.mstarlabs.com/control/znrule.html

    pid = ivPID.PID(P=.6 * K_u, I=T_u / 2, D=T_u / 8)  # ["classic" PID][1]

    pid.setPoint = .5

    state = {'i': 0, 'capacitance': {}}
    ready_for_split = threading.Event()
    dispense_completed = threading.Event()

    def on_pid_updated(sender, **kwargs):
        split_C = kwargs['split_capacitances']
        channel_C = kwargs['sensitive_capacitances']
        b_neighbours_C = channel_C[split_ctrl.b_neighbours].sum()
        state['capacitance'][time.time()] = kwargs.copy()

        if state['i'] == 0:
            state['target_C_middle'] = .66 * split_C['middle']
        elif split_C['middle'] <= state['target_C_middle']:
            ready_for_split.set()

        pid.update(split_C['b'] - alpha * target_C_b)

        if split_C['middle'] < neck_epsilon:
            dispense_completed.set()

#         print('\r%-50s' % ('[%d] C_a=%sF, C_b=%sF (neighbours: %sF), C_middle=%sF, force=%.02f' %
#                            (state['i'], si.si_format(split_C['a']),
#                             si.si_format(split_C['b']),
#                             si.si_format(b_neighbours_C),
#                             si.si_format(split_C['middle']), pid.output)), end='')
        try:
            split_ctrl.set_force(pid.output)
        except AttributeError:
            pass
        state['i'] += 1

    split_ctrl.signals.signal('split-capacitances-updated').connect(on_pid_updated, weak=False)

    # [proxy.signals.signal('sensitive-capacitances').disconnect(r)
    #  for r in proxy.signals.signal('sensitive-capacitances').receivers.values()]

    proxy.voltage = min_voltage

#     print('%sV' % si.si_format(proxy.voltage))

    split_ctrl.start(0, True)
    start = time.time()

    ready_for_split.wait(ready_wait)
    end = time.time()
    state['ready_for_split'] = end

    split_time = time.time()
    proxy.voltage = max_voltage
#     print('%sV' % si.si_format(proxy.voltage))

    dispense_completed.wait(split_wait)
    end = time.time()
    state['dispense_completed'] = end

    split_ctrl.signals.signal('split-capacitances-updated').disconnect(on_pid_updated)
    split_ctrl.stop()
    apply_duty_cycles(proxy, pd.Series(1, index=split_ctrl.b + split_ctrl.b_neighbours))
    split_capacitances = measure_sensitive()
    while split_capacitances.sum() > 2 * target_C_b:
        logging.debug('false reading for C_b: %sF' %
                      si.si_format(split_capacitances.sum()))
        apply_duty_cycles(proxy, pd.Series(1, index=split_ctrl.b + split_ctrl.b_neighbours))
        split_capacitances = measure_sensitive()
    capacitance = split_capacitances.sum()
    apply_duty_cycles(proxy, pd.Series(1, index=split_ctrl.b))
    state.update({'C_b': capacitance, 'outputs': split_ctrl.outputs,
                  'target_C_b': target_C_b, 'split_time': split_time,
                  'split_capacitances': split_capacitances})
    return state


def do_dispense(min_voltage=95, max_voltage=110, **kwargs):
    target_C_b = do_load(min_voltage)
#     print('\nTarget C: %sF' % si.si_format(target_C_b))
    split_result = do_dispense_split(target_C_b, min_voltage, max_voltage, **kwargs)
#     print('\nResult C: %sF' % si.si_format(split_result['C_b']))
    return split_result

def clean_up():
#     channels.extend([[5, 114]])
#     for d in ['right'] * 4:
#         move_dir(d)
#         time.sleep(1)
#     for d in ['up'] * 3:
#         move_dir(d)
#         time.sleep(1)

    channels.extend([[28]])
    for d in ['left'] * 2:
        move_dir(d)
        time.sleep(1)
    channels.extend([[12]])
    for d in ['right'] * 2 + ['up'] * 3:
        move_dir(d)
        time.sleep(1)

# %%
import networkx as nx

# %%
G = nx.Graph()
adjacency_list = proxy.neighbours.dropna().to_frame().reset_index(level=0).astype(int).values
adjacency_list.sort(axis=1)
G.add_edges_from(map(tuple, pd.DataFrame(adjacency_list).drop_duplicates().values))


def shortest_path(source, target):
    return nx.shortest_path(G, source, target)[1::]

# %%
def move(source, target, trail_length=1, delay=1.):
    path = list(shortest_path(source, target))
    for i in range(len(path)):
        duty_cycles = pd.Series(1, index=path[i:i + trail_length])
        apply_duty_cycles(proxy, duty_cycles)
        time.sleep(delay)

def shortest_path(source, target):
    return nx.shortest_path(G, source, target)[1::]

# %%
# alpha = .62
# alpha = 1.
#     alpha = .54


def dispense_w_retry(alpha=1., retries=10, beta=.1, **kwargs):
    try:
        apply_duty_cycles(proxy, pd.Series(1, index=middle_i + b_i))
        time.sleep(2.)

        for i in range(retries):
            result = do_dispense(alpha=alpha, **kwargs)
            ratio = result['C_b'] / result['target_C_b']
            if ratio <= 1 + beta and ratio >= 1 - beta:
                result['alpha'] = alpha
                result['i'] = i
                logging.debug('success with alpha=%.2f (C_b=%sF, target_C_b=%sF, ratio=%.2f)',
                              alpha, si.si_format(result['C_b']),
                              si.si_format(result['target_C_b']),
                              ratio)
                break
            else:
                alpha *= 1. / ratio
                logging.debug('retry with alpha=%.2f (C_b=%sF, target_C_b=%sF, ratio=%.2f)',
                              alpha, si.si_format(result['C_b']),
                              si.si_format(result['target_C_b']),
                              ratio)
                if ratio > 2:
                    map(logging.debug, str(result['split_capacitances']).splitlines())
                apply_duty_cycles(proxy, pd.Series(1, index=middle_i + b_i))
                time.sleep(2.)
                apply_duty_cycles(proxy, pd.Series(1, index=a_i + middle_i))
                time.sleep(2.)
        else:
            raise RuntimeError('Failed to reach within 10% of target volume.')
    finally:
        for r in proxy.signals.signal('sensitive-capacitances').receivers.values():
            proxy.signals.signal('sensitive-capacitances').disconnect(r)
    return result

# %%
middle_i = [22]
split_channels_ = split_channels(middle_i, a_count=2)
a_i =  split_channels_['a']
b_i =  split_channels_['b']
a_neighbours_i =  split_channels_['a_neighbours']
b_neighbours_i =  split_channels_['b_neighbours']
a_neighbours_i, a_i, middle_i, b_i, b_neighbours_i

# %%
# apply_duty_cycles(proxy, pd.Series(1, index=[26]))
a_neighbours_i, a_i, middle_i, b_i, b_neighbours_i

# %%
apply_duty_cycles(proxy, pd.Series(1, index=a_i + middle_i))

# %%
result = dispense_w_retry(max_voltage=115, ready_wait=0, split_wait=4.5, K_ux=1., alpha=.95)

# %%
# move(16, 14)
# move(16, 32)
# move(16, 107)
# move(16, 91)
# move(12, 21)
# move(107, 21)
# move(33, 21)

# %%
# proxy.refresh_drops(3.5e-12)

import itertools as it

# proxy.stop_switching_matrix()
channels_C = pd.Series(proxy.all_channel_capacitances())

for step in it.izip_longest(*[shortest_path(i, 26) for i in channels_C[channels_C > 3e-12].index]):
    channels = set(c for c in step if c is not None)
    print('\r%-50s' % channels, end='')
    apply_duty_cycles(proxy, pd.Series(1, index=sorted(channels)))
    time.sleep(1.)

# %%



# %%
proxy.stop_switching_matrix()

# %%
# move(103, 102)
# move(102, 21)
# move(12, 21)
# move(28, 21)

# %%
alphas = [1., .64]

# %%
results = []
for i in range(10):
    print(i)
    results_i = []
    try:
        result = dispense_w_retry(max_voltage=115, ready_wait=0, split_wait=2.5, K_ux=.7, alpha=alphas[0])
        results_i.append(result)
        alphas[0] = result['alpha']
        move(21, 28)

        result = dispense_w_retry(max_voltage=115, ready_wait=0, split_wait=3.5, K_ux=.7, alpha=alphas[1])
        results_i.append(result)
        alphas[1] = result['alpha']
        move(21, 22)
    finally:
        results.append(results_i)
        move(28, 90)
        move(90, 26, trail_length=3)

# %%
import json_tricks

# %%
with open('2018.10.25T16h04 - PID dispense results-03.json', 'w') as output:
    json_tricks.dump(results, output, indent=4)

# %%
!start .

# %%
proxy.stop_switching_matrix()
proxy.turn_off_all_channels()
proxy.state_of_channels = pd.Series(1, index=middle_i + b_i)

# %%
proxy.turn_off_all_channels()
proxy.state_of_channels = pd.Series(1, index=a_i + b_i)

# %%
for i in (103, 12, 28):
    move(i, 21)
# apply_duty_cycles(proxy, pd.Series(0, index=middle_i))
# do_dispense(max_voltage=110, ready_wait=5, split_wait=5, alpha=1. * alpha)

# %%
def dispense_2(alpha=1.):
    results = []

    proxy.stop_switching_matrix()
    time.sleep(.2)

    results.append(do_dispense(max_voltage=115, ready_wait=5, split_wait=2.5, alpha=alpha))
    channels.extend([[21]])
    move_dir('down')
    time.sleep(1)
    for d in ['right'] * 2:
        move_dir(d)
        time.sleep(1)
    results.append(do_dispense(max_voltage=115, ready_wait=5, split_wait=2.5, alpha=alpha))
    channels.extend([[21]])
    move_dir('down')
    time.sleep(1)
    for d in ['left'] * 2:
        move_dir(d)
        time.sleep(1)
    return results

# %%
results = []

for i in range(5):
    results.extend(dispense_2(1.))
    clean_up()

# %%
# result_dfs = []
df_results = pd.DataFrame(results)
result_dfs.append(df_results)

# %%
display(df_results[['target_C_b', 'C_b']].applymap(si.si_format))
display((df_results['target_C_b'] / df_results['C_b']).iloc[1::2].median())

# %%
# df_results.iloc[0]['capacitance']

# %%
df_results.to_dict()

# %%
state = df_results.iloc[3]
all_C = (pd.concat({t: v['sensitive_capacitances'] for t, v in state['capacitance'].items()})
         .unstack(level=1).sort_index())
all_C['b'] = all_C[b_i + b_neighbours_i].sum(axis=1)
fig, axis = plt.subplots(1)
axis.yaxis.set_major_formatter(F_formatter)
all_C.plot(legend=True, ax=axis)
axis.plot([state['split_time']] * 2, axis.get_ylim())
display(all_C.iloc[0].map(si.si_format))
all_C.iloc[-1].map(si.si_format)

# %%
channels = [[98, 16]]

def move_dir(dir_):
    channels.append(proxy.neighbours.loc[channels[-1], dir_].dropna().astype(int).tolist())
    apply_duty_cycles(proxy, pd.Series(1, index=channels[-1]))

buttons = {dir_: ipw.Button(description=dir_.title()) for dir_ in ('left', 'up', 'down', 'right')}
for dir_, b in buttons.items():
    b.on_click(ft.partial(lambda *args: move_dir(args[0]), dir_))

ipw.HBox(buttons.values())

# %%
def reset_dispense():
    apply_duty_cycles(proxy, pd.Series(1, index=middle_i + b_i))
    time.sleep(.75)
    apply_duty_cycles(proxy, pd.Series(1, index=middle_i))
    time.sleep(.75)
    apply_duty_cycles(proxy, pd.Series(1, index=a_i))

# reset_dispense()

# %%
import matplotlib.pyplot as plt

# %%
def do_dispense(proxy, min_voltage=95, max_voltage=110, epsilon=.75e-12):
    proxy.voltage = min_voltage
    # See https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    K_u=.5 / 10e-12  # Determined empirically, consistent oscillations
    T_u=2 * .365  # Oscilation period
    pid = ivPID.PID(P=.6 * K_u, I=T_u / 2, D=T_u / 8)  # ["classic" PID][1]
    print('%sV' % si.si_format(proxy.voltage))

    split_ctrl = DropSplitController(proxy, a_i, b_i, a_neighbours_i, b_neighbours_i, middle_i)
    split_ctrl.start(pid, .25, False)

    center_complete = threading.Event()

    def on_pid_updated(sender, **kwargs):
        split_C = kwargs['split_capacitances']
        if split_C['middle'] > split_C['a'] and split_C['middle'] > split_C['b'] and abs(split_C['a'] - split_C['b']) < epsilon:
            center_complete.set()

    outputs = {}
    split_ctrl.signals.signal('pid-updated').connect(on_pid_updated, weak=False)
    start = time.time()
    if center_complete.wait(10):
        end = time.time()
        print('duration: %ss' % si.si_format(end - start))
        split_ctrl.stop()
    outputs['center'] = split_ctrl.outputs.copy()

    # ------------------------------------------------

    proxy.voltage = .5 * (min_voltage + max_voltage)
    print('%sV' % si.si_format(proxy.voltage))
    pid = ivPID.PID(P=.6 * .2 * K_u, I=T_u / 2, D=T_u / 8)  # ["classic" PID][1]
    split_ctrl.start(pid, 0, True)

    center_complete = threading.Event()

    def on_pid_updated(sender, **kwargs):
        split_C = kwargs['split_capacitances']
        if split_C['middle'] < .5 * (split_C['a'] + split_C['b']) and abs(split_C['a'] - split_C['b']) < epsilon:
            center_complete.set()

    split_ctrl.signals.signal('pid-updated').connect(on_pid_updated, weak=False)
    start = time.time()

    if center_complete.wait(15):
        end = time.time()
        print('hv center duration: %ss' % si.si_format(end - start))
    # for i in range(10):
    #     if center_complete.wait(5):
    #         end = time.time()
    #         print('hv center duration: %ss' % si.si_format(end - start))
    #         break
    #     else:
    #         proxy.voltage += .1 * (.5 * (max_voltage - min_voltage))
    #         print('%sV' % si.si_format(proxy.voltage))

    # ------------------------------------------------

    split_complete = threading.Event()

    def on_pid_updated(sender, **kwargs):
        split_C = kwargs['split_capacitances']
        if split_C['middle'] < epsilon:
            split_complete.set()

    split_ctrl.signals.signal('pid-updated').connect(on_pid_updated, weak=False)

    proxy.voltage = max_voltage
    print('%sV' % si.si_format(proxy.voltage))
    # split_ctrl.K_u = min_voltage / proxy.voltage * K_u0
    start = time.time()
    if split_complete.wait(10):
        end = time.time()
        print('split duration: %ss' % si.si_format(end - start))
    split_ctrl.stop()
    apply_duty_cycles(proxy, pd.Series(1, index=a_i + b_i))
    outputs['split'] = split_ctrl.outputs.copy()
    return outputs

# %% [markdown]
# -----------------------------------------------------------

# %%
def do_split(proxy, min_voltage=85, max_voltage=110, epsilon=.75e-12):
    proxy.voltage = min_voltage
    # See https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    K_u=.5 / 10e-12  # Determined empirically, consistent oscillations
    T_u=2 * .365  # Oscilation period
    pid = ivPID.PID(P=.6 * K_u, I=T_u / 2, D=T_u / 8)  # ["classic" PID][1]
    print('%sV' % si.si_format(proxy.voltage))

    split_ctrl = DropSplitController(proxy, a_i, b_i, a_neighbours_i, b_neighbours_i, middle_i)
    split_ctrl.start(pid, .25, False)

    center_complete = threading.Event()

    def on_pid_updated(sender, **kwargs):
        split_C = kwargs['split_capacitances']
        if split_C['middle'] > split_C['a'] and split_C['middle'] > split_C['b'] and abs(split_C['a'] - split_C['b']) < epsilon:
            center_complete.set()

    outputs = {}
    split_ctrl.signals.signal('pid-updated').connect(on_pid_updated, weak=False)
    start = time.time()
    if center_complete.wait(10):
        end = time.time()
        print('duration: %ss' % si.si_format(end - start))
        split_ctrl.stop()
    outputs['center'] = split_ctrl.outputs.copy()

    # ------------------------------------------------

    proxy.voltage = .5 * (min_voltage + max_voltage)
    print('%sV' % si.si_format(proxy.voltage))
    pid = ivPID.PID(P=.6 * .2 * K_u, I=T_u / 2, D=T_u / 8)  # ["classic" PID][1]
    split_ctrl.start(pid, 0, True)

    center_complete = threading.Event()

    def on_pid_updated(sender, **kwargs):
        split_C = kwargs['split_capacitances']
        if split_C['middle'] < .5 * (split_C['a'] + split_C['b']) and abs(split_C['a'] - split_C['b']) < epsilon:
            center_complete.set()

    split_ctrl.signals.signal('pid-updated').connect(on_pid_updated, weak=False)
    start = time.time()

    if center_complete.wait(15):
        end = time.time()
        print('hv center duration: %ss' % si.si_format(end - start))
    # for i in range(10):
    #     if center_complete.wait(5):
    #         end = time.time()
    #         print('hv center duration: %ss' % si.si_format(end - start))
    #         break
    #     else:
    #         proxy.voltage += .1 * (.5 * (max_voltage - min_voltage))
    #         print('%sV' % si.si_format(proxy.voltage))

    # ------------------------------------------------

    split_complete = threading.Event()

    def on_pid_updated(sender, **kwargs):
        split_C = kwargs['split_capacitances']
        if split_C['middle'] < epsilon:
            split_complete.set()

    split_ctrl.signals.signal('pid-updated').connect(on_pid_updated, weak=False)

    proxy.voltage = max_voltage
    print('%sV' % si.si_format(proxy.voltage))
    # split_ctrl.K_u = min_voltage / proxy.voltage * K_u0
    start = time.time()
    if split_complete.wait(10):
        end = time.time()
        print('split duration: %ss' % si.si_format(end - start))
    split_ctrl.stop()
    apply_duty_cycles(proxy, pd.Series(1, index=a_i + b_i))
    outputs['split'] = split_ctrl.outputs.copy()
    return outputs

# %%
proxy.set_switching_matrix_row_count(40)
proxy.update_config(capacitance_n_samples=50)

# %%
def _merge():
    apply_duty_cycles(proxy, pd.concat([pd.Series(0, index=a_i),
                                        pd.Series(0, index=b_i),
                                        pd.Series(1, index=middle_i)]))

def _bias_left():
    apply_duty_cycles(proxy, pd.concat([pd.Series(.25, index=a_i),
                                        pd.Series(0, index=b_i),
                                        pd.Series(1, index=middle_i)]))


def _bias_right():
    apply_duty_cycles(proxy, pd.concat([pd.Series(0, index=a_i),
                                        pd.Series(.25, index=b_i),
                                        pd.Series(1, index=middle_i)]))

# %%
outputs = []

for i in range(10):
    _merge()
    time.sleep(.5)
    if i % 2 == 0:
        _bias_left()
    else:
        _bias_right()
    time.sleep(1.)
    outputs_i = do_split(proxy)
    outputs.append(outputs_i)
    time.sleep(2.)

# %%
import json_tricks

# %%
with open('2018.10.24T01h20 - PID outputs.json', 'w') as output:
    json_tricks.dump(outputs, output, indent=4)

# %%
!start .

# %%
import matplotlib.pyplot as plt

fig, axes = plt.subplots(2, figsize=(15, 8))

pid_outputs = pd.concat({i: pd.concat({k: pd.Series(o)
                                       for k, o in outputs_i.items()})
                         for i, outputs_i in enumerate(outputs)}).reorder_levels([1, 0, 2]).sort_index()
# pid_outputs.groupby(level=[0, 1]).plot(legend=True)
for axis_i, output_name_i in zip(axes, ('center', 'split')):
    axis_i.set_title('PID output during "%s" stage' % output_name_i)
    pid_outputs[output_name_i].groupby(level=0).plot(style='x-', legend=True, ax=axis_i)

# %%



# %%
# Create button to turn on _only_ the channels associated with **a** and **b** groups, i.e.,
# _not_ the **middle** group or the extended neighbours of **a** and **b**.
button_finalize_split = ipw.Button(description='Finalize split')
button_finalize_split.on_click(lambda *args: apply_duty_cycles(proxy, pd.Series(1, index=a_i + b_i)))

# Create button to merge the daughter drops in **a** and **b** back together across the
# **middle** electrode(s).
button_merge = ipw.Button(description='Merge')
button_merge.on_click(lambda *args: _merge())

button_left = ipw.Button(description='Bias left')
button_left.on_click(lambda *args: _bias_left())

button_right = ipw.Button(description='Bias right')
button_right.on_click(lambda *args:

ipw.HBox([button_finalize_split, button_merge, button_left, button_right])

# %%
proxy.stop_switching_matrix()
proxy.set_switching_matrix_row_count(30)

# %%



# %%
# proxy.voltage = 105
proxy.voltage = 85

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
on_force_change({'new': force_slider.value})

amplitude_slider = ipw.FloatSlider(description='Amplitude:', min=0, max=1, value=0)
amplitude_slider.observe(on_amplitude_change, names=['value'])
on_amplitude_change({'new': 0})

ipw.VBox([force_slider, amplitude_slider])

# %% [markdown]
# ## Re-merge drop

# %%
def move(dir_, channels):
    proxy.stop_switching_matrix()
    neighbours = proxy.neighbours.loc[channels, dir_].dropna().astype(int)
#     proxy.state_of_channels = pd.Series(1, index=neighbours)
    apply_duty_cycles(pd.Series(1, neighbours))
    return neighbours.tolist()

# %%
proxy.stop_switching_matrix()

# %%
proxy.turn_off_all_channels()
proxy.state_of_channels = pd.Series(1, index=[45])
proxy.capacitance(0)

# %%



# %%
with proxy.transaction_lock:
    sensitive_capacitances = pd.Series(proxy.y(), index=proxy.sensitive_channels)
sensitive_capacitances

# %%
channels = [119, 107, 95, 78, 66]
buttons = {dir_i: ipw.Button(description=dir_i.title()) for dir_i in ('up', 'down', 'left', 'right')}

for dir_i, b in buttons.items():
    def _foo(dir_i, *args):
        global channels
        channels = move(dir_i, channels)

    b.on_click(ft.partial(_foo, dir_i))

ipw.HBox(buttons.values())

# %%
sheet_capacitance = (sensitive_capacitances / channel_areas.loc[sensitive_capacitances.index]).median()
sheet_capacitance

# %%
import dropbot.threshold
import dropbot.chip

# %%
chip_info = dropbot.chip.chip_info(svg_path)

# %%
import trollius as asyncio

# %%
loop = asyncio.ProactorEventLoop()
asyncio.set_event_loop(loop)

# %%
chip_info['electrode_shapes'] = df_shape_infos

# %%
dir_ = 'down'
channels = proxy.neighbours.loc[channels, dir_].dropna().astype(int).tolist()
channels

# %%
loop.run_until_complete(db.threshold.execute_actuation(proxy, chip_info, sheet_capacitance, channels, duration_s=10, volume_threshold=.98))
