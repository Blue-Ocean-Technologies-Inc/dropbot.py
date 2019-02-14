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

# %%
import ivPID

# %%
# pid = ivPID.PID()

# pid.update(feedback)
# output = pid.output
# if pid.SetPoint > 0:
#     feedback += (output - (1/i))
# if i>9:
#     pid.SetPoint = 1
# time.sleep(0.02)

# feedback_list.append(feedback)
# setpoint_list.append(pid.SetPoint)
# time_list.append(i)

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
    proxy.update_state(drops_update_interval_ms=int(0))
    time.sleep(1.)
    monitor_task.cancel()

buttons = {'disconnect': ipw.Button(description='Disconnect'),
           'connect': ipw.Button(description='Connect')}
buttons['disconnect'].on_click(close)
buttons['connect'].on_click(connect)
ipw.HBox(buttons.values())

# %%
row_count = 40
proxy.set_switching_matrix_row_count(row_count)

# %%
# proxy.turn_off_all_channels()
# proxy.state_of_channels = pd.Series(1, index=[25, 29])

# %%
@cancellable
@asyncio.coroutine
def plot_vec_y():
    def _plot():
        axis.cla()
        with proxy.transaction_lock:
            capacitances = pd.Series(proxy.y(), index=proxy.sensitive_channels)
        capacitances.plot(kind='bar', ax=axis)
#         axis.relim()
#         axis.autoscale_view()
        axis.set_ylim(0, max(30e-12, capacitances.max()))
        axis.yaxis.set_major_formatter(F_formatter)
        fig.canvas.draw()

    try:
        while True:
            try:
                _plot()
            except TypeError:
                pass
            yield asyncio.From(asyncio.sleep(.4))
    except asyncio.CancelledError:
        pass

plot_thread = threading.Thread(target=plot_vec_y)
plot_thread.daemon = True
plot_thread.start()

# %%
# plot_vec_y.cancel()

# %%
proxy.update_state(voltage=115)
# Neck electrode(s).
middle = [29]
# middle = [86]
# Electrodes to split left and right.
# a, b = [24, 25], [33, 32]
a, b = [16, 25], [33, 40]
# a, b = [25], [33]
# a, b = [90, 86], [28, 32]
# a, b = [29, 90], [40, 79]

all_channels = a + b + middle
proxy.set_duty_cycle(0, all_channels)
# proxy.set_duty_cycle(0, a + b)
# proxy.set_duty_cycle(1, middle)
proxy.set_sensitive_channels(all_channels)
proxy.start_switching_matrix(row_count, 0.005)

def measure_sensitive():
    return pd.Series(proxy.y(), index=proxy.sensitive_channels)

# %%
pid = ivPID.PID(P=.25 / 10e-12)

# %%
c_ = measure_sensitive()
display(c_.map(si.si_format))
pid.SetPoint = .5 * c_[a + b].sum()
epsilon = .05 * pid.SetPoint

# %%
proxy.set_duty_cycle(1, middle)
proxy.set_sensitive_channels(all_channels)
proxy.start_switching_matrix(row_count, 0.005)

d_a_prev = None
d_b_prev = None

i = 0
with proxy.transaction_lock:
    c_ = pd.Series(proxy.y(), index=proxy.sensitive_channels)
    
steps = []
while abs(c_[a].sum() - pid.SetPoint) > epsilon:
    pid.update(c_[a].sum())
#     d_a = min(1, max(2. / row_count, abs(pid.output)))
    d_a = min(1, max(1. / row_count, abs(pid.output)))
    d_b = 0

    if pid.output <= 0:
        d_a, d_b = d_b, d_a

    data = pd.Series({'d_a': d_a, 'd_b': d_b, 'output': pid.output})
    print('\r' + 50 * ' ', end='')
    print('\r%-50s' % ('[%d] %s, %s' % (i, data.map(si.si_format).to_dict(), c_.map(si.si_format).to_dict())),
          end='')
    steps.append(pd.concat([c_, data]))
        
    if d_a_prev != d_a or d_b_prev != d_b:
        with proxy.transaction_lock:
            proxy.stop_switching_matrix()
            proxy.set_duty_cycle(d_a, a)
            proxy.set_duty_cycle(d_b, b)
            proxy.start_switching_matrix(row_count, 0.005)
        time.sleep(0.35)
        d_a_prev = d_a
        d_b_prev = d_b
    i += 1
    
    with proxy.transaction_lock:
        c_ = pd.Series(proxy.y(), index=proxy.sensitive_channels)

# Try to split.
with proxy.transaction_lock:
    proxy.stop_switching_matrix()
    proxy.set_duty_cycle(0, middle)
    proxy.set_duty_cycle(1, a + b)
    proxy.start_switching_matrix(row_count, 0.005)

# %%
df_i = pd.DataFrame(steps)[a + b + middle].plot(kind='bar')
# df_i = pd.DataFrame(steps)[a + b + middle].T
# df_i['group'] = None
# df_i.loc[a, 'group'] = 'a'
# df_i.loc[b, 'group'] = 'b'
# df_i.loc[middle, 'group'] = 'middle'
# df_i.groupby('group').sum().T.plot(kind='bar')

# %%
proxy.stop_switching_matrix()
proxy.turn_off_all_channels()
# proxy.state_of_channels = pd.Series(1, index=a[1:] + middle)
proxy.state_of_channels = pd.Series(1, index=b[:1] + middle)
# proxy.state_of_channels = pd.Series(1, index=middle + a + b)
time.sleep(.1)
proxy.turn_off_all_channels()
proxy.state_of_channels = pd.Series(1, index=middle)
proxy.turn_off_all_channels()

# %%
def foo(on_channels, off_channels, on_duty=1.):
    with proxy.transaction_lock:
        orig_state = proxy.state.copy()
        proxy.update_state(voltage=115)
        try:
            proxy.stop_switching_matrix()
            proxy.set_duty_cycle(0, off_channels)
            proxy.set_duty_cycle(on_duty, on_channels)
            proxy.set_sensitive_channels(on_channels + off_channels)
            proxy.start_switching_matrix(row_count, .005)
        finally:
            final_state = proxy.state.copy()
            proxy.update_state(**final_state[final_state != orig_state])

# %%
@asyncio.coroutine
def position(a, b, middle, ratio=.5, on_duty=.05, alpha=.05):
    try:
        foo([], a + b + middle)
        while True:
            try:
                c_i = pd.Series(proxy.y(), index=proxy.sensitive_channels)
                break
            except IndexError:
                continue
        if c_i[a].sum() > c_i[b].sum():
            a, b = b, a
            ratio = 1 - ratio
        target = ratio * c_i[a + b].sum()
        all_channels = a + b + middle
        epsilon = alpha * c_i[a + b].sum()

        done = False
        while not done:
            with proxy.transaction_lock:
                proxy.stop_switching_matrix()
                proxy.set_duty_cycle(0, b)
                proxy.set_duty_cycle(on_duty, middle + a)
                proxy.set_sensitive_channels(all_channels)
    #             print('\r%-72s' % ('off: `%s`, on: `%s`' % (b, middle + a)))
                proxy.start_switching_matrix(row_count, .005)
            while True:
                with proxy.transaction_lock:
                    c_i = pd.Series(proxy.y(), index=proxy.sensitive_channels)
                diff = abs(c_i[a].sum() - target)
                if diff <= epsilon:
                    done = True
                    break
                elif c_i[a].sum() >= target:
                    a, b = b, a
                    target = (1 - ratio) * c_i[a + b].sum()
    #                 print('change direction')
                    break
                yield asyncio.From(asyncio.sleep(0.001))
    #         display(c_i.map(si.si_format))
    except asyncio.CancelledError:
        proxy.stop_switching_matrix()

# %%
loop = asyncio.ProactorEventLoop()
asyncio.set_event_loop(loop)

# %%
import time

from IPython.display import Markdown


def on_toggle(message):
    '''
    message : dict
        Example:
        ```
        {'owner': ToggleButton(value=True, description=u'Merge'), 'new': True, 'old': False, 'name': 'value', 'type': 'change'},), kwargs: {}
        ```
    '''
    if message['new']:
        button_merge_split.description = 'Merge'
        on_center()
        foo(a + b, middle)
        for button in (button_skew, button_center):
            button.disabled = True
    else:
        button_merge_split.description = 'Split'
        foo(middle, a + b)
        for button in (button_skew, button_center):
            button.disabled = False

# Neck electrode(s).
middle = [29]
# Electrodes to split left and right.
a, b = [24, 25], [33, 32]
# a, b = [25], [33]
# Electrodes to split up and down.
# a, b = [28], [90]
# a, b = [24, 28], [90, 94]

button_merge_split = ipw.ToggleButton(description='Split')
button_merge_split.observe(on_toggle, names=['value'])

def on_skew(message):
    loop = asyncio.get_event_loop()
    try:
        task = loop.create_task(position(a, b, middle, ratio=.65, on_duty=.2))
        loop.run_until_complete(asyncio.wait([task], timeout=5))
    except asyncio.TimeoutError:
        pass
    proxy.stop_switching_matrix()

def on_center(*args):
    loop = asyncio.get_event_loop()
    task = loop.create_task(position(a, b, middle, ratio=.5, on_duty=.05, alpha=.03))
    loop.run_until_complete(task)
    proxy.stop_switching_matrix()

button_skew = ipw.Button(description='Skew')
button_skew.on_click(on_skew)
button_center = ipw.Button(description='Center')
button_center.on_click(on_center)

display(Markdown('''
# Procedure

 1. Press `Skew` to skew liquid to one side, ~3:1 ratio
 2.  _(Optional)_ Press `Center` to automatically center (using low duty cycle), i.e., ~1:1 ratio.
 3. Press `Split` to **a)** automatically center drop; and **b)** apply full duty cycle to outer electrodes and minimal duty cycle to middle electrode.
 4. Press `Merge` to merge the liquid back into a single drop.
 5. Repeat 1-4.
'''))

display(ipw.HBox([button_skew, button_center, button_merge_split]))

def reset():
    '''
    Merge liquid into single drop for consistent starting state.
    '''
    all_channels = a + b + middle
    proxy.set_duty_cycle(0, a + b + middle)
    proxy.set_sensitive_channels(all_channels)
    proxy.start_switching_matrix(row_count, 0.005)
    df_c = pd.DataFrame(proxy.y(), columns=['capacitance'], index=proxy.sensitive_channels)
    df_c['group'] = None
    df_c.loc[a, 'group'] = 'a'
    df_c.loc[b, 'group'] = 'b'
    df_c.loc[middle, 'group'] = 'middle'
    min_group = df_c.groupby('group')['capacitance'].sum().sort_values().index[0]
    if min_group == 'middle':
        # Drop has started in split state.  Trigger merge.
        on_toggle({'new': False})

reset()

# %%
# proxy.update_state(voltage=100)
# proxy.stop_switching_matrix()
# proxy.turn_off_all_channels()

# %%
# capacitances = {}
# for channel in a + b + middle:
#     proxy.turn_off_all_channels()
#     proxy.state_of_channels = pd.Series(1, index=[channel])
#     capacitances[channel] = proxy.capacitance(0)
# proxy.turn_off_all_channels()
# pd.Series(capacitances).map(si.si_format)

# %%
# ## Stop real-time capacitance plot (GTK window).
# plot_vec_y.cancel()
