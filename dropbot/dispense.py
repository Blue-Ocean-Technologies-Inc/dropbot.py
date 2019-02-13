from __future__ import (absolute_import, print_function, unicode_literals,
                        division)
from itertools import islice
import datetime as dt
import functools as ft
import logging
import os
import threading
import time

import json_tricks
import networkx as nx
import pandas as pd
import trollius as asyncio

from .ivPID import PID

EPSILON = 5e-12
NECK_EPSILON = 4e-12
BETA = 1.3
'''
pandas.Series
    Best estimate of maximum capacitance for each electrode, i.e., when
    electrode is completely covered with liquid.
'''
electrode_C = pd.Series()
log_ = []
log_lock = threading.RLock()


def output_duty_cycles(normalized_force):
    '''
    Map range `[-1, 1]` to duty cycles `(1, 0)` to `(0, 1)`

    Ideally, during a split, both **a** and **b** groups would have maximum
    duty cycle.  However, for various reasons (e.g., surface imperfections),
    the same duty cycle applied to **a** and **b** _MAY_ not result in equal
    distribution.

    This can be addressed by mapping

    ```
      -1               0                1
    (1, 0) --------- (1, 1) --------- (0, 1)
    ```
    '''
    a = 1
    b = 1 - abs(normalized_force)
    if normalized_force > 0:
        a, b = b, a
    return a, b


def apply_duty_cycles(proxy, duty_cycles, set_sensitive=True):
    '''
    Apply duty cycle to each channel; optionally set respective channels as
    sensitive.

    Parameters
    ----------
    duty_cycles : pandas.Series
        Duty cycles indexed by sensitive channel number.
    set_sensitive : bool, optional
        Configure switching matrix controller with indexed channels in
        ``duty_cycles`` as the active sensitive channels **(clears any existing
        sensitive channels)**.
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


@asyncio.coroutine
def read_C(proxy):
    '''
    Returns
    -------
    pandas.Series
        Latest capacitance of each sensitive channel.
    '''
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
def wait_on_capacitance(proxy, callback):
    '''
    Return once callback returns ``True``.

    Parameters
    ----------
    function
        Callback function accepting series of sensitive capacitances as only
        argument.

    Returns
    -------
    dict
        Dictionary containing the following keys::
        - ``C_0``: series of initial sensitive capacitances
        - ``C_T``: series of sensitive capacitances which caused callback to
          return ``True``.
        - ``i``: zero-indexed integer iteration when callback was called
    '''
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

import itertools as it


class MoveTimeout(asyncio.TimeoutError):
    def __init__(self, route, route_i, *args, **kwargs):
        self.route = route
        self.route_i = route_i
        super(MoveTimeout, self).__init__(*args, **kwargs)


@asyncio.coroutine
def move_liquid(proxy, route, trail_length=1, neighbour_epsilon=3e-12,
                wrapper=None):
    '''
    Move liquid along specified route.

    Parameters
    ----------
    route : list
        Route to follow expressed as a list of consecutive electrode IDs.
    trail_length : int, optional
        Number of electrodes to actuate at the same time along route.
    neighbour_epsilon : float, optional
        Minimum capacitance detected on head neighbour electrode before
        continuing to next actuation along route.
    '''
    def check_first(channel, threshold, capacitances):
        if (channel in capacitances) and capacitances[channel] > threshold:
            return True

    if wrapper is None:
        def wrapper(task):
            return task

    try:
        route_i = []
        # Pull liquid to cover first electrode and _at least slightly_ overlap next electrode on route.
        for route_i in it.imap(list, window(route, trail_length + 1)):
            apply_duty_cycles(proxy, pd.Series(1, index=route_i))
            print('\r%-50s' % ('actuated: `%s`' % route_i), end='')
            c_check = ft.partial(check_first, route_i[-1], neighbour_epsilon)
            state = yield asyncio\
                .From(wrapper(wait_on_capacitance(proxy, c_check)))
            apply_duty_cycles(proxy, pd.Series(1, index=route_i[1:]))
            print('\r%-50s' % ('actuated: `%s`' % route_i[1:]), end='')
            c_check = ft.partial(check_first, route_i[-1], 10e-12)
            state = yield asyncio\
                .From(wrapper(wait_on_capacitance(proxy, c_check)))
    except (asyncio.CancelledError, asyncio.TimeoutError) as exception:
        raise MoveTimeout(route, route_i)

    state = apply_duty_cycles(proxy, pd.Series(1, index=route[-trail_length:]))
    raise asyncio.Return(state)


@asyncio.coroutine
def dispense(proxy, route, alpha=1., beta=1.1, epsilon=EPSILON,
             neck_epsilon=2e-12):
    @asyncio.coroutine
    def _attempt_dispense(proxy, route, alpha=1., beta=1.1,
                          head_epsilon=10e-12, epsilon=EPSILON,
                          neck_epsilon=2e-12):
        '''
        Parameters
        ----------
        route : list
            Route to dispense along expressed as a list of consecutive electrode IDs.
        alpha : float, optional
            Corrective factor to compensate for mismatch between measured
            capacitance on head (and neighbour) electrodes and the capacitance of
            the actual dispensed volume.
        beta : float, optional
            Volume factor where target dispensed volume is the capacitance of the
            head electrodes when covered with liquid times the specified value.
        head_epsilon : float, optional
        epsilon : float, optional
        neck_epsilon : float, optional
            Capacitance floor threshold across neck electrodes before neck is
            considered "empty", i.e., dispense is considered complete.
        '''
        # 1. Move liquid along route, one electrode at a time until **head** capacitance exceeds EPSILON.
        yield asyncio.From(move_liquid(proxy, route, trail_length=1,
                                       neighbour_epsilon=epsilon))

        head = route[-2:-1]
        head_neighbours = route[-1:]
        head_index = route.index(head[0])
        neck = [route[head_index - 1]]

        apply_duty_cycles(proxy, pd.Series(1, index=head))
        C = yield asyncio.From(read_C())

        for h in head:
            electrode_C.loc[h] = max(electrode_C.loc[h] if h in electrode_C
                                     else 0, C[head].sum())
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

    alpha_ = alpha
    while True:
        target_C_head, C_head = \
            yield asyncio.From(_attempt_dispense(proxy, route, alpha=alpha,
                                                 beta=beta,
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
            with log_lock:
                log_.append(message)
            break
        else:
            alpha *= target_C_head / C_head if C_head != 0 else 1.
            message['action'] = 'dispense-retry'
            message['alpha'] = alpha
            with log_lock:
                log_.append(message)
#             print('alpha: %.2f' % alpha)
            yield asyncio.From(move_liquid(proxy, route[::-1][:4],
                                           neighbour_epsilon=3e-12))
            yield asyncio.From(move_liquid(proxy, route[:-3],
                                           neighbour_epsilon=3e-12))
    raise asyncio.Return(target_C_head, C_head, alpha)

@asyncio.coroutine
def wait_for_split(proxy, target_C_head, tail_neighbours, tail, neck, head,
                   head_neighbours, neck_epsilon=2e-12):
    '''
    Parameters
    ----------
    target_C_head : float
        Target capacitance for drop split to head electrode.
    tail_neighbours : list
        List of electrode IDs for neighbours next to tail electrode furthest
        from the neck and head.
    tail : list
        List of electrode IDs behind the neck.
    neck : list
        List of electrode IDs defining region where liquid is split.
    head : list
        List of electrode IDs indicating target location of dispensed drop.
    head_neighbours : list
        List of electrode IDs for neighbours next to head electrode furthest
        from the neck and tail.
    neck_epsilon : float
        Capacitance floor threshold across neck electrodes before neck is
        considered "empty", i.e., dispense is considered complete.
    '''
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

    pid = PID(P=.6 * K_u, I=T_u / 2, D=T_u / 8)  # ["classic" PID][1]

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
        with log_lock:
            log_.append({'action': 'apply-force', 'duty_cycles': duty_cycles,
                         'tail_duty_cycle': tail_duty_cycle, 'head_duty_cycle':
                         head_duty_cycle, 'time': time.time()})

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
    with log_lock:
        log_.append({'action': 'split-complete', 'args':
                    dict(zip(('target_C_head', 'tail_neighbours', 'tail',
                              'neck', 'head', 'head_neighbours'),
                             (target_C_head, tail_neighbours, tail, neck, head,
                              head_neighbours))), 'time': time.time()})
    raise asyncio.Return(state)


def neighbours_as_graph(neighbours):
    G = nx.Graph()
    adjacency_list = (neighbours.dropna().to_frame().reset_index(level=0)
                      .astype(int).values)
    adjacency_list.sort(axis=1)
    G.add_edges_from(map(tuple, pd.DataFrame(adjacency_list).drop_duplicates()
                         .values))
    return G


def gather_liquid(proxy, sources, target, **kwargs):
    G = neighbours_as_graph(proxy.neighbours)

    loop = asyncio.get_event_loop()
    for s in sources:
        route = nx.shortest_path(G, s, target)
        loop.run_until_complete(asyncio.wait_for(move_liquid(proxy, route,
                                                             neighbour_epsilon=2e-12,
                                                             trail_length=1),
                                                 timeout=20))
        apply_duty_cycles(proxy, pd.Series(1, index=route[-1:]))


def run_experiment(proxy, route, destinations, tunings=None):
    G = neighbours_as_graph(proxy.neighbours)
    loop = asyncio.get_event_loop()
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
        results[d] = results_i
        route_i = nx.shortest_path(G, route[-2], d)
        task_i = move_liquid(proxy, route_i, neighbour_epsilon=EPSILON,
                             trail_length=1)
        loop.run_until_complete(asyncio.wait_for(task_i, timeout=20))
        apply_duty_cycles(proxy, pd.Series(1, index=route_i[-1:]))

    gather_liquid(destinations[::-1], route[2])

    task = move_liquid(proxy, route[:4][::-1], neighbour_epsilon=EPSILON,
                       trail_length=3)
    loop.run_until_complete(asyncio.wait_for(task, timeout=20))
    apply_duty_cycles(proxy, pd.Series(1, index=route[:2]))
    return results


def test_dispenses(proxy, route, destinations, results=None):
    global apply_duty_cycles

    # source = 93
    # target = 16
    # route = nx.shortest_path(G, source, target)
    # # route = [85, 83, 82, 81, 79, 86]
    # proxy.voltage = 98

    # destinations = [22, 97]  #, 53, 66]
    # destinations = [25, 48]
    # destinations = [111, 86]
    # destinations = [21, 29]
    def log_sensitive_capacitances(message):
        message['time'] = time.time()
        log_.append(message)


    proxy.signals.signal('sensitive-capacitances')\
        .connect(log_sensitive_capacitances, weak=False)

    original_apply_duty_cycles = apply_duty_cycles

    @ft.wraps(original_apply_duty_cycles)
    def apply_duty_cycles(proxy, duty_cycles, set_sensitive=True):
        log_.append({'action': 'apply-duty-cycles', 'duty_cycles': duty_cycles,
                     'set_sensitive': set_sensitive, 'time': time.time()})
        return original_apply_duty_cycles(proxy, duty_cycles, set_sensitive=set_sensitive)

    try:
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
    return results
