# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:light
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.3'
#       jupytext_version: 1.0.2
#   kernelspec:
#     display_name: Python 2
#     language: python
#     name: python2
# ---

# +
import logging; logging.basicConfig(level=logging.DEBUG)

import dropbot as db
import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import si_prefix as si
# %matplotlib inline


def SI_formatter(suffix):
    format_str = '%%s%s' % suffix
    return mpl.ticker.FuncFormatter(lambda x, *args: format_str %
                                    si.si_format(x))

try:
    proxy.terminate()
except NameError:
    pass

proxy = db.SerialProxy(ignore=True)

default_C16 = proxy.reset_C16()
proxy.voltage = 100

df_C_pre = pd.DataFrame(proxy.on_board_capacitance() for i in range(10))
C16 = proxy.calibrate_C16(reset=False)
df_C_post = pd.DataFrame(proxy.on_board_capacitance() for i in range(10))

df_C = pd.concat([df_C_pre, df_C_post],
                 keys=['%sF' % si.si_format(c)
                       for c in default_C16, C16]).stack().reset_index()
df_C.columns = ['C16', 'i', 'nominal (F)', 'measured (F)']
g = sns.catplot(col='nominal (F)', y='measured (F)', x='C16',
                kind='box', data=df_C, sharey=False)
for ax in g.axes.ravel():
    ax.yaxis.set_major_formatter(SI_formatter('F'))
