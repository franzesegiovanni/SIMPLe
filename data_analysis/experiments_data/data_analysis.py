# -*- coding: utf-8 -*-
"""
Created on Sun Aug 28 18:05:18 2022

@author: timve
"""

import numpy as np
from datetime import datetime
import pathlib
import matplotlib.pyplot as plt

def load(filename):
        data = np.load(str(pathlib.Path().resolve()) + '/data/' + str(filename) + '.npz')
        return data
    
n = 7
data_kin = []
data_tel = []


for i in range(1,n+1):
    try:
        data_kin.append(load(f"dual_{i}_kin_dem"))
    except:
        data_kin.append(0)
        
    try:
        data_tel.append(load(f"dual_{i}_tel_dem"))
    except:
        data_tel.append(0)

plt.figure(0)
print('d')
for i ,d in enumerate(data_kin):
    plt.plot(d['recorded_traj_dual'][0], label=i)
    print(len(d['recorded_traj_dual'][0]))
plt.legend()

plt.figure(1)
print('t')
for i, t in enumerate(data_tel):
    if t != 0:
        plt.plot(t['recorded_traj_dual'][0], label=i)
        print(len(t['recorded_traj_dual'][0]))
    else:
        print(0)
        pass
plt.legend()

#%%
import scipy.stats as st
a=[965, 1795, 1180, 1205]#1795, 1180 , 1205]
b=[8028,4730, 5294, 9430] #my idea is to put 10000 for the example that failed 
(stat, p) = st.ttest_rel(a,b)

#%%
import scipy.stats as st
a=[1116, 965, 1188, 827, 1795, 1180, 1205]#1795, 1180 , 1205]
b=[1.5*9430, 8028, 1.5*9430, 1.5*9430, 4730, 5294, 9430] #my idea is to put 10000 for the example that failed 
(stat, p) = st.ttest_rel(a,b)

#%%

import plotly.graph_objects as go

categories = ['Mental Demand', 'Physical Demand', 'Temporal Demand', 'Effort', 'Frustration']

fig = go.Figure()

fig.add_trace(go.Scatterpolar(
      r=[3.57, 9, 6.86, 4.29, 2.86],
      theta=categories,
      fill='toself',
      name='Kinesthetic Teaching'
))
fig.add_trace(go.Scatterpolar(
      r=[16.43, 3.86, 10.57, 16.57, 11.71],
      theta=categories,
      fill='toself',
      name='Teleoperated Teaching'
))

fig.update_layout(
  polar=dict(
    radialaxis=dict(
      visible=True,
      range=[0, 20]
    )),
  showlegend=False
)

fig.show()
# %%
