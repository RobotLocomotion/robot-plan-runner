#%%
import os

import lcm
import numpy as np

import matplotlib.pyplot as plt
from drake import lcmt_iiwa_status, lcmt_iiwa_command

#%%
# log_path = os.path.join("/Users", "pangtao", "PycharmProjects",
#                         "lcmlog-2021-04-26.01")
log_path = os.path.join("/Users", "pangtao", "PycharmProjects",
                        "lcmlog-2021-05-15.02")
lcm_log = lcm.EventLog(log_path)

#%%
iiwa_status_list = []
iiwa_cmd_list = []

# extract time stamps and utimes of status and command messages from log file.
for event in lcm_log:
    if event.channel == "IIWA_STATUS":
        msg = lcmt_iiwa_status.decode(event.data)
        iiwa_status_list.append((event.timestamp, msg.utime))
    elif event.channel == "IIWA_COMMAND":
        msg = lcmt_iiwa_command.decode(event.data)
        iiwa_cmd_list.append((event.timestamp, msg.utime))


#%% match time stamps
# make sure iiwa_cmd_list[0].utime >= iiwa_status_list[0].utime
while iiwa_cmd_list[0][1] < iiwa_status_list[0][1]:
    iiwa_cmd_list.pop(0)

# make sure iiwa_cmd_list[-1].utime == iiwa_status_list[-1].utime
i_start_status = 0
i_end_status = 0
for i, status_times in enumerate(iiwa_status_list):
    if status_times[1] == iiwa_cmd_list[-1][1]:
        i_end_status = i
        break

iiwa_status_list = iiwa_status_list[i_start_status:i_end_status + 1]

print("number of iiwa command msgs: ", len(iiwa_cmd_list))
print("number of iiwa status msgs: ", len(iiwa_status_list))


#%% time difference between adjacent messages
def get_dt(time_list):
    n = len(time_list)
    dt_list = []
    nonuniform_count = 0
    for i in range(n - 1):
        t = time_list[i][1]
        t_next = time_list[i+1][1]
        d_utime = t_next - t
        if d_utime != 5000:
            # print(i, d_utime)
            nonuniform_count += 1
        dt_list.append((time_list[i+1][0] - time_list[i][0])/1000.)  # in milliseconds
    print("Nonuniform messages count: {} / {}".format(nonuniform_count, n))
    return np.array(dt_list)

dt_status = get_dt(iiwa_status_list)
dt_cmd = get_dt(iiwa_cmd_list)

plt.figure(dpi=150)
plt.hist(dt_cmd, label='cmd', bins=20)
plt.hist(dt_status, label='status', bins=20)
plt.ylabel('num messages')
plt.yscale('log')
plt.xlabel('ms')
plt.legend()
plt.title("time difference between adjacent messages\n" + log_path)
plt.show()

#%% time difference between cmd and status messages that have the same utime field.
dt = []
idx_cmd = 0
idx_status = 0
n_cmd = len(iiwa_cmd_list)
n_status = len(iiwa_status_list)

while idx_cmd < n_cmd and idx_status < n_status:
    utime_cmd = iiwa_cmd_list[idx_cmd][1]
    utime_status = iiwa_status_list[idx_status][1]

    if utime_cmd == utime_status:
        dt.append((iiwa_cmd_list[idx_cmd][0] -
                   iiwa_status_list[idx_status][0]) / 1000.)
        idx_cmd += 1
        idx_status += 1
    elif utime_cmd < utime_status:
        idx_cmd += 1
    elif utime_cmd > utime_status:
        idx_status += 1


#%%
dt = np.array(dt)
plt.figure(dpi=200)
plt.hist(dt, bins=20)
plt.xlabel('ms')
plt.title("time difference between cmd and status message\n" + log_path)
plt.yscale('log')
plt.ylabel('num message pairs')
plt.grid(True)
plt.show()

print("dt mean:", np.mean(dt), "dt std", np.std(dt))




