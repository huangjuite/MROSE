import pandas as pd

import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib import rc



import seaborn as sns
sns.set_style("darkgrid")


fe = fm.FontEntry(
    fname=r'/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf',
    name="Times New Roman"
)
fm.fontManager.ttflist.insert(0, fe)

fontp = {'family':'Times New Roman', 'size': 18}
pdfp = {'fonttype':42}
mathtp = {'fontset':'dejavuserif'}

# rc('text', usetex=True)
rc('font', **fontp)
rc('pdf', **pdfp)
rc('mathtext', **mathtp)

df = pd.read_csv("/tmp/radar_v_std.csv")
df["stamp"] -= df["stamp"][0]
name_map = {' radar0':'horizontal', ' radar1':'vertical'}
df["radar"] = df["radar"].apply(lambda r: name_map[r])
print(df)

fig = plt.figure(figsize=(8,8))
y_lim = 0.3

ax0 = plt.subplot(3,1,1)
x = sns.lineplot(ax=ax0, data=df, x="stamp", y="std_x", hue="radar")
ax0.set_ylabel(r"$\sigma_x\  (m/s)$")
ax0.set_xlabel("")
ax0.xaxis.set_tick_params(labelbottom=False)
ax0.set_ylim(0, y_lim)

ax1 = plt.subplot(3,1,2)
y = sns.lineplot(ax=ax1, data=df, x="stamp", y="std_y", hue="radar")
ax1.get_legend().set_visible(False)
ax1.set_ylabel(r"$\sigma_y\  (m/s)$")
ax1.set_xlabel("")
ax1.xaxis.set_tick_params(labelbottom=False)
ax1.set_ylim(0, y_lim)

ax2 = plt.subplot(3,1,3)
z = sns.lineplot(ax=ax2, data=df, x="stamp", y="std_z", hue="radar")
ax2.get_legend().set_visible(False)
ax2.set_ylabel(r"$\sigma_z\  (m/s)$")
ax2.set_xlabel("time (s)")
ax2.set_ylim(0, y_lim)

fig.tight_layout()
plt.savefig("/tmp/radar_v_std.pdf", format="pdf", bbox_inches="tight")
plt.show()