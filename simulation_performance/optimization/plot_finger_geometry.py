"""Compare local XY collision footprints: bounds versus contact contour."""
from pathlib import Path
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
base=Path(__file__).resolve().parent
fig,axes=plt.subplots(1,2,figsize=(9,6),sharex=True,sharey=True)
for ax,side in zip(axes,['left','right']):
 for name,color,fill in [('robot_9box.urdf','#3474a1',False),('robot_3box.urdf','#d97b2b',True)]:
  link=ET.parse(base/name).find(f"link[@name='finger_{side}_1']")
  for i,c in enumerate(link.findall('collision')):
   origin=[float(x) for x in c.find('origin').get('xyz').split()]
   size=[float(x) for x in c.find('geometry/box').get('size').split()]
   x,y=origin[0]*1000,origin[1]*1000;w,h=size[0]*1000,size[1]*1000
   ax.add_patch(Rectangle((x-w/2,y-h/2),w,h,fill=fill,alpha=.24 if fill else 1,edgecolor=color,facecolor=color if fill else 'none',linewidth=1.8,label=('3 boxes' if fill else '9 boxes') if i==0 else None))
 ax.set_xlim(-33,33);ax.set_ylim(-26,53);ax.set_aspect('equal');ax.set_title(f'finger_{side}_1');ax.set_xlabel('Local x (mm)');ax.grid(alpha=.15)
axes[0].set_ylabel('Local y (mm)');axes[0].legend(loc='upper left')
fig.suptitle('Collision simplification preserves bounds but changes the stepped contour')
fig.text(.5,.02,'Visuals, joints and inertia are unchanged. Added solid area can affect two-sided contact.',ha='center',fontsize=9)
fig.tight_layout(rect=[0,.10,1,.95]);fig.savefig(base/'finger_collision_comparison.png',dpi=160);fig.savefig(base/'finger_collision_comparison.svg')
