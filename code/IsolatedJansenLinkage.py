# -*- coding: utf-8 -*-

# Jansen Linkage (Strandbeest) Simulation - Isolated Single Leg
# Based on DIYwalkers.com implementation
# to animate first type in console:
# %matplotlib qt

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import copy
import math

sim_scale = 1
# Set moving to True to make Strandbeest walk across screen
moving = False
# moving = True

# create 2 dictionaries to store the joints' X and Y coordinates
xdict = {}
ydict = {}
mech = "strandbeest"
xdict[mech] = {}
ydict[mech] = {}

# Bar lengths dictionary
bar = {}
bar[mech] = {}

# Lists for animation
xlist = {}
ylist = {}

# Dictionary to store entire mechanism for each frame
entire_mech_x = {}
entire_mech_y = {}
entire_mech_x[mech] = {}
entire_mech_y[mech] = {}

# Dictionary for foot-path
footpath_x = {}
footpath_y = {}
footpath_x[mech] = {}
footpath_y[mech] = {}

# Strandbeest's linkage dimensions (Jansen's "Holy Numbers")
# Source: https://www.diywalkers.com/strandbeest-optimizer-for-lego.html
scale_jansen = 1.5
bar[mech, 1] = 1.5 * scale_jansen   # crank
bar[mech, 2] = 3.8 * scale_jansen
bar[mech, 3] = 4.15 * scale_jansen
bar[mech, 4] = 3.93 * scale_jansen
bar[mech, 5] = 4.01 * scale_jansen
bar[mech, 6] = 5.58 * scale_jansen
bar[mech, 7] = 3.94 * scale_jansen
bar[mech, 8] = 3.67 * scale_jansen
bar[mech, 9] = 6.57 * scale_jansen
bar[mech, 10] = 4.9 * scale_jansen
bar[mech, 11] = 5 * scale_jansen
bar[mech, 12] = 6.19 * scale_jansen
bar[mech, 13] = 0.78 * scale_jansen

# Frame connection distances from center of rotation
frameX = 3.8
frameY = 0.78

# Joint labels - left leg only
joint_labels = {
    1: "Crank Center",
    2: "Frame Point",
    3: "Crank End",
    4: "Upper Triangle",
    5: "Mid Joint",
    6: "Lower Triangle",
    7: "Knee Joint",
    8: "Foot"
}

# Bar labels - format: (joint1, joint2, bar_name)
bar_labels = [
    (1, 3, "Bar 1 (Crank)"),
    (2, 4, "Bar 3"),
    (3, 4, "Bar 11"),
    (2, 5, "Bar 5"),
    (4, 5, "Bar 6"),
    (2, 6, "Bar 4"),
    (3, 6, "Bar 12"),
    (5, 7, "Bar 7"),
    (6, 7, "Bar 8"),
    (6, 8, "Bar 10"),
    (7, 8, "Bar 9"),
]

# Circle algorithm helper functions
def jnt_x(ax, ay, bx, by, al, bl):
    dist = ((ax - bx)**2 + (ay - by)**2)**0.5
    sidea = (al**2 - bl**2 + dist**2) / 2 / dist
    if al - sidea > 0:
        height = (al**2 - sidea**2)**0.5
    else:
        height = 0
    Dpointx = (ax + sidea * (bx - ax) / dist)
    x1 = Dpointx + height * (ay - by) / dist
    x2 = Dpointx - height * (ay - by) / dist
    return x1, x2

def jnt_y(ax, ay, bx, by, al, bl):
    dist = ((ax - bx)**2 + (ay - by)**2)**0.5
    sidea = (al**2 - bl**2 + dist**2) / 2 / dist
    if al - sidea > 0:
        height = (al**2 - sidea**2)**0.5
    else:
        height = 0
    Dpointy = (ay + sidea * (by - ay) / dist)
    y1 = Dpointy - height * (ax - bx) / dist
    y2 = Dpointy + height * (ax - bx) / dist
    return y1, y2

# define 4 solutions for circle algo
high = 0
low = 1
left = 2
right = 3

def circle_algo(mech, j1, j2, b1, b2, i, solution):
    x1, x2 = jnt_x(xdict[mech, j1, i], ydict[mech, j1, i],
                    xdict[mech, j2, i], ydict[mech, j2, i],
                    bar[mech, b1], bar[mech, b2])
    y1, y2 = jnt_y(xdict[mech, j1, i], ydict[mech, j1, i],
                    xdict[mech, j2, i], ydict[mech, j2, i],
                    bar[mech, b1], bar[mech, b2])

    if solution == high:
        return (x1, y1) if y1 > y2 else (x2, y2)
    elif solution == low:
        return (x1, y1) if y1 < y2 else (x2, y2)
    elif solution == right:
        return (x1, y1) if x1 > x2 else (x2, y2)
    elif solution == left:
        return (x1, y1) if x1 < x2 else (x2, y2)

# Simulation parameters
rotationIncrements = 120
loop_count = rotationIncrements * 4

xcenter = {}
ycenter = {}
avgspeed = {}
xstart = {}
xchange = {}

# Strandbeest's position and movement
xcenter[mech] = 10
ycenter[mech] = 10
Jansen_Shift_dn = -1.05

if moving:
    avgspeed[mech] = -0.116 * 180 / rotationIncrements * scale_jansen / 1.4
    xstart[mech] = 35 * scale_jansen
else:
    avgspeed[mech] = 0
    xstart[mech] = 0

xchange[mech] = xstart[mech]

def calc_joints():
    """Calculate all joint positions for one complete rotation"""
    
    for i in range(rotationIncrements):
        theta = (i / (rotationIncrements - 0.0)) * 2 * math.pi
        mech = "strandbeest"
        
        # Joint 1: Crank center (fixed)
        joint = 1
        xdict[mech, joint, i] = xcenter[mech] * scale_jansen
        ydict[mech, joint, i] = ycenter[mech] * scale_jansen + Jansen_Shift_dn
        
        # Joint 2: Frame point (fixed)
        joint = 2
        xdict[mech, joint, i] = (xcenter[mech] - frameX) * scale_jansen
        ydict[mech, joint, i] = (ycenter[mech] - frameY) * scale_jansen + Jansen_Shift_dn
        
        # Joint 3: Crank end (rotates)
        joint = 3
        xdict[mech, joint, i] = xcenter[mech] * scale_jansen + bar[mech, 1] * np.cos(theta)
        ydict[mech, joint, i] = ycenter[mech] * scale_jansen + bar[mech, 1] * np.sin(theta) + Jansen_Shift_dn
        
        # Joint 4: Upper triangle point
        joint = 4
        j1 = 2
        j2 = 3
        b1 = 3
        b2 = 11
        xdict[mech, joint, i], ydict[mech, joint, i] = circle_algo(mech, j1, j2, b1, b2, i, high)
        
        # Joint 5: Mid joint
        joint = 5
        j1 = 4
        j2 = 2
        b1 = 6
        b2 = 5
        xdict[mech, joint, i], ydict[mech, joint, i] = circle_algo(mech, j1, j2, b1, b2, i, left)
        
        # Joint 6: Lower triangle point
        joint = 6
        j1 = 3
        j2 = 2
        b1 = 12
        b2 = 4
        xdict[mech, joint, i], ydict[mech, joint, i] = circle_algo(mech, j1, j2, b1, b2, i, low)
        
        # Joint 7: Knee joint
        joint = 7
        j1 = 5
        j2 = 6
        b1 = 7
        b2 = 8
        xdict[mech, joint, i], ydict[mech, joint, i] = circle_algo(mech, j1, j2, b1, b2, i, low)
        
        # Joint 8: Foot
        joint = 8
        j1 = 6
        j2 = 7
        b1 = 10
        b2 = 9
        xdict[mech, joint, i], ydict[mech, joint, i] = circle_algo(mech, j1, j2, b1, b2, i, low)
    
    # Create foot-path and joint traces
    x_foot_path_list = {}
    y_foot_path_list = {}
    mech = 'strandbeest'
    x_foot_path_list[mech] = []
    y_foot_path_list[mech] = []
    
    # Create dictionaries to store joint position traces
    joint_trace_x = {}
    joint_trace_y = {}
    for joint_num in joint_labels.keys():
        joint_trace_x[joint_num] = []
        joint_trace_y[joint_num] = []
    
    for i in range(loop_count):
        mech = 'strandbeest'
        xlist[mech] = []
        ylist[mech] = []
        
        # Plot joints in order to draw the linkage (left leg only)
        j_joint_plot = [1, 3, 4, 2, 4, 5, 2, 6, 3, 6, 8, 7, 6, 7, 5]
        
        for joint in j_joint_plot:
            xlist[mech].append(xdict[mech, joint, np.mod(i, rotationIncrements)] + xchange[mech])
            ylist[mech].append(ydict[mech, joint, np.mod(i, rotationIncrements)])
        
        # Foot position
        joint = 8
        x_foot_path_list[mech].append(xdict[mech, joint, np.mod(i, rotationIncrements)] + xchange[mech])
        y_foot_path_list[mech].append(ydict[mech, joint, np.mod(i, rotationIncrements)])
        footpath_x[mech, i] = copy.deepcopy(x_foot_path_list[mech])
        footpath_y[mech, i] = copy.deepcopy(y_foot_path_list[mech])
        
        # Store joint positions for traces
        for joint_num in joint_labels.keys():
            joint_trace_x[joint_num].append(xdict[mech, joint_num, np.mod(i, rotationIncrements)] + xchange[mech])
            joint_trace_y[joint_num].append(ydict[mech, joint_num, np.mod(i, rotationIncrements)])
        
        xchange[mech] = xchange[mech] + avgspeed[mech]
        entire_mech_x[mech, i] = xlist[mech]
        entire_mech_y[mech, i] = ylist[mech]
    
    return joint_trace_x, joint_trace_y

# Create figure with two subplots
fig1 = plt.figure(figsize=(30, 10))

# Left subplot - Animation
ax1 = fig1.add_subplot(121)
ax1.set_xlim(-5, 25)
ax1.set_ylim(-5, 25)
ax1.set_title("Strandbeest Linkage (Jansen) - Single Leg", fontsize=18)
ax1.set_aspect('equal', adjustable='box')
ax1.ticklabel_format(style='plain', axis='both')

# Right subplot - Joint position traces
ax2 = fig1.add_subplot(122)
ax2.set_xlim(-5, 25)
ax2.set_ylim(-5, 25)
ax2.set_title("Joint Position Traces", fontsize=18)
ax2.set_aspect('equal', adjustable='box')
ax2.ticklabel_format(style='plain', axis='both')
ax2.grid(True, alpha=0.3)

fig1.set_facecolor('white')

# Plot lines
line, = ax1.plot([], [], '-o', ms=round(4.5 * scale_jansen), lw=2, color='b', mfc='red')  # linkage
line2, = ax1.plot([], [], '-o', lw=0, ms=1, alpha=1, mfc='red', mec='red')  # foot-path

# Right plot - joint traces (one line per joint)
trace_lines = {}
trace_colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'cyan']
for idx, joint_num in enumerate(joint_labels.keys()):
    trace_lines[joint_num], = ax2.plot([], [], '-', lw=1.5, alpha=0.7,
                                        color=trace_colors[idx % len(trace_colors)],
                                        label=joint_labels[joint_num])
ax2.legend(loc='upper right', fontsize=8)

# Create text objects for joint labels
label_texts = []
for joint_num in joint_labels.keys():
    text_obj = ax1.text(0, 0, '', fontsize=9, ha='left', va='bottom',
                      bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
    label_texts.append((joint_num, text_obj))

# Create text objects for bar labels
bar_label_texts = []
for j1, j2, bar_name in bar_labels:
    text_obj = ax1.text(0, 0, '', fontsize=8, ha='center', va='center',
                      bbox=dict(boxstyle='round,pad=0.2', facecolor='lightblue', alpha=0.6),
                      style='italic')
    bar_label_texts.append((j1, j2, bar_name, text_obj))

def init():
    line.set_data([], [])
    line2.set_data([], [])
    for _, text_obj in label_texts:
        text_obj.set_text('')
    for _, _, _, text_obj in bar_label_texts:
        text_obj.set_text('')
    for joint_num, trace_line in trace_lines.items():
        trace_line.set_data([], [])
    return (line, line2) + tuple(text_obj for _, text_obj in label_texts) + \
           tuple(text_obj for _, _, _, text_obj in bar_label_texts) + tuple(trace_lines.values())

def animate(i):
    mech = 'strandbeest'
    line.set_data(entire_mech_x[mech, i], entire_mech_y[mech, i])
    line2.set_data(footpath_x[mech, i], footpath_y[mech, i])
    
    # Update label positions
    frame_idx = np.mod(i, rotationIncrements)
    # Calculate the current xchange offset for this frame
    current_xoffset = xstart[mech] + avgspeed[mech] * i
    
    # Update joint labels
    for joint_num, text_obj in label_texts:
        if (mech, joint_num, frame_idx) in xdict:
            x_pos = xdict[mech, joint_num, frame_idx] + current_xoffset
            y_pos = ydict[mech, joint_num, frame_idx]
            text_obj.set_position((x_pos + 0.5, y_pos + 0.5))  # Offset slightly from joint
            text_obj.set_text(joint_labels[joint_num])
        else:
            text_obj.set_text('')
    
    # Update bar labels (position at midpoint of each bar)
    for j1, j2, bar_name, text_obj in bar_label_texts:
        if (mech, j1, frame_idx) in xdict and (mech, j2, frame_idx) in xdict:
            x1 = xdict[mech, j1, frame_idx] + current_xoffset
            y1 = ydict[mech, j1, frame_idx]
            x2 = xdict[mech, j2, frame_idx] + current_xoffset
            y2 = ydict[mech, j2, frame_idx]
            
            # Position label at midpoint of bar
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            
            # Calculate bar length
            bar_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            text_obj.set_position((mid_x, mid_y))
            text_obj.set_text(f"{bar_name}\n({bar_length:.2f})")
        else:
            text_obj.set_text('')
    
    # Update joint trace lines on right plot
    for joint_num, trace_line in trace_lines.items():
        trace_line.set_data(joint_trace_x[joint_num][:i + 1], joint_trace_y[joint_num][:i + 1])
    
    return (line2, line) + tuple(text_obj for _, text_obj in label_texts) + \
           tuple(text_obj for _, _, _, text_obj in bar_label_texts) + tuple(trace_lines.values())

# Calculate all joint positions
joint_trace_x, joint_trace_y = calc_joints()

# Save a snapshot at frame 60 to show both plots
snapshot_frame = 60
animate(snapshot_frame)

# Create animation
# to make the animation run faster set "blit=True"
ani = animation.FuncAnimation(fig1, animate, frames=loop_count, interval=0, blit=False, init_func=init)

# Save animation as GIF
# Set save_gif to True to save the animation
save_gif = False
if save_gif:
    print("Saving animation as GIF... (this may take a minute)")
    ani.save('strandbeest_linkage_animation.gif', writer='pillow', fps=30, dpi=80)
    print("Animation saved as 'strandbeest_linkage_animation.gif'")

plt.show()