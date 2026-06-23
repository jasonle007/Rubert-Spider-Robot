# Rubert — Jansen Linkage Spider Robot

*A build log of a walking spider robot. Lots of failed prints. Some things that actually worked.*

<img width="500" height="650" alt="image" src="https://github.com/user-attachments/assets/4fb7b9a6-7c32-4034-8eca-43a4af94aaec" />

---

## What This Is

Rubert is a spider inspired walking robot I built as part of the WOOF3D bio inspired robotics project at UW. The goal was a Jansen linkage leg mechanism that actually walks: one servo per leg, two degrees of freedom per leg output, and a gait pattern that looks like something alive rather than something shuffling.

I named it Rubert. The club tracker says "Master the Jansen Linkage. Get this thing walking." — so that's what this is.

This repo documents the process: the CAD, the Python simulation I used to verify link geometry before printing, the print failures, and what finally worked. It's also a record of a project that's still going. The leg walks. The full robot gait is a work in progress.

---

## Context: The WOOF3D Project

This came out of the WOOF3D club's bio inspired robotics project, which split into sub teams across different problems: body and electronics, rotational/steering mechanisms, a jumping mechanism, PID stabilization, and remote control. I took on the Jansen linkage leg specifically.

The broader robot concept is a jumping spider inspired platform, roughly 150x150x75mm, with 8 legs total: 6 for walking (2 DOF each), 2 rear legs reserved for jumping (potentially 3 DOF). The walking motion uses hexapod gait logic, keeping 4 legs on the ground at any given time. The jumping legs are a separate problem the team is still working through.

My piece was getting the Jansen linkage leg to a printable, working mechanism with one servo input. Everything else depended on that being solved first.

---

## Why Jansen Linkages

The decision came down to Jansen vs. Klann, which the team debated early on. Both are planar walking mechanisms that convert rotary input into a foot trajectory. The differences matter at this scale.

Klann linkages are simpler: fewer links, easier to assemble, and the geometry is more forgiving. Another team member went deep on Klann optimization and got 20% more stride length than baseline ratios. It's a solid mechanism.

Jansen linkages use 13 bars (with a crank arm and 12 links) and produce a foot path that more closely mimics natural walking: the foot lifts, travels forward, and plants flat with low ground clearance variation during the stance phase; however the tradeoff is complexity. The geometry has to be right and is based off of Theo Jansen's "holy numbers" that specify link length ratios for this motion.

I chose Jansen because the foot path was closer to what spider locomotion actually looks like, the motion was greatly sastifying, and because I wanted the harder problem.

The specific link lengths I used in simulation (scaled by 1.5x from Jansen's original ratios):

| Link | Jansen Ratio | Scaled (1.5x) |
|------|-------------|---------------|
| Crank (bar 1) | 1.5 | 2.25 |
| Bar 2 | 3.8 | 5.70 |
| Bar 3 | 4.15 | 6.225 |
| Bar 4 | 3.93 | 5.895 |
| Bar 5 | 4.01 | 6.015 |
| Bar 6 | 5.58 | 8.37 |
| Bar 7 | 3.94 | 5.91 |
| Bar 8 | 3.67 | 5.505 |
| Bar 9 | 6.57 | 9.855 |
| Bar 10 | 4.9 | 7.35 |
| Bar 11 | 5.0 | 7.5 |
| Bar 12 | 6.19 | 9.285 |
| Frame offset X | 3.8 | — |
| Frame offset Y | 0.78 | — |

---

## Simulation Before Printing

Before cutting any geometry into CAD, the team sourced a Python simulation of the single leg linkage to verify that the foot path looked right and that the joint positions were solving correctly.

The sim uses a circle intersection algorithm to resolve each joint position through one full crank rotation, iterating across 120 angular increments. Joint 1 (crank center) and Joint 2 (frame point) are fixed. Everything else, upper triangle, mid joint, lower triangle, knee joint, and foot is computed from the two fixed anchor points and the link lengths. The solver picks between the valid intersection solutions (high/low/left/right) at each joint to enforce the correct assembly configuration.

Running it before printing caught a couple of things:

- My initial scale factor was too small. The foot path was geometrically correct but the links were going to be thin enough that they'd break in bending.
- The frame offset values (frameX = 3.8, frameY = 0.78) needed to match the physical mounting geometry exactly, otherwise the sim and the physical assembly would diverge even if the link lengths were right.

The simulation source is in `/code/IsolatedJansenLinkage.py`. To run it with animation:

```bash
pip install numpy matplotlib
# In an IPython console, first run:
# %matplotlib qt
python IsolatedJansenLinkage.py
```

Set `moving = True` in the script to see the leg translate across the screen rather than animate in place.

---

## Hardware

**Electronics (from the team's parts list):**

| Part | Model | Cost |
|------|-------|------|
| Micro Servo (x6) | MG90S | ~$25 total |
| Servo Driver | PCA9685 16-channel PWM | ~$6 |
| Microcontroller | Seeed XIAO ESP32C3 | ~$5 |
| Filament | colorFabb LW-PLA | ~$38 |

The LW-PLA was a deliberate choice being about 60% lighter than standard PLA because it foams during printing.

The PCA9685 servo driver handles PWM signal generation so the ESP32 isn't bottlenecked generating 6 servo signals simultaneously. The board communicates over I2C, which also leaves the ESP32 GPIO pins free for other things (sensors, future remote control, tilt feedback).

---

## CAD and Assembly

The SolidWorks model has 38 components. Most are link geometry, but the harder design work was in the joints.

Jansen linkages need joints that rotate freely without slop. At this scale, that means interference fit pegs and spacers rather than fasteners. My first pass at the joints had too much clearance and the links flopped. The second pass was too tight and the assembly bound up after a few cycles. By the third pass I had a working tolerance offset and kept a running log of designed vs. printed dimensions for every critical feature from that point forward.

One thing I hadn't thought through initially: the peg material matters as much as the diameter. I ended up printing pegs at higher infill than the link bodies. The peg compresses slightly on insertion, seats firmly, and holds rotation without binding. Link bodies at lower infill kept weight down without sacrificing enough strength to matter for the bending loads.

The body positions leg assemblies at specific phase offsets around a central shaft. Phase offsets in CAD are easy to set. Phase offsets in a physical assembly with tolerance stack across 38 parts are a different thing. The middle legs in particular ended up slightly off phase, which is still visible in the gait.

CAD Model:

<img width="750" height="500" alt="image" src="https://github.com/user-attachments/assets/cc2bfd85-e77f-4f1a-b474-0316dc1b10a2" />


---

## Printing

10+ prototype iterations. The things that actually caused failures:

**Layer defects from printing multiple parts simultaneously.** I was printing all the links for a single leg laid out flat on one build plate. The nozzle would do a pass across all parts, move to the next layer, and the previous layer cooled unevenly in the time between passes. This showed up as weak layer adhesion and occasional delamination under repeated load.

The fix was sequential printing: one part at a time, fully completed before moving to the next. Print time went up significantly, but the structural consistency was noticeably better.

**Link orientation and bending strength.** Jansen links are long, thin, and loaded in bending during the stance phase. Printing them flat (length along the bed) puts layer lines perpendicular to the bending stress axis, which is the weak direction for FDM. A few links snapped at layer boundaries during early testing. Printing them vertically with layer lines parallel to the long axis fixed the strength issue, at the cost of needing supports.

**Tolerancing.** Circular holes in my prints ran about 0.2mm undersized consistently, which is normal for FDM, but it varied by feature geometry. Slotted holes behaved differently than circular ones. Thin features on links warped slightly without adequate support contact. I added chamfers to most joint faces in later iterations, which made assembly easier and reduced the chance of binding during insertion.

6 Jansen Linkages Printed and Assembled:

<img width="500" height="650" alt="image" src="https://github.com/user-attachments/assets/2c6b0aee-1736-439e-bc03-3f1778cc32b0" />


---

## What Worked

By iteration 5 or so, the single leg was walking correctly. The foot traced a clean oval, lifted consistently, and didn't drag on the return stroke. The sequential printing made a real difference in joint quality.

The simulation was worth doing before printing. A few potential link collisions showed up in the CAD model that I might not have caught until the physical assembly. That saved at least two full print cycles. The SolidWorks collision detection in the motion study also helped check that adjacent links didn't interfere mid rotation.

However, CAD does not see everything and it took 3 more print iterations after that to resolve all colisions and interferences through adding slits in the legs and changing spacer width.

---

## What's Still Broken

**Middle leg phasing.** The front and rear leg pairs walk reasonably well. The middle legs are awkward. I think it's a combination of phase offset error accumulated during assembly and small geometric variation between printed legs. A more rigid central shaft would help, and tighter phase indexing at the mounting points.

**No steering.** With a fixed Jansen geometry and one continuous servo per leg, the robot walks in one direction. The team is working through a few steering approaches (rack and pinion, differential servo speed between sides), but none are implemented on Rubert yet.

**Thin feature consistency.** Features under about 3mm still come out with occasional layer adhesion issues even with sequential printing. Minimum feature thickness needs to go up, or I switch the link material for the next version.

**Full gait coordination.** A single leg walking correctly and eight legs walking together are different problems. The phasing gets set mechanically at assembly time, which means it's fixed. Getting it right across all eight legs in one assembly hasn't happened yet.

**Linkage buckle.** The linkages experience great amounts of friction with the ground and due to power issues and the servos not generating enough torque, too much weight can be placed on a leg causing the middle rectangular section to buckle inwards and collapse the linkage motion. The issue isn't permanent as the linkage can be popped out again, but this failure reveals a design flaw possibly in the linkage design it self or from the lack of structural intergrity from the 3D printed parts and spacings.

---

## What's Next

Per the team's current plan:
- Miniaturize the body, legs, and eventually the jumping mechanism
- PID control per leg, with a tilt sensor providing feedback to adjust all legs collectively
- Rotation/steering: still deciding between a rack-and-pinion mechanism and differential speed control
- Remote control via the ESP32's WiFi capability

On the Jansen side specifically: I want to rebuild the central shaft with tighter phase indexing before addressing the middle leg problem. If that doesn't fix it, the next step is going back to the simulation to check whether the phase angles I'm using are actually optimal for the gait pattern I'm targeting.

---

## Research Notes and References

Collected during the early research phase by the team:

**Linkage mechanisms:**
- [Jansen's Linkage — Wikipedia](https://en.wikipedia.org/wiki/Jansen%27s_linkage)
- [Klann Linkage — Wikipedia](https://en.wikipedia.org/wiki/Klann_linkage)
- [Analysis of Jansen and Klann Linkages — Springer](https://link.springer.com/article/10.1007/s11044-016-9532-9)
- [Walker Mechanism Thesis](http://boim.com/misc/WalkerMechanismThesis.pdf)
- [507 Mechanical Movements](https://507movements.com/)
- [Four-Bar Linkage Analysis and Synthesis — ResearchGate](https://www.researchgate.net/profile/L-Roy/publication/277584753_Analysis_and_Synthesis_of_Four_bar_Mechanism/links/55993c7a08ae21086d254047/Analysis-and-Synthesis-of-Four-bar-Mechanism.pdf)

**Spider locomotion:**
- [Arachnid Locomotion — Wikipedia](https://en.wikipedia.org/wiki/Arachnid_locomotion)
- [Unsupervised learning reveals rapid gait transitions (Journal of Experimental Biology)](https://journals.biologists.com/jeb/article/228/12/jeb250243/368261/Unsupervised-learning-reveals-rapid-gait)
- [Jumping Spider Slow Motion](https://www.youtube.com/watch?v=FfOrLNXIN-8) — only the rear 4 legs power the jump; the front legs brace landing
- [How Jumping Spiders Jump (hydraulic mechanism)](https://www.youtube.com/watch?v=hMxq0WXdxVQ)
- [Biomimetic Spider Leg Joints — ResearchGate](https://www.researchgate.net/publication/305361550_Biomimetic_Spider_Leg_Joints_A_Review_from_Biomechanical_Research_to_Compliant_Robotic_Actuators/link/68149363ded4331557410732/download)

**CAD and printing techniques explored:**
- Print in place hinges and joints
- Compliant joints for spring like behavior
- Bistable mechanisms for energy storage (jumping application)
- Sequential printing

**Inspiration projects:**
- [Crawl-E (Yudurobotics)](https://wiki.yudurobotics.com/index.php?title=Crawl-E)
- [Sesame Robot (GitHub)](https://github.com/dorianborian/sesame-robot)
- [3D Printed Biologically Inspired Robotics (Instructables)](https://www.instructables.com/3D-Printed-Biologically-Inspired-Robotics/)

---

## Files

```
/code
    IsolatedJansenLinkage.py    — Python simulation of single-leg foot path

/CAD
    /SolidWorks                 — 38-part assembly and individual part files
    /Onshape                    — Earlier iteration geometry exploration

/docs
    /tolerance-log              — Designed vs. printed dimensions across iterations
    /print-settings             — Per-part print orientation, infill, and support notes
    /research                   — Linkage analysis notes and locomotion references
```

---

## Tools

- SolidWorks (final CAD and motion study)
- Onshape (early iteration work)
- Python / matplotlib (linkage simulation)
- Arduino / ESP32 (servo control via PCA9685)
- FDM printer, PLA filament

---

## Status

Single leg: working.  
Full 6-leg gait: partially working, middle legs still off.  
Steering: not implemented.  
Jumping: team is still on it.

Updates will come as the build continues next academic year.
