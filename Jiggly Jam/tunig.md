
# button help
# '[' and ']' adjust pressure
# '1'/'2' adjust stiffness_ring
# '3'/'4' adjust stiffness_spoke
# 'd' and 'f' change damping

Global stiffness (jelly.stiffness): multiplier applied to all corrections (0..1). Lower → softer.
stiffness_ring (peripheral-peripheral) and stiffness_spoke (center-peripheral): let the shell flop while center stays stable.
iterations (constraint solver passes): fewer → softer, but may get unstable if too low.
damping (jelly.damping): closer to 1 → retains velocity (more bounce); lower → kills motion.
pressure: internal outward force — increases roundness and jiggly rebound after compression.
point masses: lighter peripheral masses → more elastic motion.
peripheral count N: fewer points → more floppy geometry.
Exact small experiments (run one at a time, then observe)

Quick floppy test (fast)
keys: press '-' until stiffness ~0.35; press Num2 until stiffness_ring ~0.5; press Num4 until stiffness_spoke ~0.5; press D to raise damping toward 0.995.
Expected: shell stretches and swings more; center remains somewhat coherent.
Squish + rebound (pressure + low ring stiffness)
Set stiffness ≈ 0.3
stiffness_ring ≈ 0.25
stiffness_spoke ≈ 0.7
iterations = 3
pressure ≈ 0.6 → 1.0 (use RBracket to increase)
damping ≈ 0.99
Expected: big squash on ground impact, then a strong rebound as pressure pushes back.
Very wiggly, floppy limbs (extreme)
stiffness ≈ 0.15
stiffness_ring ≈ 0.05
stiffness_spoke ≈ 0.4
iterations = 2
pressure = 0.0 → 0.2 (low pressure helps avoid immediate re-inflation)
reduce mass of peripheral points to 0.6
Expected: lots of slow, floppy motion; may look limp.
Bouncy but cohesive (playful)
stiffness ≈ 0.6
stiffness_ring ≈ 0.5
stiffness_spoke ≈ 0.9
iterations = 5
pressure = 0.3
damping = 0.995
Expected: lively, bouncy blob that keeps shape.
Combination notes

Low ring stiffness + moderate pressure = very squishy but retains an inflated silhouette.
Very low iterations + low stiffness can cause visible penetration/tunneling — increase substeps or iterations slightly if that happens.
If it “explodes” when low stiffness + high pressure, lower pressure or increase spoke stiffness.
Two safe code tweaks I can apply (pick one)

Add bending springs (connect i to i+2) with small rest length and low stiffness — makes the shell fold/crease naturally and looks jigglier without destroying shape.
Add nonlinear stiffness: correction scale = stiffness * (1 - exp(-k * stretchAmount)) so small stretches are soft, big stretches pull back harder (more organic). Good for floppiness plus stability.
Quick presets I can add to demo (I can implement now)

Key '5' = Floppy (apply preset values from experiment 3)
Key '6' = Squishy Rebound (experiment 2)
Key '7' = Playful Bouncy (experiment 4)