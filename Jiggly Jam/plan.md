Elevator pitch
A fast, goofy 2D platformer where each player controls a squishy, jelly character whose movement comes from soft-body physics. Multiplayer matches (local or online) let players race, fight, or cooperate by bouncing, sticking, stretching, and slinging each other around levels that reward creative physics play.

Core mechanics (short)
Soft-body player: represented as a ring of point-masses connected by springs (or a polygon with Verlet constraints). Stretch, compress, and wobble affect speed/jump.
Movement: directional input applies forces to the soft body; ground collision with bounce + friction.
Abilities: short dash (conserves momentum), cling (stick to surfaces briefly), slam (ground pound), grapple (attach to geometry or other players).
Interaction: players can push/pull by contact, grab and throw other players, and cause chain reactions (bouncing off each other's deformation).
Game modes: Race (reach goal), King-of-the-Hill (hold area), Push-the-Block (puzzle), Tag/Elimination.
Physics approach (practical options)
Mass-spring model: explicit integrator, intuitive but needs small timestep for stability.
Verlet + distance constraints: stable, predictable, easy to implement and fast — recommended for a game-y jelly feel.
Collision: circle/capsule proxies for point-masses + SAT for terrain, with position correction to avoid tunneling.
Tuning knobs: spring stiffness, damping, solver iterations, substeps.
Multiplayer architecture (recommended)
Local play: easiest — split-screen or shared-screen with independent physics simulation.
Online:
Authoritative server (recommended): server simulates authoritative physics, clients send inputs; server sends periodic state snapshots. Use client-side prediction + server reconciliation to keep responsiveness.
Deterministic lockstep is hard with soft-body physics; prefer snapshot + interpolation/rewind if authoritative.
For small player counts use UDP transport + simple reliability layer for important messages.
Safety: server enforces limits (max force, teleport checks) to prevent cheating.
Level & UX design
Level types: short race tracks, arena playgrounds, physics puzzles (move jelly through squeezes), traps that deform the jelly.
Visuals: soft shading, squash-and-stretch animations, particle goo on impact.
Audio: wet squish SFX, boings, and rhythmic music matched to movement momentum.
UI: simple lobby, match scoreboard, ping indicator, replay/ghost in races.
Phased development plan
Prototype (1–3 days)
Single-player physics demo: 1 soft-body player, ground collision, movement + jump.
Debug visualizers: show point masses/springs and center-of-mass.
Core mechanics (1–2 weeks)
Add abilities (dash, cling, slam), polish controls, tune physics.
Build sample levels and a level loader.
Local multiplayer (few days)
Shared-screen + input mapping; test interactions between two jellies.
Networked multiplayer (2–4 weeks)
Simple authoritative server prototype, client-side prediction, snapshot interpolation.
Lobby/matchmaking basics.
Polish/Release (ongoing)
Art, sound, optimization, netcode polish, analytics.
Small engineering "contract" for the initial prototype
Input: keyboard/controller axes and button events.
Output: onscreen soft-body jelly that responds to input with believable squish and a debug overlay.
Data shapes: Player state = array of N points {position, prevPosition, mass}, constraints list, global gravity, input force vector.
Error modes: instabilities from too-large timestep (mitigate by fixed timestep + substeps), tunneling (solve with substeps and collision correction), network jitter (later: prediction + interpolation).
Edge cases to plan for
Extremely high FPS/low FPS causing unstable integrator (use fixed timestep).
Overlapping players spawning inside geometry (resolve on spawn).
Network packet loss and high latency during online play (use prediction + snapshot smoothing).
Very large deformations causing geometry penetration (limit spring stretch or apply corrective constraints).