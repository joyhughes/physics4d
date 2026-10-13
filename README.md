# Four-Space Mechanics

A small rigid-body physics engine for **four spatial dimensions**, with fourteen
teaching scenes. Plain ES modules, no build step; three.js is vendored.

```
python3 serve.py          # http://localhost:8940
node tools/test.mjs       # engine checks (conservation, contacts, rolling, joints)
node tools/scenes.mjs     # run every scene headless: timing, NaN and energy audit
node tools/spinplanes.mjs # which of a hyperbox's six spin planes are stable
node tools/shot.mjs <dir> <scene…>   # headless Chrome screenshots (server must be running)
```

Live: https://joyhughes.github.io/physics4d/

## The engine (`src/world.js`, `src/collide.js`, `src/math4.js`, `src/polytopes.js`)

| | 3D (Rapier, block creatures) | 4D (this engine) |
|---|---|---|
| position, velocity | 3-vectors | 4-vectors |
| orientation | quaternion / 3×3 | 4×4 rotation matrix (Cayley steps, re-orthonormalised) |
| angular velocity, momentum, torque | 3-vectors (axes) | **bivectors**, 6 parts: xy xz xw yz yw zw |
| inertia | 3×3 tensor | 6×6 on bivectors; diagonal in the body frame, J_ab = M_a + M_b |
| shapes | many | hyperspheres (solid or hollow), hyperboxes, the five convex regular polychora, half-space floor |
| hinge | turns about a line | turns in a plane, holds the orthogonal plane fixed |
| gravity of a point mass | 1/r² | 1/r³ (`centralForce`, exponent selectable) |

Each 1/60 s frame runs 8 substeps: gravity and force fields → collision
detection → sequential impulses (10 iterations, warm-started contacts,
Coulomb friction in the 3D tangent space, ball/hinge joints with motors and
limits) → a pseudo-velocity pass that removes penetration without adding
energy → integration. Free rotation carries the world angular momentum L
through a midpoint orientation step, so precession and the unstable spins come
out right and L is conserved to round-off.

Box–box contact is a 4D separating-axis test: 8 facet normals plus 48
edge × 2-face normals (vectors orthogonal to three directions). Facet contacts
clip the incident cube's edges against the reference cube and the reference
edges against the incident cube; edge–face contacts take the closest points of
a segment and a square.

The polychora (5-cell, 16-cell, 24-cell, 120-cell, 600-cell) are built in
`src/polytopes.js` from their vertex sets and cell-centre directions, as in
the [polychoron viewer](https://joyhughes.github.io/polychora/). Their volume
and second moments come from an exact split into one 4-simplex per flag
(centre, cell, 2-face, edge), and match the textbook volumes to 1e-13. Every
regular polychoron has the same inertia in all six planes. Convex contact
tests both shapes' cell normals as separating axes (cells are tried in order
of a cheap lower bound, so few are checked), confirms with GJK, and clips the
best cell against the most opposed cell of the other shape. Hyperboxes use
the same hull description when they meet a polychoron.

Joints correct drift inside the velocity solve (SHAKE-style). The
split-impulse alternative drained ~7 % of a pendulum's energy every 10 s;
this keeps it within 0.2 %.

## Scenes (`src/scenes.js`)

1. **Slices of the fourth dimension**: a hypersphere sliding in w, invisible walls, a tesseract turning in xw.
2. **Falling and landing**: random tesseracts and hyperspheres dropped onto the 3D floor.
3. **The rolling race**: solid 4-ball rolls at ¾ g sin θ (3D: 5/7), hollow at ⅔ (3D: 3/5), sliding block at 1.
4. **Orbits under inverse-cube gravity**: no stable orbits; switch to 1/r² to compare.
5. **Simple, double and isoclinic rotation**: rotation in two planes at once.
6. **Which spins are stable?**: three of six planes are stable: a spin in plane ab tumbles iff some third axis's extent lies between a's and b's.
7. **Hyperspheres colliding**: a 5-cell rack; 4-momentum conservation.
8. **The locked room**: a cube of walls doesn't trap a 4D ball; eight walls do.
9. **Pendulums and the 4D hinge**: ball joint vs hinge plane, a double pendulum.
10. **Toppling a tower**: stacking, friction and impacts.
11. **The regular polychora**: the five 4D Platonic solids turning freely; their slices, and how many cells each slice cuts.
12. **Which polytopes roll?**: on a slope you choose, each polytope tips over only above its tipping angle (600-cell 7.8°, 120-cell 18°, 16- and 24-cell 30°, tesseract 45°, 5-cell 52.2°).
13. **Spins that never wobble**: a hyperbox tumbles; a 24-cell and a 120-cell, with isotropic inertia, keep any spin.
14. **A bin of polychora**: a dozen polychora piled in an eight-walled 4D bin, resting on tetrahedral, octahedral and dodecahedral cells.

## Viewing

The main view is the 3D slice w′ = s0. Hyperboxes and polychora cut into
polyhedra whose faces are coloured by the cell's body-frame direction (x red,
y green, z blue, w amber); hyperspheres cut into spheres
patterned by body-frame orthant so their 4D rotation shows. "Shadows" draws
each whole body projected along w′, tinted amber at +w and cyan at −w. The
w-map (bottom left) is the x–w plane from above; drag it to move the slice.
The tilt sliders rotate the view in the xw and zw planes.
