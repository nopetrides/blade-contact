# BladeContact

Reusable Unity package for feature-aware blade-to-blade contact between dynamic `Rigidbody` swords.

## Boundary

The package owns registered BladeShell pairing, authored feature classification, continuous blade-shell contact, ordinary blade slip, and bind eligibility. It returns contact impulses/velocity constraints to dynamic swords.

Unity/PhysX remains responsible for arm/servo dynamics, mass and inertia, gravity, environment contact, and all non-registered-sword interaction.

This package has no thesis-station, physical-reference, or MBC dependencies.

## Status

Scaffold only. The first functional implementation target is L1 angular contact from the Week 7 minimum-spec laboratory.
