#pragma once

// Crank-rod-beam kinematics (config.h LINK_* geometry), no hardware.
//
// Angles in degrees. phi = crank angle from 9 o'clock, positive toward
// 12 (the encoder's + direction). theta = beam angle, positive when the
// beam-end pin rises, i.e. the ball accelerates toward the hinge tower
// (p2, 45 cm) -- the same sense as DIR=1 (K-020).

#include <stdbool.h>

// Beam angle for a crank angle: exact, Newton on the rod-length constraint.
float linkage_theta_from_phi(float phi_deg);

// Crank angle for a beam angle: exact, circle-circle intersection on the
// 9 o'clock branch. Returns false (and leaves *phi_deg) if unreachable.
bool linkage_phi_from_theta(float theta_deg, float *phi_deg);

// Conversions to the encoder/step frame: counts from 6 o'clock.
float linkage_phi_from_counts(float counts_from_6);
float linkage_counts_from_phi(float phi_deg);
