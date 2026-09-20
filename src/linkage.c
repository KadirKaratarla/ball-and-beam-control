#include "linkage.h"

#include <math.h>

#include "config.h"

#define DEG2RAD (3.14159265358979f / 180.0f)
#define RAD2DEG (180.0f / 3.14159265358979f)

// Frame: origin at the shaft, x toward the hinge tower, y up.
//   crank pin   C(phi) = (-r cos phi, r sin phi)
//   pivot       P      = (dx, dy)
//   beam-end    B(th)  = (dx - d cos th, dy + d sin th)
//   constraint  |B - C| = l

static void pin(float phi, float *cx, float *cy)
{
    *cx = -LINK_CRANK_R_MM * cosf(phi);
    *cy = LINK_CRANK_R_MM * sinf(phi);
}

static void beam_end(float th, float *bx, float *by)
{
    *bx = LINK_PIVOT_DX_MM - LINK_BEAM_D_MM * cosf(th);
    *by = LINK_PIVOT_DY_MM + LINK_BEAM_D_MM * sinf(th);
}

float linkage_theta_from_phi(float phi_deg)
{
    float phi = phi_deg * DEG2RAD;
    float cx, cy;
    pin(phi, &cx, &cy);

    // First-order guess, then Newton on f(th) = |B - C|^2 - l^2.
    float s = LINK_CRANK_R_MM * sinf(phi) / LINK_BEAM_D_MM;
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    float th = asinf(s);
    for (int i = 0; i < 4; i++) {
        float bx, by;
        beam_end(th, &bx, &by);
        float ex = bx - cx, ey = by - cy;
        float f = ex * ex + ey * ey - LINK_ROD_L_MM * LINK_ROD_L_MM;
        // dB/dth = (d sin th, d cos th)
        float dfx = LINK_BEAM_D_MM * sinf(th), dfy = LINK_BEAM_D_MM * cosf(th);
        float df = 2.0f * (ex * dfx + ey * dfy);
        if (fabsf(df) < 1e-6f) break;
        th -= f / df;
    }
    return th * RAD2DEG;
}

bool linkage_phi_from_theta(float theta_deg, float *phi_deg)
{
    float bx, by;
    beam_end(theta_deg * DEG2RAD, &bx, &by);

    // Circles: radius r about the shaft (origin), radius l about B.
    float dist = sqrtf(bx * bx + by * by);
    float r = LINK_CRANK_R_MM, l = LINK_ROD_L_MM;
    if (dist > r + l || dist < fabsf(r - l) || dist < 1e-6f) return false;

    float a = (r * r - l * l + dist * dist) / (2.0f * dist);
    float h2 = r * r - a * a;
    if (h2 < 0.0f) h2 = 0.0f;
    float h = sqrtf(h2);
    float ux = bx / dist, uy = by / dist; // unit vector shaft -> B
    float mx = a * ux, my = a * uy;
    // Two candidates; the 9 o'clock branch has the pin away from the
    // tower, i.e. the more negative x.
    float c1x = mx - h * uy, c1y = my + h * ux;
    float c2x = mx + h * uy, c2y = my - h * ux;
    float cx = c1x < c2x ? c1x : c2x;
    float cy = c1x < c2x ? c1y : c2y;

    *phi_deg = atan2f(cy, -cx) * RAD2DEG;
    return true;
}

float linkage_phi_from_counts(float counts_from_6)
{
    return (counts_from_6 - ENC_LEVEL_COUNTS) * (360.0f / ENC_COUNTS_PER_REV);
}

float linkage_counts_from_phi(float phi_deg)
{
    return ENC_LEVEL_COUNTS + phi_deg * (ENC_COUNTS_PER_REV / 360.0f);
}
