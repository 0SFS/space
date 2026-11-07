import type { BodyOrbitData } from "@0sfs/mods/schema";
import { vadd, vcross, vdot, vmag, vnorm, vrotate, vscale, vsub, type Vec3 } from "./vec";

export interface Axes {
  x: [number, number, number];
  y: [number, number, number];
  z: [number, number, number];
}

function wrapPi(angle: number): number {
  const tau = Math.PI * 2;
  let x = (angle + Math.PI) % tau;
  if (x < 0) x += tau;
  return x - Math.PI;
}

/** Position and velocity about the parent, whose gravitational parameter is `mu`. */
export function keplerState(orbit: BodyOrbitData, mu: number, tSeconds: number): { r: [number, number, number]; v: [number, number, number] } {
  const a = orbit.semiMajorAxisM;
  const e = orbit.eccentricity;
  const n = Math.sqrt(mu / (a * a * a));
  const mean = wrapPi(orbit.meanAnomalyAtEpochRad + n * (tSeconds - orbit.epochSeconds));
  let eccentric = mean;
  for (let i = 0; i < 15; i++) {
    const s = Math.sin(eccentric);
    const c = Math.cos(eccentric);
    eccentric -= (eccentric - e * s - mean) / (1 - e * c);
  }
  const sinE = Math.sin(eccentric);
  const cosE = Math.cos(eccentric);
  const edot = n / (1 - e * cosE);
  const scale = Math.sqrt(1 - e * e);
  const xp = a * (cosE - e);
  const yp = a * scale * sinE;
  const xv = -a * sinE * edot;
  const yv = a * scale * cosE * edot;
  const cO = Math.cos(orbit.longitudeOfAscendingNodeRad);
  const sO = Math.sin(orbit.longitudeOfAscendingNodeRad);
  const ci = Math.cos(orbit.inclinationRad);
  const si = Math.sin(orbit.inclinationRad);
  const cw = Math.cos(orbit.argumentOfPeriapsisRad);
  const sw = Math.sin(orbit.argumentOfPeriapsisRad);
  const r11 = cO * cw - sO * sw * ci;
  const r12 = -cO * sw - sO * cw * ci;
  const r21 = sO * cw + cO * sw * ci;
  const r22 = -sO * sw + cO * cw * ci;
  const r31 = sw * si;
  const r32 = cw * si;
  return {
    r: [r11 * xp + r12 * yp, r21 * xp + r22 * yp, r31 * xp + r32 * yp],
    v: [r11 * xv + r12 * yv, r21 * xv + r22 * yv, r31 * xv + r32 * yv],
  };
}

/**
 * Body axes at `spinRad` past the Earth-facing meridian.
 * +Z is the orbit normal. +X points at Earth when `spinRad` is zero.
 */
export function bodyAxes(center: Vec3, velocity: Vec3, spinRad: number): Axes {
  const z = vnorm(vcross(center, velocity)) ?? [0, 0, 1];
  const radial = vnorm(vscale(center, -1)) ?? [1, 0, 0];
  const planar = vnorm(vsub(radial, vscale(z, vdot(radial, z)))) ?? [1, 0, 0];
  const x = vnorm(vrotate(planar, z, spinRad)) ?? planar;
  const y = vcross(z, x);
  return { x, y, z };
}

export function bodyFixedOffset(axes: Axes, radiusM: number, latRad: number, lonRad: number, altMeters: number): [number, number, number] {
  const rho = radiusM + altMeters;
  const cl = Math.cos(latRad);
  const local: Vec3 = [rho * cl * Math.cos(lonRad), rho * cl * Math.sin(lonRad), rho * Math.sin(latRad)];
  return vadd(vscale(axes.x, local[0]), vadd(vscale(axes.y, local[1]), vscale(axes.z, local[2])));
}

export interface BodyPoint {
  r: [number, number, number];
  v: [number, number, number];
  latRad: number;
  lonRad: number;
  altMeters: number;
}

/** A body-fixed point and the velocity of that point as the body orbits and spins. */
export function bodyFixedPoint(
  orbit: BodyOrbitData, mu: number, radiusM: number, rotationRadS: number,
  latRad: number, lonRad: number, altMeters: number, tSeconds: number,
): BodyPoint {
  const place = (time: number): [number, number, number] => {
    const state = keplerState(orbit, mu, time);
    const axes = bodyAxes(state.r, state.v, rotationRadS * time);
    return vadd(state.r, bodyFixedOffset(axes, radiusM, latRad, lonRad, altMeters));
  };
  const r = place(tSeconds);
  const ahead = place(tSeconds + 1e-3);
  return {
    r, latRad, lonRad, altMeters,
    v: vscale(vsub(ahead, r), 1e3),
  };
}

/** Where `r` sits on the body, and how fast that body-fixed place is moving. */
export function locateOnBody(
  orbit: BodyOrbitData, mu: number, radiusM: number, rotationRadS: number, r: Vec3, tSeconds: number,
): BodyPoint {
  const state = keplerState(orbit, mu, tSeconds);
  const axes = bodyAxes(state.r, state.v, rotationRadS * tSeconds);
  const rho = vsub(r, state.r);
  const x = vdot(rho, axes.x);
  const y = vdot(rho, axes.y);
  const z = vdot(rho, axes.z);
  const latRad = Math.atan2(z, Math.hypot(x, y));
  const lonRad = Math.atan2(y, x);
  const altMeters = vmag(rho) - radiusM;
  const point = bodyFixedPoint(orbit, mu, radiusM, rotationRadS, latRad, lonRad, altMeters, tSeconds);
  return { ...point, latRad, lonRad, altMeters };
}
