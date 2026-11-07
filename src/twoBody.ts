import type { Vec3 } from "./vec";

/**
 * Point-mass acceleration, metres per second squared.
 * The Rust twin in `wasm/crates/space-coast` keeps this multiply order.
 */
export function centralAcceleration(r: readonly [number, number, number], mu: number): [number, number, number] {
  const r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
  const inv = -mu / (r2 * Math.sqrt(r2));
  return [r[0] * inv, r[1] * inv, r[2] * inv];
}

export interface TwoBodyState {
  r: [number, number, number];
  v: [number, number, number];
}

/**
 * One velocity-Verlet step about a fixed central body.
 * `r` and `v` are relative to that body, in metres and metres per second.
 */
export function stepTwoBody(r: Vec3, v: Vec3, mu: number, dt: number): TwoBodyState {
  const a = centralAcceleration(r, mu);
  const half = dt * 0.5;
  const vHalf: [number, number, number] = [v[0] + a[0] * half, v[1] + a[1] * half, v[2] + a[2] * half];
  const rNext: [number, number, number] = [r[0] + vHalf[0] * dt, r[1] + vHalf[1] * dt, r[2] + vHalf[2] * dt];
  const aNext = centralAcceleration(rNext, mu);
  const vNext: [number, number, number] = [vHalf[0] + aNext[0] * half, vHalf[1] + aNext[1] * half, vHalf[2] + aNext[2] * half];
  return { r: rNext, v: vNext };
}
