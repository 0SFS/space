export type Vec3 = readonly [number, number, number];

export function vadd(a: Vec3, b: Vec3): [number, number, number] {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function vsub(a: Vec3, b: Vec3): [number, number, number] {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function vscale(a: Vec3, s: number): [number, number, number] {
  return [a[0] * s, a[1] * s, a[2] * s];
}

export function vdot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function vcross(a: Vec3, b: Vec3): [number, number, number] {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

export function vmag(a: Vec3): number {
  return Math.hypot(a[0], a[1], a[2]);
}

export function vnorm(a: Vec3): [number, number, number] | null {
  const m = vmag(a);
  if (!(m > 0)) return null;
  return [a[0] / m, a[1] / m, a[2] / m];
}

/** Rotate `vector` about a unit axis. Rodrigues' formula. */
export function vrotate(vector: Vec3, axis: Vec3, radians: number): [number, number, number] {
  const c = Math.cos(radians);
  const s = Math.sin(radians);
  const along = vdot(axis, vector);
  const crossed = vcross(axis, vector);
  return [
    vector[0] * c + crossed[0] * s + axis[0] * along * (1 - c),
    vector[1] * c + crossed[1] * s + axis[1] * along * (1 - c),
    vector[2] * c + crossed[2] * s + axis[2] * along * (1 - c),
  ];
}
