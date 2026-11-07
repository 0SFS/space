//! Velocity Verlet about one point mass.
//! The same multiply order as `src/twoBody.ts`. The flight calls that
//! TypeScript reference; this crate is the twin and is not loaded yet.

/// Point-mass acceleration, metres per second squared.
pub fn central_acceleration(r: [f64; 3], mu: f64) -> [f64; 3] {
    let r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    let inv = -mu / (r2 * r2.sqrt());
    [r[0] * inv, r[1] * inv, r[2] * inv]
}

/// One velocity-Verlet step. `r` and `v` are relative to the central body.
pub fn step_two_body(r: [f64; 3], v: [f64; 3], mu: f64, dt: f64) -> ([f64; 3], [f64; 3]) {
    let a = central_acceleration(r, mu);
    let half = dt * 0.5;
    let v_half = [v[0] + a[0] * half, v[1] + a[1] * half, v[2] + a[2] * half];
    let r_next = [r[0] + v_half[0] * dt, r[1] + v_half[1] * dt, r[2] + v_half[2] * dt];
    let a_next = central_acceleration(r_next, mu);
    let v_next = [
        v_half[0] + a_next[0] * half,
        v_half[1] + a_next[1] * half,
        v_half[2] + a_next[2] * half,
    ];
    (r_next, v_next)
}
