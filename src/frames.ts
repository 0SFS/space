import { ecefToGeodetic, geodeticToEcef } from "foss-earth/cameraMath";
import type { FlightState } from "@0sfs/ground-contact/flightState";
import { vadd, vscale, type Vec3 } from "./vec";

export const METERS_PER_FOOT = 0.3048;
export const METERS_PER_KNOT = 1852 / 3600;

interface Basis {
  north: [number, number, number];
  east: [number, number, number];
  up: [number, number, number];
}

function basis(latRad: number, lonRad: number): Basis {
  const sl = Math.sin(latRad);
  const cl = Math.cos(latRad);
  const so = Math.sin(lonRad);
  const co = Math.cos(lonRad);
  return {
    north: [-sl * co, -sl * so, cl],
    east: [-so, co, 0],
    up: [cl * co, cl * so, sl],
  };
}

function combine(axes: Basis, north: number, east: number, up: number): [number, number, number] {
  return vadd(vscale(axes.north, north), vadd(vscale(axes.east, east), vscale(axes.up, up)));
}

function project(axes: Basis, v: Vec3): { north: number; east: number; up: number } {
  return {
    north: v[0] * axes.north[0] + v[1] * axes.north[1] + v[2] * axes.north[2],
    east: v[0] * axes.east[0] + v[1] * axes.east[1] + v[2] * axes.east[2],
    up: v[0] * axes.up[0] + v[1] * axes.up[1] + v[2] * axes.up[2],
  };
}

/** ECEF position and its ECEF derivative, from an Earth-centered inertial state. */
export function ecefFromEci(r: Vec3, v: Vec3, thetaRad: number, omegaRadS: number): { r: [number, number, number]; v: [number, number, number] } {
  const c = Math.cos(thetaRad);
  const s = Math.sin(thetaRad);
  const x = c * r[0] + s * r[1];
  const y = -s * r[0] + c * r[1];
  const z = r[2];
  return {
    r: [x, y, z],
    v: [
      c * v[0] + s * v[1] + omegaRadS * y,
      -s * v[0] + c * v[1] - omegaRadS * x,
      v[2],
    ],
  };
}

/** Inverse of `ecefFromEci`. `v` is the ECEF derivative, the velocity over the ground. */
export function eciFromEcef(r: Vec3, v: Vec3, thetaRad: number, omegaRadS: number): { r: [number, number, number]; v: [number, number, number] } {
  const c = Math.cos(thetaRad);
  const s = Math.sin(thetaRad);
  const px = v[0] - omegaRadS * r[1];
  const py = v[1] + omegaRadS * r[0];
  return {
    r: [c * r[0] - s * r[1], s * r[0] + c * r[1], r[2]],
    v: [c * px - s * py, s * px + c * py, v[2]],
  };
}

export function inertialFromFlight(state: FlightState, thetaRad: number, omegaRadS: number): { r: [number, number, number]; v: [number, number, number] } {
  const lat = state.latDeg * Math.PI / 180;
  const lon = state.lonDeg * Math.PI / 180;
  const ecef = geodeticToEcef(lat, lon, state.altMeters);
  const axes = basis(lat, lon);
  const velocity = combine(axes, state.northVelocityFps * METERS_PER_FOOT, state.eastVelocityFps * METERS_PER_FOOT, state.verticalSpeedFps * METERS_PER_FOOT);
  return eciFromEcef([ecef.x, ecef.y, ecef.z], velocity, thetaRad, omegaRadS);
}

function wrapLongitude(deg: number): number {
  return ((deg + 180) % 360 + 360) % 360 - 180;
}

export function flightFromInertial(r: Vec3, v: Vec3, thetaRad: number, omegaRadS: number, attitude: FlightState): FlightState {
  const ecef = ecefFromEci(r, v, thetaRad, omegaRadS);
  const geo = ecefToGeodetic(ecef.r[0], ecef.r[1], ecef.r[2]);
  const axes = basis(geo.latRad, geo.lonRad);
  const ned = project(axes, ecef.v);
  const speed = Math.hypot(ned.north, ned.east, ned.up);
  return {
    latDeg: geo.latRad * 180 / Math.PI,
    lonDeg: wrapLongitude(geo.lonRad * 180 / Math.PI),
    altMeters: geo.altMeters,
    rollRad: attitude.rollRad,
    pitchRad: attitude.pitchRad,
    headingRad: attitude.headingRad,
    airspeedKts: speed / METERS_PER_KNOT,
    northVelocityFps: ned.north / METERS_PER_FOOT,
    eastVelocityFps: ned.east / METERS_PER_FOOT,
    verticalSpeedFps: ned.up / METERS_PER_FOOT,
    throttleNorm: attitude.throttleNorm,
  };
}
