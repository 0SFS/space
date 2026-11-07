import type { FlightState } from "@0sfs/ground-contact/flightState";
import type { BodyModData, MissionCompleteWhen, MissionModData, SpaceModData } from "@0sfs/mods/schema";
import { SPACE_DEPARTURE_ALTITUDE_M } from "@0sfs/mods/schema";
import { flightFromInertial, inertialFromFlight, METERS_PER_FOOT, METERS_PER_KNOT } from "./frames";
import { bodyAxes, bodyFixedPoint, keplerState, locateOnBody } from "./orbit";
import { stepTwoBody } from "./twoBody";
import { vadd, vmag, vscale, vsub, type Vec3 } from "./vec";

export interface SpaceBodyListing {
  id: string;
  name: string;
}

export interface SpaceMissionListing {
  id: string;
  name: string;
  summary: string;
  missingBodyIds: readonly string[];
  phases: readonly { id: string; label: string }[];
  active: boolean;
  activePhaseId: string | null;
  complete: boolean;
}

export interface SpaceSnapshot {
  installed: boolean;
  departureAltitudeM: number;
  bodies: readonly SpaceBodyListing[];
  missions: readonly SpaceMissionListing[];
}

export interface MissionPlacementPlan {
  name: string;
  label: string;
  state: FlightState;
  location: {
    latDeg: number;
    lonDeg: number;
    altMeters: number;
    headingDeg: number;
    airspeedKts: number;
    flightPathDeg: number;
  };
}

interface Ephemeris {
  id: string;
  r: [number, number, number];
  v: [number, number, number];
  mu: number;
  radiusM: number;
  soiM: number;
}

interface Coast {
  r: [number, number, number];
  v: [number, number, number];
  attitude: FlightState;
}

const MAX_SPEED_MPS = 20_000;
const listeners = new Set<() => void>();
let installed = false;
let earth: SpaceModData["earth"] | null = null;
let time = 0;
let bodies = new Map<string, BodyModData>();
let missions = new Map<string, MissionModData>();
let coast: Coast | null = null;
let activeBody = "earth";
let activeMission: string | null = null;
let problem: string | null = null;
let publishedPhase = "";
let snapshot: SpaceSnapshot = emptySnapshot();

function emptySnapshot(): SpaceSnapshot {
  return { installed: false, departureAltitudeM: SPACE_DEPARTURE_ALTITUDE_M, bodies: [], missions: [] };
}

function publish(): void {
  snapshot = {
    installed,
    departureAltitudeM: SPACE_DEPARTURE_ALTITUDE_M,
    bodies: [...bodies.values()].map(body => ({ id: body.id, name: body.name })),
    missions: [...missions.values()].map(mission => {
      const progress = mission.id === activeMission ? phaseProgress(mission) : { phaseId: null, complete: false };
      return {
        id: mission.id,
        name: mission.name,
        summary: mission.summary,
        missingBodyIds: missingBodies(mission),
        phases: mission.phases.map(phase => ({ id: phase.id, label: phase.label })),
        active: mission.id === activeMission,
        activePhaseId: progress.phaseId,
        complete: progress.complete,
      };
    }),
  };
  for (const listener of listeners) listener();
}

export function subscribeSpaceSession(listener: () => void): () => void {
  listeners.add(listener);
  return () => listeners.delete(listener);
}

export function getSpaceSnapshot(): SpaceSnapshot {
  return snapshot;
}

export function spaceInstalled(): boolean {
  return installed;
}

export function departureAltitudeM(): number {
  return SPACE_DEPARTURE_ALTITUDE_M;
}

export function activeBodyId(): string | null {
  return installed ? activeBody : null;
}

export function installSpace(space: SpaceModData): void {
  if (installed) return;
  if (space.apiVersion !== 1 || space.originBodyId !== "earth" || space.departureAltitudeM !== SPACE_DEPARTURE_ALTITUDE_M) {
    throw new Error("The space mod does not match this flight.");
  }
  earth = space.earth;
  installed = true;
  time = 0;
  coast = null;
  activeBody = "earth";
  problem = null;
  publish();
}

export function uninstallSpace(): void {
  installed = false;
  earth = null;
  time = 0;
  bodies = new Map();
  missions = new Map();
  coast = null;
  activeBody = "earth";
  activeMission = null;
  problem = null;
  publishedPhase = "";
  publish();
}

export function registerBody(body: BodyModData): void {
  if (!installed) throw new Error("Install the space mod before a body.");
  if (body.id === "earth" || bodies.has(body.id)) throw new Error(`Body ${body.id} is already registered.`);
  bodies.set(body.id, body);
  publish();
}

export function unregisterBody(id: string): void {
  if (!bodies.delete(id)) return;
  if (activeBody === id) activeBody = "earth";
  publish();
}

export function registerMission(mission: MissionModData): void {
  if (!installed) throw new Error("Install the space mod before a mission.");
  if (missions.has(mission.id)) throw new Error(`Mission ${mission.id} is already registered.`);
  missions.set(mission.id, mission);
  publish();
}

export function unregisterMission(id: string): void {
  if (!missions.delete(id)) return;
  if (activeMission === id) activeMission = null;
  publish();
}

export function bodyEphemeris(id: string): { r: readonly [number, number, number]; v: readonly [number, number, number] } | null {
  if (!installed || !earth) return null;
  if (id === "earth") return { r: [0, 0, 0], v: [0, 0, 0] };
  const body = bodies.get(id);
  if (!body) return null;
  return keplerState(body.orbit, earth.gravitationalParameterM3S2, time);
}

function missingBodies(mission: MissionModData): string[] {
  return mission.requiredBodyIds.filter(id => id !== "earth" && !bodies.has(id));
}

function thetaAt(at: number): number {
  return (earth?.rotationRadS ?? 0) * at;
}

export function invalidateSpaceCoast(): void {
  coast = null;
  activeBody = "earth";
  problem = null;
}

/** Remembers `missionId` as the scenario being flown, once the pilot has agreed to the move. */
export function seedSpaceCoast(state: FlightState, missionId?: string): void {
  if (!installed || !earth) return;
  const inertial = inertialFromFlight(state, thetaAt(time), earth.rotationRadS);
  coast = { r: inertial.r, v: inertial.v, attitude: state };
  activeBody = ownerAt(inertial.r, time).id;
  problem = null;
  if (missionId && missions.has(missionId) && missingBodies(missions.get(missionId)!).length === 0) activeMission = missionId;
  noteProgress();
}

export function spaceOwnsFlight(state: FlightState): boolean {
  if (!installed) return false;
  if (coast) return true;
  return state.altMeters >= SPACE_DEPARTURE_ALTITUDE_M;
}

function earthEphemeris(): Ephemeris {
  return {
    id: "earth", r: [0, 0, 0], v: [0, 0, 0],
    mu: earth!.gravitationalParameterM3S2, radiusM: earth!.equatorialRadiusM, soiM: earth!.soiRadiusM,
  };
}

function keplerOrEarth(id: string, at: number): { r: [number, number, number]; v: [number, number, number] } {
  if (id === "earth" || !earth) return { r: [0, 0, 0], v: [0, 0, 0] };
  const body = bodies.get(id);
  return body ? keplerState(body.orbit, earth.gravitationalParameterM3S2, at) : { r: [0, 0, 0], v: [0, 0, 0] };
}

function ownerAt(r: Vec3, at: number): Ephemeris {
  const home = earthEphemeris();
  let best: Ephemeris | null = vmag(r) <= home.soiM ? home : null;
  let bestSoi = best ? home.soiM : Number.POSITIVE_INFINITY;
  for (const body of bodies.values()) {
    const state = keplerState(body.orbit, home.mu, at);
    if (vmag(vsub(r, state.r)) >= body.soiRadiusM || body.soiRadiusM >= bestSoi) continue;
    best = {
      id: body.id, r: state.r, v: state.v, mu: body.gravitationalParameterM3S2,
      radiusM: body.equatorialRadiusM, soiM: body.soiRadiusM,
    };
    bestSoi = body.soiRadiusM;
  }
  return best ?? home;
}

function flightAt(r: Vec3, v: Vec3, at: number, attitude: FlightState): FlightState {
  return flightFromInertial(r, v, thetaAt(at), earth!.rotationRadS, attitude);
}

function holdOnSurface(relR: [number, number, number], relV: [number, number, number], radiusM: number): { r: [number, number, number]; v: [number, number, number] } {
  const dist = vmag(relR);
  if (!(dist > 0) || dist >= radiusM) return { r: relR, v: relV };
  const n = vscale(relR, 1 / dist);
  const radial = n[0] * relV[0] + n[1] * relV[1] + n[2] * relV[2];
  const v = radial < 0 ? vsub(relV, vscale(n, radial)) : relV;
  return { r: vscale(n, radiusM), v: [v[0], v[1], v[2]] };
}

export function coastFlightStep(state: FlightState, dt: number): FlightState {
  if (!installed || !earth || !Number.isFinite(dt) || dt <= 0) return state;
  if (!coast) {
    const seeded = inertialFromFlight(state, thetaAt(time), earth.rotationRadS);
    coast = { r: seeded.r, v: seeded.v, attitude: state };
  }
  const owner = ownerAt(coast.r, time);
  if (vmag(coast.r) > earth.soiRadiusM && owner.id === "earth") {
    problem = "Outside every installed sphere of influence.";
    return flightAt(coast.r, coast.v, time, coast.attitude);
  }
  const stepped = stepTwoBody(vsub(coast.r, owner.r), vsub(coast.v, owner.v), owner.mu, dt);
  const held = owner.id === "earth" ? stepped : holdOnSurface(stepped.r, stepped.v, owner.radiusM);
  const nextTime = time + dt;
  const origin = keplerOrEarth(owner.id, nextTime);
  const absolute = { r: vadd(origin.r, held.r), v: vadd(origin.v, held.v) };
  const flight = flightAt(absolute.r, absolute.v, nextTime, coast.attitude);
  const rejected = envelopeReasons(flight);
  if (rejected.length > 0) {
    problem = rejected[0] ?? "Space flight left the envelope.";
    return flightAt(coast.r, coast.v, time, coast.attitude);
  }
  time = nextTime;
  const holder = ownerAt(absolute.r, time);
  if (flight.altMeters < SPACE_DEPARTURE_ALTITUDE_M && holder.id === "earth") {
    coast = null;
    activeBody = "earth";
    problem = null;
    noteProgress();
    return flight;
  }
  coast = { r: absolute.r, v: absolute.v, attitude: flight };
  activeBody = holder.id;
  problem = null;
  noteProgress();
  return flight;
}

function envelopeReasons(state: FlightState): string[] {
  const reasons: string[] = [];
  for (const [key, value] of Object.entries(state)) {
    if (!Number.isFinite(value)) reasons.push(`${key} is ${value}`);
  }
  if (Math.abs(state.latDeg) > 90) reasons.push(`latDeg ${state.latDeg.toFixed(4)} outside +-90`);
  if (Math.abs(state.lonDeg) > 180) reasons.push(`lonDeg ${state.lonDeg.toFixed(4)} outside +-180`);
  const speed = Math.hypot(state.northVelocityFps, state.eastVelocityFps, state.verticalSpeedFps) * METERS_PER_FOOT;
  if (speed > MAX_SPEED_MPS) reasons.push(`speed ${speed.toFixed(0)} m/s beyond 20000`);
  return reasons;
}

export function spaceStateReasons(state: FlightState): string[] {
  return problem ? [problem, ...envelopeReasons(state)] : envelopeReasons(state);
}

function bodyAltitudeAndSpeed(bodyId: string): { altMeters: number; speedMps: number } | null {
  if (!coast || !earth) return null;
  if (bodyId === "earth") {
    const flight = flightAt(coast.r, coast.v, time, coast.attitude);
    return {
      altMeters: flight.altMeters,
      speedMps: Math.hypot(flight.northVelocityFps, flight.eastVelocityFps, flight.verticalSpeedFps) * METERS_PER_FOOT,
    };
  }
  const body = bodies.get(bodyId);
  if (!body) return null;
  const place = locateOnBody(body.orbit, earth.gravitationalParameterM3S2, body.equatorialRadiusM, body.rotationRadS, coast.r, time);
  return { altMeters: place.altMeters, speedMps: vmag(vsub(coast.v, place.v)) };
}

function conditionMet(bodyId: string, when: MissionCompleteWhen): boolean {
  if (when.kind === "body") return activeBody === when.bodyId;
  const sample = bodyAltitudeAndSpeed(bodyId);
  if (!sample) return false;
  if (when.kind === "altitude-below") return sample.altMeters < when.altMeters;
  if (when.kind === "altitude-above") return sample.altMeters > when.altMeters;
  return sample.speedMps < when.speedMps;
}

function phaseProgress(mission: MissionModData): { phaseId: string | null; complete: boolean } {
  for (const phase of mission.phases) {
    if (!conditionMet(mission.bodyId, phase.completeWhen)) return { phaseId: phase.id, complete: false };
  }
  return { phaseId: null, complete: true };
}

function noteProgress(): void {
  const mission = activeMission ? missions.get(activeMission) : undefined;
  const progress = mission ? phaseProgress(mission) : { phaseId: null, complete: false };
  const key = `${activeMission ?? ""}:${progress.phaseId ?? ""}:${progress.complete}`;
  if (key === publishedPhase) return;
  publishedPhase = key;
  publish();
}

function flightFromStart(latDeg: number, lonDeg: number, altMeters: number, northMps: number, eastMps: number, upMps: number, headingDeg: number): FlightState {
  const speed = Math.hypot(northMps, eastMps, upMps);
  const heading = speed < 0.05 ? headingDeg : Math.atan2(eastMps, northMps) * 180 / Math.PI;
  const gamma = speed < 0.05 ? 0 : Math.atan2(upMps, Math.hypot(northMps, eastMps)) * 180 / Math.PI;
  return {
    latDeg, lonDeg, altMeters,
    rollRad: 0,
    pitchRad: gamma * Math.PI / 180,
    headingRad: heading * Math.PI / 180,
    airspeedKts: speed / METERS_PER_KNOT,
    northVelocityFps: northMps / METERS_PER_FOOT,
    eastVelocityFps: eastMps / METERS_PER_FOOT,
    verticalSpeedFps: upMps / METERS_PER_FOOT,
    throttleNorm: 0,
  };
}

export function missionPlacementPlan(id: string): MissionPlacementPlan | null {
  const mission = missions.get(id);
  if (!installed || !earth || !mission || missingBodies(mission).length > 0) return null;
  const start = mission.start;
  let state: FlightState;
  if (mission.bodyId === "earth") {
    state = flightFromStart(start.latDeg, start.lonDeg, start.altMeters, start.northMps, start.eastMps, start.upMps, start.headingDeg);
  } else {
    const body = bodies.get(mission.bodyId);
    if (!body) return null;
    const lat = start.latDeg * Math.PI / 180;
    const lon = start.lonDeg * Math.PI / 180;
    const point = bodyFixedPoint(body.orbit, earth.gravitationalParameterM3S2, body.equatorialRadiusM, body.rotationRadS, lat, lon, start.altMeters, time);
    const center = keplerState(body.orbit, earth.gravitationalParameterM3S2, time);
    const axes = bodyAxes(center.r, center.v, body.rotationRadS * time);
    const cl = Math.cos(lat);
    const north = vadd(vscale(axes.x, -Math.sin(lat) * Math.cos(lon)), vadd(vscale(axes.y, -Math.sin(lat) * Math.sin(lon)), vscale(axes.z, Math.cos(lat))));
    const east = vadd(vscale(axes.x, -Math.sin(lon)), vscale(axes.y, Math.cos(lon)));
    const up = vadd(vscale(axes.x, cl * Math.cos(lon)), vadd(vscale(axes.y, cl * Math.sin(lon)), vscale(axes.z, Math.sin(lat))));
    const vNed = vadd(vscale(north, start.northMps), vadd(vscale(east, start.eastMps), vscale(up, start.upMps)));
    state = flightAt(point.r, vadd(point.v, vNed), time, flightFromStart(0, 0, 0, 0, 0, 0, start.headingDeg));
    if (Math.hypot(start.northMps, start.eastMps, start.upMps) < 0.05) {
      state = { ...state, headingRad: start.headingDeg * Math.PI / 180, pitchRad: 0 };
    }
  }
  const horizontal = Math.hypot(state.northVelocityFps, state.eastVelocityFps);
  return {
    name: mission.name,
    label: `${mission.name}: ${mission.bodyId} ${start.latDeg.toFixed(2)}°, ${start.lonDeg.toFixed(2)}°, ${start.altMeters.toFixed(0)} m`,
    state,
    location: {
      latDeg: state.latDeg,
      lonDeg: state.lonDeg,
      altMeters: state.altMeters,
      headingDeg: state.headingRad * 180 / Math.PI,
      airspeedKts: state.airspeedKts,
      flightPathDeg: (Math.atan2(state.verticalSpeedFps, horizontal) * 180) / Math.PI,
    },
  };
}
