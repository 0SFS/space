import { readFileSync } from "node:fs";

const manifest = JSON.parse(readFileSync("public/0sfs-mod.json", "utf8"));
const failures = [];
const fail = (message) => failures.push(message);
const finite = (value) => typeof value === "number" && Number.isFinite(value);

if (manifest.schemaVersion !== 1) fail("schemaVersion must be 1.");
if (manifest.kind !== "plugin") fail("kind must be plugin.");
if (manifest.id !== "0sfs/space") fail("id must be 0sfs/space.");
if (manifest.execution !== "declarative") fail("execution must be declarative.");
if (manifest.js || manifest.wasm) fail("Space ships no JavaScript and no WASM entry.");
if (manifest.dependencies?.length) fail("Space does not depend on another mod.");
const space = manifest.space;
if (!space || space.apiVersion !== 1 || space.originBodyId !== "earth" || space.departureAltitudeM !== 100000) {
  fail("space needs apiVersion 1, origin earth, and departure 100000 m.");
}
const earth = space?.earth;
if (!earth || earth.id !== "earth" || !finite(earth.gravitationalParameterM3S2) || earth.gravitationalParameterM3S2 <= 0
  || earth.equatorialRadiusM !== 6378137 || !(earth.soiRadiusM > earth.equatorialRadiusM) || !(earth.rotationRadS > 0)) {
  fail("earth needs the WGS84 radius, a gravitational parameter, a sphere of influence, and a rotation rate.");
}
const provenance = manifest.provenance ?? {};
if (provenance.owner !== "0SFS" || provenance.repository !== "https://github.com/0SFS/space" || provenance.license !== "AGPL-3.0-only") {
  fail("provenance must name 0SFS/space under the AGPL.");
}

if (failures.length) {
  console.error(failures.join("\n"));
  process.exit(1);
}
console.log(`${manifest.id} is a valid ${manifest.kind} mod.`);
