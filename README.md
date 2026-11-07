# Space

Opt-in flight beyond Earth's atmosphere. A flight that has not added this mod
stops at 100 km, the same boundary as the atmospheric envelope. Adding it lets
the aircraft coast under gravity from that altitude out to Earth's sphere of
influence, and into the sphere of influence of any body mod that is also
installed.

This mod is not part of `0SFS/default-modpack`. Earth stays the world the
simulator draws. A body mod such as the Moon registers a destination. A mission
mod places the aircraft and names the phases of a flight. Neither ships with
this package.

Above 100 km the coast replaces JSBSim for as long as the craft stays there.
Engines do not push in space. Crossing back down through 100 km returns the
aircraft to JSBSim. The airspeed shown is speed over the Earth, including the
speed of a craft riding with another body.

The numeric step is the TypeScript velocity Verlet in `src/twoBody.ts`.
`wasm/crates/space-coast` is the same arithmetic and is not loaded by the flight.

## Add it

In the Mods tab, choose **This computer** and add `0SFS/space`, or add
`0SFS/space` from GitHub once this repository is published. Removing it
restores the 100 km limit.

```sh
npm run validate
```

Topics: `0sfs-mod` and `0sfs-plugin-mod`. The contract is
[SPEC-SPACE](https://github.com/0SFS/docs/blob/main/proposals/SPEC-SPACE.md).

## Community

- [Code of conduct](https://github.com/0SFS/.github/blob/main/CODE_OF_CONDUCT.md)
- [Contributing](https://github.com/0SFS/.github/blob/main/CONTRIBUTING.md)
- [License](LICENSE)
- [Security](https://github.com/0SFS/.github/blob/main/SECURITY.md)
