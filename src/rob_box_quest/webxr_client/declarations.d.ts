// Ambient declarations for runtime-only npm modules that ship no .d.ts.
//
// These packages are used in scripts/*.mjs (untyped JS) and in
// tests/*.test.ts. The scripts don't care about types; the tests do.
// Rather than sprinkle `as any` everywhere, centralise the cast here so
// the actual call sites stay strict.

declare module "draco3dgltf" {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const mod: any;
  export default mod;
}

declare module "meshoptimizer" {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  export const MeshoptEncoder: any;
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  export const MeshoptDecoder: any;
}