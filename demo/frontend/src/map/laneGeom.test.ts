import { describe, expect, it } from "vitest";
import { laneCorridorCorners } from "./laneGeom";

describe("laneCorridorCorners", () => {
  it("builds a width-aligned corridor from centerline", () => {
    const c = laneCorridorCorners(0, 0, 2, 0, 1);
    expect(c).toHaveLength(4);
    // perpendicular to +X is +Y / -Y
    expect(c[0].y).toBeCloseTo(0.5);
    expect(c[3].y).toBeCloseTo(-0.5);
    expect(c[1].x).toBeCloseTo(2);
  });
});
