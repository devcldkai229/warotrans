import { describe, expect, it } from "vitest";
import {
  MapCoordinateTransformer,
  type MapMeta,
} from "./MapCoordinateTransformer";

const meta: MapMeta = {
  width: 60,
  height: 62,
  resolution: 0.05,
  origin_x: -0.409,
  origin_y: -2.357,
  origin_yaw: 0,
};

describe("MapCoordinateTransformer", () => {
  const xf = new MapCoordinateTransformer(meta);

  it("maps origin to bottom-left pixel", () => {
    const { col, row } = xf.mapToPixel(meta.origin_x, meta.origin_y);
    expect(col).toBeCloseTo(0, 6);
    expect(row).toBeCloseTo(meta.height - 1, 6);
  });

  it("maps opposite corner to top-right", () => {
    const x = meta.origin_x + (meta.width - 1) * meta.resolution;
    const y = meta.origin_y + (meta.height - 1) * meta.resolution;
    const { col, row } = xf.mapToPixel(x, y);
    expect(col).toBeCloseTo(meta.width - 1, 6);
    expect(row).toBeCloseTo(0, 6);
  });

  it("inverts Y: higher map y → smaller row_from_top", () => {
    const x = meta.origin_x + 1;
    const low = xf.mapToPixel(x, meta.origin_y);
    const high = xf.mapToPixel(x, meta.origin_y + 1);
    expect(high.row).toBeLessThan(low.row);
  });

  it("round-trips pixel → map → pixel", () => {
    const samples = [
      [0, 0],
      [10.5, 20.25],
      [59, 61],
      [30, 31],
    ] as const;
    for (const [col, row] of samples) {
      const { x, y } = xf.pixelToMap(col, row);
      const back = xf.mapToPixel(x, y);
      expect(Math.abs(back.col - col)).toBeLessThan(0.5);
      expect(Math.abs(back.row - row)).toBeLessThan(0.5);
    }
  });

  it("round-trips map → pixel → map", () => {
    const points = [
      [meta.origin_x, meta.origin_y],
      [meta.origin_x + 1, meta.origin_y + 0.5],
      [0, 0],
    ] as const;
    for (const [x, y] of points) {
      const { col, row } = xf.mapToPixel(x, y);
      const back = xf.pixelToMap(col, row);
      expect(Math.abs(back.x - x)).toBeLessThan(meta.resolution / 2);
      expect(Math.abs(back.y - y)).toBeLessThan(meta.resolution / 2);
    }
  });

  it("round-trips with non-zero origin_yaw", () => {
    const yaw = Math.PI / 6;
    const m: MapMeta = {
      width: 40,
      height: 40,
      resolution: 0.05,
      origin_x: -1,
      origin_y: -1,
      origin_yaw: yaw,
    };
    const t = new MapCoordinateTransformer(m);
    for (const [x, y] of [
      [0, 0],
      [-0.5, 0.25],
      [0.8, -0.3],
    ] as const) {
      const { col, row } = t.mapToPixel(x, y);
      const back = t.pixelToMap(col, row);
      expect(Math.abs(back.x - x)).toBeLessThan(1e-9);
      expect(Math.abs(back.y - y)).toBeLessThan(1e-9);
    }
  });
});
