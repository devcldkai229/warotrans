/**
 * Map metadata from /api/map/meta — ROS OccupancyGrid info.
 * PNG row 0 = top of image (browser Y down).
 *
 * When origin_yaw ≠ 0, cell offsets are rotated around the map origin
 * before applying origin_x / origin_y (ROS OccupancyGrid convention).
 */
export interface MapMeta {
  width: number;
  height: number;
  resolution: number;
  origin_x: number;
  origin_y: number;
  origin_yaw: number;
}

/**
 * Convert between ROS map metres and browser image pixels.
 * Pan/zoom must NOT alter this conversion — apply view transform after.
 */
export class MapCoordinateTransformer {
  readonly meta: MapMeta;
  private readonly c: number;
  private readonly s: number;

  constructor(meta: MapMeta) {
    if (meta.width < 1 || meta.height < 1) {
      throw new Error("map width/height must be >= 1");
    }
    if (meta.resolution <= 0) {
      throw new Error("map resolution must be > 0");
    }
    this.meta = meta;
    this.c = Math.cos(meta.origin_yaw);
    this.s = Math.sin(meta.origin_yaw);
  }

  /** ROS map (x, y) metres → image (col, row_from_top) pixels. */
  mapToPixel(x: number, y: number): { col: number; row: number } {
    const m = this.meta;
    const dx = x - m.origin_x;
    const dy = y - m.origin_y;
    const localX = this.c * dx + this.s * dy;
    const localY = -this.s * dx + this.c * dy;
    const col = localX / m.resolution;
    const rowFromBottom = localY / m.resolution;
    const row = m.height - 1 - rowFromBottom;
    return { col, row };
  }

  /** Image (col, row_from_top) pixels → ROS map (x, y) metres. */
  pixelToMap(col: number, row: number): { x: number; y: number } {
    const m = this.meta;
    const rowFromBottom = m.height - 1 - row;
    const localX = col * m.resolution;
    const localY = rowFromBottom * m.resolution;
    const x = m.origin_x + this.c * localX - this.s * localY;
    const y = m.origin_y + this.s * localX + this.c * localY;
    return { x, y };
  }

  /** Tip of orientation arrow in map frame. */
  tipInMap(
    x: number,
    y: number,
    yaw: number,
    lengthM: number,
  ): { x: number; y: number } {
    return {
      x: x + lengthM * Math.cos(yaw),
      y: y + lengthM * Math.sin(yaw),
    };
  }
}
