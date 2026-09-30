/** Derive corridor quad from centerline + width (ROS map metres). */
export function laneCorridorCorners(
  startX: number,
  startY: number,
  endX: number,
  endY: number,
  widthM: number,
): { x: number; y: number }[] {
  const dx = endX - startX;
  const dy = endY - startY;
  const length = Math.hypot(dx, dy);
  const hw = widthM * 0.5;
  if (length < 1e-9) {
    return [
      { x: startX - hw, y: startY - hw },
      { x: startX + hw, y: startY - hw },
      { x: startX + hw, y: startY + hw },
      { x: startX - hw, y: startY + hw },
    ];
  }
  const ux = dx / length;
  const uy = dy / length;
  const px = -uy;
  const py = ux;
  return [
    { x: startX + px * hw, y: startY + py * hw },
    { x: endX + px * hw, y: endY + py * hw },
    { x: endX - px * hw, y: endY - py * hw },
    { x: startX - px * hw, y: startY - py * hw },
  ];
}
