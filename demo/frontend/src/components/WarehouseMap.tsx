import {
  forwardRef,
  useCallback,
  useEffect,
  useImperativeHandle,
  useRef,
  useState,
  type PointerEvent as ReactPointerEvent,
} from "react";
import {
  MapCoordinateTransformer,
  type MapMeta,
} from "../map/MapCoordinateTransformer";
import { laneCorridorCorners } from "../map/laneGeom";
import type {
  EditorMode,
  Endpoint,
  EndpointType,
  Lane,
  LaneDirection,
  NavPath,
  NavState,
  RobotPose,
  TrafficZone,
  ZonePoint,
  ZoneType,
} from "../types";
import { TYPE_COLORS, ZONE_COLORS } from "../types";

export interface WarehouseMapHandle {
  fitMap: () => void;
}

export interface EndpointDraft {
  x: number;
  y: number;
  yaw: number;
  name: string;
  type: EndpointType;
  editingId?: string;
}

export interface LaneDraft {
  startX: number;
  startY: number;
  endX: number;
  endY: number;
  name: string;
  widthMeters: number;
  direction: LaneDirection;
  placedStart: boolean;
  editingId?: string;
}

export interface ZoneDraft {
  points: ZonePoint[];
  name: string;
  type: ZoneType;
  editingId?: string;
}

export type Selection =
  | { kind: "endpoint"; id: string }
  | { kind: "lane"; id: string }
  | { kind: "zone"; id: string }
  | null;

type DragKind =
  | null
  | "pan"
  | "yaw"
  | "ep-move"
  | "lane-start"
  | "lane-end"
  | "zone-vertex";

interface Props {
  meta: MapMeta;
  imageUrl: string;
  endpoints: Endpoint[];
  lanes: Lane[];
  zones: TrafficZone[];
  selection: Selection;
  robot: RobotPose;
  path: NavPath;
  nav: NavState;
  mode: EditorMode;
  endpointDraft: EndpointDraft | null;
  laneDraft: LaneDraft | null;
  zoneDraft: ZoneDraft | null;
  hoverMap: { x: number; y: number } | null;
  onHoverMap: (p: { x: number; y: number } | null) => void;
  onEndpointDraftChange: (d: EndpointDraft | null) => void;
  onLaneDraftChange: (d: LaneDraft | null) => void;
  onZoneDraftChange: (d: ZoneDraft | null) => void;
  onSelect: (s: Selection) => void;
  onLanePatch: (id: string, patch: Partial<Lane>) => void;
  onZonePatch: (id: string, patch: Partial<TrafficZone>) => void;
  onEndpointPatch: (id: string, patch: Partial<Endpoint>) => void;
  onFinishZone: () => void;
}

function mapToScreen(
  xf: MapCoordinateTransformer,
  x: number,
  y: number,
  offset: { x: number; y: number },
  scale: number,
) {
  const { col, row } = xf.mapToPixel(x, y);
  return { sx: offset.x + col * scale, sy: offset.y + row * scale };
}

export const WarehouseMap = forwardRef<WarehouseMapHandle, Props>(
  function WarehouseMap(props, ref) {
    const {
      meta,
      imageUrl,
      endpoints,
      lanes,
      zones,
      selection,
      robot,
      path,
      nav,
      mode,
      endpointDraft,
      laneDraft,
      zoneDraft,
      hoverMap,
      onHoverMap,
      onEndpointDraftChange,
      onLaneDraftChange,
      onZoneDraftChange,
      onSelect,
      onLanePatch,
      onZonePatch,
      onEndpointPatch,
      onFinishZone,
    } = props;

    const canvasRef = useRef<HTMLCanvasElement>(null);
    const imgRef = useRef<HTMLImageElement | null>(null);
    const xf = useRef(new MapCoordinateTransformer(meta));
    xf.current = new MapCoordinateTransformer(meta);

    const [scale, setScale] = useState(4);
    const [offset, setOffset] = useState({ x: 40, y: 40 });
    const [hoverId, setHoverId] = useState<Selection>(null);
    const drag = useRef<{
      kind: DragKind;
      lastX: number;
      lastY: number;
      vertexIndex?: number;
      endpointId?: string;
    }>({ kind: null, lastX: 0, lastY: 0 });
    const lastClick = useRef(0);

    const fitMap = useCallback(() => {
      const canvas = canvasRef.current;
      if (!canvas) return;
      const pad = 24;
      const w = canvas.clientWidth - pad * 2;
      const h = canvas.clientHeight - pad * 2;
      if (w <= 0 || h <= 0) return;
      const s = Math.min(w / meta.width, h / meta.height);
      setScale(Math.max(1, s));
      setOffset({
        x: pad + (w - meta.width * s) / 2,
        y: pad + (h - meta.height * s) / 2,
      });
    }, [meta.height, meta.width]);

    useImperativeHandle(ref, () => ({ fitMap }), [fitMap]);

    const screenToMap = useCallback(
      (sx: number, sy: number) => {
        const col = (sx - offset.x) / scale;
        const row = (sy - offset.y) / scale;
        return xf.current.pixelToMap(col, row);
      },
      [offset, scale],
    );

    const drawLane = useCallback(
      (
        ctx: CanvasRenderingContext2D,
        lane: {
          startX: number;
          startY: number;
          endX: number;
          endY: number;
          widthMeters: number;
          direction: LaneDirection;
          name: string;
        },
        selected: boolean,
        preview: boolean,
        hovered: boolean,
      ) => {
        const corners = laneCorridorCorners(
          lane.startX,
          lane.startY,
          lane.endX,
          lane.endY,
          lane.widthMeters,
        );
        ctx.beginPath();
        corners.forEach((c, i) => {
          const p = mapToScreen(xf.current, c.x, c.y, offset, scale);
          if (i === 0) ctx.moveTo(p.sx, p.sy);
          else ctx.lineTo(p.sx, p.sy);
        });
        ctx.closePath();
        ctx.fillStyle = selected
          ? "rgba(59, 130, 246, 0.38)"
          : hovered
            ? "rgba(59, 130, 246, 0.3)"
            : preview
              ? "rgba(59, 130, 246, 0.2)"
              : "rgba(37, 99, 235, 0.22)";
        ctx.fill();
        ctx.strokeStyle = selected || hovered ? "#93c5fd" : "#3b82f6";
        ctx.lineWidth = selected ? 2.5 : hovered ? 2 : 1;
        ctx.stroke();

        const a = mapToScreen(xf.current, lane.startX, lane.startY, offset, scale);
        const b = mapToScreen(xf.current, lane.endX, lane.endY, offset, scale);
        ctx.beginPath();
        ctx.moveTo(a.sx, a.sy);
        ctx.lineTo(b.sx, b.sy);
        ctx.strokeStyle = "#e2e8f0";
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]);
        ctx.stroke();
        ctx.setLineDash([]);

        const dx = b.sx - a.sx;
        const dy = b.sy - a.sy;
        const len = Math.hypot(dx, dy) || 1;
        const ux = dx / len;
        const uy = dy / len;
        const nx = -uy;
        const ny = ux;
        const drawArrow = (cx: number, cy: number, sx: number, sy: number) => {
          const ang = Math.atan2(sy, sx);
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          ctx.lineTo(cx - 8 * Math.cos(ang - 0.45), cy - 8 * Math.sin(ang - 0.45));
          ctx.lineTo(cx - 8 * Math.cos(ang + 0.45), cy - 8 * Math.sin(ang + 0.45));
          ctx.closePath();
          ctx.fillStyle = "#f8fafc";
          ctx.fill();
        };
        const n = Math.max(1, Math.floor(len / 36));
        for (let i = 1; i <= n; i++) {
          const t = i / (n + 1);
          const cx = a.sx + dx * t;
          const cy = a.sy + dy * t;
          if (lane.direction === "BIDIRECTIONAL") {
            drawArrow(cx + nx * 5, cy + ny * 5, ux, uy);
            drawArrow(cx - nx * 5, cy - ny * 5, -ux, -uy);
          } else {
            drawArrow(cx, cy, ux, uy);
          }
        }

        ctx.fillStyle = "#cbd5e1";
        ctx.font = "11px system-ui, sans-serif";
        ctx.fillText(lane.name, (a.sx + b.sx) / 2 + 6, (a.sy + b.sy) / 2 - 6);

        if (selected || preview) {
          for (const p of [a, b]) {
            ctx.beginPath();
            ctx.arc(p.sx, p.sy, 6, 0, Math.PI * 2);
            ctx.fillStyle = "#fbbf24";
            ctx.fill();
            ctx.strokeStyle = "#fff";
            ctx.stroke();
          }
        }
      },
      [offset, scale],
    );

    const draw = useCallback(() => {
      const canvas = canvasRef.current;
      const img = imgRef.current;
      if (!canvas || !img) return;
      const ctx = canvas.getContext("2d");
      if (!ctx) return;

      const dpr = window.devicePixelRatio || 1;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      canvas.width = Math.floor(w * dpr);
      canvas.height = Math.floor(h * dpr);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

      ctx.fillStyle = "#1e1e24";
      ctx.fillRect(0, 0, w, h);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(img, offset.x, offset.y, meta.width * scale, meta.height * scale);

      // Zones (under lanes)
      for (const z of zones) {
        if (!z.enabled || z.polygon.length < 2) continue;
        const selected = selection?.kind === "zone" && selection.id === z.id;
        const hovered = hoverId?.kind === "zone" && hoverId.id === z.id;
        const color = ZONE_COLORS[z.type];
        ctx.beginPath();
        z.polygon.forEach((pt, i) => {
          const p = mapToScreen(xf.current, pt.x, pt.y, offset, scale);
          if (i === 0) ctx.moveTo(p.sx, p.sy);
          else ctx.lineTo(p.sx, p.sy);
        });
        ctx.closePath();
        ctx.fillStyle = selected ? `${color}55` : hovered ? `${color}44` : `${color}33`;
        ctx.fill();
        ctx.strokeStyle = color;
        ctx.lineWidth = selected ? 2.5 : hovered ? 2 : 1.5;
        ctx.stroke();
        const c0 = z.polygon[0];
        const label = mapToScreen(xf.current, c0.x, c0.y, offset, scale);
        ctx.fillStyle = "#e2e8f0";
        ctx.font = "11px system-ui, sans-serif";
        ctx.fillText(`${z.name} (${z.type})`, label.sx + 4, label.sy - 4);
        if (selected) {
          z.polygon.forEach((pt) => {
            const p = mapToScreen(xf.current, pt.x, pt.y, offset, scale);
            ctx.beginPath();
            ctx.arc(p.sx, p.sy, 5, 0, Math.PI * 2);
            ctx.fillStyle = "#fbbf24";
            ctx.fill();
          });
        }
      }

      // Zone draft
      if (zoneDraft && zoneDraft.points.length > 0) {
        const pts = [...zoneDraft.points];
        if (hoverMap && mode === "zone") pts.push(hoverMap);
        ctx.beginPath();
        pts.forEach((pt, i) => {
          const p = mapToScreen(xf.current, pt.x, pt.y, offset, scale);
          if (i === 0) ctx.moveTo(p.sx, p.sy);
          else ctx.lineTo(p.sx, p.sy);
        });
        if (zoneDraft.points.length >= 3) ctx.closePath();
        ctx.fillStyle = "rgba(56, 189, 248, 0.2)";
        if (zoneDraft.points.length >= 3) ctx.fill();
        ctx.strokeStyle = "#38bdf8";
        ctx.lineWidth = 1.5;
        ctx.stroke();
        zoneDraft.points.forEach((pt) => {
          const p = mapToScreen(xf.current, pt.x, pt.y, offset, scale);
          ctx.beginPath();
          ctx.arc(p.sx, p.sy, 4, 0, Math.PI * 2);
          ctx.fillStyle = "#38bdf8";
          ctx.fill();
        });
      }

      // Lanes
      for (const lane of lanes) {
        if (!lane.enabled) continue;
        drawLane(
          ctx,
          lane,
          selection?.kind === "lane" && selection.id === lane.id,
          false,
          hoverId?.kind === "lane" && hoverId.id === lane.id,
        );
      }
      if (laneDraft?.placedStart) {
        const endX = hoverMap && mode === "lane" ? hoverMap.x : laneDraft.endX;
        const endY = hoverMap && mode === "lane" ? hoverMap.y : laneDraft.endY;
        drawLane(
          ctx,
          {
            ...laneDraft,
            endX,
            endY,
          },
          true,
          true,
          false,
        );
      }

      // Path
      if (path.poses.length > 1) {
        ctx.beginPath();
        path.poses.forEach((p, i) => {
          const s = mapToScreen(xf.current, p.x, p.y, offset, scale);
          if (i === 0) ctx.moveTo(s.sx, s.sy);
          else ctx.lineTo(s.sx, s.sy);
        });
        ctx.strokeStyle = "#22c55e";
        ctx.lineWidth = 2.5;
        ctx.stroke();
      }

      // Goal
      if (nav.status === "NAVIGATING" && nav.goal_x != null && nav.goal_y != null) {
        const g = mapToScreen(xf.current, nav.goal_x, nav.goal_y, offset, scale);
        ctx.beginPath();
        ctx.arc(g.sx, g.sy, 10, 0, Math.PI * 2);
        ctx.strokeStyle = "#38bdf8";
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      const drawMarker = (
        x: number,
        y: number,
        yaw: number,
        label: string,
        color: string,
        selected: boolean,
      ) => {
        const o = mapToScreen(xf.current, x, y, offset, scale);
        const tip = xf.current.tipInMap(x, y, yaw, 0.35);
        const tipS = mapToScreen(xf.current, tip.x, tip.y, offset, scale);
        ctx.beginPath();
        ctx.arc(o.sx, o.sy, selected ? 8 : 6, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
        if (selected) {
          ctx.strokeStyle = "#fff";
          ctx.lineWidth = 2;
          ctx.stroke();
        }
        ctx.beginPath();
        ctx.moveTo(o.sx, o.sy);
        ctx.lineTo(tipS.sx, tipS.sy);
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.fillStyle = "#eee";
        ctx.font = "12px system-ui, sans-serif";
        ctx.fillText(label, o.sx + 10, o.sy - 8);
      };

      for (const ep of endpoints) {
        if (!ep.enabled) continue;
        drawMarker(
          ep.x,
          ep.y,
          ep.yaw,
          ep.name,
          TYPE_COLORS[ep.type],
          selection?.kind === "endpoint" && selection.id === ep.id,
        );
      }
      if (endpointDraft) {
        drawMarker(
          endpointDraft.x,
          endpointDraft.y,
          endpointDraft.yaw,
          endpointDraft.name || "NEW",
          TYPE_COLORS[endpointDraft.type],
          true,
        );
      }
      if (robot.valid) {
        drawMarker(robot.x, robot.y, robot.yaw, "ROBOT", "#ef4444", false);
      }
    }, [
      drawLane,
      endpointDraft,
      endpoints,
      laneDraft,
      lanes,
      meta.height,
      meta.width,
      mode,
      nav,
      hoverId,
      hoverMap,
      offset,
      path,
      robot,
      scale,
      selection,
      zoneDraft,
      zones,
    ]);

    useEffect(() => {
      const img = new Image();
      img.onload = () => {
        imgRef.current = img;
        fitMap();
        draw();
      };
      img.src = imageUrl;
    }, [imageUrl, draw, fitMap]);

    useEffect(() => {
      draw();
    }, [draw]);

    const hitEndpoint = (sx: number, sy: number) => {
      for (const ep of endpoints) {
        if (!ep.enabled) continue;
        const p = mapToScreen(xf.current, ep.x, ep.y, offset, scale);
        if (Math.hypot(sx - p.sx, sy - p.sy) < 12) return ep;
      }
      return null;
    };

    const hitLaneHandle = (sx: number, sy: number) => {
      for (const lane of lanes) {
        if (!lane.enabled) continue;
        const a = mapToScreen(xf.current, lane.startX, lane.startY, offset, scale);
        const b = mapToScreen(xf.current, lane.endX, lane.endY, offset, scale);
        if (Math.hypot(sx - a.sx, sy - a.sy) < 10) return { lane, which: "start" as const };
        if (Math.hypot(sx - b.sx, sy - b.sy) < 10) return { lane, which: "end" as const };
        // corridor rough hit via centerline distance
        const dx = b.sx - a.sx;
        const dy = b.sy - a.sy;
        const len2 = dx * dx + dy * dy || 1;
        let t = ((sx - a.sx) * dx + (sy - a.sy) * dy) / len2;
        t = Math.max(0, Math.min(1, t));
        const cx = a.sx + t * dx;
        const cy = a.sy + t * dy;
        const dist = Math.hypot(sx - cx, sy - cy);
        if (dist < Math.max(8, (lane.widthMeters * scale) / meta.resolution / 2)) {
          return { lane, which: "body" as const };
        }
      }
      return null;
    };

    const hitZoneVertex = (sx: number, sy: number) => {
      for (const z of zones) {
        if (!z.enabled) continue;
        for (let i = 0; i < z.polygon.length; i++) {
          const p = mapToScreen(xf.current, z.polygon[i].x, z.polygon[i].y, offset, scale);
          if (Math.hypot(sx - p.sx, sy - p.sy) < 9) return { zone: z, index: i };
        }
        // body: point in polygon screen approx via first vertex select
      }
      // select zone by proximity to any edge midpoint
      for (const z of zones) {
        if (!z.enabled || z.polygon.length < 3) continue;
        let cx = 0;
        let cy = 0;
        z.polygon.forEach((pt) => {
          cx += pt.x;
          cy += pt.y;
        });
        cx /= z.polygon.length;
        cy /= z.polygon.length;
        const c = mapToScreen(xf.current, cx, cy, offset, scale);
        if (Math.hypot(sx - c.sx, sy - c.sy) < 28) return { zone: z, index: -1 };
      }
      return null;
    };

    const onPointerDown = (e: ReactPointerEvent) => {
      const rect = canvasRef.current!.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;
      (e.target as HTMLElement).setPointerCapture(e.pointerId);
      const mapPt = screenToMap(sx, sy);
      const now = Date.now();
      const dbl = now - lastClick.current < 350;
      lastClick.current = now;

      if (mode === "pan" || e.button === 1) {
        drag.current = { kind: "pan", lastX: sx, lastY: sy };
        return;
      }

      if (mode === "endpoint" && e.button === 0) {
        if (endpointDraft) {
          const tip = xf.current.tipInMap(
            endpointDraft.x,
            endpointDraft.y,
            endpointDraft.yaw,
            0.35,
          );
          const tipS = mapToScreen(xf.current, tip.x, tip.y, offset, scale);
          if (Math.hypot(sx - tipS.sx, sy - tipS.sy) < 16) {
            drag.current = { kind: "yaw", lastX: sx, lastY: sy };
            return;
          }
        }
        onEndpointDraftChange({
          x: mapPt.x,
          y: mapPt.y,
          yaw: endpointDraft?.yaw ?? 0,
          name: endpointDraft?.name ?? "Endpoint",
          type: endpointDraft?.type ?? "PARK",
          editingId: endpointDraft?.editingId,
        });
        return;
      }

      if (mode === "lane" && e.button === 0) {
        if (!laneDraft || !laneDraft.placedStart) {
          onLaneDraftChange({
            startX: mapPt.x,
            startY: mapPt.y,
            endX: mapPt.x,
            endY: mapPt.y,
            name: laneDraft?.name ?? "Lane",
            widthMeters: laneDraft?.widthMeters ?? 0.6,
            direction: laneDraft?.direction ?? "BIDIRECTIONAL",
            placedStart: true,
            editingId: laneDraft?.editingId,
          });
        } else {
          onLaneDraftChange({
            ...laneDraft,
            endX: mapPt.x,
            endY: mapPt.y,
          });
        }
        return;
      }

      if (mode === "zone" && e.button === 0) {
        const pts = zoneDraft?.points ?? [];
        if (pts.length >= 3) {
          const first = mapToScreen(xf.current, pts[0].x, pts[0].y, offset, scale);
          if (dbl || Math.hypot(sx - first.sx, sy - first.sy) < 12) {
            onFinishZone();
            return;
          }
        }
        onZoneDraftChange({
          points: [...pts, { x: mapPt.x, y: mapPt.y }],
          name: zoneDraft?.name ?? "Zone",
          type: zoneDraft?.type ?? "JUNCTION",
          editingId: zoneDraft?.editingId,
        });
        return;
      }

      if (mode === "select" && e.button === 0) {
        const ep = hitEndpoint(sx, sy);
        if (ep) {
          onSelect({ kind: "endpoint", id: ep.id });
          const tip = xf.current.tipInMap(ep.x, ep.y, ep.yaw, 0.35);
          const tipS = mapToScreen(xf.current, tip.x, tip.y, offset, scale);
          if (Math.hypot(sx - tipS.sx, sy - tipS.sy) < 14) {
            drag.current = {
              kind: "yaw",
              lastX: sx,
              lastY: sy,
              endpointId: ep.id,
            };
          } else {
            drag.current = {
              kind: "ep-move",
              lastX: sx,
              lastY: sy,
              endpointId: ep.id,
            };
          }
          return;
        }
        const lh = hitLaneHandle(sx, sy);
        if (lh) {
          onSelect({ kind: "lane", id: lh.lane.id });
          if (lh.which === "start") {
            drag.current = { kind: "lane-start", lastX: sx, lastY: sy };
          } else if (lh.which === "end") {
            drag.current = { kind: "lane-end", lastX: sx, lastY: sy };
          } else {
            drag.current = { kind: "pan", lastX: sx, lastY: sy };
          }
          return;
        }
        const zh = hitZoneVertex(sx, sy);
        if (zh) {
          onSelect({ kind: "zone", id: zh.zone.id });
          if (zh.index >= 0) {
            drag.current = {
              kind: "zone-vertex",
              lastX: sx,
              lastY: sy,
              vertexIndex: zh.index,
            };
          }
          return;
        }
        onSelect(null);
        drag.current = { kind: "pan", lastX: sx, lastY: sy };
      }
    };

    const onPointerMove = (e: ReactPointerEvent) => {
      const rect = canvasRef.current!.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;
      const mapPt = screenToMap(sx, sy);
      if (mode === "lane" || mode === "zone") onHoverMap(mapPt);
      else onHoverMap(null);

      if (!drag.current.kind && mode === "select") {
        const ep = hitEndpoint(sx, sy);
        if (ep) setHoverId({ kind: "endpoint", id: ep.id });
        else {
          const lh = hitLaneHandle(sx, sy);
          if (lh) setHoverId({ kind: "lane", id: lh.lane.id });
          else {
            const zh = hitZoneVertex(sx, sy);
            if (zh) setHoverId({ kind: "zone", id: zh.zone.id });
            else setHoverId(null);
          }
        }
      }

      if (!drag.current.kind) return;

      if (drag.current.kind === "pan") {
        const dx = sx - drag.current.lastX;
        const dy = sy - drag.current.lastY;
        setOffset((o) => ({ x: o.x + dx, y: o.y + dy }));
        drag.current.lastX = sx;
        drag.current.lastY = sy;
        return;
      }
      if (drag.current.kind === "yaw") {
        if (endpointDraft && mode === "endpoint") {
          const yaw = Math.atan2(mapPt.y - endpointDraft.y, mapPt.x - endpointDraft.x);
          onEndpointDraftChange({ ...endpointDraft, yaw });
          return;
        }
        if (drag.current.endpointId) {
          const ep = endpoints.find((e) => e.id === drag.current.endpointId);
          if (ep) {
            const yaw = Math.atan2(mapPt.y - ep.y, mapPt.x - ep.x);
            onEndpointPatch(ep.id, { yaw });
          }
          return;
        }
      }
      if (drag.current.kind === "ep-move" && drag.current.endpointId) {
        onEndpointPatch(drag.current.endpointId, { x: mapPt.x, y: mapPt.y });
        return;
      }
      if (drag.current.kind === "lane-start" && selection?.kind === "lane") {
        onLanePatch(selection.id, { startX: mapPt.x, startY: mapPt.y });
        return;
      }
      if (drag.current.kind === "lane-end" && selection?.kind === "lane") {
        onLanePatch(selection.id, { endX: mapPt.x, endY: mapPt.y });
        return;
      }
      if (
        drag.current.kind === "zone-vertex" &&
        selection?.kind === "zone" &&
        drag.current.vertexIndex != null
      ) {
        const z = zones.find((zz) => zz.id === selection.id);
        if (!z) return;
        const polygon = z.polygon.map((p, i) =>
          i === drag.current.vertexIndex ? { x: mapPt.x, y: mapPt.y } : p,
        );
        onZonePatch(selection.id, { polygon });
      }
    };

    const onPointerUp = () => {
      drag.current.kind = null;
    };

    const onWheel = (e: React.WheelEvent) => {
      e.preventDefault();
      const factor = e.deltaY > 0 ? 0.9 : 1.1;
      const rect = canvasRef.current!.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      const newScale = Math.min(20, Math.max(1, scale * factor));
      const col = (mx - offset.x) / scale;
      const row = (my - offset.y) / scale;
      setScale(newScale);
      setOffset({ x: mx - col * newScale, y: my - row * newScale });
    };

    useEffect(() => {
      const onKey = (ev: KeyboardEvent) => {
        if (ev.key === "Enter" && mode === "zone" && (zoneDraft?.points.length ?? 0) >= 3) {
          onFinishZone();
        }
      };
      window.addEventListener("keydown", onKey);
      return () => window.removeEventListener("keydown", onKey);
    }, [mode, onFinishZone, zoneDraft]);

    return (
      <canvas
        ref={canvasRef}
        className="warehouse-canvas"
        onWheel={onWheel}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
      />
    );
  },
);
