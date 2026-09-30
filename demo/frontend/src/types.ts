export type EndpointType =
  | "PICKUP"
  | "DROPOFF"
  | "PARK"
  | "QUEUE"
  | "WAIT";

export type LaneDirection = "ONE_WAY" | "BIDIRECTIONAL";

export type ZoneType =
  | "JUNCTION"
  | "KEEP_OUT"
  | "SPEED_LIMIT"
  | "SINGLE_ROBOT";

export type NavStatus =
  | "IDLE"
  | "NAVIGATING"
  | "SUCCEEDED"
  | "FAILED"
  | "CANCELLED";

export type EditorMode =
  | "select"
  | "pan"
  | "endpoint"
  | "lane"
  | "zone";

export interface Endpoint {
  id: string;
  name: string;
  type: EndpointType;
  x: number;
  y: number;
  yaw: number;
  enabled: boolean;
}

export interface EndpointCreate {
  name: string;
  type: EndpointType;
  x: number;
  y: number;
  yaw: number;
  enabled?: boolean;
}

export interface Lane {
  id: string;
  name: string;
  startX: number;
  startY: number;
  endX: number;
  endY: number;
  widthMeters: number;
  direction: LaneDirection;
  enabled: boolean;
  speedLimit?: number | null;
}

export interface LaneCreate {
  name: string;
  startX: number;
  startY: number;
  endX: number;
  endY: number;
  widthMeters: number;
  direction: LaneDirection;
  enabled?: boolean;
  speedLimit?: number | null;
}

export interface ZonePoint {
  x: number;
  y: number;
}

export interface TrafficZone {
  id: string;
  name: string;
  type: ZoneType;
  polygon: ZonePoint[];
  enabled: boolean;
  maxSpeedMps?: number | null;
  capacity?: number | null;
}

export interface ZoneCreate {
  name: string;
  type: ZoneType;
  polygon: ZonePoint[];
  enabled?: boolean;
  maxSpeedMps?: number | null;
  capacity?: number | null;
}

export interface MapVersion {
  version: number;
  frame_id: string;
  endpoints: Endpoint[];
  lanes: Lane[];
  zones: TrafficZone[];
}

export interface NavState {
  status: NavStatus;
  endpoint_id: string | null;
  message: string;
  goal_x?: number | null;
  goal_y?: number | null;
  goal_yaw?: number | null;
  mode?: string;
  lane_ids?: string[];
  speed_limit_mps?: number | null;
  speed_limit_percent?: number | null;
  filters_applied_at?: number | null;
}

export interface RobotPose {
  x: number;
  y: number;
  yaw: number;
  timestamp: number;
  valid: boolean;
}

export interface PathPose {
  x: number;
  y: number;
  yaw?: number;
}

export interface NavPath {
  frame_id: string;
  poses: PathPose[];
}

export const ENDPOINT_TYPES: EndpointType[] = [
  "PICKUP",
  "DROPOFF",
  "PARK",
  "QUEUE",
  "WAIT",
];

export const ZONE_TYPES: ZoneType[] = [
  "JUNCTION",
  "KEEP_OUT",
  "SPEED_LIMIT",
  "SINGLE_ROBOT",
];

export const TYPE_COLORS: Record<EndpointType, string> = {
  PICKUP: "#22c55e",
  DROPOFF: "#3b82f6",
  PARK: "#f59e0b",
  QUEUE: "#a855f7",
  WAIT: "#64748b",
};

export const ZONE_COLORS: Record<ZoneType, string> = {
  JUNCTION: "#38bdf8",
  KEEP_OUT: "#ef4444",
  SPEED_LIMIT: "#f59e0b",
  SINGLE_ROBOT: "#a855f7",
};
