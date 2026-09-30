import type {
  Endpoint,
  EndpointCreate,
  Lane,
  LaneCreate,
  MapVersion,
  NavPath,
  NavState,
  RobotPose,
  TrafficZone,
  ZoneCreate,
} from "./types";
import type { MapMeta } from "./map/MapCoordinateTransformer";

async function json<T>(res: Response): Promise<T> {
  if (!res.ok) {
    const text = await res.text();
    throw new Error(text || res.statusText);
  }
  return res.json() as Promise<T>;
}

export async function fetchMapMeta(): Promise<MapMeta> {
  return json(await fetch("/api/map/meta"));
}

export function mapImageUrl(): string {
  return `/api/map/image.png?t=${Date.now()}`;
}

export async function listEndpoints(): Promise<Endpoint[]> {
  return json(await fetch("/api/endpoints"));
}

export async function createEndpoint(body: EndpointCreate): Promise<Endpoint> {
  return json(
    await fetch("/api/endpoints", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    }),
  );
}

export async function updateEndpoint(
  id: string,
  body: Partial<EndpointCreate>,
): Promise<Endpoint> {
  return json(
    await fetch(`/api/endpoints/${id}`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    }),
  );
}

export async function deleteEndpoint(id: string): Promise<void> {
  const res = await fetch(`/api/endpoints/${id}`, { method: "DELETE" });
  if (!res.ok) throw new Error(await res.text());
}

export async function listLanes(): Promise<Lane[]> {
  return json(await fetch("/api/lanes"));
}

export async function createLane(body: LaneCreate): Promise<Lane> {
  return json(
    await fetch("/api/lanes", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    }),
  );
}

export async function updateLane(
  id: string,
  body: Partial<LaneCreate>,
): Promise<Lane> {
  return json(
    await fetch(`/api/lanes/${id}`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    }),
  );
}

export async function deleteLane(id: string): Promise<void> {
  const res = await fetch(`/api/lanes/${id}`, { method: "DELETE" });
  if (!res.ok) throw new Error(await res.text());
}

export async function listZones(): Promise<TrafficZone[]> {
  return json(await fetch("/api/zones"));
}

export async function createZone(body: ZoneCreate): Promise<TrafficZone> {
  return json(
    await fetch("/api/zones", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    }),
  );
}

export async function updateZone(
  id: string,
  body: Partial<ZoneCreate>,
): Promise<TrafficZone> {
  return json(
    await fetch(`/api/zones/${id}`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    }),
  );
}

export async function deleteZone(id: string): Promise<void> {
  const res = await fetch(`/api/zones/${id}`, { method: "DELETE" });
  if (!res.ok) throw new Error(await res.text());
}

export async function exportSemanticMap(): Promise<MapVersion> {
  return json(await fetch("/api/semantic-map"));
}

export async function importSemanticMap(body: MapVersion): Promise<MapVersion> {
  return json(
    await fetch("/api/semantic-map", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    }),
  );
}

export async function navigateLanes(
  lane_ids: string[],
  spacing_m = 0.35,
): Promise<NavState> {
  return json(
    await fetch("/api/navigate/lanes", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ lane_ids, spacing_m }),
    }),
  );
}

export async function syncFilters(): Promise<Record<string, unknown>> {
  return json(await fetch("/api/filters/sync", { method: "POST" }));
}

export async function navigateTo(id: string): Promise<NavState> {
  return json(await fetch(`/api/navigate/${id}`, { method: "POST" }));
}

export async function cancelNavigate(): Promise<NavState> {
  return json(await fetch("/api/navigate/cancel", { method: "POST" }));
}

export async function fetchNavStatus(): Promise<NavState> {
  return json(await fetch("/api/nav/status"));
}

export async function fetchRobot(): Promise<RobotPose> {
  return json(await fetch("/api/robot"));
}

export async function fetchPath(): Promise<NavPath> {
  return json(await fetch("/api/path"));
}

export function connectWs(onMessage: (data: unknown) => void): WebSocket {
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${window.location.host}/ws`);
  ws.onmessage = (ev) => {
    try {
      onMessage(JSON.parse(ev.data as string));
    } catch {
      /* ignore */
    }
  };
  return ws;
}
