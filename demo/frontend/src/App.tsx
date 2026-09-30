import { useCallback, useEffect, useRef, useState } from "react";
import {
  cancelNavigate,
  connectWs,
  createEndpoint,
  createLane,
  createZone,
  deleteEndpoint,
  deleteLane,
  deleteZone,
  exportSemanticMap,
  fetchMapMeta,
  fetchNavStatus,
  fetchPath,
  fetchRobot,
  importSemanticMap,
  listEndpoints,
  listLanes,
  listZones,
  mapImageUrl,
  navigateLanes,
  navigateTo,
  syncFilters,
  updateEndpoint,
  updateLane,
  updateZone,
} from "./api";
import {
  WarehouseMap,
  type EndpointDraft,
  type LaneDraft,
  type Selection,
  type WarehouseMapHandle,
  type ZoneDraft,
} from "./components/WarehouseMap";
import type { MapMeta } from "./map/MapCoordinateTransformer";
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
  ZoneType,
} from "./types";
import { ENDPOINT_TYPES, ZONE_TYPES } from "./types";

const emptyNav: NavState = {
  status: "IDLE",
  endpoint_id: null,
  message: "",
};
const emptyPath: NavPath = { frame_id: "map", poses: [] };

export default function App() {
  const mapRef = useRef<WarehouseMapHandle>(null);
  const fileRef = useRef<HTMLInputElement>(null);

  const [meta, setMeta] = useState<MapMeta | null>(null);
  const [imageUrl, setImageUrl] = useState("");
  const [endpoints, setEndpoints] = useState<Endpoint[]>([]);
  const [lanes, setLanes] = useState<Lane[]>([]);
  const [zones, setZones] = useState<TrafficZone[]>([]);
  const [laneRoute, setLaneRoute] = useState<string[]>([]);
  const [selection, setSelection] = useState<Selection>(null);
  const [mode, setMode] = useState<EditorMode>("select");
  const [endpointDraft, setEndpointDraft] = useState<EndpointDraft | null>(null);
  const [laneDraft, setLaneDraft] = useState<LaneDraft | null>(null);
  const [zoneDraft, setZoneDraft] = useState<ZoneDraft | null>(null);
  const [hoverMap, setHoverMap] = useState<{ x: number; y: number } | null>(null);
  const [nav, setNav] = useState<NavState>(emptyNav);
  const [path, setPath] = useState<NavPath>(emptyPath);
  const [robot, setRobot] = useState<RobotPose>({
    x: 0,
    y: 0,
    yaw: 0,
    timestamp: 0,
    valid: false,
  });
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  const reloadSemantic = useCallback(async () => {
    const [eps, lns, zns] = await Promise.all([
      listEndpoints(),
      listLanes(),
      listZones(),
    ]);
    setEndpoints(eps);
    setLanes(lns);
    setZones(zns);
  }, []);

  const reload = useCallback(async () => {
    try {
      setError(null);
      const [m, n, r, p] = await Promise.all([
        fetchMapMeta(),
        fetchNavStatus(),
        fetchRobot(),
        fetchPath(),
      ]);
      setMeta(m);
      setImageUrl(mapImageUrl());
      setNav(n);
      setRobot(r);
      setPath(p);
      await reloadSemantic();
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    }
  }, [reloadSemantic]);

  useEffect(() => {
    void reload();
  }, [reload]);

  useEffect(() => {
    const ws = connectWs((raw) => {
      const msg = raw as Record<string, unknown>;
      if (msg.type === "nav" && msg.nav) setNav(msg.nav as NavState);
      if (msg.type === "robot" && msg.robot) setRobot(msg.robot as RobotPose);
      if (msg.type === "path" && msg.path) setPath(msg.path as NavPath);
      if (msg.type === "speed_limit") {
        setNav((n) => ({
          ...n,
          speed_limit_mps: msg.speed_limit_mps as number | null,
          speed_limit_percent: msg.speed_limit_percent as number | null,
        }));
      }
      if (msg.type === "map_updated") {
        setImageUrl(mapImageUrl());
        if (msg.meta) setMeta(msg.meta as MapMeta);
      }
      if (msg.type === "snapshot") {
        if (msg.nav) setNav(msg.nav as NavState);
        if (msg.robot) setRobot(msg.robot as RobotPose);
        if (msg.path) setPath(msg.path as NavPath);
      }
    });
    return () => ws.close();
  }, []);

  const clearDrafts = () => {
    setEndpointDraft(null);
    setLaneDraft(null);
    setZoneDraft(null);
  };

  const setTool = (m: EditorMode) => {
    setMode(m);
    clearDrafts();
    if (m === "endpoint") {
      setEndpointDraft({
        x: 0,
        y: 0,
        yaw: 0,
        name: "Endpoint",
        type: "PARK",
      });
    }
    if (m === "lane") {
      setLaneDraft({
        startX: 0,
        startY: 0,
        endX: 0,
        endY: 0,
        name: "Lane",
        widthMeters: 0.6,
        direction: "BIDIRECTIONAL",
        placedStart: false,
      });
    }
    if (m === "zone") {
      setZoneDraft({
        points: [],
        name: "Zone",
        type: "JUNCTION",
      });
    }
  };

  const selectedEndpoint =
    selection?.kind === "endpoint"
      ? endpoints.find((e) => e.id === selection.id) ?? null
      : null;
  const selectedLane =
    selection?.kind === "lane"
      ? lanes.find((l) => l.id === selection.id) ?? null
      : null;
  const selectedZone =
    selection?.kind === "zone"
      ? zones.find((z) => z.id === selection.id) ?? null
      : null;

  const saveEndpointDraft = async () => {
    if (!endpointDraft) return;
    setBusy(true);
    try {
      if (endpointDraft.editingId) {
        await updateEndpoint(endpointDraft.editingId, endpointDraft);
      } else {
        await createEndpoint({ ...endpointDraft, enabled: true });
      }
      clearDrafts();
      setMode("select");
      await reloadSemantic();
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  };

  const saveLaneDraft = async () => {
    if (!laneDraft?.placedStart) return;
    setBusy(true);
    try {
      const body = {
        name: laneDraft.name,
        startX: laneDraft.startX,
        startY: laneDraft.startY,
        endX: laneDraft.endX,
        endY: laneDraft.endY,
        widthMeters: laneDraft.widthMeters,
        direction: laneDraft.direction,
        enabled: true,
      };
      if (laneDraft.editingId) await updateLane(laneDraft.editingId, body);
      else await createLane(body);
      clearDrafts();
      setMode("select");
      await reloadSemantic();
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  };

  const finishZone = async () => {
    if (!zoneDraft || zoneDraft.points.length < 3) return;
    setBusy(true);
    try {
      const body = {
        name: zoneDraft.name,
        type: zoneDraft.type,
        polygon: zoneDraft.points,
        enabled: true,
        capacity: zoneDraft.type === "SINGLE_ROBOT" ? 1 : null,
      };
      if (zoneDraft.editingId) await updateZone(zoneDraft.editingId, body);
      else await createZone(body);
      clearDrafts();
      setMode("select");
      await reloadSemantic();
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  };

  const doDelete = async () => {
    if (!selection) return;
    setBusy(true);
    try {
      if (selection.kind === "endpoint") await deleteEndpoint(selection.id);
      if (selection.kind === "lane") await deleteLane(selection.id);
      if (selection.kind === "zone") await deleteZone(selection.id);
      setSelection(null);
      await reloadSemantic();
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  };

  const doExport = async () => {
    const data = await exportSemanticMap();
    const blob = new Blob([JSON.stringify(data, null, 2)], {
      type: "application/json",
    });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = "warotrans-semantic-map.json";
    a.click();
    URL.revokeObjectURL(a.href);
  };

  const doImportFile = async (file: File) => {
    setBusy(true);
    try {
      const text = await file.text();
      const json = JSON.parse(text);
      await importSemanticMap(json);
      setSelection(null);
      await reloadSemantic();
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  };

  const patchLaneLive = async (id: string, patch: Partial<Lane>) => {
    setLanes((prev) =>
      prev.map((l) => (l.id === id ? { ...l, ...patch } : l)),
    );
    try {
      await updateLane(id, patch);
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
      await reloadSemantic();
    }
  };

  const patchZoneLive = async (id: string, patch: Partial<TrafficZone>) => {
    setZones((prev) =>
      prev.map((z) => (z.id === id ? { ...z, ...patch } : z)),
    );
    try {
      await updateZone(id, patch);
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
      await reloadSemantic();
    }
  };

  const patchEndpointLive = async (id: string, patch: Partial<Endpoint>) => {
    setEndpoints((prev) =>
      prev.map((e) => (e.id === id ? { ...e, ...patch } : e)),
    );
    try {
      await updateEndpoint(id, patch);
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
      await reloadSemantic();
    }
  };

  const laneReady =
    !!laneDraft?.placedStart &&
    (laneDraft.endX !== laneDraft.startX || laneDraft.endY !== laneDraft.startY);

  return (
    <div className="app">
      <header className="header">
        <h1>Warehouse — Semantic Navigation (Phase 3)</h1>
        <div className={`nav-badge status-${nav.status.toLowerCase()}`}>
          {nav.status}
          {nav.message ? ` — ${nav.message}` : ""}
        </div>
      </header>

      {error && (
        <div className="banner error">
          {error}
          <button type="button" onClick={() => setError(null)}>
            ×
          </button>
        </div>
      )}

      <div className="layout">
        <main className="map-pane">
          {meta && imageUrl ? (
            <WarehouseMap
              ref={mapRef}
              meta={meta}
              imageUrl={imageUrl}
              endpoints={endpoints}
              lanes={lanes}
              zones={zones}
              selection={selection}
              robot={robot}
              path={path}
              nav={nav}
              mode={mode}
              endpointDraft={endpointDraft}
              laneDraft={laneDraft}
              zoneDraft={zoneDraft}
              hoverMap={hoverMap}
              onHoverMap={setHoverMap}
              onEndpointDraftChange={setEndpointDraft}
              onLaneDraftChange={setLaneDraft}
              onZoneDraftChange={setZoneDraft}
              onSelect={(s) => {
                setSelection(s);
                setMode("select");
                clearDrafts();
              }}
              onLanePatch={(id, p) => void patchLaneLive(id, p)}
              onZonePatch={(id, p) => void patchZoneLive(id, p)}
              onEndpointPatch={(id, p) => void patchEndpointLive(id, p)}
              onFinishZone={() => void finishZone()}
            />
          ) : (
            <div className="map-placeholder">
              Đang chờ map từ Nav2 (/map)…
              <button type="button" onClick={() => void reload()}>
                Retry
              </button>
            </div>
          )}
        </main>

        <aside className="side">
          <section>
            <h2>Tools</h2>
            <div className="btn-row">
              {(
                [
                  ["select", "Select"],
                  ["endpoint", "Endpoint"],
                  ["lane", "Lane"],
                  ["zone", "Zone"],
                  ["pan", "Pan"],
                ] as const
              ).map(([m, label]) => (
                <button
                  key={m}
                  type="button"
                  className={mode === m ? "active" : ""}
                  onClick={() => setTool(m)}
                >
                  {label}
                </button>
              ))}
              <button type="button" onClick={() => mapRef.current?.fitMap()}>
                Fit Map
              </button>
            </div>
            <p className="hint">
              Lane: click START rồi END. Zone: click đỉnh, Enter / double-click /
              click điểm đầu để đóng. Semantic overlays chưa ràng buộc Nav2.
            </p>
            <div className="btn-row">
              <button type="button" onClick={() => void doExport()}>
                Export JSON
              </button>
              <button type="button" onClick={() => fileRef.current?.click()}>
                Import JSON
              </button>
              <button
                type="button"
                disabled={busy}
                onClick={() => {
                  setBusy(true);
                  void syncFilters()
                    .then((r) => {
                      console.info("filters", r);
                    })
                    .catch((e) =>
                      setError(e instanceof Error ? e.message : String(e)),
                    )
                    .finally(() => setBusy(false));
                }}
              >
                Sync Filters
              </button>
              <input
                ref={fileRef}
                type="file"
                accept="application/json,.json"
                hidden
                onChange={(e) => {
                  const f = e.target.files?.[0];
                  if (f) void doImportFile(f);
                  e.target.value = "";
                }}
              />
            </div>
            <p className="hint">
              Sync Filters → Nav2 KeepoutFilter / SpeedFilter (LoadMap). Lane
              nav dùng NavigateThroughPoses; path vẫn là /plan thật.
            </p>
          </section>

          {mode === "endpoint" && endpointDraft && (
            <section>
              <h2>New / Edit Endpoint</h2>
              <label>
                Name
                <input
                  value={endpointDraft.name}
                  onChange={(e) =>
                    setEndpointDraft({ ...endpointDraft, name: e.target.value })
                  }
                />
              </label>
              <label>
                Type
                <select
                  value={endpointDraft.type}
                  onChange={(e) =>
                    setEndpointDraft({
                      ...endpointDraft,
                      type: e.target.value as EndpointType,
                    })
                  }
                >
                  {ENDPOINT_TYPES.map((t) => (
                    <option key={t} value={t}>
                      {t}
                    </option>
                  ))}
                </select>
              </label>
              <label>
                yaw (rad)
                <input
                  type="number"
                  step="0.01"
                  value={endpointDraft.yaw}
                  onChange={(e) =>
                    setEndpointDraft({
                      ...endpointDraft,
                      yaw: Number(e.target.value),
                    })
                  }
                />
              </label>
              <div className="btn-row">
                <button
                  type="button"
                  disabled={busy}
                  onClick={() => void saveEndpointDraft()}
                >
                  Save
                </button>
              </div>
            </section>
          )}

          {mode === "lane" && laneDraft && (
            <section>
              <h2>New Lane</h2>
              <p className="hint">
                {laneDraft.placedStart
                  ? "Click END trên map, rồi Save."
                  : "Click START trên map."}
              </p>
              <label>
                Name
                <input
                  value={laneDraft.name}
                  onChange={(e) =>
                    setLaneDraft({ ...laneDraft, name: e.target.value })
                  }
                />
              </label>
              <label>
                Width (m)
                <input
                  type="number"
                  step="0.05"
                  min="0.1"
                  value={laneDraft.widthMeters}
                  onChange={(e) =>
                    setLaneDraft({
                      ...laneDraft,
                      widthMeters: Number(e.target.value),
                    })
                  }
                />
              </label>
              <label>
                Direction
                <select
                  value={laneDraft.direction}
                  onChange={(e) =>
                    setLaneDraft({
                      ...laneDraft,
                      direction: e.target.value as LaneDirection,
                    })
                  }
                >
                  <option value="BIDIRECTIONAL">BIDIRECTIONAL</option>
                  <option value="ONE_WAY">ONE_WAY</option>
                </select>
              </label>
              <div className="btn-row">
                <button
                  type="button"
                  disabled={busy || !laneReady}
                  onClick={() => void saveLaneDraft()}
                >
                  Save Lane
                </button>
              </div>
            </section>
          )}

          {mode === "zone" && zoneDraft && (
            <section>
              <h2>New Zone</h2>
              <p className="hint">
                Đỉnh: {zoneDraft.points.length} (cần ≥3). Enter hoặc double-click
                để đóng.
              </p>
              <label>
                Name
                <input
                  value={zoneDraft.name}
                  onChange={(e) =>
                    setZoneDraft({ ...zoneDraft, name: e.target.value })
                  }
                />
              </label>
              <label>
                Type
                <select
                  value={zoneDraft.type}
                  onChange={(e) =>
                    setZoneDraft({
                      ...zoneDraft,
                      type: e.target.value as ZoneType,
                    })
                  }
                >
                  {ZONE_TYPES.map((t) => (
                    <option key={t} value={t}>
                      {t}
                    </option>
                  ))}
                </select>
              </label>
              <div className="btn-row">
                <button
                  type="button"
                  disabled={busy || zoneDraft.points.length < 3}
                  onClick={() => void finishZone()}
                >
                  Finish Zone
                </button>
              </div>
            </section>
          )}

          {selectedEndpoint && mode === "select" && (
            <section>
              <h2>Endpoint</h2>
              <p>
                <strong>{selectedEndpoint.name}</strong> ({selectedEndpoint.type})
              </p>
              <p className="mono">
                x={selectedEndpoint.x.toFixed(3)} y=
                {selectedEndpoint.y.toFixed(3)} yaw=
                {selectedEndpoint.yaw.toFixed(3)}
              </p>
              <div className="btn-row">
                <button
                  type="button"
                  disabled={busy}
                  onClick={() => void navigateTo(selectedEndpoint.id).then(setNav)}
                >
                  Navigate
                </button>
                {nav.status === "NAVIGATING" && (
                  <button
                    type="button"
                    onClick={() => void cancelNavigate().then(setNav)}
                  >
                    Cancel Nav
                  </button>
                )}
                <button type="button" disabled={busy} onClick={() => void doDelete()}>
                  Delete
                </button>
              </div>
            </section>
          )}

          {selectedLane && mode === "select" && (
            <section>
              <h2>Lane</h2>
              <label>
                Name
                <input
                  value={selectedLane.name}
                  onChange={(e) =>
                    void patchLaneLive(selectedLane.id, { name: e.target.value })
                  }
                />
              </label>
              <label>
                Width (m)
                <input
                  type="number"
                  step="0.05"
                  value={selectedLane.widthMeters}
                  onChange={(e) =>
                    void patchLaneLive(selectedLane.id, {
                      widthMeters: Number(e.target.value),
                    })
                  }
                />
              </label>
              <label>
                Direction
                <select
                  value={selectedLane.direction}
                  onChange={(e) =>
                    void patchLaneLive(selectedLane.id, {
                      direction: e.target.value as LaneDirection,
                    })
                  }
                >
                  <option value="BIDIRECTIONAL">BIDIRECTIONAL</option>
                  <option value="ONE_WAY">ONE_WAY</option>
                </select>
              </label>
              <div className="btn-row">
                <button
                  type="button"
                  onClick={() =>
                    setLaneRoute((prev) =>
                      prev.includes(selectedLane.id)
                        ? prev
                        : [...prev, selectedLane.id],
                    )
                  }
                >
                  Add to route
                </button>
                <button type="button" disabled={busy} onClick={() => void doDelete()}>
                  Delete
                </button>
              </div>
            </section>
          )}

          <section>
            <h2>Lane route (Phase 3)</h2>
            <p className="hint">
              Thứ tự NavigateThroughPoses. JUNCTION cho phép nối lane gần nhau.
            </p>
            <ol className="ep-list">
              {laneRoute.map((id, i) => {
                const ln = lanes.find((l) => l.id === id);
                return (
                  <li key={`${id}-${i}`}>
                    {i + 1}. {ln?.name ?? id}
                  </li>
                );
              })}
            </ol>
            <div className="btn-row">
              <button
                type="button"
                disabled={busy || laneRoute.length === 0}
                onClick={() => {
                  setBusy(true);
                  void navigateLanes(laneRoute)
                    .then(setNav)
                    .catch((e) =>
                      setError(e instanceof Error ? e.message : String(e)),
                    )
                    .finally(() => setBusy(false));
                }}
              >
                Navigate Lanes
              </button>
              <button type="button" onClick={() => setLaneRoute([])}>
                Clear route
              </button>
              {nav.status === "NAVIGATING" && (
                <button
                  type="button"
                  onClick={() => void cancelNavigate().then(setNav)}
                >
                  Cancel Nav
                </button>
              )}
            </div>
            {nav.speed_limit_mps != null && (
              <p className="mono">
                speed_limit: {nav.speed_limit_mps.toFixed(3)} m/s
              </p>
            )}
          </section>

          {selectedZone && mode === "select" && (
            <section>
              <h2>Zone</h2>
              <label>
                Name
                <input
                  value={selectedZone.name}
                  onChange={(e) =>
                    void patchZoneLive(selectedZone.id, { name: e.target.value })
                  }
                />
              </label>
              <label>
                Type
                <select
                  value={selectedZone.type}
                  onChange={(e) =>
                    void patchZoneLive(selectedZone.id, {
                      type: e.target.value as ZoneType,
                    })
                  }
                >
                  {ZONE_TYPES.map((t) => (
                    <option key={t} value={t}>
                      {t}
                    </option>
                  ))}
                </select>
              </label>
              {selectedZone.type === "SPEED_LIMIT" && (
                <label>
                  maxSpeedMps
                  <input
                    type="number"
                    step="0.05"
                    value={selectedZone.maxSpeedMps ?? 0.2}
                    onChange={(e) =>
                      void patchZoneLive(selectedZone.id, {
                        maxSpeedMps: Number(e.target.value),
                      })
                    }
                  />
                </label>
              )}
              {selectedZone.type === "SINGLE_ROBOT" && (
                <label>
                  capacity
                  <input
                    type="number"
                    min={1}
                    value={selectedZone.capacity ?? 1}
                    onChange={(e) =>
                      void patchZoneLive(selectedZone.id, {
                        capacity: Number(e.target.value),
                      })
                    }
                  />
                </label>
              )}
              <div className="btn-row">
                <button
                  type="button"
                  disabled={busy}
                  onClick={() => {
                    const poly = selectedZone.polygon;
                    if (poly.length < 2) return;
                    const last = poly[poly.length - 1];
                    const prev = poly[poly.length - 2];
                    void patchZoneLive(selectedZone.id, {
                      polygon: [
                        ...poly,
                        {
                          x: (last.x + prev.x) / 2,
                          y: (last.y + prev.y) / 2,
                        },
                      ],
                    });
                  }}
                >
                  Add vertex
                </button>
                <button
                  type="button"
                  disabled={busy || selectedZone.polygon.length <= 3}
                  onClick={() =>
                    void patchZoneLive(selectedZone.id, {
                      polygon: selectedZone.polygon.slice(0, -1),
                    })
                  }
                >
                  Remove vertex
                </button>
              </div>
              <button type="button" disabled={busy} onClick={() => void doDelete()}>
                Delete
              </button>
            </section>
          )}

          <section>
            <h2>Objects</h2>
            <p className="hint">
              EP {endpoints.length} · Lane {lanes.length} · Zone {zones.length}
            </p>
            <ul className="ep-list">
              {endpoints.map((ep) => (
                <li key={ep.id}>
                  <button
                    type="button"
                    className={
                      selection?.kind === "endpoint" && selection.id === ep.id
                        ? "active"
                        : ""
                    }
                    onClick={() => {
                      setSelection({ kind: "endpoint", id: ep.id });
                      setMode("select");
                      clearDrafts();
                    }}
                  >
                    ● {ep.name}
                  </button>
                </li>
              ))}
              {lanes.map((ln) => (
                <li key={ln.id}>
                  <button
                    type="button"
                    className={
                      selection?.kind === "lane" && selection.id === ln.id
                        ? "active"
                        : ""
                    }
                    onClick={() => {
                      setSelection({ kind: "lane", id: ln.id });
                      setMode("select");
                      clearDrafts();
                    }}
                  >
                    ▬ {ln.name}
                  </button>
                </li>
              ))}
              {zones.map((z) => (
                <li key={z.id}>
                  <button
                    type="button"
                    className={
                      selection?.kind === "zone" && selection.id === z.id
                        ? "active"
                        : ""
                    }
                    onClick={() => {
                      setSelection({ kind: "zone", id: z.id });
                      setMode("select");
                      clearDrafts();
                    }}
                  >
                    ▢ {z.name}
                  </button>
                </li>
              ))}
            </ul>
          </section>

          <section>
            <h2>Robot (Phase 1)</h2>
            {robot.valid ? (
              <p className="mono">
                x={robot.x.toFixed(3)} y={robot.y.toFixed(3)} yaw=
                {robot.yaw.toFixed(3)}
              </p>
            ) : (
              <p className="hint">Chưa có TF map→base_footprint</p>
            )}
            <p className="hint">Path poses: {path.poses.length}</p>
          </section>
        </aside>
      </div>
    </div>
  );
}
