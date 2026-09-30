"""FastAPI entry — WaroTrans Demo Phase 1–3 (semantic nav integration)."""

from __future__ import annotations

import asyncio
import json
from contextlib import asynccontextmanager
from pathlib import Path
from typing import List

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response
from fastapi.staticfiles import StaticFiles

from .ros_gateway import gateway
from .session import (
    Endpoint,
    EndpointCreate,
    EndpointUpdate,
    Lane,
    LaneCreate,
    LaneNavigateRequest,
    LaneUpdate,
    MapVersion,
    NavState,
    RobotPose,
    TrafficZone,
    ZoneCreate,
    ZoneUpdate,
    store,
)

FRONTEND_DIST = Path(__file__).resolve().parents[2] / "frontend" / "dist"


def _try_sync_filters() -> None:
    try:
        gateway.sync_filters()
    except Exception as exc:
        print(f"[demo] filter sync skipped: {exc}")


@asynccontextmanager
async def lifespan(app: FastAPI):
    try:
        gateway.start()
    except Exception as exc:
        print(f"[demo] ROS gateway start failed: {exc}")
    yield
    try:
        gateway.stop()
    except Exception:
        pass


app = FastAPI(
    title="WaroTrans Demo Phase 3",
    version="0.3.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
def health() -> dict:
    return {
        "ok": True,
        "phase": 3,
        "map_ready": gateway.get_meta() is not None,
        "nav": store.nav.model_dump(),
        "counts": {
            "endpoints": len(store.list_endpoints()),
            "lanes": len(store.list_lanes()),
            "zones": len(store.list_zones()),
        },
    }


@app.get("/api/map/meta")
def map_meta() -> dict:
    meta = gateway.get_meta()
    if meta is None:
        raise HTTPException(503, "Map not available yet (is Nav2 map_server active?)")
    return meta.to_dict()


@app.get("/api/map/image.png")
def map_image() -> Response:
    png = gateway.get_png()
    if png is None:
        raise HTTPException(503, "Map image not available yet")
    return Response(content=png, media_type="image/png")


@app.get("/api/endpoints", response_model=List[Endpoint])
def list_endpoints() -> List[Endpoint]:
    return store.list_endpoints()


@app.post("/api/endpoints", response_model=Endpoint)
def create_endpoint(body: EndpointCreate) -> Endpoint:
    return store.create(body)


@app.get("/api/endpoints/{endpoint_id}", response_model=Endpoint)
def get_endpoint(endpoint_id: str) -> Endpoint:
    ep = store.get(endpoint_id)
    if ep is None:
        raise HTTPException(404, "Endpoint not found")
    return ep


@app.put("/api/endpoints/{endpoint_id}", response_model=Endpoint)
def update_endpoint(endpoint_id: str, body: EndpointUpdate) -> Endpoint:
    ep = store.update(endpoint_id, body)
    if ep is None:
        raise HTTPException(404, "Endpoint not found")
    return ep


@app.delete("/api/endpoints/{endpoint_id}")
def delete_endpoint(endpoint_id: str) -> dict:
    if not store.delete(endpoint_id):
        raise HTTPException(404, "Endpoint not found")
    return {"ok": True}


@app.get("/api/lanes", response_model=List[Lane])
def list_lanes() -> List[Lane]:
    return store.list_lanes()


@app.post("/api/lanes", response_model=Lane)
def create_lane(body: LaneCreate) -> Lane:
    return store.create_lane(body)


@app.put("/api/lanes/{lane_id}", response_model=Lane)
def update_lane(lane_id: str, body: LaneUpdate) -> Lane:
    lane = store.update_lane(lane_id, body)
    if lane is None:
        raise HTTPException(404, "Lane not found")
    return lane


@app.delete("/api/lanes/{lane_id}")
def delete_lane(lane_id: str) -> dict:
    if not store.delete_lane(lane_id):
        raise HTTPException(404, "Lane not found")
    return {"ok": True}


@app.get("/api/zones", response_model=List[TrafficZone])
def list_zones() -> List[TrafficZone]:
    return store.list_zones()


@app.post("/api/zones", response_model=TrafficZone)
def create_zone(body: ZoneCreate) -> TrafficZone:
    try:
        zone = store.create_zone(body)
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from None
    _try_sync_filters()
    return zone


@app.put("/api/zones/{zone_id}", response_model=TrafficZone)
def update_zone(zone_id: str, body: ZoneUpdate) -> TrafficZone:
    try:
        zone = store.update_zone(zone_id, body)
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from None
    if zone is None:
        raise HTTPException(404, "Zone not found")
    _try_sync_filters()
    return zone


@app.delete("/api/zones/{zone_id}")
def delete_zone(zone_id: str) -> dict:
    if not store.delete_zone(zone_id):
        raise HTTPException(404, "Zone not found")
    _try_sync_filters()
    return {"ok": True}


@app.get("/api/semantic-map", response_model=MapVersion)
def export_semantic_map() -> MapVersion:
    return store.export_map()


@app.put("/api/semantic-map", response_model=MapVersion)
def import_semantic_map(body: MapVersion) -> MapVersion:
    result = store.import_map(body)
    _try_sync_filters()
    return result


@app.post("/api/filters/sync")
def sync_filters() -> dict:
    try:
        return gateway.sync_filters()
    except RuntimeError as exc:
        raise HTTPException(503, str(exc)) from None


@app.post("/api/navigate/lanes")
def navigate_lanes(body: LaneNavigateRequest) -> dict:
    try:
        return gateway.navigate_lanes(body.lane_ids, spacing_m=body.spacing_m)
    except KeyError as exc:
        raise HTTPException(404, str(exc)) from None
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from None
    except TimeoutError as exc:
        raise HTTPException(503, str(exc)) from None
    except RuntimeError as exc:
        raise HTTPException(503, str(exc)) from None


@app.post("/api/navigate/cancel")
def cancel_navigate() -> dict:
    return gateway.cancel()


@app.post("/api/navigate/{endpoint_id}")
def navigate(endpoint_id: str) -> dict:
    try:
        return gateway.navigate_to_endpoint(endpoint_id)
    except KeyError:
        raise HTTPException(404, "Endpoint not found") from None
    except TimeoutError as exc:
        raise HTTPException(503, str(exc)) from None
    except RuntimeError as exc:
        raise HTTPException(503, str(exc)) from None


@app.get("/api/nav/status", response_model=NavState)
def nav_status() -> NavState:
    return store.nav


@app.get("/api/robot", response_model=RobotPose)
def robot_pose() -> RobotPose:
    return store.robot


@app.get("/api/path")
def get_path() -> dict:
    return store.path.model_dump()


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    loop = asyncio.get_running_loop()
    queue: asyncio.Queue = asyncio.Queue()

    def on_event(payload: dict) -> None:
        loop.call_soon_threadsafe(queue.put_nowait, payload)

    gateway.add_ws_listener(on_event)
    await ws.send_text(json.dumps({"type": "snapshot", **gateway.snapshot()}))
    try:
        while True:
            try:
                recv_task = asyncio.create_task(ws.receive_text())
                queue_task = asyncio.create_task(queue.get())
                done, pending = await asyncio.wait(
                    {recv_task, queue_task},
                    return_when=asyncio.FIRST_COMPLETED,
                )
                for task in pending:
                    task.cancel()
                if queue_task in done:
                    payload = queue_task.result()
                    await ws.send_text(json.dumps(payload))
                if recv_task in done:
                    _ = recv_task.result()
            except WebSocketDisconnect:
                break
    finally:
        gateway.remove_ws_listener(on_event)


if FRONTEND_DIST.is_dir():
    app.mount("/", StaticFiles(directory=str(FRONTEND_DIST), html=True), name="frontend")
