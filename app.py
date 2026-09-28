"""Vectoriseur de logos — API + site (FastAPI, déployable sur Vercel).

POST /api/vectorize          corps = octets de l'image (PNG/JPG/WebP)
     ?format=json (défaut)   -> {svg, svg_transparent, width, height, fg, bg, shapes, arcs, ms}
     ?format=svg             -> le SVG directement (image/svg+xml)
     &transparent=1          -> avec format=svg : sans fond
GET  /api/health             -> {"ok": true}
"""
from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse, Response
from starlette.concurrency import run_in_threadpool

from vectorizer import vectorize

MAX_BYTES = 4_400_000  # Vercel limite le corps des requêtes à 4,5 Mo

app = FastAPI(title="Vectoriseur de logos", docs_url=None, redoc_url=None)

CORS = {
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Methods": "POST, GET, OPTIONS",
    "Access-Control-Allow-Headers": "Content-Type",
}


def _error(status: int, message: str) -> JSONResponse:
    return JSONResponse({"error": message}, status_code=status, headers=CORS)


@app.get("/api/health")
async def health(request: Request):
    return JSONResponse({"ok": True}, headers=CORS)


@app.options("/api/vectorize")
async def vectorize_options(request: Request):
    return Response(status_code=204, headers=CORS)


@app.post("/api/vectorize")
async def vectorize_endpoint(request: Request):
    data = await request.body()
    if not data:
        return _error(400, "Aucune image reçue.")
    if len(data) > MAX_BYTES:
        return _error(413, "Image trop lourde (max 4,4 Mo).")
    try:
        result = await run_in_threadpool(vectorize, data)
    except ValueError as e:
        return _error(422, str(e))
    except Exception:
        return _error(422, "Impossible de lire cette image. Utilisez un PNG, JPG ou WebP.")

    fmt = request.query_params.get("format", "json").lower()
    if fmt == "svg":
        transparent = request.query_params.get("transparent", "0").lower() in ("1", "true", "yes")
        svg = result["svg_transparent"] if transparent else result["svg"]
        return Response(svg, media_type="image/svg+xml", headers=CORS)
    return JSONResponse(result, headers=CORS)


@app.get("/")
async def index(request: Request):
    # Normalement servi directement par le CDN depuis public/ ; ceci est un repli.
    return FileResponse("public/index.html")
