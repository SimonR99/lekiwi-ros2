#!/usr/bin/env python3
"""
Moondream Vision Server for Dino AI

FastAPI server that provides vision analysis using Moondream2 model.
Supports image captioning, visual Q&A, object detection, and pointing.
"""

from fastapi import FastAPI, UploadFile, File, Form, HTTPException
from fastapi.responses import JSONResponse
from transformers import AutoModelForCausalLM
from PIL import Image, ImageDraw
import io
import base64
import logging
import torch
from typing import Optional
import uvicorn

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Moondream Vision Server",
    description="Vision analysis server using Moondream2 for Dino AI",
    version="1.0.0"
)

# Global model variable
model = None

@app.on_event("startup")
async def load_model():
    """Load Moondream model on startup"""
    global model
    try:
        logger.info("Loading Moondream2 model...")
        
        # Check if CUDA is available
        device = "cuda" if torch.cuda.is_available() else "cpu"
        logger.info(f"Using device: {device}")
        
        # Load model with appropriate device mapping
        if device == "cuda":
            model = AutoModelForCausalLM.from_pretrained(
                "vikhyatk/moondream2",
                revision="2024-04-02",  # Use stable revision
                trust_remote_code=True,
                device_map={"": "cuda"},
                torch_dtype=torch.float16
            )
        else:
            model = AutoModelForCausalLM.from_pretrained(
                "vikhyatk/moondream2",
                revision="2024-04-02",
                trust_remote_code=True,
                torch_dtype=torch.float32
            )
            model = model.to("cpu")
        
        logger.info("Moondream2 model loaded successfully!")
        
    except Exception as e:
        logger.error(f"Failed to load Moondream model: {e}")
        raise


def read_image_from_upload(image_file: UploadFile) -> Image.Image:
    """Read PIL Image from uploaded file"""
    contents = image_file.file.read()
    return Image.open(io.BytesIO(contents))


def read_image_from_base64(image_b64: str) -> Image.Image:
    """Read PIL Image from base64 string"""
    image_data = base64.b64decode(image_b64)
    return Image.open(io.BytesIO(image_data))


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "model_loaded": model is not None,
        "device": "cuda" if torch.cuda.is_available() else "cpu"
    }


@app.post("/caption")
async def caption_image(image: UploadFile = File(...), length: str = Form("normal")):
    """Generate image caption"""
    if model is None:
        raise HTTPException(status_code=503, detail="Model not loaded")
    
    try:
        pil_image = read_image_from_upload(image)
        
        if length == "short":
            result = model.caption(pil_image, length="short")
        else:
            result = model.caption(pil_image, length="normal")
        
        return {"caption": result}
        
    except Exception as e:
        logger.error(f"Caption generation failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/query")
async def visual_query(image: UploadFile = File(...), query: str = Form(...)):
    """Answer visual questions about image"""
    if model is None:
        raise HTTPException(status_code=503, detail="Model not loaded")
    
    try:
        pil_image = read_image_from_upload(image)
        result = model.query(pil_image, query)
        
        return {"answer": result}
        
    except Exception as e:
        logger.error(f"Visual query failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/analyze")
async def analyze_image(request_data: dict):
    """Analyze image with prompt (compatible with Dino AI)"""
    if model is None:
        raise HTTPException(status_code=503, detail="Model not loaded")
    
    try:
        # Extract image and prompt from request
        image_b64 = request_data.get("image")
        prompt = request_data.get("prompt", "Describe what you see")
        
        if not image_b64:
            raise HTTPException(status_code=400, detail="No image provided")
        
        # Decode image
        pil_image = read_image_from_base64(image_b64)
        
        # Generate analysis
        result = model.query(pil_image, prompt)
        
        return {"description": result}
        
    except Exception as e:
        logger.error(f"Image analysis failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/detect")
async def detect_objects(image: UploadFile = File(...), target: str = Form(...)):
    """Detect specific objects in image"""
    if model is None:
        raise HTTPException(status_code=503, detail="Model not loaded")
    
    try:
        pil_image = read_image_from_upload(image)
        result = model.detect(pil_image, target)
        
        return {"detection": result}
        
    except Exception as e:
        logger.error(f"Object detection failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/point")
async def point_target(image: UploadFile = File(...), query: str = Form(...)):
    """Point to objects in image"""
    if model is None:
        raise HTTPException(status_code=503, detail="Model not loaded")
    
    try:
        pil_image = read_image_from_upload(image)
        result = model.point(pil_image, query)
        
        return {"points": result}
        
    except Exception as e:
        logger.error(f"Pointing failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/point/image")
async def point_and_save(image: UploadFile = File(...), query: str = Form(...)):
    """Point to objects and save image with visual markers"""
    if model is None:
        raise HTTPException(status_code=503, detail="Model not loaded")
    
    try:
        pil_image = read_image_from_upload(image)
        points = model.point(pil_image, query)["points"]

        # Draw points on image
        draw = ImageDraw.Draw(pil_image)
        for point in points:
            x = int(point["x"] * pil_image.width)
            y = int(point["y"] * pil_image.height)
            draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill="red", outline="red")

        # Save image
        output_path = "output_image_with_points.jpeg"
        pil_image.save(output_path)
        
        return JSONResponse(content={
            "message": "Image saved with points", 
            "output_path": output_path,
            "points": points
        })
        
    except Exception as e:
        logger.error(f"Point and save failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/info")
async def model_info():
    """Get model information"""
    return {
        "model_name": "vikhyatk/moondream2",
        "revision": "2024-04-02",
        "capabilities": [
            "image_captioning",
            "visual_question_answering", 
            "object_detection",
            "pointing"
        ],
        "endpoints": [
            "/caption",
            "/query", 
            "/analyze",
            "/detect",
            "/point",
            "/point/image"
        ]
    }


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Moondream Vision Server")
    parser.add_argument("--host", default="127.0.0.1", help="Host to bind to")
    parser.add_argument("--port", type=int, default=5000, help="Port to bind to")
    parser.add_argument("--workers", type=int, default=1, help="Number of workers")
    
    args = parser.parse_args()
    
    logger.info(f"Starting Moondream server on {args.host}:{args.port}")
    
    uvicorn.run(
        "moondream_server:app",
        host=args.host,
        port=args.port,
        workers=args.workers,
        log_level="info"
    )