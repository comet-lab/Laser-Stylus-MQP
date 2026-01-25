import * as fabric from 'fabric';
import { Position, ShapeType } from './types';
import * as Utils from './utils';

export class DrawingTracker {
    private fCanvas: fabric.Canvas;
    private video: HTMLVideoElement;
    private apiBaseUrl: string;
    private onShapeComplete: () => void;

    // State for Drag-to-Create
    private currentShapeType: ShapeType | null = null;
    private isCreatingShape: boolean = false;
    private shapeStartPos: { x: number, y: number } | null = null;
    private activeShape: fabric.FabricObject | null = null;
    private hasPlacedShape: boolean = false;

    // Heat marker mode
    private isMarkerMode: boolean = false;
    private markerPoints: Position[] = [];
    private markerObjects: fabric.Object[] = [];

    // --- CONFIG: Visual Defaults ---
    private readonly SHAPE_DEFAULTS = {
        fill: 'transparent',
        stroke: '#ff0000',
        strokeWidth: 6,
        strokeUniform: true,
        strokeLineCap: 'round' as const,
        strokeLineJoin: 'round' as const,

        // Prevent borders scaling when resizing objects
        objectCaching: false,

        cornerColor: '#007AFF',
        cornerStrokeColor: '#ffffff',
        borderColor: '#007AFF',
        cornerStyle: 'circle' as const,
        cornerSize: 12,
        touchCornerSize: 24,
        transparentCorners: false,
        padding: 10,

        originX: 'left' as const,
        originY: 'top' as const
    };

    constructor(
        canvas: HTMLCanvasElement,
        video: HTMLVideoElement,
        apiBaseUrl: string = `http://${window.location.hostname}:443`,
        onShapeComplete: () => void = () => { }
    ) {
        this.video = video;
        this.apiBaseUrl = apiBaseUrl;
        this.onShapeComplete = onShapeComplete;

        // Checks the DOM element for an existing fabric instance and kills it.
        // Currently still getting related error
        const el = canvas as any;
        if (el.__canvas) {
            // Fabric stores the instance on the element in .__canvas
            el.__canvas.dispose();
            el.__canvas = undefined;
        }

        this.fCanvas = new fabric.Canvas(canvas, {
            selection: false,
            preserveObjectStacking: true,
            containerClass: 'fabric-canvas-container'
        });

        fabric.FabricObject.ownDefaults = {
            ...fabric.FabricObject.ownDefaults,
            ...(this.SHAPE_DEFAULTS as any)
        };

        const brush = new fabric.PencilBrush(this.fCanvas);
        brush.width = 6;
        brush.color = '#ff0000';
        this.fCanvas.freeDrawingBrush = brush;

        this.fCanvas.on('mouse:down', this.onMouseDown.bind(this));
        this.fCanvas.on('mouse:move', this.onMouseMove.bind(this));
        this.fCanvas.on('mouse:up', this.onMouseUp.bind(this));

        this.fCanvas.on('path:created', (e: any) => {
            if (e.path) {
                e.path.set({
                    ...this.SHAPE_DEFAULTS,
                    strokeUniform: false
                });
                e.path.setCoords();
            }

            this.hasPlacedShape = true;
            this.fCanvas.isDrawingMode = false;
            this.fCanvas.defaultCursor = 'default';
            this.onShapeComplete();
        });
    }

    public dispose(): void {
        if (this.fCanvas) {
            this.fCanvas.dispose();
        }
    }

    // --- Interaction Handlers ---

    private onMouseDown(opt: any) {
    if (this.isMarkerMode) {
        const pointer = this.fCanvas.getScenePoint(opt.e);

        // Add marker dot
        this.markerPoints.push({ x: pointer.x, y: pointer.y });

        const dot = new fabric.Circle({
            left: pointer.x,
            top: pointer.y,
            radius: 6,
            fill: '#ff0000',
            originX: 'center',
            originY: 'center',
            selectable: false,
            evented: false
        });

        this.fCanvas.add(dot);
        this.markerObjects.push(dot);

        this.fCanvas.requestRenderAll();
        return;
    }
        if (this.hasPlacedShape) return;

        if (!this.currentShapeType || this.currentShapeType === 'freehand') return;
        if (opt.target) return;

        this.isCreatingShape = true;

        const pointer = this.fCanvas.getScenePoint(opt.e);
        this.shapeStartPos = { x: pointer.x, y: pointer.y };

        const commonOpts = {
            left: pointer.x,
            top: pointer.y,
            ...this.SHAPE_DEFAULTS
        };

        switch (this.currentShapeType) {
            case 'square':
                this.activeShape = new fabric.Rect({ ...commonOpts, width: 0, height: 0 });
                break;
            case 'circle':
                this.activeShape = new fabric.Ellipse({ ...commonOpts, rx: 0, ry: 0 });
                break;
            case 'triangle':
                this.activeShape = new fabric.Triangle({ ...commonOpts, width: 0, height: 0 });
                break;
            case 'line':
                this.activeShape = new fabric.Line([pointer.x, pointer.y, pointer.x, pointer.y], {
                    ...commonOpts,
                    fill: 'transparent'
                });
                break;
        }

        if (this.activeShape) {
            this.fCanvas.add(this.activeShape);
            this.fCanvas.setActiveObject(this.activeShape);
        }
    }

    private onMouseMove(opt: any) {
        if (!this.isCreatingShape || !this.activeShape || !this.shapeStartPos) return;

        const pointer = this.fCanvas.getScenePoint(opt.e);
        const w = Math.abs(pointer.x - this.shapeStartPos.x);
        const h = Math.abs(pointer.y - this.shapeStartPos.y);

        const left = Math.min(pointer.x, this.shapeStartPos.x);
        const top = Math.min(pointer.y, this.shapeStartPos.y);

        if (this.activeShape instanceof fabric.Line) {
            this.activeShape.set({ x2: pointer.x, y2: pointer.y });
        } else if (this.activeShape instanceof fabric.Ellipse) {
            this.activeShape.set({ left, top, rx: w / 2, ry: h / 2 });
        } else {
            this.activeShape.set({ left, top, width: w, height: h });
        }

        this.activeShape.setCoords();
        this.fCanvas.requestRenderAll();
    }

    private onMouseUp() {
        if (this.isCreatingShape) {
            this.isCreatingShape = false;
            if (this.activeShape) {
                // Ensure shape is not microscopic (Click vs Drag)
                if (this.activeShape instanceof fabric.Line) {
                    const x1 = this.activeShape.x1 || 0;
                    const y1 = this.activeShape.y1 || 0;
                    const x2 = this.activeShape.x2 || 0;
                    const y2 = this.activeShape.y2 || 0;
                    const dist = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
                    if (dist < 5) this.activeShape.set({ x2: x1 + 50, y2: y1 + 50 });
                } else if (this.activeShape instanceof fabric.Ellipse) {
                    if ((this.activeShape.rx || 0) < 2) this.activeShape.set({ rx: 25, ry: 25 });
                } else {
                    if ((this.activeShape.width || 0) < 5) this.activeShape.set({ width: 50, height: 50 });
                }

                this.activeShape.setCoords();
                this.hasPlacedShape = true;
                this.fCanvas.requestRenderAll();
                this.onShapeComplete();
            }
        }
    }

    // --- Public API ---

    public setShapeType(type: ShapeType | null): void {
        if (this.hasPlacedShape && type !== null) {
            return;
        }

        this.currentShapeType = type;

        if (type === 'freehand') {
            this.fCanvas.isDrawingMode = true;
            this.fCanvas.discardActiveObject();
            this.fCanvas.requestRenderAll();
        } else {
            this.fCanvas.isDrawingMode = false;
            this.fCanvas.defaultCursor = type ? 'crosshair' : 'default';
        }
    }

    public clearDrawing(): void {
        this.fCanvas.clear();
        this.activeShape = null;
        this.hasPlacedShape = false;

        // Reset freehand mode if it was selected
        if (this.currentShapeType === 'freehand') {
            this.fCanvas.isDrawingMode = true;
            this.fCanvas.requestRenderAll();
        }
    }

    public disableDrawing(): void {
        this.fCanvas.isDrawingMode = false;
        this.currentShapeType = null;
        this.fCanvas.discardActiveObject();

        this.fCanvas.forEachObject(o => {
            o.selectable = false;
            o.evented = false;
        });

        this.fCanvas.requestRenderAll();
        this.fCanvas.defaultCursor = 'default';
    }

    public enableDrawing(): void {
        this.fCanvas.forEachObject(o => {
            o.selectable = true;
            o.evented = true;
        });
    }

    public isDrawingEnabled(): boolean {
        return this.fCanvas.isDrawingMode || this.fCanvas.getObjects().length > 0;
    }

    public hasShape(): boolean {
        return this.hasPlacedShape || this.fCanvas.getObjects().length > 0;
    }

    public updateCanvasSize(width: number, height: number): void {
        this.fCanvas.setDimensions({ width, height });
    }

    public render(): void {
        this.fCanvas.renderAll();
    }

    // Heat marker methods
    public enableMarkerMode(): void {
    this.clearDrawing();
    this.disableDrawing();

    this.isMarkerMode = true;
    this.markerPoints = [];
    this.markerObjects = [];

    this.fCanvas.defaultCursor = 'crosshair';
}

public disableMarkerMode(): void {
    this.isMarkerMode = false;
    this.fCanvas.defaultCursor = 'default';
}

public getHeatMarkers(): Position[] {
    return this.markerPoints;
}

public getHeatMarkersInVideoSpace(): Position[] {
    return this.markerPoints.map(p => ({
        x: (p.x / this.fCanvas.getWidth()) * this.video.videoWidth,
        y: (p.y / this.fCanvas.getHeight()) * this.video.videoHeight
    }));
}

// end heat method marker 

    // --- Execution & Export Logic ---
    public async executePath(speed: number, raster_type: string, density: number, isFillEnabled: boolean): Promise<any> {
        const pixels = this.generatePixelPath();

        // Convert canvas pixels to video-space coordinates for the movement path
        const videoPixels = pixels.map(p => ({
            x: (p.x / this.fCanvas.getWidth()) * this.video.videoWidth,
            y: (p.y / this.fCanvas.getHeight()) * this.video.videoHeight
        }));

        // --- Generate the Raster Mask Blob ---
        const width = this.fCanvas.getWidth();
        const height = this.fCanvas.getHeight();
        const tempCanvas = document.createElement('canvas');
        tempCanvas.width = width;
        tempCanvas.height = height;
        const ctx = tempCanvas.getContext('2d');

        if (!ctx) throw new Error("Could not create temp context");

        // 1. Initialize background to white (non-engrave area)
        ctx.fillStyle = "#ffffff";
        ctx.fillRect(0, 0, width, height);

        // 2. Setup drawing styles for the mask (black = engrave area)
        ctx.fillStyle = "#000000";
        ctx.strokeStyle = "#000000";
        ctx.lineWidth = 1;

        // 3. Draw every object from the Fabric canvas onto the mask
        this.fCanvas.getObjects().forEach(obj => {
            ctx.save();

            // Fabric objects have a path or coordinate array we can iterate
            if (obj instanceof fabric.Path) {
                ctx.beginPath();
                const pathData = (obj as any).path; // Access the SVG command array

                pathData.forEach((cmd: any) => {
                    const type = cmd[0];
                    switch (type) {
                        case 'M': ctx.moveTo(cmd[1], cmd[2]); break;
                        case 'L': ctx.lineTo(cmd[1], cmd[2]); break;
                        case 'Q': ctx.quadraticCurveTo(cmd[1], cmd[2], cmd[3], cmd[4]); break;
                        case 'C': ctx.bezierCurveTo(cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]); break;
                        case 'Z': ctx.closePath(); break;
                    }
                });

                if (isFillEnabled) {
                    // Ensure the shape is closed for the raster even if the user didn't finish the loop
                    ctx.closePath();
                    // 'nonzero' ensures self-intersecting loops (like a fish tail) stay filled
                    ctx.fill('nonzero');
                }

                // Stroke as well to ensure the boundary pixels are captured
                ctx.stroke();

            } else if (obj instanceof fabric.Rect || obj instanceof fabric.Triangle || obj instanceof fabric.Ellipse) {
                obj.render(ctx);
            }

            ctx.restore();
        });

        const blob = await new Promise<Blob | null>(resolve => tempCanvas.toBlob(resolve, 'image/png'));
        if (!blob) throw new Error("Failed to generate image blob");

        // --- Bundle everything into FormData ---
        const formData = new FormData();
        formData.append('speed', (Number(speed) / 1000).toString());
        formData.append('raster_type', raster_type);
        formData.append('density', density.toString());
        formData.append('pixels', JSON.stringify(videoPixels));
        formData.append('file', blob, 'path.png');

        const response = await fetch(`${this.apiBaseUrl}/api/execute`, {
            method: 'POST',
            body: formData
        });

        if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.detail || "Execution failed");
        }

        return await response.json();
    }

    public async updateViewSettings(isTransformedViewOn: boolean, isThermalViewOn: boolean): Promise<any> {
        const response = await fetch(`${this.apiBaseUrl}/api/view_settings`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ isTransformedViewOn, isThermalViewOn })
        });
        return await response.json();
    }

    // --- Internal: Geometry Bridge ---

    private generatePixelPath(): Position[] {
        const objects = this.fCanvas.getObjects();
        const pixels: Position[] = [];

        for (const obj of objects) {
            if (obj instanceof fabric.Path) {
                const pathObj = obj as any;
                if (pathObj.path) {
                    let lastX = 0, lastY = 0;
                    for (const cmd of pathObj.path) {
                        const type = cmd[0];
                        const x = cmd[1];
                        const y = cmd[2];
                        if (type === 'M') {
                            lastX = x; lastY = y;
                        } else if (type === 'L' || type === 'Q') {
                            const gen = Utils.generateLinePixels(lastX, lastY, x, y);
                            for (const p of gen) pixels.push(p);
                            lastX = x; lastY = y;
                        }
                    }
                }
            }
            else if (obj instanceof fabric.Triangle) {
                const triangle = obj as fabric.Triangle;

                // 1. Get the object's transformation matrix (handles position, rotation, scale)
                const matrix = triangle.calcTransformMatrix();

                // 2. Define the 3 local vertices of a Fabric Triangle
                // Fabric defines triangles with (0,0) at the center of the bounding box
                const w = triangle.width;
                const h = triangle.height;

                const localPoints = [
                    new fabric.Point(0, -h / 2),      // Top Center
                    new fabric.Point(w / 2, h / 2),   // Bottom Right
                    new fabric.Point(-w / 2, h / 2)   // Bottom Left
                ];

                // 3. Transform local points to absolute canvas coordinates
                const vertices = localPoints.map(p => p.transform(matrix));

                // 4. Generate pixels for the three edges
                for (let i = 0; i < vertices.length; i++) {
                    const start = vertices[i];
                    const end = vertices[(i + 1) % vertices.length];
                    const gen = Utils.generateLinePixels(start.x, start.y, end.x, end.y);
                    for (const p of gen) pixels.push(p);
                }
            }
            else if (obj.type === 'ellipse' || obj.type === 'circle') {
                const ellipse = obj as fabric.Ellipse;
                const center = ellipse.getCenterPoint();
                const rx = ellipse.rx * ellipse.scaleX;
                const ry = ellipse.ry * ellipse.scaleY;
                const rotation = (ellipse.angle || 0) * (Math.PI / 180);

                const steps = Math.max(120, Math.floor((rx + ry) * 2));
                let prevX = 0, prevY = 0;

                for (let i = 0; i <= steps; i++) {
                    const t = (i / steps) * 2 * Math.PI;
                    const rawX = rx * Math.cos(t);
                    const rawY = ry * Math.sin(t);
                    const rotX = rawX * Math.cos(rotation) - rawY * Math.sin(rotation);
                    const rotY = rawX * Math.sin(rotation) + rawY * Math.cos(rotation);
                    const finalX = center.x + rotX;
                    const finalY = center.y + rotY;

                    if (i > 0) {
                        const gen = Utils.generateLinePixels(prevX, prevY, finalX, finalY);
                        for (const p of gen) pixels.push(p);
                    }
                    prevX = finalX;
                    prevY = finalY;
                }
            }
            else {
                const coords = obj.getCoords();
                for (let i = 0; i < coords.length; i++) {
                    const start = coords[i];
                    const end = coords[(i + 1) % coords.length];
                    if (obj.type === 'line' && i === coords.length - 1) continue;
                    const gen = Utils.generateLinePixels(start.x, start.y, end.x, end.y);
                    for (const p of gen) pixels.push(p);
                }
            }
        }
        return pixels;
    }
}