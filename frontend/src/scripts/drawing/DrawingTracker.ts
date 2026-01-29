import * as fabric from 'fabric';
import { Position, ShapeType } from './types';
import * as Utils from './utils';

export class DrawingTracker {
    private fCanvas: fabric.Canvas;
    private video: HTMLVideoElement;
    private apiBaseUrl: string;
    private onShapeComplete: () => void;
    private onFixturesChange: () => void;

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

    // Fixtures mode
    private isFixturesMode: boolean = false;
    private fixturesCanvas: HTMLCanvasElement | null = null;
    private fixturesCtx: CanvasRenderingContext2D | null = null;
    private currentBrushType: 'round' | 'square' | null = null;
    private currentBrushSize: number = 50;
    private isErasing: boolean = false;
    private isFixturesDrawing: boolean = false;
    private lastFixturesPoint: { x: number, y: number } | null = null;
    private fixturesApplied: boolean = false;

    // --- CONFIG: Visual Defaults ---
    private readonly SHAPE_DEFAULTS = {
        fill: 'transparent',
        stroke: '#007AFF',
        strokeWidth: 6,
        strokeUniform: true,
        strokeLineCap: 'round' as const,
        strokeLineJoin: 'round' as const,
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
        onShapeComplete: () => void = () => { },
        onFixturesChange: () => void = () => { }
    ) {
        this.video = video;
        this.apiBaseUrl = apiBaseUrl;
        this.onShapeComplete = onShapeComplete;
        this.onFixturesChange = onFixturesChange;

        const el = canvas as any;
        if (el.__canvas) {
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
        brush.color = '#007AFF';
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

        this.createFixturesCanvas(canvas);
    }

    private createFixturesCanvas(mainCanvas: HTMLCanvasElement): void {
        this.fixturesCanvas = document.createElement('canvas');
        this.fixturesCanvas.id = 'fixturesCanvas';
        
        // Set internal dimensions to match main canvas
        this.fixturesCanvas.width = mainCanvas.width;
        this.fixturesCanvas.height = mainCanvas.height;
        
        // CSS already handles positioning via #fixturesCanvas styles
        // Just set opacity here (prevents "dots" at overlaps)
        this.fixturesCanvas.style.opacity = '0.6';
        
        this.fixturesCtx = this.fixturesCanvas.getContext('2d', { willReadFrequently: true });
        
        // Append to viewport (same parent as main canvas)
        mainCanvas.parentElement?.appendChild(this.fixturesCanvas);
    }

    public dispose(): void {
        if (this.fCanvas) {
            this.fCanvas.dispose();
        }
        if (this.fixturesCanvas) {
            this.fixturesCanvas.remove();
        }
    }

    // --- Fixtures Mode Methods ---
    public enableFixturesMode(): void {
        this.isFixturesMode = true;
        this.disableDrawing();
        
        if (this.fixturesCanvas) {
            this.fixturesCanvas.classList.add('active');
            this.fixturesCanvas.style.pointerEvents = 'auto';
        }
    }

    public disableFixturesMode(): void {
        this.isFixturesMode = false;
        this.disableFixturesBrush();
        
        if (this.fixturesCanvas) {
            if (this.hasFixtures()) {
                this.fixturesCanvas.classList.add('active');
                this.fixturesCanvas.style.pointerEvents = 'none';
            } else {
                this.fixturesCanvas.classList.remove('active');
            }
        }
    }

    public showFixtures(): void {
        if (this.fixturesCanvas && this.hasFixtures()) {
            this.fixturesCanvas.classList.add('active');
            this.fixturesCanvas.style.pointerEvents = 'none';
        }
    }

    public setFixturesBrush(brushType: 'round' | 'square', size: number, isEraser: boolean): void {
        this.currentBrushType = brushType;
        this.currentBrushSize = size;
        this.isErasing = isEraser;

        if (this.fixturesCanvas) {
            this.fixturesCanvas.style.cursor = 'crosshair';
            this.fixturesCanvas.style.pointerEvents = 'auto';
        }

        if (this.fixturesCanvas && !this.fixturesCanvas.onpointerdown) {
            this.fixturesCanvas.onpointerdown = this.onFixturesPointerDown.bind(this);
            this.fixturesCanvas.onpointermove = this.onFixturesPointerMove.bind(this);
            this.fixturesCanvas.onpointerup = this.onFixturesPointerUp.bind(this);
            this.fixturesCanvas.onpointercancel = this.onFixturesPointerUp.bind(this);
        }
    }

    public disableFixturesBrush(): void {
        this.currentBrushType = null;
        
        if (this.fixturesCanvas) {
            this.fixturesCanvas.style.cursor = 'default';
            this.fixturesCanvas.onpointerdown = null;
            this.fixturesCanvas.onpointermove = null;
            this.fixturesCanvas.onpointerup = null;
            this.fixturesCanvas.onpointercancel = null;
        }
    }


    //Convert client coordinates to canvas coordinates accounting for scaling
    private getCanvasCoordinates(e: PointerEvent, canvas: HTMLCanvasElement): { x: number, y: number } {
        const rect = canvas.getBoundingClientRect();
        
        // Get the actual canvas internal dimensions
        const canvasWidth = canvas.width;
        const canvasHeight = canvas.height;
        
        // Get the displayed CSS dimensions
        const displayWidth = rect.width;
        const displayHeight = rect.height;
        
        // Calculate scaling factors
        const scaleX = canvasWidth / displayWidth;
        const scaleY = canvasHeight / displayHeight;
        
        // Get client coordinates relative to canvas
        const clientX = e.clientX - rect.left;
        const clientY = e.clientY - rect.top;
        
        // Scale to canvas coordinates
        const x = clientX * scaleX;
        const y = clientY * scaleY;
        
        return { x, y };
    }

    private onFixturesPointerDown(e: PointerEvent): void {
        if (!this.currentBrushType || !this.fixturesCanvas || !this.fixturesCtx) return;

        this.isFixturesDrawing = true;
        this.fixturesCanvas.setPointerCapture(e.pointerId);

        const { x, y } = this.getCanvasCoordinates(e, this.fixturesCanvas);
        this.lastFixturesPoint = { x, y };
        this.drawFixturesBrush(x, y);

        this.fixturesApplied = false;
        this.onFixturesChange();
    }

    private onFixturesPointerMove(e: PointerEvent): void {
        if (!this.isFixturesDrawing || !this.fixturesCanvas) return;

        const { x, y } = this.getCanvasCoordinates(e, this.fixturesCanvas);
        this.drawFixturesBrush(x, y);
        this.lastFixturesPoint = { x, y };
    }

    private onFixturesPointerUp(e: PointerEvent): void {
        if (!this.fixturesCanvas) return;
        
        this.isFixturesDrawing = false;
        this.lastFixturesPoint = null;
        this.fixturesCanvas.releasePointerCapture(e.pointerId);
    }

    private drawFixturesBrush(x: number, y: number): void {
        if (!this.fixturesCtx || !this.currentBrushType) return;

        // Setup Opacity/Composite
        if (this.isErasing) {
            this.fixturesCtx.globalCompositeOperation = 'destination-out';
            this.fixturesCtx.fillStyle = '#E69F00';
            this.fixturesCtx.strokeStyle = '#E69F00';
        } else {
            this.fixturesCtx.globalCompositeOperation = 'source-over';
            this.fixturesCtx.fillStyle = '#E69F00';
            this.fixturesCtx.strokeStyle = '#E69F00';
        }

        if (this.currentBrushType === 'round') {
            // Round Brush Logic
            this.fixturesCtx.lineWidth = this.currentBrushSize;
            this.fixturesCtx.lineCap = 'round';
            this.fixturesCtx.lineJoin = 'round';

            if (this.lastFixturesPoint) {
                this.fixturesCtx.beginPath();
                this.fixturesCtx.moveTo(this.lastFixturesPoint.x, this.lastFixturesPoint.y);
                this.fixturesCtx.lineTo(x, y);
                this.fixturesCtx.stroke();
            }
        } else {
            // Square Brush Logic
            const size = this.currentBrushSize;
            const halfSize = size / 2;

            if (this.lastFixturesPoint) {
                const p1 = this.lastFixturesPoint;
                const p2 = { x, y };
                
                const dx = p2.x - p1.x;
                const dy = p2.y - p1.y;
                const distance = Math.sqrt(dx * dx + dy * dy);
                const steps = Math.ceil(distance);

                for (let i = 0; i <= steps; i++) {
                    const t = steps === 0 ? 0 : i / steps;
                    const cx = p1.x + (dx * t);
                    const cy = p1.y + (dy * t);
                    
                    this.fixturesCtx.fillRect(cx - halfSize, cy - halfSize, size, size);
                }
            } else {
                // Just a dot (single click)
                this.fixturesCtx.fillRect(x - halfSize, y - halfSize, size, size);
            }
        }

        // Reset composite
        this.fixturesCtx.globalCompositeOperation = 'source-over';
    }

    public hasFixtures(): boolean {
        if (!this.fixturesCanvas || !this.fixturesCtx) return false;

        const imageData = this.fixturesCtx.getImageData(0, 0, this.fixturesCanvas.width, this.fixturesCanvas.height);
        const data = imageData.data;

        for (let i = 0; i < data.length; i += 4) {
            if (data[i + 3] > 0 && data[i] > 200) {
                return true;
            }
        }
        return false;
    }

    public canApplyFixtures(): boolean {
        return this.hasFixtures() && !this.fixturesApplied;
    }

    public clearFixtures(): void {
        if (!this.fixturesCtx || !this.fixturesCanvas) return;

        this.fixturesCtx.clearRect(0, 0, this.fixturesCanvas.width, this.fixturesCanvas.height);
        this.fixturesApplied = false;
        
        if (!this.isFixturesMode) {
            this.fixturesCanvas.classList.remove('active');
        } else {
            this.fixturesCanvas.classList.add('active');
            this.fixturesCanvas.style.pointerEvents = 'auto';
        }
        
        this.onFixturesChange();
    }

    public async executeFixtures(): Promise<any> {
        if (!this.fixturesCanvas || !this.fixturesCtx) throw new Error("Fixtures canvas not initialized");

        const maskCanvas = document.createElement('canvas');
        maskCanvas.width = this.fixturesCanvas.width;
        maskCanvas.height = this.fixturesCanvas.height;
        const maskCtx = maskCanvas.getContext('2d');
        
        if (!maskCtx) throw new Error("Could not create mask context");

        maskCtx.fillStyle = '#ffffff';
        maskCtx.fillRect(0, 0, maskCanvas.width, maskCanvas.height);

        const fixturesData = this.fixturesCtx.getImageData(0, 0, this.fixturesCanvas.width, this.fixturesCanvas.height);
        const maskData = maskCtx.getImageData(0, 0, maskCanvas.width, maskCanvas.height);

        for (let i = 0; i < fixturesData.data.length; i += 4) {
            const r = fixturesData.data[i];
            const a = fixturesData.data[i + 3];
            if (a > 0 && r > 200) {
                maskData.data[i] = 0;     // R
                maskData.data[i + 1] = 0; // G
                maskData.data[i + 2] = 0; // B
                maskData.data[i + 3] = 255; // A
            }
        }

        maskCtx.putImageData(maskData, 0, 0);

        const blob = await new Promise<Blob | null>(resolve =>
            maskCanvas.toBlob(resolve, 'image/png')
        );
        
        if (!blob) throw new Error("Failed to generate fixtures blob");

        const formData = new FormData();
        formData.append('file', blob, 'fixtures.png');

        const response = await fetch(`${this.apiBaseUrl}/api/fixtures`, {
            method: 'POST',
            body: formData
        });

        if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.detail || "Fixtures execution failed");
        }

        this.fixturesApplied = true;
        return await response.json();
    }

    // --- Interaction Handlers ---
    private onMouseDown(opt: any) {
        if (this.isMarkerMode) {
            const pointer = this.fCanvas.getScenePoint(opt.e);
            this.markerPoints.push({ x: pointer.x, y: pointer.y });

            const dot = new fabric.Circle({
                left: pointer.x,
                top: pointer.y,
                radius: 6,
                fill: '#007AFF',
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
        
        if (this.fixturesCanvas && this.fixturesCtx) {
            const tempCanvas = document.createElement('canvas');
            tempCanvas.width = this.fixturesCanvas.width;
            tempCanvas.height = this.fixturesCanvas.height;
            const tempCtx = tempCanvas.getContext('2d');

            if (tempCtx) {
                tempCtx.drawImage(this.fixturesCanvas, 0, 0);
            }

            // Update internal dimensions
            this.fixturesCanvas.width = width;
            this.fixturesCanvas.height = height;
            
            // CSS 100% will automatically adjust to match viewport

            if (tempCtx) {
                this.fixturesCtx.drawImage(tempCanvas, 0, 0, width, height);
            }
        }
    }

    public render(): void {
        this.fCanvas.renderAll();
        
        if (this.hasFixtures() && this.fixturesCanvas && !this.fixturesCanvas.classList.contains('active')) {
            this.fixturesCanvas.classList.add('active');
        }
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

    public async submitHeatMarkers(markers: Position[]): Promise<any> {
        if (!this.apiBaseUrl) throw new Error("API base URL not set");

        const videoMarkers = markers.map(p => ({
            x: p.x,
            y: p.y
        }));

        const formData = new FormData();
        formData.append('markers', JSON.stringify(videoMarkers));

        const response = await fetch(`${this.apiBaseUrl}/api/heat_markers`, {
            method: 'POST',
            body: formData
        });

        if (!response.ok) {
            const errorText = await response.text();
            throw new Error(errorText || "Failed to submit heat markers");
        }

        return await response.json();
    }

    // --- Execution & Export Logic ---
    public async executePath(speed: number, raster_type: string, density: number, isFillEnabled: boolean): Promise<any> {
        const pixels = this.generatePixelPath();
        const videoPixels = pixels.map(p => ({
            x: (p.x / this.fCanvas.getWidth()) * this.video.videoWidth,
            y: (p.y / this.fCanvas.getHeight()) * this.video.videoHeight
        }));

        const width = this.fCanvas.getWidth();
        const height = this.fCanvas.getHeight();

        const tempCanvas = document.createElement('canvas');
        tempCanvas.width = width;
        tempCanvas.height = height;
        const ctx = tempCanvas.getContext('2d', {
            willReadFrequently: true,
            alpha: false
        });

        if (!ctx) throw new Error("Could not create temp context");

        ctx.imageSmoothingEnabled = false;
        (ctx as any).mozImageSmoothingEnabled = false;
        (ctx as any).webkitImageSmoothingEnabled = false;
        (ctx as any).msImageSmoothingEnabled = false;

        // Background is always white
        ctx.fillStyle = "#ffffff";
        ctx.fillRect(0, 0, width, height);

        // Configure Context based on Fill Mode
        if (isFillEnabled) {
            ctx.fillStyle = "#000000"; // Black fill for raster
            ctx.strokeStyle = "transparent";
        } else {
            ctx.strokeStyle = "#000000"; // Black stroke for vector/outline
            ctx.fillStyle = "transparent";
            ctx.lineWidth = 1;
            ctx.lineCap = 'square';
            ctx.lineJoin = 'miter';
        }

        this.fCanvas.getObjects().forEach(obj => {
            ctx.save();

            if (obj instanceof fabric.Path) {
                ctx.beginPath();
                const pathData = (obj as any).path;
                let lastX = 0, lastY = 0;

                pathData.forEach((cmd: any) => {
                    const type = cmd[0];
                    switch (type) {
                        case 'M':
                            lastX = Math.round(cmd[1]);
                            lastY = Math.round(cmd[2]);
                            ctx.moveTo(lastX + 0.5, lastY + 0.5);
                            break;
                        case 'L':
                            const x = Math.round(cmd[1]);
                            const y = Math.round(cmd[2]);
                            ctx.lineTo(x + 0.5, y + 0.5);
                            lastX = x;
                            lastY = y;
                            break;
                        case 'Q':
                            const qx = Math.round(cmd[3]);
                            const qy = Math.round(cmd[4]);
                            ctx.quadraticCurveTo(Math.round(cmd[1]) + 0.5, Math.round(cmd[2]) + 0.5, qx + 0.5, qy + 0.5);
                            lastX = qx;
                            lastY = qy;
                            break;
                        case 'C':
                            const cx = Math.round(cmd[5]);
                            const cy = Math.round(cmd[6]);
                            ctx.bezierCurveTo(
                                Math.round(cmd[1]) + 0.5, Math.round(cmd[2]) + 0.5,
                                Math.round(cmd[3]) + 0.5, Math.round(cmd[4]) + 0.5,
                                cx + 0.5, cy + 0.5
                            );
                            lastX = cx;
                            lastY = cy;
                            break;
                        case 'Z':
                            ctx.closePath();
                            break;
                    }
                });
                
                if (isFillEnabled) ctx.fill();
                else ctx.stroke();
            } else if (obj instanceof fabric.Rect) {
                const rect = obj as fabric.Rect;
                const left = Math.round(rect.left || 0);
                const top = Math.round(rect.top || 0);
                const width = Math.round((rect.width || 0) * (rect.scaleX || 1));
                const height = Math.round((rect.height || 0) * (rect.scaleY || 1));
                
                if (isFillEnabled) ctx.fillRect(left + 0.5, top + 0.5, width, height);
                else ctx.strokeRect(left + 0.5, top + 0.5, width, height);
            } else if (obj instanceof fabric.Triangle) {
                const triangle = obj as fabric.Triangle;
                const matrix = triangle.calcTransformMatrix();
                const w = triangle.width;
                const h = triangle.height;

                const localPoints = [
                    new fabric.Point(0, -h / 2),
                    new fabric.Point(w / 2, h / 2),
                    new fabric.Point(-w / 2, h / 2)
                ];

                const vertices = localPoints.map(p => p.transform(matrix));

                ctx.beginPath();
                ctx.moveTo(Math.round(vertices[0].x) + 0.5, Math.round(vertices[0].y) + 0.5);
                ctx.lineTo(Math.round(vertices[1].x) + 0.5, Math.round(vertices[1].y) + 0.5);
                ctx.lineTo(Math.round(vertices[2].x) + 0.5, Math.round(vertices[2].y) + 0.5);
                ctx.closePath();
                
                if (isFillEnabled) ctx.fill();
                else ctx.stroke();
            } else if (obj instanceof fabric.Ellipse) {
                const ellipse = obj as fabric.Ellipse;
                const center = ellipse.getCenterPoint();
                const rx = Math.round(ellipse.rx * ellipse.scaleX);
                const ry = Math.round(ellipse.ry * ellipse.scaleY);

                ctx.beginPath();
                ctx.ellipse(
                    center.x, center.y,
                    rx, ry,
                    (ellipse.angle || 0) * Math.PI / 180,
                    0, 2 * Math.PI
                );
                
                if (isFillEnabled) ctx.fill();
                else ctx.stroke();
            } else if (obj instanceof fabric.Line) {
                const line = obj as fabric.Line;
                ctx.beginPath();
                ctx.moveTo(Math.round(line.x1 || 0) + 0.5, Math.round(line.y1 || 0) + 0.5);
                ctx.lineTo(Math.round(line.x2 || 0) + 0.5, Math.round(line.y2 || 0) + 0.5);
                ctx.stroke();
            }

            ctx.restore();
        });

        // Ensure binary contrast
        const imageData = ctx.getImageData(0, 0, width, height);
        const data = imageData.data;

        for (let i = 0; i < data.length; i += 4) {
            if (data[i] < 255 || data[i+1] < 255 || data[i+2] < 255) {
                data[i] = 0;
                data[i+1] = 0;
                data[i+2] = 0;
                data[i+3] = 255;
            }
        }

        ctx.putImageData(imageData, 0, 0);

        const blob = await new Promise<Blob | null>(resolve => tempCanvas.toBlob(resolve, 'image/png'));
        if (!blob) throw new Error("Failed to generate image blob");

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
                const matrix = triangle.calcTransformMatrix();
                const w = triangle.width;
                const h = triangle.height;

                const localPoints = [
                    new fabric.Point(0, -h / 2),
                    new fabric.Point(w / 2, h / 2),
                    new fabric.Point(-w / 2, h / 2)
                ];

                const vertices = localPoints.map(p => p.transform(matrix));

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