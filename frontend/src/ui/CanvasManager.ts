//frontend/src/ui/CanvasManager.ts

import * as fabric from 'fabric';
import { Position, ShapeType } from './types';
import * as Utils from '../utils/math_utils';

export class CanvasManager {
    private fCanvas: fabric.Canvas;
    private video: HTMLVideoElement;
    private apiBaseUrl: string;
    private onShapeComplete: () => void;
    private onFixturesChange: () => void;
    private onMarkersChange: () => void;
    private onHeatAreaDefined: () => void;

    // --- State: Drawing Mode ---
    private currentShapeType: ShapeType | null = null;
    private isCreatingShape: boolean = false;
    private shapeStartPos: { x: number, y: number } | null = null;
    private activeShape: fabric.FabricObject | null = null;
    private hasPlacedShape: boolean = false;

    // --- State: Heat Marker Mode ---
    private isMarkerMode: boolean = false;
    private markerObjects: fabric.Group[] = [];

    // --- State: Heat Area Mode ---
    private isHeatAreaMode: boolean = false;
    private isDrawingHeatArea: boolean = false;
    private heatStartPos: { x: number, y: number } | null = null;
    private activeHeatRect: fabric.Rect | null = null;

    // --- State: Fixtures Mode ---
    private isFixturesMode: boolean = false;
    private fixturesCanvas: HTMLCanvasElement | null = null;
    private fixturesCtx: CanvasRenderingContext2D | null = null;
    private currentBrushType: 'round' | 'square' | null = null;
    private currentBrushSize: number = 50;
    private isErasing: boolean = false;
    private isFixturesDrawing: boolean = false;
    private lastFixturesPoint: { x: number, y: number } | null = null;
    private fixturesApplied: boolean = false;
    private rawDrawingBackup: ImageData | null = null;

    // --- State: Canvas Sizing ---
    private prevWidth: number;
    private prevHeight: number;

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
        originY: 'top' as const,

    };

    // --- CONFIG: Other Constants ---
    private readonly BORDER_THICKNESS = 4;

    constructor(
        canvas: HTMLCanvasElement,
        video: HTMLVideoElement,
        apiBaseUrl: string = `http://${window.location.hostname}:443`,
        onShapeComplete: () => void = () => { },
        onFixturesChange: () => void = () => { },
        onMarkersChange: () => void = () => { },
        onHeatAreaDefined: () => void
    ) {
        this.video = video;
        this.apiBaseUrl = apiBaseUrl;
        this.onShapeComplete = onShapeComplete;
        this.onFixturesChange = onFixturesChange;
        this.onMarkersChange = onMarkersChange;
        this.onHeatAreaDefined = onHeatAreaDefined;

        const el = canvas as any;
        if (el.__canvas) {
            el.__canvas.dispose();
            el.__canvas = undefined;
        }

        this.fCanvas = new fabric.Canvas(canvas, {
            selection: false,
            preserveObjectStacking: true,
            containerClass: 'fabric-canvas-container',
            enablePointerEvents: true
        });


        this.prevWidth = canvas.width;
        this.prevHeight = canvas.height;

        fabric.FabricObject.ownDefaults = {
            ...fabric.FabricObject.ownDefaults,
            ...(this.SHAPE_DEFAULTS as any)
        };

        const brush = new fabric.PencilBrush(this.fCanvas);
        brush.width = 6;
        brush.color = '#007AFF';

        this.fCanvas.freeDrawingBrush = brush;

        this.fCanvas.on('mouse:down', (opt) => {
            if (this.isHeatAreaMode) this.onHeatMouseDown(opt);
            else this.onMouseDown(opt);
        });
        this.fCanvas.on('mouse:move', (opt) => {
            if (this.isHeatAreaMode) this.onHeatMouseMove(opt);
            else this.onMouseMove(opt);
        });
        this.fCanvas.on('mouse:up', (opt) => {
            if (this.isHeatAreaMode) this.onHeatMouseUp();
            else this.onMouseUp();
        });
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
            this.ensureMarkersOnTop();
            this.onShapeComplete();
        });

        this.createFixturesCanvas(canvas);
    }

    private createFixturesCanvas(mainCanvas: HTMLCanvasElement): void {
        this.fixturesCanvas = document.createElement('canvas');
        this.fixturesCanvas.id = 'fixturesCanvas';
        this.fixturesCanvas.width = mainCanvas.width;
        this.fixturesCanvas.height = mainCanvas.height;
        this.fixturesCanvas.style.opacity = '0.6';
        this.fixturesCtx = this.fixturesCanvas.getContext('2d', { willReadFrequently: true });
        mainCanvas.parentElement?.appendChild(this.fixturesCanvas);
    }

    // =========================================================================
    // Core Lifecycle Methods
    // =========================================================================

    public dispose(): void {
        if (this.fCanvas) {
            this.fCanvas.dispose();
        }
        if (this.fixturesCanvas) {
            this.fixturesCanvas.remove();
        }
    }

    // =========================================================================
    // Fixtures Mode Logic
    // =========================================================================

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

    private getCanvasCoordinates(e: PointerEvent, canvas: HTMLCanvasElement): { x: number, y: number } {
        const rect = canvas.getBoundingClientRect();
        const canvasWidth = canvas.width;
        const canvasHeight = canvas.height;
        const displayWidth = rect.width;
        const displayHeight = rect.height;
        const scaleX = canvasWidth / displayWidth;
        const scaleY = canvasHeight / displayHeight;
        const clientX = e.clientX - rect.left;
        const clientY = e.clientY - rect.top;
        const x = clientX * scaleX;
        const y = clientY * scaleY;
        return { x, y };
    }

    private onFixturesPointerDown(e: PointerEvent): void {
        if (!this.currentBrushType || !this.fixturesCanvas || !this.fixturesCtx) return;
        if (this.fixturesApplied) {
            this.restoreDrawingState();
        }
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

    private restoreDrawingState(): void {
        if (!this.fixturesCtx || !this.rawDrawingBackup) return;
        
        // Put the raw cyan pixels back
        this.fixturesCtx.putImageData(this.rawDrawingBackup, 0, 0);
        
        // Reset flags
        this.fixturesApplied = false;
        this.rawDrawingBackup = null;
    }

    private drawFixturesBrush(x: number, y: number): void {
        if (!this.fixturesCtx || !this.currentBrushType) return;
        if (this.isErasing) {
            this.fixturesCtx.globalCompositeOperation = 'destination-out';
            this.fixturesCtx.fillStyle = '#007AFF';
            this.fixturesCtx.strokeStyle = '#007AFF';
        } else {
            this.fixturesCtx.globalCompositeOperation = 'source-over';
            this.fixturesCtx.fillStyle = '#007AFF';
            this.fixturesCtx.strokeStyle = '#007AFF';
        }

        if (this.currentBrushType === 'round') {
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
                this.fixturesCtx.fillRect(x - halfSize, y - halfSize, size, size);
            }
        }
        this.fixturesCtx.globalCompositeOperation = 'source-over';
    }

    public hasFixtures(): boolean {
        if (!this.fixturesCanvas || !this.fixturesCtx) return false;
        const imageData = this.fixturesCtx.getImageData(0, 0, this.fixturesCanvas.width, this.fixturesCanvas.height);
        const data = imageData.data;
        for (let i = 0; i < data.length; i += 4) {
            if (data[i + 3] > 0) {
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
        this.rawDrawingBackup = null; // Clear backup
        
        this.fixturesCtx.globalCompositeOperation = 'source-over';
        
        if (!this.isFixturesMode) {
            this.fixturesCanvas.classList.remove('active');
        } else {
            this.fixturesCanvas.classList.add('active');
            this.fixturesCanvas.style.pointerEvents = 'auto';
        }
        this.onFixturesChange();
    }

    public async clearFixturesOnServer(): Promise<void> {
        const blob = await this.generateBlankFixtureBlob();
        await this.uploadFixtures(blob);
    }

    public async executeFixtures(): Promise<any> {
        if (!this.fixturesCanvas || !this.fixturesCtx) throw new Error("Fixtures canvas not initialized");
        
        const width = this.fixturesCanvas.width;
        const height = this.fixturesCanvas.height;

        // 1. Capture the current RAW state (The Cyan Blobs)
        const currentDrawing = this.fixturesCtx.getImageData(0, 0, width, height);
        this.rawDrawingBackup = currentDrawing; // Save backup for later editing

        const data = currentDrawing.data;

        // 2. Prepare Server Mask (Black=Forbidden, White=Safe)
        const maskCanvas = document.createElement('canvas');
        maskCanvas.width = width;
        maskCanvas.height = height;
        const maskCtx = maskCanvas.getContext('2d');
        if (!maskCtx) throw new Error("Could not create mask context");

        const maskImageData = maskCtx.createImageData(width, height);
        const maskPixels = maskImageData.data;

        // 3. Prepare Visual Feedback (Dark background, Transparent safe zone, Red Border)
        const visualImageData = this.fixturesCtx.createImageData(width, height);
        const visualPixels = visualImageData.data;

        // Helper: Is this pixel part of the user's drawing? (Alpha > 0)
        // We use the backup data because we are writing to new arrays
        const isSafe = (idx: number) => data[idx + 3] > 0;

        // Helper: Check for border condition with thickness
        // Returns true if ANY pixel within 'radius' is Forbidden (not drawn)
        const isNearEdge = (cx: number, cy: number, radius: number) => {
            // Optimization: check cardinal directions first (fast fail)
            const idxTop = ((cy - radius) * width + cx) * 4;
            const idxBottom = ((cy + radius) * width + cx) * 4;
            const idxLeft = (cy * width + (cx - radius)) * 4;
            const idxRight = (cy * width + (cx + radius)) * 4;

            // Bounds checks + Safety check
            if (cy - radius >= 0 && !isSafe(idxTop)) return true;
            if (cy + radius < height && !isSafe(idxBottom)) return true;
            if (cx - radius >= 0 && !isSafe(idxLeft)) return true;
            if (cx + radius < width && !isSafe(idxRight)) return true;

            return false;
        };

        for (let y = 0; y < height; y++) {
            for (let x = 0; x < width; x++) {
                const i = (y * width + x) * 4;

                if (isSafe(i)) {
                    // --- SAFE ZONE (User Drawn) ---

                    // Server Mask: White (Allowed)
                    maskPixels[i] = 255; maskPixels[i + 1] = 255; maskPixels[i + 2] = 255; maskPixels[i + 3] = 255;

                    // Visual Feedback:
                    // Check if we are near the edge to draw the Red Border
                    if (isNearEdge(x, y, this.BORDER_THICKNESS)) {
                        // Border: Solid Red
                        visualPixels[i] = 255;     // R
                        visualPixels[i + 1] = 0;   // G
                        visualPixels[i + 2] = 0;   // B
                        visualPixels[i + 3] = 255; // Alpha
                    } else {
                        // Inner Safe Zone: Transparent (Show Video)
                        visualPixels[i + 3] = 0; 
                    }
                } else {
                    // --- FORBIDDEN ZONE (Empty) ---

                    // Server Mask: Black (Forbidden)
                    maskPixels[i + 3] = 255; // Solid Alpha for PNG

                    //Dark overlay
                    visualPixels[i] = 0;       // R
                    visualPixels[i + 1] = 0;   // G
                    visualPixels[i + 2] = 0;   // B
                    visualPixels[i + 3] = 180; // Darken the camera view
                }
            }
        }

        //Update the Canvas with the Visual Feedback
        this.fixturesCtx.putImageData(visualImageData, 0, 0);

        //Send mask to server
        maskCtx.putImageData(maskImageData, 0, 0);
        
        const blob = await new Promise<Blob | null>(resolve => maskCanvas.toBlob(resolve, 'image/png'));
        if (!blob) throw new Error("Failed to generate fixtures blob");
        
        const result = await this.uploadFixtures(blob);
        
        //Mark as applied so we know to restore backup on next draw
        this.fixturesApplied = true; 
        
        return result;
    }

    // =========================================================================
    // Heat Area Logic
    // =========================================================================

    public enableHeatAreaMode(): void {
        this.disableDrawing(); // Ensure standard flags are off

        this.isHeatAreaMode = true;
        this.fCanvas.defaultCursor = 'crosshair';
        this.fCanvas.selection = false; // Disable group selection
        this.fCanvas.discardActiveObject();
        this.fCanvas.requestRenderAll();
    }

    public disableHeatAreaMode(): void {
        this.isHeatAreaMode = false;
        this.isDrawingHeatArea = false;
        this.fCanvas.defaultCursor = 'default';
        if (this.activeHeatRect) {
            this.fCanvas.remove(this.activeHeatRect);
            this.activeHeatRect = null;
        }
    }

    public async resetHeatArea(): Promise<void> {
        // Send a black mask to clear the setting on backend
        await this.sendHeatMaskToServer(null); // null triggers the "blank" generation
    }

    private onHeatMouseDown(opt: any) {
        if (!this.fCanvas.getElement()) return;

        this.isDrawingHeatArea = true;
        const pointer = this.fCanvas.getScenePoint(opt.e);
        this.heatStartPos = { x: pointer.x, y: pointer.y };

        // Create the specific "Heat Style" rectangle
        this.activeHeatRect = new fabric.Rect({
            left: pointer.x,
            top: pointer.y,
            width: 0,
            height: 0,
            fill: 'rgba(255, 69, 0, 0.3)', // Translucent Orange
            stroke: '#ff4500',            // Solid Orange (for base)
            strokeWidth: 2,
            strokeDashArray: [6, 6],      // Dotted/Dashed
            selectable: false,            // NOT MOVABLE
            evented: false,               // NO EVENTS
            hasControls: false,           // NO HANDLES
            hasBorders: false,
            originX: 'left',
            originY: 'top'
        });

        this.fCanvas.add(this.activeHeatRect);
    }

    private onHeatMouseMove(opt: any) {
        if (!this.isDrawingHeatArea || !this.activeHeatRect || !this.heatStartPos) return;

        const pointer = this.fCanvas.getScenePoint(opt.e);

        // Calculate geometry allowing for dragging in any direction (negative width/height handling)
        const origX = this.heatStartPos.x;
        const origY = this.heatStartPos.y;

        const left = Math.min(origX, pointer.x);
        const top = Math.min(origY, pointer.y);
        const width = Math.abs(origX - pointer.x);
        const height = Math.abs(origY - pointer.y);

        this.activeHeatRect.set({ left, top, width, height });
        this.fCanvas.requestRenderAll();
    }

    private async onHeatMouseUp() {
        if (!this.isDrawingHeatArea || !this.activeHeatRect) return;

        this.isDrawingHeatArea = false;

        // Prevent tiny accidental clicks from registering as masks
        if (this.activeHeatRect.width * this.activeHeatRect.scaleX < 5 ||
            this.activeHeatRect.height * this.activeHeatRect.scaleY < 5) {
            this.fCanvas.remove(this.activeHeatRect);
            this.activeHeatRect = null;
            return;
        }

        // 1. Generate Mask and Send to Backend
        await this.sendHeatMaskToServer(this.activeHeatRect);

        // 2. Remove the visual rectangle (as requested: "finish drawing... it will disappear")
        this.fCanvas.remove(this.activeHeatRect);
        this.activeHeatRect = null;
        this.fCanvas.requestRenderAll();

        // 3. Notify UI to enable the Reset Button
        this.onHeatAreaDefined();
    }

    private async sendHeatMaskToServer(rect: fabric.Rect | null): Promise<void> {
        const width = this.fCanvas.getWidth();
        const height = this.fCanvas.getHeight();

        const tempCanvas = document.createElement('canvas');
        tempCanvas.width = width;
        tempCanvas.height = height;
        const ctx = tempCanvas.getContext('2d');
        if (!ctx) return;

        // Background: Black (No Heat)
        ctx.fillStyle = '#ffffff';
        ctx.fillRect(0, 0, width, height);

        // Foreground: White (Heat Active) - Only if a rect is provided
        if (rect) {
            ctx.fillStyle = '#000000';
            // We must account for scale if Fabric applied any, though we created it raw
            const w = rect.width * (rect.scaleX || 1);
            const h = rect.height * (rect.scaleY || 1);
            ctx.fillRect(rect.left, rect.top, w, h);
        }

        const blob = await new Promise<Blob | null>(resolve => tempCanvas.toBlob(resolve, 'image/png'));
        if (!blob) return;

        await this.uploadHeatMask(blob);
    }

    // =========================================================================
    // Drawing Interaction Handlers
    // =========================================================================

    private async onMouseDown(opt: any) {
        if (this.isMarkerMode) {
            const pointer = this.fCanvas.getScenePoint(opt.e);
            if (opt.target && opt.target.type === 'group' && opt.target._isMarker) {
                const group = opt.target as fabric.Group;
                const groupCenter = group.getCenterPoint();
                const clickRelX = pointer.x - groupCenter.x;
                const clickRelY = pointer.y - groupCenter.y;

                const closeBoxObj = group.getObjects()[3];

                const distToX = Math.sqrt(
                    Math.pow(clickRelX - closeBoxObj.left, 2) +
                    Math.pow(clickRelY - closeBoxObj.top, 2)
                );

                if (distToX < 15) {
                    this.removeMarker(group);
                    await this.submitHeatMarkers(this.getHeatMarkersInVideoSpace());
                    return;
                }
                return;
            }
            this.addHeatMarker(pointer.x, pointer.y);
            await this.submitHeatMarkers(this.getHeatMarkersInVideoSpace());
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

    // =========================================================================
    // Public Drawing API
    // =========================================================================

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
        const objects = this.fCanvas.getObjects();
        objects.forEach(obj => {
            if (!(obj as any)._isMarker) {
                this.fCanvas.remove(obj);
            }
        });

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
            if (!(o as any)._isMarker) {
                o.selectable = false;
                o.evented = false;
            }
        });
        this.fCanvas.requestRenderAll();
        this.fCanvas.defaultCursor = 'default';
    }

    public enableDrawing(): void {
        this.fCanvas.forEachObject(o => {
            if (!(o as any)._isMarker) {
                o.selectable = true;
                o.evented = true;
            }
        });
    }

    public isDrawingEnabled(): boolean {
        return this.fCanvas.isDrawingMode || this.fCanvas.getObjects().length > 0;
    }

    public hasShape(): boolean {
        return this.hasPlacedShape || this.fCanvas.getObjects().some(o => !(o as any)._isMarker);
    }

    public updateCanvasSize(width: number, height: number): void {
        // 1. Calculate the scaling factor based on the change
        // Prevent division by zero if initialized incorrectly
        const scaleX = this.prevWidth ? width / this.prevWidth : 1;
        const scaleY = this.prevHeight ? height / this.prevHeight : 1;

        // 2. Resize the actual fabric canvas container
        this.fCanvas.setDimensions({ width, height });

        // 3. Iterate through all objects to reposition them relative to the new size
        this.fCanvas.getObjects().forEach(obj => {
            // Scale the position
            const newLeft = obj.left * scaleX;
            const newTop = obj.top * scaleY;

            obj.set({
                left: newLeft,
                top: newTop
            });

            // Also scale specific custom properties for Markers
            if ((obj as any)._isMarker) {
                (obj as any)._tipX = (obj as any)._tipX * scaleX;
                (obj as any)._tipY = (obj as any)._tipY * scaleY;
            }
            obj.setCoords(); // Critical, recalculate hitboxes
        });

        // 4. Handle Fixtures (Raster) Canvas Scaling
        // We scale the bitmap image to fit the new size
        if (this.fixturesCanvas && this.fixturesCtx) {
            // Create a temporary copy of the current fixtures
            const tempCanvas = document.createElement('canvas');
            tempCanvas.width = this.prevWidth;
            tempCanvas.height = this.prevHeight;
            const tempCtx = tempCanvas.getContext('2d');

            if (tempCtx) {
                tempCtx.drawImage(this.fixturesCanvas, 0, 0);
            }
            this.fixturesCanvas.width = width;
            this.fixturesCanvas.height = height;

            if (tempCtx) {
                this.fixturesCtx.drawImage(tempCanvas, 0, 0, width, height);
            }
        }
        this.prevWidth = width;
        this.prevHeight = height;

        this.fCanvas.requestRenderAll();
    }

    public render(): void {
        this.fCanvas.renderAll();
        if (this.hasFixtures() && this.fixturesCanvas && !this.fixturesCanvas.classList.contains('active')) {
            this.fixturesCanvas.classList.add('active');
        }
    }

    // =========================================================================
    // Heat Marker Logic
    // =========================================================================

    public enableMarkerMode(): void {
        // We only clear drawing SHAPES, not markers
        this.disableDrawing();
        this.isMarkerMode = true;
        this.fCanvas.defaultCursor = 'crosshair';

        this.markerObjects.forEach(m => {
            m.selectable = true;
            m.evented = true;
            m.hoverCursor = 'pointer';
        });
    }

    public disableMarkerMode(): void {
        this.isMarkerMode = false;
        this.fCanvas.defaultCursor = 'default';
        this.markerObjects.forEach(m => {
            m.selectable = false;
            m.evented = false;
        });
    }

    private ensureMarkersOnTop(): void {
        this.markerObjects.forEach(markerGroup => {
            this.fCanvas.bringObjectToFront(markerGroup);
        });
        this.fCanvas.requestRenderAll();
    }

    public showMarkers(): void {
        this.markerObjects.forEach(m => {
            m.visible = true;
        });
        this.fCanvas.requestRenderAll();
    }

    public hideMarkers(): void {
        this.markerObjects.forEach(m => {
            m.visible = false;
        });
        this.fCanvas.requestRenderAll();
    }

    public hasMarkers(): boolean {
        return this.markerObjects.length > 0;
    }

    private addHeatMarker(x: number, y: number): void {
        const arrow = new fabric.Triangle({
            width: 20,
            height: 20,
            fill: '#151b21B3',
            stroke: '#e0f2e0',
            strokeWidth: 2,
            left: 0,
            top: 18,
            originX: 'center',
            originY: 'top',
            flipY: true
        });
        const box = new fabric.Rect({
            fill: '#151b21B3',
            width: 70,
            height: 25,
            stroke: '#e0f2e0',
            strokeWidth: 2,
            rx: 4,
            ry: 4,
            left: 24,
            top: 20,
            originX: 'center',
            originY: 'bottom'
        });
        const text = new fabric.Text('...°C', {
            fontSize: 14,
            fill: '#ffffff',
            fontFamily: 'IBM Plex Sans',
            left: 14,
            top: 16,
            originX: 'center',
            originY: 'bottom'
        });
        const closeBox = new fabric.Rect({
            width: 20,
            height: 20,
            rx: 4,
            ry: 4,
            fill: '#151b21B3',
            strokeWidth: 2,
            left: 46,
            top: 6.5,
            originX: 'center',
            originY: 'center'
        });
        const closeText = new fabric.Text('×', {
            fontSize: 14,
            fill: '#ffffff',
            left: 46,
            top: 6.5,
            originX: 'center',
            originY: 'center',
            fontFamily: 'Arial'
        });

        const group = new fabric.Group([arrow, box, text, closeBox, closeText], {
            left: x,
            top: y,
            originX: 'center',
            originY: 'center',
            selectable: true,
            hasControls: false,
            hasBorders: false,
            lockMovementX: true,
            lockMovementY: true,
            hoverCursor: 'default'
        });
        const arrowObj = group.getObjects()[0];
        const tipOffsetY = arrowObj.top + arrowObj.height;
        const tipOffsetX = arrowObj.left;

        group.set({
            left: x - tipOffsetX,
            top: y - tipOffsetY
        });

        (group as any)._isMarker = true;
        (group as any)._textObj = text;
        (group as any)._tipX = x;
        (group as any)._tipY = y;

        this.fCanvas.add(group);
        this.markerObjects.push(group);
        this.fCanvas.requestRenderAll();
        this.ensureMarkersOnTop();

        // Notify UI
        this.onMarkersChange();
    }

    private removeMarker(markerGroup: fabric.Group): void {
        this.fCanvas.remove(markerGroup);
        this.markerObjects = this.markerObjects.filter(m => m !== markerGroup);
        this.fCanvas.requestRenderAll();
        // Notify UI
        this.onMarkersChange();
    }

    public clearMarkers(): void {
        this.markerObjects.forEach(m => this.fCanvas.remove(m));
        this.markerObjects = [];
        this.fCanvas.requestRenderAll();
        // Notify UI
        this.onMarkersChange();
    }

    public getHeatMarkersInVideoSpace(): Position[] {
        return this.markerObjects.map(m => {
            const tipX = (m as any)._tipX;
            const tipY = (m as any)._tipY;
            return {
                x: (tipX / this.fCanvas.getWidth()) * this.video.videoWidth,
                y: (tipY / this.fCanvas.getHeight()) * this.video.videoHeight
            };
        });
    }

    public updateMarkerTemperatures(markersData: { x: number, y: number, temp?: number }[]): void {
        if (markersData.length !== this.markerObjects.length) {
            return;
        }
        this.markerObjects.forEach((markerGroup, index) => {
            const data = markersData[index];
            if (data && data.temp !== undefined) {
                const textObj = (markerGroup as any)._textObj as fabric.Text;
                textObj.set('text', `${data.temp.toFixed(1)}°`);
            }
        });
        this.fCanvas.requestRenderAll();
    }

    public async submitHeatMarkers(markers: Position[]): Promise<any> {
        return this.postData('/api/heat_markers', { markers: JSON.stringify(markers) }, true);
    }

    // =========================================================================
    // Execution & Export Logic
    // =========================================================================

    public async executePath(speed: number, raster_type: string, density: number, isFillEnabled: boolean): Promise<any> {
        // 1. Always generate the vector path (pixels)
        const pixels = this.generatePixelPath();
        const videoPixels = pixels.map(p => ({
            x: (p.x / this.fCanvas.getWidth()) * this.video.videoWidth,
            y: (p.y / this.fCanvas.getHeight()) * this.video.videoHeight
        }));

        const formData = new FormData();
        formData.append('speed', (Number(speed) / 1000).toString());
        if (isFillEnabled) {
            formData.append('raster_type', raster_type);
        }
        formData.append('density', density.toString());
        formData.append('pixels', JSON.stringify(videoPixels));
        formData.append('is_fill', isFillEnabled.toString());

        // 2. Only generate raster image if fill is enabled
        if (isFillEnabled) {
            const width = this.fCanvas.getWidth();
            const height = this.fCanvas.getHeight();
            const tempCanvas = document.createElement('canvas');
            tempCanvas.width = width;
            tempCanvas.height = height;
            const ctx = tempCanvas.getContext('2d', { willReadFrequently: true, alpha: false });
            if (!ctx) throw new Error("Could not create temp context");

            ctx.imageSmoothingEnabled = false;
            (ctx as any).mozImageSmoothingEnabled = false;
            (ctx as any).webkitImageSmoothingEnabled = false;
            (ctx as any).msImageSmoothingEnabled = false;

            // White background
            ctx.fillStyle = "#ffffff";
            ctx.fillRect(0, 0, width, height);

            // Black fill for shapes
            ctx.fillStyle = "#000000";
            ctx.strokeStyle = "transparent";

            this.fCanvas.getObjects().forEach(obj => {
                if ((obj as any)._isMarker) return; // Skip markers
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
                    ctx.fill();
                } else if (obj instanceof fabric.Rect) {
                    const rect = obj as fabric.Rect;
                    const left = Math.round(rect.left || 0);
                    const top = Math.round(rect.top || 0);
                    const width = Math.round((rect.width || 0) * (rect.scaleX || 1));
                    const height = Math.round((rect.height || 0) * (rect.scaleY || 1));
                    ctx.fillRect(left + 0.5, top + 0.5, width, height);
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
                    ctx.fill();
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
                    ctx.fill();
                } else if (obj instanceof fabric.Line) {
                    // Lines cannot be filled in raster mode usually, but if needed:
                    const line = obj as fabric.Line;
                    ctx.beginPath();
                    ctx.moveTo(Math.round(line.x1 || 0) + 0.5, Math.round(line.y1 || 0) + 0.5);
                    ctx.lineTo(Math.round(line.x2 || 0) + 0.5, Math.round(line.y2 || 0) + 0.5);
                    ctx.stroke(); // Fallback for line visibility
                }
                ctx.restore();
            });

            const imageData = ctx.getImageData(0, 0, width, height);
            const data = imageData.data;
            for (let i = 0; i < data.length; i += 4) {
                if (data[i] < 255 || data[i + 1] < 255 || data[i + 2] < 255) {
                    data[i] = 0;
                    data[i + 1] = 0;
                    data[i + 2] = 0;
                    data[i + 3] = 255;
                }
            }
            ctx.putImageData(imageData, 0, 0);

            const blob = await new Promise<Blob | null>(resolve => tempCanvas.toBlob(resolve, 'image/png'));
            if (!blob) throw new Error("Failed to generate image blob");
            formData.append('file', blob, 'path.png');
        }

        return this.postFormData('/api/execute', formData);
    }

    public async updateViewSettings(isTransformedViewOn: boolean, isThermalViewOn: boolean = false): Promise<any> {
        return this.postJson('/api/view_settings', { isTransformedViewOn, isThermalViewOn });
    }

    private generatePixelPath(): Position[] {
        const objects = this.fCanvas.getObjects();
        const pixels: Position[] = [];

        for (const obj of objects) {
            if ((obj as any)._isMarker) continue;

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
                for (let i = 0; i < vertices.length; i++) {
                    const start = vertices[i];
                    const end = vertices[(i + 1) % vertices.length];
                    const gen = Utils.generateLinePixels(start.x, start.y, end.x, end.y);
                    for (const p of gen) pixels.push(p);
                }
            } else if (obj.type === 'ellipse' || obj.type === 'circle') {
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
            } else {
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

    // =========================================================================
    // Private API Helpers
    // =========================================================================

    private async generateBlankFixtureBlob(): Promise<Blob> {
        const width = this.fixturesCanvas?.width ?? 640;
        const height = this.fixturesCanvas?.height ?? 480;
        const blankCanvas = document.createElement('canvas');
        blankCanvas.width = width;
        blankCanvas.height = height;
        const ctx = blankCanvas.getContext('2d');
        if (!ctx) throw new Error("Could not create blank canvas context");
        ctx.fillStyle = '#ffffff';
        ctx.fillRect(0, 0, width, height);
        const blob = await new Promise<Blob | null>(resolve => blankCanvas.toBlob(resolve, 'image/png'));
        if (!blob) throw new Error("Failed to generate blank fixtures blob");
        return blob;
    }

    private async uploadFixtures(blob: Blob): Promise<any> {
        const formData = new FormData();
        formData.append('file', blob, 'fixtures.png');
        return this.postFormData('/api/fixtures', formData);
    }

    private async uploadHeatMask(blob: Blob): Promise<void> {
        const formData = new FormData();
        formData.append('file', blob, 'heat_mask.png');
        try {
            await this.postFormData('/api/heat_area', formData);
        } catch (e) {
            console.error("Error uploading heat mask:", e);
        }
    }

    private async postFormData(endpoint: string, formData: FormData): Promise<any> {
        const response = await fetch(`${this.apiBaseUrl}${endpoint}`, {
            method: 'POST',
            body: formData
        });
        if (!response.ok) {
            const errorData = await response.json().catch(() => ({ detail: response.statusText }));
            throw new Error(errorData.detail || `Request to ${endpoint} failed`);
        }
        return response.json();
    }

    private async postJson(endpoint: string, data: any): Promise<any> {
        const response = await fetch(`${this.apiBaseUrl}${endpoint}`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(data)
        });
        return response.json();
    }

    private async postData(endpoint: string, data: Record<string, string>, isFormData: boolean = false): Promise<any> {
        const formData = new FormData();
        for (const key in data) {
            formData.append(key, data[key]);
        }
        const response = await fetch(`${this.apiBaseUrl}${endpoint}`, {
            method: 'POST',
            body: formData
        });
        if (!response.ok) {
            const errorText = await response.text();
            throw new Error(errorText || `Request to ${endpoint} failed`);
        }
        return response.json();
    }
}