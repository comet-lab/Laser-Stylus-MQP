export interface Position {
    x: number;
    y: number;
}

export type ShapeType = 'freehand' | 'square' | 'circle' | 'triangle' | 'line';

export interface Shape {
    type: ShapeType;
    startPos: Position;
    endPos: Position;
    rotation: number; // Rotation in radians
}

export interface BoundingBox {
    minX: number;
    minY: number;
    maxX: number;
    maxY: number;
    centerX: number;
    centerY: number;
    width: number;
    height: number;
}

export type HandleType = 'nw' | 'ne' | 'sw' | 'se' | 'n' | 's' | 'e' | 'w' | 'move' | 'rot' | null;

export interface DragOffsets {
    start: Position;
    end: Position;
}

export type DrawingMode = 'batch' | 'realtime';

export interface RealTimeCallbacks {
    onStart: () => void;
    onMove: (x: number, y: number) => void;
    onEnd: () => void;
}