//frontend/src/ui/types.ts

export interface Position {
    x: number;
    y: number;
}

export type ShapeType = 'square' | 'circle' | 'triangle' | 'line' | 'freehand';

export interface MarkerData {
    x: number;
    y: number;
    temp?: number;
}

export interface TrackerCallbacks {
    onShapeComplete?: () => void;
    onFixturesChange?: () => void;
    onMarkersChange?: () => void;
    onHeatAreaDefined?: () => void;
}