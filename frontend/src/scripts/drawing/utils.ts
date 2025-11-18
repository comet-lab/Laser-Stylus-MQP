// drawingUtils.ts
import { Position, Shape, BoundingBox } from './types';

// Rotate a point around a center
export const rotatePoint = (point: Position, center: Position, angle: number): Position => {
    if (angle === 0) return point;
    const cos = Math.cos(angle);
    const sin = Math.sin(angle);
    const dx = point.x - center.x;
    const dy = point.y - center.y;
    return {
        x: center.x + (dx * cos - dy * sin),
        y: center.y + (dx * sin + dy * cos)
    };
};

export const getLocalBoundingBox = (shape: Shape): BoundingBox => {
    const { startPos, endPos } = shape;
    const minX = Math.min(startPos.x, endPos.x);
    const maxX = Math.max(startPos.x, endPos.x);
    const minY = Math.min(startPos.y, endPos.y);
    const maxY = Math.max(startPos.y, endPos.y);
    const width = Math.abs(endPos.x - startPos.x);
    const height = Math.abs(endPos.y - startPos.y);

    return {
        minX, minY, maxX, maxY,
        centerX: minX + width / 2,
        centerY: minY + height / 2,
        width, height
    };
};

export const getShapeVertices = (shape: Shape): Position[] => {
    const { type, startPos, endPos, rotation } = shape;
    const bbox = getLocalBoundingBox(shape);
    const center = { x: bbox.centerX, y: bbox.centerY };
    const vertices: Position[] = [];

    if (type === 'square') {
        vertices.push(rotatePoint({ x: bbox.minX, y: bbox.minY }, center, rotation)); // TL
        vertices.push(rotatePoint({ x: bbox.maxX, y: bbox.minY }, center, rotation)); // TR
        vertices.push(rotatePoint({ x: bbox.maxX, y: bbox.maxY }, center, rotation)); // BR
        vertices.push(rotatePoint({ x: bbox.minX, y: bbox.maxY }, center, rotation)); // BL
    } else if (type === 'triangle') {
        const midX = (bbox.minX + bbox.maxX) / 2;
        vertices.push(rotatePoint({ x: midX, y: bbox.minY }, center, rotation));
        vertices.push(rotatePoint({ x: bbox.maxX, y: bbox.maxY }, center, rotation));
        vertices.push(rotatePoint({ x: bbox.minX, y: bbox.maxY }, center, rotation));
    } else if (type === 'line') {
        vertices.push(startPos);
        vertices.push(endPos);
    }

    return vertices;
};

export const getHandlePositions = (shape: Shape): Record<string, Position> | null => {
    const bbox = getLocalBoundingBox(shape);
    const center = { x: bbox.centerX, y: bbox.centerY };
    const rot = shape.rotation;

    const rawHandles: Record<string, Position> = {
        nw: { x: bbox.minX, y: bbox.minY },
        ne: { x: bbox.maxX, y: bbox.minY },
        sw: { x: bbox.minX, y: bbox.maxY },
        se: { x: bbox.maxX, y: bbox.maxY },
        n:  { x: bbox.centerX, y: bbox.minY },
        s:  { x: bbox.centerX, y: bbox.maxY },
        e:  { x: bbox.maxX, y: bbox.centerY },
        w:  { x: bbox.minX, y: bbox.centerY },
    };

    const rotatedHandles: Record<string, Position> = {} as any;
    for (const [key, pos] of Object.entries(rawHandles)) {
        rotatedHandles[key] = rotatePoint(pos, center, rot);
    }

    // Add rotation handle (sticking up 25px from Top/North)
    const topPoint = { x: bbox.centerX, y: bbox.minY - 25 };
    rotatedHandles['rot'] = rotatePoint(topPoint, center, rot);

    return rotatedHandles;
};

// Bresenham's line algorithm generator
export function* generateLinePixels(x0: number, y0: number, x1: number, y1: number, brushSize: number = 5) {
    x0 = Math.floor(x0); y0 = Math.floor(y0); x1 = Math.floor(x1); y1 = Math.floor(y1);
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = x0 < x1 ? 1 : -1;
    const sy = y0 < y1 ? 1 : -1;
    let err = dx - dy;

    while (true) {
        for (let bx = -brushSize; bx <= brushSize; bx++) {
            for (let by = -brushSize; by <= brushSize; by++) {
                yield { x: x0 + bx, y: y0 + by };
            }
        }
        if (x0 === x1 && y0 === y1) break;
        const e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}