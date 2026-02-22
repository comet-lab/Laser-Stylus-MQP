//frontend/src/utils/math_utils.ts

// Bresenham's line algorithm generator
// Used to convert the vector shapes from Fabric into a pixel list for the laser
export function* generateLinePixels(x0: number, y0: number, x1: number, y1: number) {
    // Ensure integers immediately
    x0 = Math.floor(x0); y0 = Math.floor(y0); x1 = Math.floor(x1); y1 = Math.floor(y1);
    
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = x0 < x1 ? 1 : -1;
    const sy = y0 < y1 ? 1 : -1;
    let err = dx - dy;

    while (true) {
        yield { x: x0, y: y0 };

        if (x0 === x1 && y0 === y1) break;
        
        const e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}