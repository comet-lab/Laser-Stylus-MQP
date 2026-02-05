// tests/math_utils.test.ts
import { describe, it, expect } from 'vitest';
import { generateLinePixels } from '../src/utils/math_utils';

/** Helper: drain a generator into an array */
function collect(gen: Generator<{ x: number; y: number }>): { x: number; y: number }[] {
    return [...gen];
}

describe('generateLinePixels – degenerate cases', () => {
    it('single point when start === end', () => {
        const pixels = collect(generateLinePixels(5, 5, 5, 5));
        expect(pixels).toHaveLength(1);
        expect(pixels[0]).toEqual({ x: 5, y: 5 });
    });

    it('single point at origin', () => {
        expect(collect(generateLinePixels(0, 0, 0, 0))).toEqual([{ x: 0, y: 0 }]);
    });
});

describe('generateLinePixels – axis-aligned lines', () => {
    it('horizontal line left → right', () => {
        const pixels = collect(generateLinePixels(0, 3, 4, 3));
        // Must include both endpoints
        expect(pixels[0]).toEqual({ x: 0, y: 3 });
        expect(pixels[pixels.length - 1]).toEqual({ x: 4, y: 3 });
        // All y values are 3
        pixels.forEach(p => expect(p.y).toBe(3));
        // x is monotonically increasing
        for (let i = 1; i < pixels.length; i++) {
            expect(pixels[i].x).toBeGreaterThanOrEqual(pixels[i - 1].x);
        }
    });

    it('horizontal line right → left (negative direction)', () => {
        const pixels = collect(generateLinePixels(4, 3, 0, 3));
        expect(pixels[0]).toEqual({ x: 4, y: 3 });
        expect(pixels[pixels.length - 1]).toEqual({ x: 0, y: 3 });
        for (let i = 1; i < pixels.length; i++) {
            expect(pixels[i].x).toBeLessThanOrEqual(pixels[i - 1].x);
        }
    });

    it('vertical line top → bottom', () => {
        const pixels = collect(generateLinePixels(2, 0, 2, 5));
        expect(pixels[0]).toEqual({ x: 2, y: 0 });
        expect(pixels[pixels.length - 1]).toEqual({ x: 2, y: 5 });
        pixels.forEach(p => expect(p.x).toBe(2));
    });

    it('vertical line bottom → top', () => {
        const pixels = collect(generateLinePixels(2, 5, 2, 0));
        expect(pixels[0]).toEqual({ x: 2, y: 5 });
        expect(pixels[pixels.length - 1]).toEqual({ x: 2, y: 0 });
    });
});

describe('generateLinePixels – diagonal / general lines', () => {
    it('45° diagonal has equal dx and dy pixel counts', () => {
        const pixels = collect(generateLinePixels(0, 0, 10, 10));
        expect(pixels[0]).toEqual({ x: 0, y: 0 });
        expect(pixels[pixels.length - 1]).toEqual({ x: 10, y: 10 });
        // Bresenham on a perfect diagonal: length == dx + 1
        expect(pixels).toHaveLength(11);
    });

    it('steep line (dy > dx) produces correct endpoint', () => {
        const pixels = collect(generateLinePixels(1, 0, 2, 8));
        expect(pixels[pixels.length - 1]).toEqual({ x: 2, y: 8 });
        // Every consecutive pair differs by at most 1 in each axis
        for (let i = 1; i < pixels.length; i++) {
            expect(Math.abs(pixels[i].x - pixels[i - 1].x)).toBeLessThanOrEqual(1);
            expect(Math.abs(pixels[i].y - pixels[i - 1].y)).toBeLessThanOrEqual(1);
        }
    });

    it('shallow line (dx > dy) produces correct endpoint', () => {
        const pixels = collect(generateLinePixels(0, 1, 8, 2));
        expect(pixels[pixels.length - 1]).toEqual({ x: 8, y: 2 });
        for (let i = 1; i < pixels.length; i++) {
            expect(Math.abs(pixels[i].x - pixels[i - 1].x)).toBeLessThanOrEqual(1);
            expect(Math.abs(pixels[i].y - pixels[i - 1].y)).toBeLessThanOrEqual(1);
        }
    });
});

describe('generateLinePixels – floating-point inputs', () => {
    it('truncates floats to integers before drawing', () => {
        const pixels = collect(generateLinePixels(0.7, 0.9, 3.2, 3.8));
        // After Math.floor: (0,0) → (3,3)
        expect(pixels[0]).toEqual({ x: 0, y: 0 });
        expect(pixels[pixels.length - 1]).toEqual({ x: 3, y: 3 });
        // All coordinates are integers
        pixels.forEach(p => {
            expect(Number.isInteger(p.x)).toBe(true);
            expect(Number.isInteger(p.y)).toBe(true);
        });
    });

    it('negative floats are floored correctly', () => {
        const pixels = collect(generateLinePixels(-0.5, -0.5, -0.5, -0.5));
        // Math.floor(-0.5) === -1
        expect(pixels).toEqual([{ x: -1, y: -1 }]);
    });
});

describe('generateLinePixels – determinism & connectivity', () => {
    it('is deterministic – same inputs produce identical output', () => {
        const a = collect(generateLinePixels(3, 7, 18, 2));
        const b = collect(generateLinePixels(3, 7, 18, 2));
        expect(a).toEqual(b);
    });

    it('every pixel is 4-connected or 8-connected to its neighbour', () => {
        const pixels = collect(generateLinePixels(0, 0, 15, 9));
        for (let i = 1; i < pixels.length; i++) {
            const dx = Math.abs(pixels[i].x - pixels[i - 1].x);
            const dy = Math.abs(pixels[i].y - pixels[i - 1].y);
            // Chebyshev distance must be exactly 1
            expect(Math.max(dx, dy)).toBe(1);
        }
    });
});