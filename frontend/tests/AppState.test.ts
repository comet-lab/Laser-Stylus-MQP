// tests/AppState.test.ts
import { describe, it, expect } from 'vitest';
import { createAppState, AppState } from '../src/core/AppState';

describe('createAppState', () => {
    it('returns a fresh object with all expected keys', () => {
        const state = createAppState();
        const keys: (keyof AppState)[] = [
            'laserConfirmationTimeout',
            'robotConfirmationTimeout',
            'selectedShape',
            'drawnShapeType',
            'isRealTimeDrawing',
            'latestRealTimePos',
            'fillEnabled',
            'selectedRasterPattern',
            'currentMode',
            'selectedBrushType',
            'isEraserActive',
        ];
        keys.forEach(k => expect(state).toHaveProperty(k));
    });

    it('hardware timeouts default to null', () => {
        const s = createAppState();
        expect(s.laserConfirmationTimeout).toBeNull();
        expect(s.robotConfirmationTimeout).toBeNull();
    });

    it('drawing state defaults are null / false', () => {
        const s = createAppState();
        expect(s.selectedShape).toBeNull();
        expect(s.drawnShapeType).toBeNull();
        expect(s.isRealTimeDrawing).toBe(false);
        expect(s.latestRealTimePos).toBeNull();
    });

    it('fill defaults to disabled with line_raster pattern', () => {
        const s = createAppState();
        expect(s.fillEnabled).toBe(false);
        expect(s.selectedRasterPattern).toBe('line_raster');
    });

    it('current mode defaults to drawing', () => {
        expect(createAppState().currentMode).toBe('drawing');
    });

    it('brush / eraser state defaults are null / false', () => {
        const s = createAppState();
        expect(s.selectedBrushType).toBeNull();
        expect(s.isEraserActive).toBe(false);
    });

    it('each call returns an independent object (no shared reference)', () => {
        const a = createAppState();
        const b = createAppState();
        a.fillEnabled = true;
        expect(b.fillEnabled).toBe(false);
    });
});
