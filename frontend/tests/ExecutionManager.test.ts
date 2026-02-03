// tests/ExecutionManager.test.ts
import { describe, it, expect, beforeEach, vi, type Mock } from 'vitest';
import { ExecutionManager } from '../src/features/drawing/ExecutionManager';
import { createAppState, AppState } from '../src/core/AppState';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
function makeUI(speedVal = '25', densityVal = '10') {
    return {
        speedInput:         { value: speedVal } as any,
        rasterDensityInput: { value: densityVal } as any,
        executeBtn:         { disabled: false } as any,
        prepareBtn:         { disabled: false } as any,
        preparePopup:       { classList: { remove: vi.fn() } } as any,
        toggleButtons:      [{ disabled: false }, { disabled: false }] as any,
    } as any;
}

function makeCanvasManager(executePath: Mock = vi.fn().mockResolvedValue({})) {
    return {
        executePath,
        clearDrawing:  vi.fn(),
        setShapeType:  vi.fn(),
        enableDrawing: vi.fn(),
        hasShape:      vi.fn(() => true),
    } as any;
}

// ---------------------------------------------------------------------------
describe('ExecutionManager – onShapeComplete', () => {
    let state: AppState, updateDraw: Mock, em: ExecutionManager;

    beforeEach(() => {
        state      = createAppState();
        updateDraw = vi.fn();
        em         = new ExecutionManager(makeUI(), state, () => makeCanvasManager(), updateDraw);
    });

    it('records drawnShapeType from selectedShape', () => {
        state.selectedShape = 'circle';
        em.onShapeComplete();
        expect(state.drawnShapeType).toBe('circle');
        expect(updateDraw).toHaveBeenCalled();
    });

    it('does NOT record when selectedShape is null', () => {
        state.selectedShape = null;
        em.onShapeComplete();
        expect(state.drawnShapeType).toBeNull();
        expect(updateDraw).not.toHaveBeenCalled();
    });

    it('does NOT record when selectedShape is "marker"', () => {
        state.selectedShape = 'marker';
        em.onShapeComplete();
        expect(state.drawnShapeType).toBeNull();
        expect(updateDraw).not.toHaveBeenCalled();
    });

    it('works for every valid ShapeType', () => {
        const shapes: any[] = ['freehand', 'square', 'circle', 'triangle', 'line'];
        shapes.forEach(shape => {
            state.selectedShape  = shape;
            state.drawnShapeType = null;
            em.onShapeComplete();
            expect(state.drawnShapeType).toBe(shape);
        });
    });
});

// ---------------------------------------------------------------------------
describe('ExecutionManager – executePath', () => {
    let ui: any, state: AppState, cm: any, updateDraw: Mock, em: ExecutionManager;

    beforeEach(() => {
        ui         = makeUI('50', '8');
        state      = createAppState();
        state.selectedShape = 'square';
        state.fillEnabled   = true;
        state.selectedRasterPattern = 'spiral_raster';
        cm         = makeCanvasManager();
        updateDraw = vi.fn();
        em         = new ExecutionManager(ui, state, () => cm, updateDraw);
        
        // --- FIX: Stub window.alert explicitly for JSDOM ---
        window.alert = vi.fn(); 
    });

    it('calls cm.executePath with correct arguments', async () => {
        await em.executePath();

        expect(cm.executePath).toHaveBeenCalledWith(
            50,                  // speed parsed from input
            'spiral_raster',     // state.selectedRasterPattern
            8,                   // density parsed from input
            true,                // state.fillEnabled
        );
    });

    it('disables execute/prepare buttons before the call', async () => {
        // Make executePath hang so we can inspect mid-flight
        cm.executePath = vi.fn(() => new Promise(() => {}));  // never resolves
        em.executePath();   // fire-and-forget
        expect(ui.executeBtn.disabled).toBe(true);
        expect(ui.prepareBtn.disabled).toBe(true);
    });

    it('on success: clears drawing, nulls drawnShapeType, re-arms tool, closes popup', async () => {
        await em.executePath();

        expect(cm.clearDrawing).toHaveBeenCalled();
        expect(state.drawnShapeType).toBeNull();
        expect(cm.setShapeType).toHaveBeenCalledWith('square');
        expect(cm.enableDrawing).toHaveBeenCalled();
        expect(ui.preparePopup.classList.remove).toHaveBeenCalledWith('active');
    });

    it('on success: re-enables all toggle buttons', async () => {
        await em.executePath();
        ui.toggleButtons.forEach((btn: any) => expect(btn.disabled).toBe(false));
    });

    it('on failure: re-enables execute/prepare buttons, does not clear drawing', async () => {
        cm.executePath = vi.fn().mockRejectedValue(new Error('server down'));

        await em.executePath();   // swallows the error internally

        expect(ui.executeBtn.disabled).toBe(false);
        expect(ui.prepareBtn.disabled).toBe(false);
        expect(cm.clearDrawing).not.toHaveBeenCalled();
    });

    it('returns early when canvasManager is null', async () => {
        const emNull = new ExecutionManager(ui, state, () => null, updateDraw);
        await expect(emNull.executePath()).resolves.toBeUndefined();
    });

    it('alerts and aborts on NaN speed', async () => {
        ui.speedInput.value = 'abc';
        await em.executePath();
        expect(window.alert).toHaveBeenCalledWith('Invalid speed');
        expect(cm.executePath).not.toHaveBeenCalled();
    });

    it('alerts and aborts on zero speed', async () => {
        ui.speedInput.value = '0';
        await em.executePath();
        expect(window.alert).toHaveBeenCalledWith('Invalid speed');
    });

    it('alerts and aborts on negative speed', async () => {
        ui.speedInput.value = '-10';
        await em.executePath();
        expect(window.alert).toHaveBeenCalledWith('Invalid speed');
    });

    it('passes density 0 when fill is disabled', async () => {
        state.fillEnabled = false;
        await em.executePath();
        expect(cm.executePath).toHaveBeenCalledWith(50, 'spiral_raster', 0, false);
    });

    it('does NOT re-arm tool when selectedShape is marker', async () => {
        state.selectedShape = 'marker';
        await em.executePath();
        expect(cm.setShapeType).not.toHaveBeenCalled();
    });
});

// ---------------------------------------------------------------------------
describe('ExecutionManager – clearDrawing', () => {
    let state: AppState, cm: any, updateDraw: Mock, em: ExecutionManager;

    beforeEach(() => {
        state      = createAppState();
        state.selectedShape    = 'triangle';
        state.drawnShapeType   = 'triangle';
        cm         = makeCanvasManager();
        updateDraw = vi.fn();
        em         = new ExecutionManager(makeUI(), state, () => cm, updateDraw);
    });

    it('calls cm.clearDrawing and nulls drawnShapeType', () => {
        em.clearDrawing();
        expect(cm.clearDrawing).toHaveBeenCalled();
        expect(state.drawnShapeType).toBeNull();
        expect(updateDraw).toHaveBeenCalled();
    });

    it('re-arms the selected shape tool', () => {
        em.clearDrawing();
        expect(cm.setShapeType).toHaveBeenCalledWith('triangle');
        expect(cm.enableDrawing).toHaveBeenCalled();
    });

    it('does NOT re-arm when selectedShape is null', () => {
        state.selectedShape = null;
        em.clearDrawing();
        expect(cm.setShapeType).not.toHaveBeenCalled();
    });

    it('does NOT re-arm when selectedShape is marker', () => {
        state.selectedShape = 'marker';
        em.clearDrawing();
        expect(cm.setShapeType).not.toHaveBeenCalled();
    });

    it('is safe when canvasManager is null', () => {
        const emNull = new ExecutionManager(makeUI(), state, () => null, updateDraw);
        expect(() => emNull.clearDrawing()).not.toThrow();
    });
});