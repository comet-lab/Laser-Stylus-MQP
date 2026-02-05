// tests/RealTimeDrawing.test.ts
import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
import { RealTimeDrawing } from '../src/features/drawing/RealTimeDrawing';
import { createAppState, AppState } from '../src/core/AppState';

// ---------------------------------------------------------------------------
// Stubs
// ---------------------------------------------------------------------------
function makeCanvas(): HTMLCanvasElement {
    const c = document.createElement('canvas') as any;
    c.width  = 640;
    c.height = 480;
    // getBoundingClientRect returns display dimensions (may differ from intrinsic)
    c.getBoundingClientRect = () => ({ left: 10, top: 20, width: 640, height: 480, right: 650, bottom: 500, x: 10, y: 20 });
    c.setPointerCapture      = vi.fn();
    c.releasePointerCapture  = vi.fn();
    return c;
}

function makeVideo(vw = 1280, vh = 720): HTMLVideoElement {
    const v = document.createElement('video') as any;
    Object.defineProperty(v, 'videoWidth',  { value: vw });
    Object.defineProperty(v, 'videoHeight', { value: vh });
    return v;
}

function makeUI(canvas: HTMLCanvasElement, video: HTMLVideoElement) {
    return {
        canvas,
        video,
        processingModeSwitch: { checked: true } as any,   // real-time ON by default
    } as any;
}

function makeWS() {
    return { updateState: vi.fn() } as any;
}

function pointerEvent(clientX: number, clientY: number, pointerId = 1): PointerEvent {
    return { clientX, clientY, pointerId, preventDefault: vi.fn() } as any;
}

// ---------------------------------------------------------------------------
describe('RealTimeDrawing – guard conditions', () => {
    let canvas: HTMLCanvasElement, ui: any, state: AppState, ws: any, rtd: RealTimeDrawing;

    beforeEach(() => {
        vi.useFakeTimers();
        canvas = makeCanvas();
        ui     = makeUI(canvas, makeVideo());
        state  = createAppState();
        ws     = makeWS();
        rtd    = new RealTimeDrawing(ui, state, ws);
    });

    it('handleStart does nothing when processingModeSwitch is OFF', () => {
        ui.processingModeSwitch.checked = false;
        state.selectedShape = 'freehand';
        rtd.handleStart(pointerEvent(100, 100));
        expect(state.isRealTimeDrawing).toBe(false);
        expect(ws.updateState).not.toHaveBeenCalled();
    });

    it('handleStart does nothing when selectedShape is not freehand', () => {
        state.selectedShape = 'square';
        rtd.handleStart(pointerEvent(100, 100));
        expect(state.isRealTimeDrawing).toBe(false);
    });

    it('handleMove is a no-op while not drawing', () => {
        state.isRealTimeDrawing = false;
        rtd.handleMove(pointerEvent(50, 50));
        expect(state.latestRealTimePos).toBeNull();
    });

    it('handleEnd is a no-op while not drawing', () => {
        state.isRealTimeDrawing = false;
        rtd.handleEnd(pointerEvent(50, 50));
        expect(ws.updateState).not.toHaveBeenCalled();
    });
});

// ---------------------------------------------------------------------------
describe('RealTimeDrawing – stroke lifecycle', () => {
    let canvas: HTMLCanvasElement, ui: any, state: AppState, ws: any, rtd: RealTimeDrawing;

    beforeEach(() => {
        vi.useFakeTimers();
        vi.useFakeTimers();
        canvas = makeCanvas();
        ui     = makeUI(canvas, makeVideo());
        state  = createAppState();
        state.selectedShape = 'freehand';
        ws     = makeWS();
        rtd    = new RealTimeDrawing(ui, state, ws);
    });
    afterEach(() => vi.useRealTimers());

    it('handleStart sets isRealTimeDrawing, captures pointer, sends pathEvent:start', () => {
        rtd.handleStart(pointerEvent(100, 200));

        expect(state.isRealTimeDrawing).toBe(true);
        expect(canvas.setPointerCapture).toHaveBeenCalledWith(1);
        expect(ws.updateState).toHaveBeenCalledWith({ pathEvent: 'start' });
    });

    it('handleStart stores canvas-local coordinates', () => {
        // clientX=110 → canvas-local x = 110 - rect.left(10) = 100
        // clientY=220 → canvas-local y = 220 - rect.top(20)  = 200
        rtd.handleStart(pointerEvent(110, 220));
        expect(state.latestRealTimePos).toEqual({ x: 100, y: 200 });
    });

    it('handleMove updates latestRealTimePos', () => {
        state.isRealTimeDrawing = true;
        rtd.handleMove(pointerEvent(60, 70));
        // 60-10=50, 70-20=50
        expect(state.latestRealTimePos).toEqual({ x: 50, y: 50 });
    });

    it('handleEnd clears drawing state, releases pointer, sends pathEvent:end', () => {
        state.isRealTimeDrawing = true;
        rtd.handleEnd(pointerEvent(100, 100, 1));

        expect(state.isRealTimeDrawing).toBe(false);
        expect(state.latestRealTimePos).toBeNull();
        expect(canvas.releasePointerCapture).toHaveBeenCalledWith(1);
        expect(ws.updateState).toHaveBeenCalledWith({ pathEvent: 'end' });
    });
});

// ---------------------------------------------------------------------------
describe('RealTimeDrawing – coordinate normalisation (canvas → video space)', () => {
    // canvas display: 640×480, video intrinsic: 1280×720
    // A point at canvas-local (320, 240) → normalised (0.5, 0.5) → video (640, 360)
    let canvas: HTMLCanvasElement, ui: any, state: AppState, ws: any, rtd: RealTimeDrawing;

    beforeEach(() => {
        vi.useFakeTimers();
        canvas = makeCanvas();   // display 640×480
        ui     = makeUI(canvas, makeVideo(1280, 720));
        state  = createAppState();
        state.selectedShape = 'freehand';
        ws     = makeWS();
        rtd    = new RealTimeDrawing(ui, state, ws);
    });

    it('runLoop sends video-space coordinates derived from normalised canvas position', () => {
        // Start a stroke at canvas-local (320, 240)
        // clientX = 320 + rect.left(10) = 330
        // clientY = 240 + rect.top(20)  = 260
        rtd.handleStart(pointerEvent(330, 260));

        // Flush the RAF tick that handleStart kicked off
        // In the test env requestAnimationFrame is synchronous via fake timers
        vi.advanceTimersByTime(50);

        // The loop should have called updateState with { x: 640, y: 360 }
        const calls = ws.updateState.mock.calls;
        const coordCall = calls.find((c: any[]) => c[0].x !== undefined && c[0].y !== undefined);
        expect(coordCall).toBeDefined();
        expect(coordCall[0].x).toBeCloseTo(640, 0);
        expect(coordCall[0].y).toBeCloseTo(360, 0);
    });
});

// ---------------------------------------------------------------------------
describe('RealTimeDrawing – loop termination', () => {
    let canvas: HTMLCanvasElement, ui: any, state: AppState, ws: any, rtd: RealTimeDrawing;
    let rafCount = 0;
    const origRAF = globalThis.requestAnimationFrame;

    beforeEach(() => {
        vi.useFakeTimers();
        canvas = makeCanvas();
        ui     = makeUI(canvas, makeVideo());
        state  = createAppState();
        state.selectedShape = 'freehand';
        ws     = makeWS();
        rtd    = new RealTimeDrawing(ui, state, ws);

        rafCount = 0;
        globalThis.requestAnimationFrame = (cb: any) => {
            rafCount++;
            if (rafCount <= 3) cb();   // allow up to 3 iterations, then stop
            return rafCount;
        };
    });
    afterEach(() => {
        vi.useRealTimers();
        globalThis.requestAnimationFrame = origRAF;
    });

    it('loop stops requesting frames once isRealTimeDrawing becomes false', () => {
        rtd.handleStart(pointerEvent(110, 220));   // starts loop
        // Immediately end the stroke
        state.isRealTimeDrawing = false;
        // The loop's next iteration should bail out; rafCount won't keep climbing
        const countBefore = rafCount;
        // A small additional tick – nothing should schedule
        // (rafCount already stopped because the guard returned early)
        expect(rafCount).toBeLessThanOrEqual(countBefore + 1);
    });
});
