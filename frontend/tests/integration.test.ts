// tests/integration.test.ts
/**
 * Cross-module integration tests.  Each scenario wires together the real
 * classes (with only external I/O mocked) to verify the interactions that
 * unit tests cannot cover.
 */
import { describe, it, expect, beforeEach, afterEach, vi, type Mock } from 'vitest';
import { createAppState, AppState } from '../src/core/AppState';
import { HardwareController }       from '../src/features/hardware/HardwareController';
import { RealTimeDrawing }          from '../src/features/drawing/RealTimeDrawing';
import { ExecutionManager }         from '../src/features/drawing/ExecutionManager';
import { ToolHandler }              from '../src/features/drawing/ToolHandler';
import { ModeManager }              from '../src/features/settings/ModeManager';

// ===========================================================================
// Shared factory helpers
// ===========================================================================
function makeClassList(...initial: string[]) {
    const set = new Set(initial);
    return {
        add:      (c: string)  => set.add(c),
        remove:   (c: string)  => set.delete(c),
        contains: (c: string)  => set.has(c),
        toggle:   (c: string, force?: boolean) => {
            if (force !== undefined) { force ? set.add(c) : set.delete(c); }
            else { set.has(c) ? set.delete(c) : set.add(c); }
        },
        _raw: set,
    };
}
function makeEl(...classes: string[]) {
    return { classList: makeClassList(...classes) } as any;
}
function makeBtn(...classes: string[]) {
    const el = makeEl(...classes);
    el.disabled = false;
    el.style    = { pointerEvents: 'auto' };
    return el;
}
function makeCanvas() {
    const c = document.createElement('canvas') as any;
    c.width  = 640; c.height = 480;
    c.getBoundingClientRect = () => ({ left: 0, top: 0, width: 640, height: 480, right: 640, bottom: 480, x: 0, y: 0 });
    c.setPointerCapture      = vi.fn();
    c.releasePointerCapture  = vi.fn();
    return c;
}
function makeVideo(vw = 1280, vh = 720) {
    const v = document.createElement('video') as any;
    Object.defineProperty(v, 'videoWidth',  { value: vw });
    Object.defineProperty(v, 'videoHeight', { value: vh });
    return v;
}

function fullUI() {
    return {
        canvas:   makeCanvas(),
        video:    makeVideo(),
        viewport: { offsetWidth: 640, offsetHeight: 480 } as any,

        processingModeSwitch: { checked: false } as any,
        batchUiElements:      [makeEl(), makeEl()],
        toggleButtons:        [makeBtn(), makeBtn(), makeBtn(), makeBtn(), makeBtn()],

        drawingTools:   makeEl(),
        thermalTools:   makeEl(),
        fixturesTools:  makeEl(),
        drawingUiElements:  [makeEl()],
        thermalUiElements:  [makeEl()],
        fixturesUiElements: [makeEl()],

        penBtn:      makeBtn(),
        squareBtn:   makeBtn(),
        circleBtn:   makeBtn(),
        triangleBtn: makeBtn(),
        lineBtn:     makeBtn(),
        markerBtn:   makeBtn(),
        heatAreaBtn: makeBtn(),
        clearMarkersBtn: makeBtn(),

        roundBrushBtn:   makeBtn(),
        squareBrushBtn:  makeBtn(),
        eraserBrushBtn:  makeBtn(),
        brushSizeSlider: { value: '50' } as any,
        clearBoundaryBtn: makeBtn(),
        applyFixturesBtn: makeBtn(),

        laserBtn: makeBtn(),
        robotBtn: makeBtn(),

        speedInput:         { value: '30' } as any,
        rasterDensityInput: { value: '5' }  as any,
        executeBtn:         makeBtn(),
        prepareBtn:         makeBtn(),
        clearBtn:           makeBtn(),
        preparePopup:       makeEl(),

        robotMarker:          makeEl(),
        averageHeatDisplay:   { textContent: '' } as any,
        resetHeatAreaBtn:     makeBtn(),
    } as any;
}

function fullCM() {
    return {
        setShapeType:          vi.fn(),
        enableDrawing:         vi.fn(),
        disableDrawing:        vi.fn(),
        enableMarkerMode:      vi.fn(),
        disableMarkerMode:     vi.fn(),
        enableHeatAreaMode:    vi.fn(),
        disableHeatAreaMode:   vi.fn(),
        enableFixturesMode:    vi.fn(),
        disableFixturesMode:   vi.fn(),
        setFixturesBrush:      vi.fn(),
        disableFixturesBrush:  vi.fn(),
        clearFixtures:         vi.fn(),
        clearFixturesOnServer: vi.fn().mockResolvedValue({}),
        executeFixtures:       vi.fn().mockResolvedValue({}),
        hasFixtures:           vi.fn(() => false),
        canApplyFixtures:      vi.fn(() => false),
        hasMarkers:            vi.fn(() => false),
        hasShape:              vi.fn(() => false),
        clearDrawing:          vi.fn(),
        executePath:           vi.fn().mockResolvedValue({}),
        showMarkers:           vi.fn(),
        showFixtures:          vi.fn(),
    } as any;
}

function makeWS() {
    return { updateState: vi.fn(), connect: vi.fn() } as any;
}

// ===========================================================================
// 1. Hardware interlock – laser OFF kills robot (and vice-versa)
// ===========================================================================
describe('Integration: hardware safety interlock round-trip', () => {
    let ui: any, state: AppState, ws: any, hw: HardwareController;

    beforeEach(() => {
        vi.useFakeTimers();
        ui    = fullUI();
        state = createAppState();
        ws    = makeWS();
        hw    = new HardwareController(ui, state, ws);
    });
    afterEach(() => vi.useRealTimers());

    it('laser OFF → robot OFF payload, then server ACK clears both locks', () => {
        // Simulate both buttons appearing "active" in the UI
        ui.laserBtn.classList.add('active');
        ui.robotBtn.classList.add('active');

        // Turn laser off
        hw.changeLaserState(false);

        // Verify combined payload
        expect(ws.updateState).toHaveBeenCalledWith(
            expect.objectContaining({ isLaserOn: false, isRobotOn: false })
        );

        // Both pointers locked
        expect(ui.laserBtn.style.pointerEvents).toBe('none');
        expect(ui.robotBtn.style.pointerEvents).toBe('none');

        // Simulate server ACKs arriving
        hw.applyServerLaserState(false);
        hw.applyServerRobotState(false);

        expect(ui.laserBtn.style.pointerEvents).toBe('auto');
        expect(ui.robotBtn.style.pointerEvents).toBe('auto');
        expect(ui.laserBtn.classList.contains('active')).toBe(false);
        expect(ui.robotBtn.classList.contains('active')).toBe(false);
    });

    it('robot OFF → laser OFF mirrors the same pattern', () => {
        ui.laserBtn.classList.add('active');
        ui.robotBtn.classList.add('active');

        hw.changeRobotState(false);

        expect(ws.updateState).toHaveBeenCalledWith(
            expect.objectContaining({ isRobotOn: false, isLaserOn: false })
        );

        hw.applyServerRobotState(false);
        hw.applyServerLaserState(false);

        expect(ui.robotBtn.style.pointerEvents).toBe('auto');
        expect(ui.laserBtn.style.pointerEvents).toBe('auto');
    });

    it('turning one ON does NOT affect the other', () => {
        hw.changeLaserState(true);
        // Only isLaserOn in the payload
        expect(ws.updateState).toHaveBeenCalledWith({ isLaserOn: true });
    });
});

// ===========================================================================
// 2. Tool-selection → execution pipeline
// ===========================================================================
describe('Integration: select tool → draw → execute → re-arm', () => {
    let ui: any, state: AppState, cm: any, th: ToolHandler, em: ExecutionManager;
    let updateDraw: Mock;

    beforeEach(() => {
        ui         = fullUI();
        state      = createAppState();
        cm         = fullCM();
        updateDraw = vi.fn();

        th = new ToolHandler(ui, state, () => cm);
        em = new ExecutionManager(ui, state, () => cm, updateDraw);
    });

    it('full pipeline: select square → shape complete → execute → clear & re-arm', async () => {
        // 1. User selects square
        th.handleShapeSelection(ui.squareBtn, 'square');
        expect(state.selectedShape).toBe('square');

        // 2. User finishes drawing (CanvasManager callback fires)
        em.onShapeComplete();
        expect(state.drawnShapeType).toBe('square');

        // 3. User clicks Execute
        await em.executePath();

        // 4. Verify execute was called with correct args
        expect(cm.executePath).toHaveBeenCalledWith(30, 'line_raster', 0, false);

        // 5. Verify cleanup: drawing cleared, tool re-armed
        expect(cm.clearDrawing).toHaveBeenCalled();
        expect(state.drawnShapeType).toBeNull();
        expect(cm.setShapeType).toHaveBeenCalledWith('square');
        expect(cm.enableDrawing).toHaveBeenCalled();
    });

    it('clear button mid-pipeline re-arms without executing', () => {
        th.handleShapeSelection(ui.circleBtn, 'circle');
        em.onShapeComplete();   // shape exists

        em.clearDrawing();

        expect(cm.clearDrawing).toHaveBeenCalled();
        expect(state.drawnShapeType).toBeNull();
        expect(cm.setShapeType).toHaveBeenCalledWith('circle');
        expect(cm.executePath).not.toHaveBeenCalled();
    });
});

// ===========================================================================
// 3. Mode-switch teardown sequences
// ===========================================================================
describe('Integration: mode-switch teardown → setup sequences', () => {
    let ui: any, state: AppState, cm: any, mm: ModeManager, th: ToolHandler;

    beforeEach(() => {
        ui    = fullUI();
        state = createAppState();
        cm    = fullCM();

        th = new ToolHandler(ui, state, () => cm);
        mm = new ModeManager(
            ui, state, () => cm,
            () => th.updateDrawButtonState(),
            () => th.updateFixturesButtonState(),
            () => th.updateThermalButtonState(),
        );
    });

    it('drawing → fixtures → drawing preserves drawn shape across round-trip', () => {
        // Start in drawing with a shape
        state.currentMode    = 'drawing';
        state.selectedShape  = 'line';
        state.drawnShapeType = 'line';

        // Switch to fixtures
        mm.switchMode('fixturesBtn');
        expect(state.currentMode).toBe('fixtures');

        // Switch back to drawing
        mm.switchMode('drawingBtn');
        expect(state.currentMode).toBe('drawing');

        // drawnShapeType survived the round-trip
        expect(state.drawnShapeType).toBe('line');
    });

    it('fixtures with unapplied changes → thermal clears fixtures', () => {
        state.currentMode = 'fixtures';
        cm.canApplyFixtures.mockReturnValue(true);   // unapplied

        mm.switchMode('thermalBtn');

        expect(cm.clearFixtures).toHaveBeenCalled();
        expect(state.selectedBrushType).toBeNull();
        expect(state.isEraserActive).toBe(false);
    });

    it('thermal → drawing cleans up marker mode', () => {
        state.currentMode   = 'thermal';
        state.selectedShape = 'marker';
        ui.markerBtn.classList.add('selected');

        mm.switchMode('drawingBtn');

        expect(cm.disableMarkerMode).toHaveBeenCalled();
        expect(ui.markerBtn.classList.contains('selected')).toBe(false);
    });
});

// ===========================================================================
// 4. Real-time stroke lifecycle (pointer → WS → end)
// ===========================================================================
describe('Integration: real-time drawing full stroke lifecycle', () => {
    let ui: any, state: AppState, ws: any, rtd: RealTimeDrawing;

    beforeEach(() => {
        ui    = fullUI();
        ui.processingModeSwitch.checked = true;
        state = createAppState();
        state.selectedShape = 'freehand';
        ws    = makeWS();
        rtd   = new RealTimeDrawing(ui, state, ws);
    });

    function pe(cx: number, cy: number, id = 1) {
        return { clientX: cx, clientY: cy, pointerId: id, preventDefault: vi.fn() } as any;
    }

    it('start → move → end produces start event, coordinate updates, end event', () => {
        // Start
        rtd.handleStart(pe(100, 200));
        expect(ws.updateState).toHaveBeenCalledWith({ pathEvent: 'start' });
        expect(state.isRealTimeDrawing).toBe(true);

        // Move
        rtd.handleMove(pe(150, 250));
        expect(state.latestRealTimePos).toEqual({ x: 150, y: 250 });

        // End
        rtd.handleEnd(pe(150, 250));
        expect(ws.updateState).toHaveBeenCalledWith({ pathEvent: 'end' });
        expect(state.isRealTimeDrawing).toBe(false);
        expect(state.latestRealTimePos).toBeNull();
    });

    it('move events before start are silently ignored', () => {
        rtd.handleMove(pe(300, 300));
        // latestRealTimePos stays null because isRealTimeDrawing is false
        expect(state.latestRealTimePos).toBeNull();
    });

    it('end event before start is silently ignored', () => {
        rtd.handleEnd(pe(0, 0));
        // No pathEvent:end sent because drawing never started
        expect(ws.updateState).not.toHaveBeenCalled();
    });

    it('pointer capture is acquired on start and released on end', () => {
        rtd.handleStart(pe(50, 50, 7));
        expect(ui.canvas.setPointerCapture).toHaveBeenCalledWith(7);

        rtd.handleEnd(pe(50, 50, 7));
        expect(ui.canvas.releasePointerCapture).toHaveBeenCalledWith(7);
    });
});

// ===========================================================================
// 5. Fixture brush → apply → button-state flow
// ===========================================================================
describe('Integration: fixture brush workflow with button-state updates', () => {
    let ui: any, state: AppState, cm: any, th: ToolHandler;

    beforeEach(() => {
        ui    = fullUI();
        state = createAppState();
        cm    = fullCM();
        th    = new ToolHandler(ui, state, () => cm);
    });

    it('round brush → apply → buttons reflect applied state', async () => {
        // Enable round brush
        th.handleBrushSelection('round');
        expect(state.selectedBrushType).toBe('round');
        expect(cm.setFixturesBrush).toHaveBeenCalled();

        // Apply fixtures
        await th.applyFixtures();

        // After apply brush state is cleared
        expect(state.selectedBrushType).toBeNull();
        expect(state.isEraserActive).toBe(false);
        expect(cm.disableFixturesBrush).toHaveBeenCalled();
    });

    it('round brush → clear → server clear called, brush reset', async () => {
        th.handleBrushSelection('round');
        await th.clearFixtures();

        expect(cm.clearFixtures).toHaveBeenCalled();
        expect(cm.clearFixturesOnServer).toHaveBeenCalled();
        expect(state.selectedBrushType).toBeNull();
    });

    it('eraser is disabled when no fixtures exist', () => {
        cm.hasFixtures.mockReturnValue(false);
        th.updateFixturesButtonState();
        expect((ui.eraserBrushBtn as any).disabled).toBe(true);
    });
});

// ===========================================================================
// 6. Thermal tool toggle mutual exclusion
// ===========================================================================
describe('Integration: marker ↔ heat-area mutual exclusion', () => {
    let ui: any, state: AppState, cm: any, th: ToolHandler;

    beforeEach(() => {
        ui    = fullUI();
        state = createAppState();
        cm    = fullCM();
        th    = new ToolHandler(ui, state, () => cm);
    });

    it('marker ON → heat ON: marker is turned off first', () => {
        th.selectThermalTool('marker');
        expect(cm.enableMarkerMode).toHaveBeenCalled();

        th.selectThermalTool('heat');
        expect(cm.disableMarkerMode).toHaveBeenCalled();
        expect(cm.enableHeatAreaMode).toHaveBeenCalled();

        expect(ui.markerBtn.classList.contains('selected')).toBe(false);
        expect(ui.heatAreaBtn.classList.contains('selected')).toBe(true);
    });

    it('heat ON → marker ON: heat is turned off first', () => {
        th.selectThermalTool('heat');
        th.selectThermalTool('marker');

        expect(cm.disableHeatAreaMode).toHaveBeenCalled();
        expect(cm.enableMarkerMode).toHaveBeenCalled();

        expect(ui.heatAreaBtn.classList.contains('selected')).toBe(false);
        expect(ui.markerBtn.classList.contains('selected')).toBe(true);
    });

    it('toggling the same tool twice returns to idle state', () => {
        th.selectThermalTool('marker');
        th.selectThermalTool('marker');

        expect(state.selectedShape).toBeNull();
        expect(ui.markerBtn.classList.contains('selected')).toBe(false);
    });
});
